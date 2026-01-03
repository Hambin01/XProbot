#!/usr/bin/env python3
"""
Author: lei.zeng@leapting.com, cong.liu@leapting.com
"""
import getpass
import numpy as np
import math
import cv2
from scipy.optimize import curve_fit
import rospy
from sensor_msgs.msg import Image
import torch
from ultralytics import YOLO
from cv_bridge import CvBridge
import time
from std_msgs.msg import Header, String
import parsing_yolo8_model_name
from dynamic_reconfigure.server import Server
from yolov8_result.cfg import Yolov8Config
from skimage import measure

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

import base64

def cv2_to_base64(cv_image):
    img_msg = String()
    img_encode = cv2.imencode('.png', cv_image)[1]
    base64_data_str = str(base64.b64encode(img_encode))[2:-1]
    img_msg.data = base64_data_str
    return img_msg

class Yolov8Process(object):
    def __init__(self):
        model_ab_path = relative_to_absolute_path(
            rospy.get_param("~model_folder_path", "~/catkin_ws/src/yolov8_result/model"))
        try:
            self.model_info = parsing_yolo8_model_name.parsing_name(
                model_ab_path)
        except Exception as e:
            rospy.logerr('Yolov8Process parsing model error: %s' % str(e))
            return

        self.model = YOLO('%s/%s' %
                          (model_ab_path, self.model_info['model_name']))
        self.search_max = rospy.get_param("~search_max", True)
        self.search_centroid = rospy.get_param("~search_centroid", True)
        self.search_corner = rospy.get_param("~search_corner", False)
        # image frame, lefter-up corner is origin
        self.distance_ratio_x = rospy.get_param("~distance_ratio_x", 0.5)
        self.distance_ratio_y = rospy.get_param("~distance_ratio_y", 0.5)
        self.bridge = CvBridge()
        self.raw_img = None
        self.seg_img = None
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        self.coordinate = []
        self.filter_radius = rospy.get_param("~filter_radius", 15)
        self.filter_amplitude = rospy.get_param("~filter_amplitude", 110)
        self.filter_max_area = rospy.get_param("~filter_max_area", 9000)
        self.refine_enable = rospy.get_param("~refine_enable", True)
        self.corner_show_enable = rospy.get_param("~corner_show_enable", True)
        
        self.compressed_pub_enable = rospy.get_param("~compressed_pub_enable", True)
        self.compressed_raw_pub = rospy.Publisher('compressed_raw_base64', String, queue_size=1, latch=True)
        self.compressed_raw_timer = rospy.Timer(rospy.Duration(1), self.compressed_raw_cb)
        self.compressed_res_pub = rospy.Publisher('compressed_res_base64', String, queue_size=1, latch=True)
        self.compressed_res_timer = rospy.Timer(rospy.Duration(1), self.compressed_res_cb)

        self.img_pub = rospy.Publisher('yolov8_res_img', Image, queue_size=1)
        # if self.search_corner:
        #     self.points_pub = rospy.Publisher(
        #         'post_processing_img_mark_points', Image, queue_size=1)
        self.corner_pub = rospy.Publisher('post_corner_coordinate', PoseArray, queue_size=1)
        self.dy_recfg_srv = Server(Yolov8Config, self.dy_recfg_cb)
        rospy.Subscriber('image_publisher/image_raw',
                         Image, self.image_cb, queue_size=1)
        self.last_time = 0
        self.min_rest = rospy.get_param("~min_rest", 0.5)

        self.if_enable = False
        rospy.Subscriber('trig',
                         Header,
                         self.trig_cb,
                         queue_size=10)
        self.model_predict_first()

    def dy_recfg_cb(self, config, level):
        self.distance_ratio_x = config['distance_ratio_x']
        self.distance_ratio_y = config['distance_ratio_y']
        return config

    def trig_cb(self, msg):
        if msg.frame_id == "enable_yolov8" and msg.seq >= 1:
            rospy.loginfo("enable yolov8")
            self.if_enable = True
        elif msg.frame_id == "enable_yolov8" and msg.seq == 0:
            rospy.loginfo("unable yolov8")
            self.if_enable = False

    def model_predict_first(self):
        devicee = torch.device(0) if torch.cuda.is_available() else torch.device('cpu')
        self.model.predict(source=np.zeros((100,100,3)), save=False, save_txt=False, device=devicee, conf=self.model_info['conf'], iou=self.model_info['iou'], retina_masks=True)
        rospy.loginfo("first prediction finished")
    
    def compressed_raw_cb(self, event):
        if (not self.if_enable) or (not self.compressed_pub_enable) or (self.raw_img is None):
            return
        compressed_start = time.time()
        resized_raw_img = cv2.resize(self.raw_img, None, fx=0.2, fy=0.2, interpolation=cv2.INTER_AREA)
        compressed_end = time.time()
        self.compressed_raw_pub.publish(cv2_to_base64(resized_raw_img))
        print('compressed raw time: '+str(round((compressed_end-compressed_start)*1000,1))+'ms')
    
    def compressed_res_cb(self, event):
        if (not self.if_enable) or (not self.compressed_pub_enable) or (self.seg_img is None):
            return
        compressed_start = time.time()
        resized_seg_img = cv2.resize(self.seg_img, None, fx=0.2, fy=0.2, interpolation=cv2.INTER_AREA)
        compressed_end = time.time()
        self.compressed_res_pub.publish(cv2_to_base64(resized_seg_img))
        print('compressed res time: '+str(round((compressed_end-compressed_start)*1000,1))+'ms')
    
    def image_cb(self, image_msg):

        if (not self.if_enable) or (self.min_rest > 0 and time.time() - self.last_time < self.min_rest):
            return

        self.raw_img = self.bridge.imgmsg_to_cv2(
            image_msg, desired_encoding='passthrough')
            
        devicee = torch.device(0) if torch.cuda.is_available() else torch.device('cpu')
        results = self.model.predict(source=self.raw_img, save=False, save_txt=False, device=devicee, conf=self.model_info['conf'], iou=self.model_info['iou'], retina_masks=True)

        for result in results:
            if result.masks is None:
                rospy.loginfo("no result")
                continue
            res_tensor = result.masks.data
            try:
                n = res_tensor.size()[0]
            except:
                n = 0
            if n == 0:
                rospy.loginfo("no result")
                continue
            else:
                self.last_time = time.time()

            max_obj = {'idx': 0, 'area': 0, 'dist_to_c': math.inf}
            for i in range(0, n):
                np_img = res_tensor[i].cpu().numpy()
                # cv2.imwrite("/home/leizeng/catkin_ws/src/my_pkg/yolov8_result/src/res%s.png" % str(i), np.uint8(np_img*255))
                if self.search_max:
                    s = np.sum(np_img)
                    if s > max_obj['area']:
                        max_obj['area'] = s
                        max_obj['idx'] = i
                elif self.search_centroid:
                    cx, cy = self.calculate_centroid(np.uint8(np_img*255))
                    d = np.sqrt((cx - image_msg.width*self.distance_ratio_x)**2 +
                                (cy - image_msg.height*self.distance_ratio_y)**2)
                    if d < max_obj['dist_to_c']:
                        max_obj['dist_to_c'] = d
                        max_obj['idx'] = i

                else:
                    self.img_pub.publish(self.bridge.cv2_to_imgmsg(
                        np.uint8(np_img*255), encoding="mono8"))
            if self.search_max or self.search_centroid:
                np_img = res_tensor[max_obj['idx']].cpu().numpy()
                self.seg_img = np.uint8(np_img*255)
                if self.refine_enable:
                    if self.segment_refine():
                        self.img_pub.publish(self.bridge.cv2_to_imgmsg(
                            self.seg_img, encoding="mono8"))
                else:
                    self.img_pub.publish(self.bridge.cv2_to_imgmsg(self.seg_img, encoding="mono8"))

    def calculate_centroid(self, np_img):
        _, thresh = cv2.threshold(np_img, 127, 255, 0)
        M = cv2.moments(thresh)

        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return (cX, cY)

    def segment_refine(self):
        image = self.raw_img
        (r, g, b) = cv2.split(image)
        image = cv2.merge((b, g, r))
        # RGB2HSV
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  # RGB转HSV空间
        #  提取V空间
        v_channel = hsv_image[:, :, 2]
        # s_channel = hsv_image[:, :, 1]
        #  对通道进行CLAHE直方图均衡（降低过曝光的影响）
        clip_limit = 10
        tile_grid_size = (8, 8)
        clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=tile_grid_size)
        equalized_v_channel = clahe.apply(v_channel)
        # equalized_s_channel = clahe.apply(s_channel)
        # 最后合并
        hsv_image[:, :, 2] = equalized_v_channel
        # hsv_image[:, :, 1] = equalized_s_channel
        image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)  # 转换回RGB空间
        #  ROI
        rett, bina = cv2.threshold(self.connected_component(
            self.seg_img).astype('uint8'), 0, 255, cv2.THRESH_BINARY)  # 消除误分割的无关区域
        seg_rgb_img = cv2.cvtColor(bina, cv2.COLOR_GRAY2RGB)  # 将分割区域转为彩色，便于后续融合操作
        pengzhang = cv2.dilate(seg_rgb_img, self.kernel, iterations=6)  # 将初步分割的结果扩大
        _, dst = cv2.threshold(pengzhang, 125, 255, cv2.THRESH_BINARY_INV)  # 膨胀后的二值图反转,反转后目标部分像素值为0，与原图相加即提取ROI

        output = cv2.add(image, dst)  # 得到ROI图片
        #  上下采样，降低运算时间
        py_down = cv2.pyrDown(output)
        mean = cv2.pyrMeanShiftFiltering(py_down, 10, 100)
        py_up = cv2.pyrUp(mean)
        gamma = self.gamma_trans(py_up, 2)  # 均值漂移+gamma校正（分割前预处理）
        #  分割
        gray = cv2.cvtColor(gamma, cv2.COLOR_BGR2GRAY)  # 灰度化ROI，为后续分割作准备
        _, dst1 = cv2.threshold(
            gray, 200, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)  # 分割
        gray_output = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)  # 灰度化ROI彩色图像
        _, dst2 = cv2.threshold(
            gray_output, 200, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        lty = self.connected_component(dst1).astype('uint8')
        _, binary = cv2.threshold(lty, 0, 255, cv2.THRESH_BINARY)  # 对dst1进行连通域处理，去除小区域
        binary2 = cv2.add(dst2, binary)  # dst1由于预处理的影响，可能导致边缘发生变化，与dst2融合，增加边缘细节
        lty1 = self.connected_component(binary2).astype('uint8')
        _, binary3 = cv2.threshold(lty1, 0, 255, cv2.THRESH_BINARY)  # 去除小区域
        closing = cv2.morphologyEx(
            binary, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10)))  # 闭操作，填充部分孔洞区域
        contours, hier = cv2.findContours(
            closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 找轮廓
        img = cv2.fillPoly(closing, contours, color=255)  # 根据轮廓，填充封闭的轮廓内部孔洞
        edges = cv2.Canny(img, 50, 200, apertureSize=3)  # 对于大的缺口，用直线检测画线条，封闭轮廓
        dlines = cv2.HoughLinesP(
            edges, 1, np.pi / 180, 22, minLineLength=50, maxLineGap=10)
        for line in dlines:
            x0 = int(round(line[0][0]))
            y0 = int(round(line[0][1]))
            x1 = int(round(line[0][2]))
            y1 = int(round(line[0][3]))
            cv2.line(img, (x0, y0), (x1, y1), 255, 1, cv2.LINE_AA)
        contours1, hier1 = cv2.findContours(
            img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 再填充
        cv2.fillPoly(img, contours1, color=255)
        img_fushi = cv2.erode(
            img, kernel=cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)), iterations=1)
        binary3 = cv2.add(img_fushi, binary3)  # 对img进行适当腐蚀，然后融合，进一步填充binary3
        # 对binary3腐蚀，得到更精确的直线检测
        bina_fushi = cv2.erode(
            binary3, kernel=cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)), iterations=2)
        edges_fushi = cv2.Canny(bina_fushi, 50, 200, apertureSize=3)  # 提取binary3腐蚀后的边缘，准备直线检测
        binary3_rgb = cv2.cvtColor(binary3, cv2.COLOR_GRAY2RGB)
        img_result = image.copy()
        bina_fushi_copy = bina_fushi.copy()
        #  顶点检测
        Points, points = self.corner_detection(edges_fushi)  # 后者是正常点
        result = self.segment_filling(bina_fushi_copy, Points, points)
        for point in points:
            x = int(round(point[0]))
            y = int(round(point[1]))
            cv2.circle(binary3_rgb, (x, y), 3, (0, 0, 255), 2)
            cv2.circle(img_result, (x, y), 2, (0, 0, 255), 2)
        
        if self.search_corner:
            # self.points_pub.publish(self.bridge.cv2_to_imgmsg(
            #     np.uint8(binary3_rgb), encoding="mono8"))
            # self.points_pub.publish(self.bridge.cv2_to_imgmsg(
            #     np.uint8(img_result), encoding="mono8"))
            self.corner_pub.publish(points_to_posearray(points))
            if self.corner_show_enable:
                cv2.imshow('corner', img_result)
                cv2.waitKey(10)

        if self.warring_func(result, n=5, s=0.15, d=50, co=350):
            self.seg_img = result
            return True
        else:
            return False

    def warring_func(self, img, n, s, d, co):
        res = True
        if True:
            S = 0  # 面积
            labels = measure.label(img, connectivity=2)
            properties = measure.regionprops(labels)
            for prop in properties:
                S += prop.area
                self.coordinate.append(prop.centroid)
            if S / (img.shape[0] * img.shape[1]) < s:
                rospy.logwarn(
                    'segmentation area too small, may not be complete.')
                res = False
            if math.sqrt((self.coordinate[0][1] - img.shape[1] / 2) ** 2 + (
                    self.coordinate[0][0] - img.shape[0] / 2) ** 2) > co:
                rospy.logwarn('abnormal position of the segment.')
                res = False

        if len(self.coordinate) == n:
            Dvalue = 0
            for i in range(0, n):
                if i < 1:
                    continue
                else:
                    Dvalue += math.sqrt((self.coordinate[i][1] - self.coordinate[i - 1][1]) ** 2 + (
                            self.coordinate[i][0] - self.coordinate[i - 1][0]) ** 2)
            if Dvalue / (n - 1) > d:
                rospy.logwarn('unstable segment')
                res = False
            del self.coordinate[0]

        return res

    def gamma_trans(self, img, gamma):  # gamma函数处理
        gamma_table = [np.power(x / 255.0, gamma) *
                       255.0 for x in range(256)]  # 建立映射表
        gamma_table = np.round(np.array(gamma_table)
                               ).astype(np.uint8)  # 颜色值为整数
        return cv2.LUT(img, gamma_table)  # 图片颜色查表。另外可以根据光强（颜色）均匀化原则设计自适应算法。

    def connected_component(self, image):
        # 标记输入的3D图像
        label, num = measure.label(
            image, connectivity=2, return_num=True)  # 获取二值图像的连通域标签和标签数量
        if num < 1:
            return image

        # 获取对应的region对象
        region = measure.regionprops(label)
        # 获取每一块区域面积并排序
        num_list = [i for i in range(1, num + 1)]  # 将每一个标签存入该列表
        area_list = [region[i - 1].area for i in num_list]  # 获取对应标签的面积存入该列表
        num_list_sorted = sorted(
            num_list, key=lambda x: area_list[x - 1])[::-1]  # 排序一下面积的大小存入一个列表
        # 去除面积较小的连通域
        count = len(num_list_sorted)
        indexs = range(count)
        if count > 1:
            for index in indexs[:]:
                # 存放面积的列表和存放标签的列表对应相同的index，如果一个连通域面积小于16，则把对应标签的连通域标0（去除）
                if area_list[index] < self.filter_max_area:
                    label[label == num_list[index]] = 0
        return label

    def cross_point(self, image, line1, line2):
        k0 = line1[0]
        b0 = line1[1]
        k1 = line2[0]
        b1 = line2[1]
        h, w = image.shape[0], image.shape[1]
        if np.isclose(k0, k1):
            if np.isclose(b0, b1):
                return None
            else:
                return None  # 无交点
        else:
            x = (b1 - b0) / (k0 - k1)
            y = k0 * x + b0
            if 40 <= x <= w - 40 and 20 <= y <= h - 20:
                return (x, y)
            if np.isclose(y, 0, atol=0.1) or np.isclose(y, h - 1, atol=0.1):
                return (x, y)
            else:
                return None

    def corner_detection(self, edge):
        h, w = edge.shape[0], edge.shape[1]
        dlines = cv2.HoughLinesP(edge, 1, np.pi / 180, 90, minLineLength=30, maxLineGap=55)
        K, KK, A, B, K1, K2, K3, K4, XMIN, XMAX, YMIN, YMAX, IN \
            = [], [], [], [], [], [], [], [], [], [], [], [], []
        mid_x, mid_y = [], []
        count1, count2, count3, count4, switch = 0, 0, 0, 0, 0
        for line in dlines:
            x0 = int(round(line[0][0]))
            y0 = int(round(line[0][1]))
            x1 = int(round(line[0][2]))
            y1 = int(round(line[0][3]))
            mid_x.append((int((x0 + x1) / 2), int((y0 + y1) / 2)))
            mid_y.append((int((y0 + y1) / 2), int((x0 + x1) / 2)))
            if x1 - x0 == 0:
                print('存在垂直的直线')
                switch = 1
                K.append((y1 - y0) / (x1 - (x0 + 0.1)))
                KK.append(
                    ((y1 - y0) / (x1 - (x0 + 0.1)), (y0 - ((y1 - y0) / (x1 - (x0 + 0.1))) * (x0 + 0.1)), (x0, y0),
                     (x1, y1)))
                B.append(y0 - ((y1 - y0) / (x1 - (x0 + 0.1))) * (x0 + 0.1))
                pass
            else:
                K.append((y1 - y0) / (x1 - x0))
                KK.append(((y1 - y0) / (x1 - x0), (y0 - ((y1 - y0) / (x1 - x0)) * x0), (x0, y0), (x1, y1)))
                B.append(y0 - ((y1 - y0) / (x1 - x0)) * x0)
        # 最值点筛选
        for xm in mid_x:
            if abs(max(mid_x)[0] - xm[0]) < (w * 1 / 30):  # 如果最值点有距离很近的线段质心点，计算二者的距离
                XMAX.append(
                    (np.sqrt(((xm[0] - (w - 1)) ** 2) + ((xm[1] - (h - 1) / 2) ** 2)), xm))  # 选择离图像边缘中心最近的质心点
            if abs(min(mid_x)[0] - xm[0]) < (w * 1 / 30):
                XMIN.append(
                    (np.sqrt(((xm[0] - 0) ** 2) + ((xm[1] - (h - 1) / 2) ** 2)), xm))
        for ym in mid_y:
            if abs(max(mid_y)[0] - ym[0]) < (h * 1 / 20):
                YMAX.append(
                    (np.sqrt(((ym[0] - (h - 1)) ** 2) + ((ym[1] - (w - 1) / 2) ** 2)), ym))
            if abs(min(mid_y)[0] - ym[0]) < (h * 1 / 20):
                YMIN.append(
                    (np.sqrt(((ym[0] - 0) ** 2) + ((ym[1] - (w - 1) / 2) ** 2)), ym))
        A.append(KK[mid_x.index(min(XMAX)[1])])  # right
        A.append(KK[mid_x.index(min(XMIN)[1])])  # left
        A.append(KK[mid_y.index(min(YMAX)[1])])  # down
        A.append(KK[mid_y.index(min(YMIN)[1])])  # up
        # 没有四条直线时，删除A中相同的直线
        for a in range(0, len(A)):
            for s in range(a + 1, len(A)):
                if abs(A[a][0]) <= 2:
                    if (abs(A[a][0] - A[s][0]) < 0.05) * (abs(A[a][1] - A[s][1]) < 25):
                        del A[s]
                        A.append((0, 99999))
                if 2 < abs(A[a][0]) <= 30:
                    if (abs(A[a][0] - A[s][0]) < 0.5) * (abs(A[a][1] - A[s][1]) < 200):
                        del A[s]
                        A.append((0, 99999))
                if abs(A[a][0]) > 30:
                    if (abs(A[a][0] - A[s][0]) < 25) * (abs(A[a][1] - A[s][1]) < 25000):
                        del A[s]
                        A.append((0, 99999))
                else:
                    pass
        indices_to_remove = [mid_x.index(min(XMAX)[1]), mid_x.index(min(XMIN)[1]),
                             mid_y.index(min(YMAX)[1]), mid_y.index(min(YMIN)[1])]
        NEW_KK = [item for index, item in enumerate(KK) if index not in indices_to_remove]
        K1.append((A[2][0], A[2][1]))
        k0 = A[2][0]
        b0 = A[2][1]
        K2.append((A[3][0], A[3][1]))
        k1 = A[3][0]
        b1 = A[3][1]
        K3.append((A[0][0], A[0][1]))
        k2 = A[0][0]
        b2 = A[0][1]
        K4.append((A[1][0], A[1][1]))
        k3 = A[1][0]
        b3 = A[1][1]
        # 由筛选出的四条直线根据k/b进一步搜集斜率和b相近的直线，为拟合做准备
        for i in range(0, len(NEW_KK)):
            if (abs(NEW_KK[i][0] - k0) <= 0.05) * (abs(NEW_KK[i][1] - b0) <= 35):
                K1.append((NEW_KK[i][0], NEW_KK[i][1]))
            else:
                if (abs(NEW_KK[i][0] - k1) <= 0.05) * (abs(NEW_KK[i][1] - b1) <= 35):
                    K2.append((NEW_KK[i][0], NEW_KK[i][1]))
        for i in range(0, len(NEW_KK)):
            if abs(k2) > 1:
                if (abs(NEW_KK[i][0] - k2) <= 1) * (abs(NEW_KK[i][1] - b2) <= 230):
                    K3.append((NEW_KK[i][0], NEW_KK[i][1]))
            else:
                if (abs(NEW_KK[i][0] - k2) <= 0.1) * (abs(NEW_KK[i][1] - b2) <= 20):
                    K3.append((NEW_KK[i][0], NEW_KK[i][1]))
            if abs(k3) > 1:
                if (abs(NEW_KK[i][0] - k3) <= 1) * (abs(NEW_KK[i][1] - b3) <= 230):
                    K4.append((NEW_KK[i][0], NEW_KK[i][1]))
            else:
                if (abs(NEW_KK[i][0] - k3) <= 0.1) * (abs(NEW_KK[i][1] - b3) <= 20):
                    K4.append((NEW_KK[i][0], NEW_KK[i][1]))
        for i in range(len(KK)):
            IN.append(K[i] + B[i])
        # 计算斜率k和b的均值
        if len(K1) == 0 or K1[0][1] == 99999:
            k0, b0 = 0, 99999
        else:
            binary_image1 = np.zeros((720, 1280), dtype=np.uint8)
            for k in K1:
                x0 = dlines[IN.index(k[0] + k[1])][0][0]
                y0 = dlines[IN.index(k[0] + k[1])][0][1]
                x1 = dlines[IN.index(k[0] + k[1])][0][2]
                y1 = dlines[IN.index(k[0] + k[1])][0][3]
                cv2.line(binary_image1, (x0, y0), (x1, y1), 255, 1, cv2.LINE_AA)
            if x0 == x1:
                count1 = 1
            white_pixels = np.where(binary_image1 >= 1)
            xdata = white_pixels[1]
            ydata = white_pixels[0]
            if count1 == 1:
                k0, b0 = K1[0][0], K1[0][1]
            else:
                params, covariance = curve_fit(linear_model, xdata, ydata)
                k0, b0 = params
        if len(K2) == 0 or K2[0][1] == 99999:
            k1, b1 = 0, 99999
        else:
            binary_image1 = np.zeros((720, 1280), dtype=np.uint8)
            for k in K2:
                x0 = dlines[IN.index(k[0] + k[1])][0][0]
                y0 = dlines[IN.index(k[0] + k[1])][0][1]
                x1 = dlines[IN.index(k[0] + k[1])][0][2]
                y1 = dlines[IN.index(k[0] + k[1])][0][3]
                cv2.line(binary_image1, (x0, y0), (x1, y1), 255, 1, cv2.LINE_AA)
            if x0 == x1:
                count2 = 1
            white_pixels = np.where(binary_image1 >= 1)
            xdata = white_pixels[1]
            ydata = white_pixels[0]
            if count2 == 1:
                k1, b1 = K2[0][0], K2[0][1]
            else:
                params, covariance = curve_fit(linear_model, xdata, ydata)
                k1, b1 = params
        if len(K3) == 0 or K3[0][1] == 99999:
            k2, b2 = 0, 99999
        else:
            binary_image1 = np.zeros((720, 1280), dtype=np.uint8)
            for k in K3:
                x0 = dlines[IN.index(k[0] + k[1])][0][0]
                y0 = dlines[IN.index(k[0] + k[1])][0][1]
                x1 = dlines[IN.index(k[0] + k[1])][0][2]
                y1 = dlines[IN.index(k[0] + k[1])][0][3]
                cv2.line(binary_image1, (x0, y0), (x1, y1), 255, 1, cv2.LINE_AA)
            if x0 == x1:
                count3 = 1
            white_pixels = np.where(binary_image1 >= 1)
            xdata = white_pixels[1]
            ydata = white_pixels[0]
            if count3 == 1:
                k2, b2 = K3[0][0], K3[0][1]
            else:
                params, covariance = curve_fit(linear_model, xdata, ydata)
                k2, b2 = params
        if len(K4) == 0 or K4[0][1] == 99999:
            k3, b3 = 0, 99999
        else:
            binary_image1 = np.zeros((720, 1280), dtype=np.uint8)
            for k in K4:
                x0 = dlines[IN.index(k[0] + k[1])][0][0]
                y0 = dlines[IN.index(k[0] + k[1])][0][1]
                x1 = dlines[IN.index(k[0] + k[1])][0][2]
                y1 = dlines[IN.index(k[0] + k[1])][0][3]
                cv2.line(binary_image1, (x0, y0), (x1, y1), 255, 1, cv2.LINE_AA)
                if x0 == x1:
                    count4 = 1
            white_pixels = np.where(binary_image1 >= 1)
            xdata = white_pixels[1]
            ydata = white_pixels[0]
            if count4 == 1:
                k3, b3 = K4[0][0], K4[0][1]
            else:
                params, covariance = curve_fit(linear_model, xdata, ydata)
                k3, b3 = params
        Calculate = [(k0, b0), (k1, b1), (k2, b2), (k3, b3), (0, 0), (0, edge.shape[0] - 1)]
        Calculate1 = [(k0, b0), (k1, b1), (k2, b2), (k3, b3)]  # 正常点
        Points, Points1 = [], []
        for l in range(0, len(Calculate)):
            for m in range(l + 1, len(Calculate)):
                if self.cross_point(edge, Calculate[l], Calculate[m]) == None:
                    pass
                else:
                    Points.append(self.cross_point(
                        edge, Calculate[l], Calculate[m]))
        for l in range(0, len(Calculate1)):
            for m in range(l + 1, len(Calculate1)):
                if self.cross_point(edge, Calculate1[l], Calculate1[m]) == None:
                    pass
                else:
                    Points1.append(self.cross_point(
                        edge, Calculate1[l], Calculate1[m]))
        return Points, Points1
    
    def segment_filling(self, image, points, Points):
        # 首先，计算四条边界有0-255或者255-0跳变的像素坐标
        w = image.shape[1]
        h = image.shape[0]
        co = []
        # 第一行
        for x in range(0, w - 1):
            ex_value = int(image[0, x])
            fl_value = int(image[0, x + 1])
            if ex_value - fl_value == -255:
                # print(ex_value, fl_value)
                co.append((x + 1, 0))
            if ex_value - fl_value == 255:
                # print(ex_value, fl_value)
                co.append((x, 0))
        # 最后一行
        for x in range(0, w - 1):
            ex_value = int(image[h - 1, x])
            fl_value = int(image[h - 1, x + 1])
            if ex_value - fl_value == -255:
                co.append((x + 1, h - 1))
            if ex_value - fl_value == 255:
                co.append((x, h - 1))
        #  接着，比较point和跳变点的距离，距离近的是需要的点，保留
        c_need = []
        for g in range(0, len(co)):
            for f in range(0, len(points)):
                if np.isclose(0, points[f][1], atol=0.1) or np.isclose(h - 1, points[f][1], atol=0.1):
                    distance = math.sqrt(
                        (co[g][0] - points[f][0]) ** 2 + (co[g][1] - points[f][1]) ** 2)
                    if distance <= 80:
                        c_need.append(points[f])
                    else:
                        pass
                else:
                    continue
        #  筛选出来的点和正常的Points融合
        points = Points + c_need
        for p in range(0, len(points)):
            for j in range(p + 1, len(points)):
                x0 = int(points[p][0])
                y0 = int(points[p][1])
                x1 = int(points[j][0])
                y1 = int(points[j][1])
                cv2.line(image, (x0, y0), (x1, y1), 255, 1, cv2.LINE_AA)
        contours2, hier2 = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 再填充
        cv2.fillPoly(image, contours2, color=255)
        return image


def relative_to_absolute_path(relative_path):
    print(relative_path)
    if relative_path[0] == '~':
        return '/home/' + getpass.getuser() + relative_path[1:]
    else:
        return relative_path

def linear_model(x, m, b):
    return m * x + b

def points_to_posearray(points_list):
    pa_msg = PoseArray()
    pa_msg.header.stamp = rospy.Time.now()
    # pa_msg.header.frame_id = 'corner_points'
    poses_temp = []
    for p in points_list:
        pose_temp = Pose()
        # position
        pose_temp.position.x = p[0]
        pose_temp.position.y = p[1]
        pose_temp.position.z = 0
        # orientation
        pose_temp.orientation.x = 0
        pose_temp.orientation.y = 0
        pose_temp.orientation.z = 0
        pose_temp.orientation.w = 1
        poses_temp.append(pose_temp)
    pa_msg.poses = poses_temp[:]
    return pa_msg

def main():
    rospy.init_node('yolov8_result_node')
    Yolov8Process()
    try:
        rospy.spin()
    except:
        print("Shutting down...")


if __name__ == '__main__':
    main()
