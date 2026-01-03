#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from rospy import Time

class ScanTimestampConverter:
    def __init__(self):
        # 初始化节点
        rospy.init_node('scan_timestamp_converter', anonymous=True)
        
        # 创建发布者，发布转换后的LaserScan到/scan1话题
        self.scan_pub = rospy.Publisher('/scan1', LaserScan, queue_size=10)
        
        # 创建订阅者，订阅/scan话题，回调函数处理数据
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        rospy.loginfo("Scan Timestamp Converter Node Started!")
        rospy.loginfo("Subscribed to: /scan")
        rospy.loginfo("Publishing to: /scan1")

    def scan_callback(self, data):
        """
        回调函数：处理收到的LaserScan数据，修改时间戳为当前时间
        """
        # 创建新的LaserScan消息（复制原数据）
        new_scan = LaserScan()
        new_scan = data  # 复制所有激光雷达数据
        
        # 将时间戳修改为当前时间（两种方式可选，根据需求选择）
        # 方式1：使用rospy.get_rostime() - 获取ROS系统当前时间（推荐）
        new_scan.header.stamp = rospy.get_rostime()
        
        # 方式2：使用rospy.Time.now() - 获取系统墙钟时间（与ROS时间同步时效果一致）
        # new_scan.header.stamp = rospy.Time.now()
        
        # 可选：打印时间戳转换信息（调试用）
        rospy.logdebug("Converted timestamp - Old: %f -> New: %f" % 
                      (data.header.stamp.to_sec(), new_scan.header.stamp.to_sec()))
        
        # 发布转换后的消息
        self.scan_pub.publish(new_scan)

if __name__ == '__main__':
    try:
        # 创建节点实例并运行
        converter = ScanTimestampConverter()
        # 保持节点运行
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Scan Timestamp Converter Node Shutdown!")
        pass
