#!/usr/bin/env python3

"""
Author: lei.zeng@tu-dortmund.de
"""

import rospy
import math
import numpy as np
import random
from geometry_msgs.msg import Twist, Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import time
import getpass
import yaml
import tf
import tf2_ros
from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import zlib
from median_player.msg import MedianPlayActionGoal


rospy.init_node('double_localization_manager')
ROS_MELODIC = False
if rospy.get_param("/rosdistro")[:7] == 'melodic':
    ROS_MELODIC = True
    from apriltag_ros.msg import AprilTagDetectionArray
    from flexbe_core import BehaviorLibrary
    from flexbe_msgs.msg import BEStatus


def relative_to_absolute_path(relative_path):
    if relative_path[0] == '~':
        absolute_path = '/home/' + getpass.getuser() + relative_path[1:]
    else:
        absolute_path = relative_path
    return absolute_path


class Localization():
    def __init__(self):
        self._namespace = rospy.get_namespace()[1:]
        self._namespace = ''

        self._buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._buf)
        self.jump = rospy.get_param("~jump", 1.0)
        self.either_or = rospy.get_param("~either_or", True)

        self._tag_num = rospy.get_param("~tag_num", 3)
        self._tag_error = rospy.get_param("~tag_error", 0.5)

        self.amcl_supply_error = rospy.get_param(
            "~amcl_supply_error", 0.25)
        self.amcl_relocation_error = rospy.get_param(
            "~amcl_relocation_error", 0.25)
        self.correct_distance = rospy.get_param(
            "~correct_distance", 5)

        self.reflection_supply_error = rospy.get_param(
            "~reflection_supply_error", 0.2)
        self.reflection_relocation_error = rospy.get_param(
            "~reflection_relocation_error", 0.4)
        self.correct_dist = rospy.get_param(
            "~reflection_correct_distance", 10)

        self.amcl_error, self.amcl_shift = 0, 0
        self._last_x, self._last_y = 0, 0
        self._amcl_stamp = time.time()

        self.reflection_error, self.reflection_shift = 0, 0
        self._last_xr, self._last_yr = 0, 0
        self._amcl_ref_stamp = time.time()

        self._global_x, self._global_y = 0, 0
        self._global_twist = 0

        self.initial_state = False

        self.pub_diag = rospy.Publisher(
            '/diagnostics', DiagnosticArray, queue_size=10)
        self.pub_amcl = rospy.Publisher('amcl_pose_filtered',
                                        PoseWithCovarianceStamped, queue_size=1)
        self.pub_initial = rospy.Publisher(
            'initialpose', PoseWithCovarianceStamped, latch=True, queue_size=10)
        self.pub_initial_origin = rospy.Publisher(
            'initialpose_0', PoseWithCovarianceStamped,  latch=True, queue_size=10)
        self.pub_initial_reflection = rospy.Publisher(
            'initialpose_reflection', PoseWithCovarianceStamped, latch=True, queue_size=10)
        self.pub_set_pose = rospy.Publisher(
            'set_pose_global', PoseWithCovarianceStamped, latch=True, queue_size=10)
        self.pub_amcl_reflection = rospy.Publisher('amcl_pose_filtered_reflection',
                                                   PoseWithCovarianceStamped, latch=True, queue_size=1)

        self.alertGoalPub = rospy.Publisher('alert/median_player_server/goal',
                                            MedianPlayActionGoal,
                                            queue_size=1)
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped,
                         self.amcl_pose_cb, queue_size=1)
        rospy.Subscriber("initialpose", PoseWithCovarianceStamped,
                         self.initialpose_cb, queue_size=10)
        rospy.Subscriber("global_pose", Odometry,
                         self.global_pose_cb, queue_size=1)
        rospy.Subscriber("amcl_pose_reflection", PoseWithCovarianceStamped,
                         self.amcl_reflection_cb, queue_size=1)

        if ROS_MELODIC:
            special_area_path = rospy.get_param(
                rospy.get_namespace()+"rosbridge_system/special_area_path", '')
            self.tags_dict = {}
            self._registering = False
            self._tag_pose_list = []
            try:
                if special_area_path != '':
                    special_area_path = relative_to_absolute_path(
                        special_area_path)
                    rospy.loginfo('[DL] Special area Path: %s' %
                                  special_area_path)
                    with open(special_area_path, 'r') as stream:
                        try:
                            content = (yaml.safe_load(stream)).items()
                            tags = [
                                a for a in content if a[0][:3] == 'tag']
                            for t in tags:
                                self.tags_dict[t[1]['tag_id']
                                               ] = self.getTagMat(t)
                            self.pubTagPose = rospy.Publisher(
                                'tag_pose', PoseWithCovarianceStamped, queue_size=10)
                            rospy.Subscriber(
                                'camera/tag_detections', AprilTagDetectionArray, self.tagsCallback, queue_size=5)
                        except:
                            rospy.logwarn('[DL] Failed to load special areas')
            except:
                rospy.logwarn('[DL] Failed to load special areas')

            if rospy.get_param(rospy.get_namespace()+"rosbridge_system/use_flexbe", True):
                BeLib = BehaviorLibrary()
                self._check_num_dict = {}
                for key in BeLib._behavior_lib:
                    be_filepath_new = BeLib.get_sourcecode_filepath(key)
                    with open(be_filepath_new, "r") as f:
                        be_content_new = f.read()
                        behavior_checksum = self.to_int16(
                            zlib.adler32(be_content_new))
                        self._check_num_dict[
                            behavior_checksum] = BeLib._behavior_lib[key]['name']
                rospy.Subscriber('flexbe/status',
                                 BEStatus,
                                 self.flexbeStatusCallback,
                                 queue_size=10)

    def flexbeStatusCallback(self, fStatusMsg):
        try:
            if self._check_num_dict[self.to_int16(fStatusMsg.behavior_id)] == "tag_register" and fStatusMsg.code == 0:
                self._registering = True
            else:
                self._registering = False
        except Exception as e:
            rospy.logwarn("flexbe status exception: %s" % str(e))

    def dist_xy(self, x, y):
        return math.sqrt(x**2+y**2)

    def tagsCallback(self, tag_msg):
        for t in tag_msg.detections:
            try:
                tag_id = t.id[0]
                if abs(t.pose.pose.pose.position.z) < t.size[0]*50 and self._registering is False:
                    tform_t_b = self._buf.lookup_transform(self._namespace + 'tag_' + str(tag_id),
                                                           self._namespace + 'base_link',
                                                           rospy.Time(0),
                                                           rospy.Duration(0.1))
                    trans = tform_t_b.transform.translation
                    rot = tform_t_b.transform.rotation
                    Tt_b = tf.TransformerROS().fromTranslationRotation((trans.x, trans.y, trans.z),
                                                                       (rot.x, rot.y, rot.z, rot.w))
                    Tm_b = np.dot(self.tags_dict['tag_' + str(tag_id)], Tt_b)
                    _, _, yaw = tf.transformations.euler_from_matrix(Tm_b)
                    pos = tf.transformations.translation_from_matrix(Tm_b)
                    self._tag_pose_list.append((pos[0], pos[1], yaw))

                    if len(self._tag_pose_list) >= self._tag_num:
                        x_list = map(lambda f: f[0], self._tag_pose_list)
                        y_list = map(lambda f: f[1], self._tag_pose_list)
                        yaw_list = map(lambda f: f[2], self._tag_pose_list)

                        t_dx = max(x_list)-min(x_list)
                        t_dy = max(y_list) - min(y_list)
                        t_dyaw = max(yaw_list)-min(yaw_list)
                        if_ok = (t_dx < self._tag_error) or (
                            t_dy < self._tag_error) or (t_dyaw < self._tag_error)
                        if not if_ok:
                            self._tag_pose_list = []
                            return
                        # print round(t_dx, 3),  round(t_dy, 3),  round(t_dyaw, 3)
                        x = np.median(x_list)
                        y = np.median(y_list)
                        yaw = np.median(yaw_list)
                        tag_pose_msg = PoseWithCovarianceStamped()
                        tag_pose_msg.header.stamp = rospy.Time.now()
                        tag_pose_msg.header.frame_id = 'map'
                        pose_msg = Pose()
                        pose_msg.position.x, pose_msg.position.y = x, y
                        (pose_msg.orientation.x, pose_msg.orientation.y,
                         pose_msg.orientation.z, pose_msg.orientation.w) = tuple(tf.transformations.quaternion_from_euler(
                             0, 0, yaw))
                        tag_pose_msg.pose.pose = pose_msg
                        tag_pose_msg.pose.covariance[0] = t_dx
                        tag_pose_msg.pose.covariance[7] = t_dy
                        tag_pose_msg.pose.covariance[-1] = t_dyaw
                        self.pubTagPose.publish(tag_pose_msg)

                        if self.dist_xy(self._global_x-x, self._global_y-y) > 2 \
                                and abs(t.pose.pose.pose.position.z) < 2 and abs(t.pose.pose.pose.position.x) < 0.3 \
                                and self._global_twist < 0.05:
                            self.pub_initial.publish(tag_pose_msg)
                            rospy.loginfo(
                                '[DL] pose recovery by tag %s' % str(tag_id))
                            alert_goal_msg = MedianPlayActionGoal()
                            alert_goal_msg.goal.mode = "single"
                            alert_goal_msg.goal.sound_id = 28
                            self.alertGoalPub.publish(alert_goal_msg)
                        self._tag_pose_list.pop(0)
                else:
                    self._tag_pose_list = []
            except Exception as e:
                pass
            if not tag_msg.detections and len(self._tag_pose_list) > 0:
                self._tag_pose_list = []

    def to_int16(self, n):
        return np.array([n], 'int16')[0]

    def getTagMat(self, tag):
        trans = tuple(tag[1]['position'])
        rot = tuple(tag[1]['orientation'])
        return tf.TransformerROS().fromTranslationRotation(trans, rot)

    def initialpose_cb(self, initialMsg):
        set_pose_msg = PoseWithCovarianceStamped()
        set_pose_msg.header.frame_id = initialMsg.header.frame_id
        set_pose_msg.header.stamp = rospy.Time.now()
        set_pose_msg.pose.pose = initialMsg.pose.pose
        set_pose_msg.pose.covariance = initialMsg.pose.covariance
        self.pub_set_pose.publish(set_pose_msg)
        rospy.loginfo('[DL] resetting global ekf')

        self.pub_initial_origin.publish(initialMsg)
        rospy.loginfo('[DL] initialized amcl')

        self.pub_initial_reflection.publish(initialMsg)
        rospy.loginfo('[DL] initialized amcl reflection')

    def amcl_pose_cb(self, amclMsg):
        if self.initial_state:
            time.sleep(0.2)
            initialPose = PoseWithCovarianceStamped()
            initialPose.header.frame_id = amclMsg.header.frame_id
            initialPose.header.stamp = rospy.Time.now()
            initialPose.pose.pose = amclMsg.pose.pose
            initialPose.pose.covariance = amclMsg.pose.covariance
            self.pub_set_pose.publish(initialPose)
            # rospy.loginfo('[DL] initializing global ekf ')
            print(initialPose)
            self.initial_state = False

        uncertainty = amclMsg.pose.covariance[0] + \
            amclMsg.pose.covariance[7] + amclMsg.pose.covariance[-1]
        self.amcl_error = np.sqrt(abs(uncertainty))
        self.amcl_shift = self.dist_xy(
            amclMsg.pose.pose.position.x - self._global_x, amclMsg.pose.pose.position.y-self._global_y)
        self._amcl_stamp = time.time()
        if self.either_or and self._amcl_stamp - self._amcl_ref_stamp > 10:
            self.reflection_error = 100

        state = DiagnosticStatus()
        # rospy.loginfo('amcl_error: %f,%f',
        #               self.amcl_supply_error, self.amcl_error)
        # rospy.loginfo('amcl_shift: %f,%f',
        #               self.reflection_error, self.amcl_shift)
        if self.amcl_error < self.amcl_supply_error and self.amcl_shift < self.jump:
            if (not self.either_or) or (self.either_or and self.amcl_error <= self.reflection_error):
                self.pub_amcl.publish(amclMsg)
            state.level = 0
            state.message = 'OK'
        else:
            state.level = 1
            state.message = 'Warning'

        state.name = 'amcl_origin_class'
        diagnostic_msg = DiagnosticArray()
        diagnostic_msg.header.stamp = rospy.Time.now()
        diagnostic_msg.status.append(state)
        # self.pub_diag.publish(diagnostic_msg)

    def amcl_reflection_cb(self, amcl_reflection_msg):
        uncertaintyReflection = amcl_reflection_msg.pose.covariance[0] + \
            amcl_reflection_msg.pose.covariance[7] + \
            amcl_reflection_msg.pose.covariance[-1]
        self.reflection_error = np.sqrt(uncertaintyReflection)
        self.reflection_shift = self.dist_xy(
            amcl_reflection_msg.pose.pose.position.x - self._global_x, amcl_reflection_msg.pose.pose.position.y-self._global_y)
        self._amcl_ref_stamp = time.time()
        if self.either_or and self._amcl_ref_stamp - self._amcl_stamp > 10:
            self.amcl_error = 100

        state = DiagnosticStatus()
        if self.reflection_error < self.reflection_supply_error and self.reflection_shift < self.jump:
            if (not self.either_or) or (self.either_or and self.amcl_error >= self.reflection_error):
                self.pub_amcl_reflection.publish(amcl_reflection_msg)
            state.level = 0
            state.message = 'OK'
        else:
            state.level = 1
            state.message = 'Warning'

        state.name = 'amcl_reflection_class'
        diagnostic_msg = DiagnosticArray()
        diagnostic_msg.header.stamp = rospy.Time.now()
        diagnostic_msg.status.append(state)
        # self.pub_diag.publish(diagnostic_msg)

    def global_pose_cb(self, global_pose_msg):
        global_pose = PoseWithCovarianceStamped()
        global_pose.header = global_pose_msg.header
        global_pose.header.stamp = rospy.Time.now()
        global_pose.pose.pose = global_pose_msg.pose.pose
        self._global_x, self._global_y = global_pose_msg.pose.pose.position.x, global_pose_msg.pose.pose.position.y
        self._global_twist = abs(
            global_pose_msg.twist.twist.linear.x) + abs(global_pose_msg.twist.twist.angular.z)
        global_pose.pose.covariance = (0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0, 0.05, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.01)

        deltaPosition = self.dist_xy(
            self._global_x - self._last_x,  self._global_y - self._last_y)
        if (self.amcl_error > self.amcl_relocation_error and deltaPosition > self.correct_distance) \
                or self.amcl_shift > self.jump:
            self.pub_initial_origin.publish(global_pose)
            # rospy.loginfo('[DL] initialized amcl by global ekf')
            self._last_x, self._last_y = self._global_x, self._global_y

        deltaPositionReflection = self.dist_xy(
            self._global_x - self._last_xr, self._global_y - self._last_yr)
        if (self.reflection_error > self.reflection_relocation_error and deltaPositionReflection > self.correct_dist) \
                or self.reflection_shift > self.jump:
            self.pub_initial_reflection.publish(global_pose)
            # rospy.loginfo('[DL] initialized amcl reflecttion by global ekf')
            self._last_xr, self._last_yr = self._global_x, self._global_y


def main():
    Localization()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
