#! /usr/bin/env python
"""
Author: lei.zeng@tu-dortmund.de
"""

import time
import math
from geometry_msgs.msg import Twist
import numpy as np
import tf
import tf2_ros
import actionlib
import rospy
import shelf_detector.msg
from nav_msgs.msg import Odometry
from one_euro_filter import OneEuroFilter
from shelf_detector.cfg import HomingConfig
from dynamic_reconfigure.server import Server


class HomingAction(object):
    def __init__(self):
        self._namespace = rospy.get_namespace()[1:]
        self._namespace = ''

        self._buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._buf)
        self._br_target = tf.TransformBroadcaster()

        self._goal_frame = ''
        self.__goal_reached = False
        self.__goal_switch = False
        self.__goal_defination = False
        self._max_v_x = rospy.get_param("~max_vel_x", 0.06)
        self._max_omega = rospy.get_param("~max_omega", 0.50)
        self._max_rot = rospy.get_param("~max_rot", 0.25)

        self._k_rho = rospy.get_param("~k_rho", 1)
        self._k_phi = rospy.get_param("~k_phi", -1)
        self._k_alpha = rospy.get_param("~k_alpha", -3)
        self._control_rate = rospy.get_param("~control_rate", 50)
        self._use_filter = rospy.get_param("~use_filter", True)

        self._feedback = shelf_detector.msg.HomingControlActionFeedback()
        self._result = shelf_detector.msg.HomingControlActionResult()
        self.agv_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.omega_filter = OneEuroFilter(t0=time.time(),
                                          x0=0.0,
                                          dx0=0.0,
                                          min_cutoff=1.0,
                                          beta=0.3,
                                          d_cutoff=1.0)

        self.srv = Server(HomingConfig, self.srvCallback)
        self._action_name = 'homing_control_server'
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            shelf_detector.msg.HomingControlAction,
            execute_cb=self.execute_cb,
            auto_start=False)
        self._as.start()

    def srvCallback(self, config, level):
        self._max_v_x = config['max_vel_x']
        self._max_omega = config['max_omega']
        self._max_rot = config['max_rot']
        self._k_rho = config['k_rho']
        self._k_phi = config['k_phi']
        self._k_alpha = config['k_alpha']
        self._use_filter = config['use_filter']
        return config

    def detectOdomCallback(self, odomMsg):
        if self._goal_frame == 'base_link' and not self.__goal_reached:
            if self.__goal_switch:
                try:
                    tform_o_b = self._buf.lookup_transform(
                        self._namespace + "odom",
                        self._namespace + "base_link",
                        rospy.Time(0),
                        timeout=rospy.Duration(1))

                    trans_o_b = (tform_o_b.transform.translation.x,
                                 tform_o_b.transform.translation.y, 0)
                    rot_o_b = (tform_o_b.transform.rotation.x,
                               tform_o_b.transform.rotation.y,
                               tform_o_b.transform.rotation.z,
                               tform_o_b.transform.rotation.w)
                    self._To_b = tf.TransformerROS().fromTranslationRotation(
                        trans_o_b, rot_o_b)
                    self.__goal_switch = False
                except:
                    rospy.logwarn("[HC]: TF Exception")
                    return

                self._To_t = np.dot(self._To_b, self._Tb_t)
                self._rot_target = tf.transformations.quaternion_from_matrix(
                    self._To_t)
                self._trans_target = tf.transformations.translation_from_matrix(
                    self._To_t)
            try:
                self._br_target.sendTransform(self._trans_target,
                                              self._rot_target,
                                              rospy.Time.now(),
                                              self._namespace + "base_target",
                                              self._namespace + "odom")
            except:
                pass

        elif self._goal_frame != 'base_link' and not self.__goal_reached and self.__goal_defination:
            try:
                self._br_target.sendTransform(
                    self._trans_b_t, self._rot_b_t, rospy.Time.now(),
                    self._namespace + "base_target",
                    self._namespace + self._goal_frame)
            except:
                pass

    def homingInitialization(self, goalMsg):
        self._goal_frame = goalMsg.homing_goal
        self._trans_b_t = (goalMsg.x_goal, goalMsg.y_goal, 0)
        self._rot_b_t = tf.transformations.quaternion_from_euler(
            0.0, 0.0, goalMsg.yaw_goal)
        self._rot_b_t = tuple(self._rot_b_t)
        self._Tb_t = tf.TransformerROS().fromTranslationRotation(
            self._trans_b_t, self._rot_b_t)

        self.__goal_switch = (goalMsg.homing_goal == 'base_link')
        self.__goal_defination = (goalMsg.homing_goal != 'base_link' and
                                  (goalMsg.x_goal != 0 or goalMsg.y_goal != 0
                                   or goalMsg.yaw_goal != 0))

    def execute_cb(self, goal):
        self.sub_odom = rospy.Subscriber("odom",
                                         Odometry,
                                         self.detectOdomCallback,
                                         queue_size=1)
        tf2_ros.Buffer.clear(self._buf)
        rospy.loginfo("[HC] new goal")
        self.__goal_reached = False
        self.homingInitialization(goal)

        rate = rospy.Rate(self._control_rate)
        self._feedback.feedback.xy_distance = 100
        self._feedback.feedback.yaw_distance = 100
        robot_frame = 'base_link'
        if goal.homing_goal != robot_frame \
                and (abs(goal.x_goal) + abs(goal.y_goal) + abs(goal.yaw_goal) == 0):
            target_frame = goal.homing_goal
        else:
            target_frame = 'base_target'

        rough_xy_error = rospy.get_param("~rough_xy_error", 0.005)  # 0.005
        x_error = rospy.get_param("~x_error", 0.002)  # 0.002
        yaw_error = rospy.get_param("~yaw_error", 0.002)  # 0.002
        cmd = Twist()

        try:
            tform = self._buf.lookup_transform(
                self._namespace + robot_frame,
                self._namespace + target_frame,
                rospy.Time(0),
                timeout=rospy.Duration(0.05)).transform
            trans, rot = self.get_trans_rot(tform)
            distance_target = math.sqrt(trans[0]**2 + trans[1]**2)
            _, _, phi = self.poseError(trans, rot)
            distance_target_x = abs(trans[0])
        except:
            distance_target, distance_target_x, phi = 1, 1, 1

        ts = time.time()
        self.omega_filter(time.time(), 0)
        while distance_target > rough_xy_error:
            if self.homing_preempt():
                return
            try:
                tform = self._buf.lookup_transform(
                    self._namespace + robot_frame,
                    self._namespace + target_frame,
                    rospy.Time(0),
                    timeout=rospy.Duration(0.05)).transform
                trans, rot = self.get_trans_rot(tform)

                distance_target = math.sqrt(trans[0]**2 + trans[1]**2)
                distance_target_x = abs(trans[0])
                (rho, alpha, phi) = self.poseError(trans, rot)
                linear, angular = self.homingControl(rho, alpha, phi)
                cmd.linear.x = linear
                # cmd.angular.z = angular
                if self._use_filter:
                    cmd.angular.z = round(
                        self.omega_filter(time.time(), angular), 5)
                else:
                    cmd.angular.z = angular
                self.agv_vel.publish(cmd)
                self.feedback_publish(distance_target, phi)
            except:
                rospy.logwarn("[HC]: TF Exception")
            rate.sleep()
        print "\033[0;37;42m[STEP 1]\033[0m", ' xy_error:', distance_target, ', yaw error:', phi

        while abs(phi) > yaw_error:
            if self.homing_preempt():
                return
            try:
                tform = self._buf.lookup_transform(
                    self._namespace + robot_frame,
                    self._namespace + target_frame,
                    rospy.Time(0),
                    timeout=rospy.Duration(0.05)).transform
                trans, rot = self.get_trans_rot(tform)

                distance_target = math.sqrt(trans[0]**2 + trans[1]**2)
                distance_target_x = abs(trans[0])
                (rho, alpha, phi) = self.poseError(trans, rot)
                cmd.linear.x = 0
                angular = phi
                angular = np.clip(angular, (-1)*self._max_rot, self._max_rot)
                if self._use_filter:
                    cmd.angular.z = round(
                        self.omega_filter(time.time(), angular), 5)
                else:
                    cmd.angular.z = angular
                self.agv_vel.publish(cmd)
                self.feedback_publish(distance_target, phi)
            except:
                rospy.logwarn("[HC]: TF Exception")
            rate.sleep()
        print "\033[0;37;42m[STEP 2]\033[0m", ' xy_error:', distance_target, ', yaw error:', phi

        while distance_target_x > x_error:
            if self.homing_preempt():
                return
            try:
                tform = self._buf.lookup_transform(
                    self._namespace + robot_frame,
                    self._namespace + target_frame,
                    rospy.Time(0),
                    timeout=rospy.Duration(0.05)).transform
                trans, rot = self.get_trans_rot(tform)

                distance_target_x = abs(trans[0])
                distance_target = math.sqrt(trans[0]**2 + trans[1]**2)
                (rho, alpha, phi) = self.poseError(trans, rot)
                linear, angular = self.homingControl(rho, alpha, phi)
                cmd.linear.x = linear
                cmd.angular.z = 0
                self.agv_vel.publish(cmd)
                self.feedback_publish(distance_target, phi)
            except:
                rospy.logwarn("[HC]: TF Exception")
            rate.sleep()
        print "\033[0;37;42m[STEP 3]\033[0m", ' xy_error:', distance_target, ', yaw error:', phi
        cmd.linear.x = 0
        cmd.angular.z = 0
        self.agv_vel.publish(cmd)

        if True:
            self._result.result.xy_error = distance_target
            self._result.result.yaw_error = phi
            self._result.result.x_error = distance_target_x
            self._result.result.y_error = math.sqrt(abs(distance_target**2 -
                                                        distance_target_x**2))
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result.result)
        else:
            rospy.logerr('%s: Failed to reach the goal' % self._action_name)
            self._as.set_aborted()

        self.__goal_reached = True
        self.sub_odom.unregister()

    def get_trans_rot(self, tform):
        trans = [
            tform.translation.x, tform.translation.y,
            tform.translation.z
        ]
        rot = [
            tform.rotation.x, tform.rotation.y, tform.rotation.z,
            tform.rotation.w
        ]
        return trans, rot

    def feedback_publish(self, xy, phi):
        self._feedback.feedback.xy_distance = xy
        self._feedback.feedback.yaw_distance = phi
        self._as.publish_feedback(self._feedback.feedback)

    def homing_preempt(self):
        if self._as.is_preempt_requested():
            self.agv_vel.publish(Twist())
            rospy.logwarn('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            self.sub_odom.unregister()
            return True
        else:
            return False

    def cart2pol(self, x, y):
        dist = np.sqrt(x**2 + y**2)
        deg = math.atan2(y, x)
        return (deg, dist)

    def angdiff(self, alpha, beta):
        delta = (alpha - beta) % (2 * math.pi)
        if delta > math.pi:
            delta = delta - 2 * math.pi
        return delta

    def poseError(self, trans, rot):
        (alpha, rho) = self.cart2pol(trans[0], trans[1])
        alpha = self.angdiff(0, alpha)
        _, _, theta = tf.transformations.euler_from_quaternion(rot)
        phi = theta
        return (rho, alpha, phi)

    def homingControl(self, rho, alpha, phi):
        v = self._k_rho * rho
        if alpha < -math.pi / 2 or alpha > math.pi / 2:
            v = -v
            alpha = self.angdiff(alpha, math.pi)
        v = np.clip(v, -self._max_v_x, self._max_v_x)
        omega = self._k_alpha * alpha + self._k_phi * phi
        omega = np.clip(omega, -self._max_omega, self._max_omega)
        omega = np.clip(omega, -max(0.1, rho*1.0), max(0.1, rho*1.0))
        return (v, omega)


def main():
    rospy.init_node('homing_control_server')
    HomingAction()
    try:
        rospy.spin()
    except:
        pass


if __name__ == '__main__':
    main()
