#!/usr/bin/env python
"""
Author: lei.zeng@tu-dortmund.de
"""

import copy
import getpass
import json
import os
import subprocess
import time
import xml.etree.ElementTree as ET
import zlib
from shutil import copyfile, copytree, rmtree

import dynamic_reconfigure.client
import git
import numpy as np
import rosgraph
import roslaunch
import rospy
import tf
import tf2_ros
import yaml
from actionlib_msgs.msg import GoalID
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from flexbe_core import BehaviorLibrary
from geometry_msgs.msg import (Point, PointStamped, PoseArray, PoseStamped,
                               PoseWithCovarianceStamped, Twist)
from median_player.msg import MedianPlayActionFeedback, MedianPlayActionGoal
# import threading
from move_base_msgs.msg import MoveBaseActionResult
from nav_msgs.msg import Odometry, Path
from scheduling_msgs.msg import MapStream
from sensor_msgs.msg import (BatteryState, Joy, JoyFeedback, JoyFeedbackArray,
                             LaserScan)
from shapely.geometry import Point as SPoint
from shapely.geometry import LinearRing, LineString, MultiPoint, Polygon
from std_msgs.msg import Bool, ColorRGBA, Empty, Float32, Header, String
from visualization_msgs.msg import Marker, MarkerArray
from yocs_msgs.msg import (NavigationControl, NavigationControlStatus,
                           WaypointList)
import re

rospy.init_node('rosbridge_system')
ROS_MELODIC = False
if rospy.get_param("/rosdistro")[:7] == 'melodic':
    ROS_MELODIC = True
    from apriltag_ros.msg import AprilTagDetectionArray
    from flexbe_msgs.msg import BehaviorExecutionActionGoal, BehaviorRequest, BEStatus

RS_START = time.time()
USE_FLEXBE = rospy.get_param("~use_flexbe", False)


class systemTask(object):
    def __init__(self):
        self._namespace = rospy.get_namespace()
        self.startSLAM, self.excutedSLAM, self.slamReset, self.slamContinue = False, False, False, False
        self.saveMap = False
        self._switch_str = ''
        self.slamGoOn, self.slamPause = False, False
        self._trashing = False
        self._robot_active = False
        self._charging, self._detecting = False, False

        self.startDetector, self.executedDetector = False, False
        self.detectorTarget = ""

        self.battery_percentage = 0.7
        self._behavior_state = 'free'  # 'free'

        self.amcl_x, self.amcl_y, self.amcl_yaw = 0, 0, 0
        self.odom_x, self.odom_y, self.odom_yaw = 0, 0, 0

        self.updateRequestFlag = False
        self.dbparam_git_task = False
        self.bgm_pause = False

        self.cmd_vel_linear, self.cmd_vel_angular = [0], [0]
        self.turn_judge, self.back_judge = 'none', False
        self.cmd_vel_music = rospy.get_param("~cmd_vel_music", False)

        self.odom_vel_linear, self.odom_vel_angular = [0], [0]

        self.turn_threshold_omega = rospy.get_param("~turn_threshold_omega",
                                                    0.2)
        self.turn_threshold_num = rospy.get_param("~turn_threshold_num", 10)
        self.odom_vel_music = rospy.get_param("~odom_vel_music", True)
        self._bgm_id = rospy.get_param("~bgm_id", 10)
        self.bgm_vel = rospy.get_param("~bgm_vel", 0.05)
        self.back_alert_vel = rospy.get_param("~back_alert_vel", -0.05)

        self.obstacle_distances = []
        self.obstacle_alert_distance = rospy.get_param(
            "~obstacle_alert_distance", 1.5)
        self.close_obstacle = "false"

        if_teb_turn_judge = rospy.get_param("~if_teb_turn_judge", True)
        self.teb_turn_yaw = rospy.get_param("~teb_turn_yaw", 0.5)
        self.teb_turn_judge = 'none'

        self.if_bgm_canceled, self.is_alerting = False, False

        self._departure_alert = rospy.get_param("~departure_alert", True)
        self._battery_alert = rospy.get_param("~battery_alert", True)
        self._low_battery = rospy.get_param("~low_battery", -0.1)

        self._trash_wait_time = rospy.get_param("~trash_wait_time", 100)
        self.initialization_completition = False
        self.phb_applied_timeout = rospy.get_param("~phb_applied_timeout", 60)
        self.phb_wait_timeout = rospy.get_param("~phb_wait_timeout", 60)

        self.area_apply_dist = rospy.get_param("~area_apply_dist", 10)
        self.area_stop_dist = rospy.get_param("~area_stop_dist", 5)

        if USE_FLEXBE:
            self._flexbe_site = {}
            self._flexbe_site_path = relative_to_absolute_path(
                rospy.get_param(
                    "~waypoints_path",
                    "~/catkin_ws/dbparam/flexbe_waypoints.yaml"))
            if os.path.isfile(self._flexbe_site_path):
                rospy.logdebug('[RS] %s already exists' %
                               self._flexbe_site_path)
                with open(self._flexbe_site_path) as site_f:
                    self._flexbe_site = yaml.safe_load(site_f)
            else:
                with open(self._flexbe_site_path, 'w') as fp:
                    pass
                with open(self._flexbe_site_path, "w") as f:
                    c = {'initial_pose': {'frame_id': 'map',
                                          'pose': {
                                              'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
                                              'position': {'x': 0, 'y': 0, 'z': 0}
                                          }
                                          },
                         'charge_point': {'frame_id': 'map',
                                          'pose': {
                                              'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
                                              'position': {'x': 0, 'y': 0, 'z': 0}
                                          }
                                          }
                         }
                    yaml.dump(c, f)
                    self._flexbe_site = c
                    rospy.loginfo('[RS] %s created' % self._flexbe_site_path)

        special_area_path = rospy.get_param(
            "~special_area_path", '~/catkin_ws/dbparam/special_areas.yaml')
        self._special_areas, self._in_special_area = None, False
        self._alley_areas, self._alley_info = [], []
        self._in_alley = False
        self._special_info = []
        self._tag_areas = {}
        self._door_areas = None
        self._ob_ts = time.time()
        self._diversion = {'mode': 'off',
                           'stamp': 0,
                           'start_pose': [0, 0, 0],
                           'fit_dist': rospy.get_param("~fit_dist", 0.06),
                           'on_timeout': rospy.get_param("~diversion_on_timeout", -1),
                           'off_timeout': rospy.get_param("~diversion_off_timeout", 100),
                           'max_offtrack':  rospy.get_param("~max_offtrack", 1.5),
                           'max_offyaw':  rospy.get_param("~max_offyaw", 1.57),
                           'dy_params': {},
                           'pre_params': {}}
        self._phb_areas = {}
        self._phb_pass = {'alert': 'todo', 'todo': 'todo',
                          'state': 'initial', 'timeout': False, 'exit': 'none'}
        self._phb_pass_list = []
        self._bgm_stamp = 0
        try:
            if special_area_path != '':
                self.special_area_path = relative_to_absolute_path(
                    special_area_path)
                rospy.loginfo('[RS] Special area Path: %s' %
                              self.special_area_path)
                self.alleyPub = rospy.Publisher('alley_ee',
                                                PointStamped,
                                                queue_size=1,
                                                latch=True)
                if os.path.isfile(self.special_area_path):
                    rospy.logdebug('[RS] %s already exists' %
                                   self.special_area_path)
                else:
                    with open(self.special_area_path, 'w') as fp:
                        pass
                    with open(self.special_area_path, "w") as f:
                        c = {}
                        yaml.dump(c, f)
                        rospy.loginfo('[RS] create new file: %s' %
                                      self.special_area_path)
                with open(self.special_area_path, 'r') as stream:
                    try:
                        f = yaml.safe_load(stream)
                        if 'diversion' in f:
                            self._diversion['dy_params'] = f['diversion']
                            self._diversion['pre_params'] = self.getParamsDictionary(
                                self._diversion['dy_params'].keys())
                        areas = f.items()
                        self._special_areas = [
                            a for a in areas if a[0][:4] == 'area'
                        ]
                        self._door_areas = [
                            a for a in areas if a[0][:4] == 'door'
                        ]
                        self._alley_areas = [
                            a for a in areas if a[0][:5] == 'alley'
                        ]
                        if rospy.get_param("~enable_phb", True):
                            prohibition_areas = [
                                a for a in areas if ((a[0][:11] == 'prohibition' or a[0][:3] == 'phb') and a[0] != 'phb_pass')]
                        else:
                            prohibition_areas = []
                        for p in prohibition_areas:
                            if 'coords' not in p[1]:
                                if len(p[1]) < 3:
                                    rospy.logwarn('%s ignored' % p[0])
                                    continue
                                ref_list = []
                                p_close = p[1]+[p[1][0]]
                                for n in range(len(p[1])):
                                    a, b = p_close[n], p_close[n+1]
                                    dx, dy = b[0]-a[0], b[1]-a[1]
                                    if abs(dx) >= abs(dy):
                                        x_ref, y_ref = dx/abs(dx), 0
                                    else:
                                        x_ref, y_ref = 0, dy/abs(dy)
                                    ref_list.append(
                                        (np.sqrt(dx**2+dy**2), x_ref, y_ref, n))
                                ref_list_od = sorted(ref_list)
                                gate_a = (p_close[ref_list_od[0][3]], p_close[
                                    ref_list_od[0][3]+1])
                                gate_b = (p_close[ref_list_od[1][3]], p_close[
                                    ref_list_od[1][3]+1])
                                # print 'gateab:', gate_a, gate_b
                                edge = ref_list[min(ref_list_od[0][3], ref_list_od[1][3])+1: max(
                                    ref_list_od[0][3], ref_list_od[1][3])]
                                x_ref, y_ref = 0, 0
                                for r in edge:
                                    x_ref += r[1]
                                    y_ref += r[2]
                                self._phb_areas[p[0]] = {
                                    "coords": p[1], "polygon":  Polygon(p[1]), "x_ref": x_ref, "y_ref": y_ref, 'if_amcl_in': False,
                                    "gate_a": LineString(gate_a), "gate_b": LineString(gate_b), 'robot_dist': 100}
                            else:
                                if len(p[1]['coords']) < 3:
                                    rospy.logwarn('%s ignored' % p[0])
                                    continue
                                self._phb_areas[p[0]] = {
                                    "coords": p[1]['coords'], "polygon":  Polygon(p[1]['coords']), "x_ref": 0, "y_ref": 0, 'if_amcl_in': False,
                                    "gate_a": LineString(p[1]['gate_a']), "gate_b": LineString(p[1]['gate_b']), 'robot_dist': 100}
                        for phb in self._phb_areas:
                            coord_ring = self._phb_areas[phb]["coords"] + [
                                self._phb_areas[phb]["coords"][0]]
                            edges = []
                            for i in range(len(coord_ring)-1):
                                edges.append(LineString(
                                    [coord_ring[i], coord_ring[i+1]]))
                            self._phb_areas[phb]['edges'] = edges
                        if 'phb_pass' in f:
                            self._phb_pass = f['phb_pass']
                            self._phb_pass['exit'] = 'none'
                            if self._phb_pass['todo'] == 'applied':
                                # self._phb_pass_list = [(0, self._phb_pass)]
                                self._phb_pass['todo'] = 'leaving'
                                self._phb_pass['timeout'] = False
                                self._phb_pass['state'] = 'leaving by initialization'
                                rospy.logwarn(
                                    '[RS] %s to be reset at the restart' % self._phb_pass['area'])
                    except Exception as e:
                        rospy.logwarn(
                            '[RS] Failed to load special areas: %s' % str(e))
        except:
            rospy.logwarn('[RS] Failed to load specail areas')

        (self.safe_door_xmin, self.safe_door_xmax, self.safe_door_ymin,
         self.safe_door_ymax) = (rospy.get_param("~safe_door_xmin", 0),
                                 rospy.get_param("~safe_door_xmax", 0),
                                 rospy.get_param("~safe_door_ymin", 0),
                                 rospy.get_param("~safe_door_ymax", 0))
        self._initialization_task = rospy.get_param("~initialization_task", '')
        self._home_point = rospy.get_param("~home_point", 'home_point')

        self.musicControlPub = rospy.Publisher('music/median_control',
                                               Header,
                                               queue_size=1)
        self.musicCancelPub = rospy.Publisher(
            'music/median_player_server/cancel', GoalID, queue_size=1)
        self.musicGoalpub = rospy.Publisher('music/median_player_server/goal',
                                            MedianPlayActionGoal,
                                            queue_size=1)
        self.alertControlPub = rospy.Publisher('alert/median_control',
                                               Header,
                                               queue_size=1)
        self.alertCancelPub = rospy.Publisher(
            'alert/median_player_server/cancel', GoalID, queue_size=1)
        self.alertGoalPub = rospy.Publisher('alert/median_player_server/goal',
                                            MedianPlayActionGoal,
                                            queue_size=1)
        self.cancelHomingPub = rospy.Publisher('homing_control_server/cancel',
                                               GoalID,
                                               queue_size=1)
        self.cancelMoveBasePub = rospy.Publisher('move_base/cancel',
                                                 GoalID,
                                                 queue_size=1)
        self.taskSwitchPub = rospy.Publisher('task_switch',
                                             Header,
                                             queue_size=10)
        self.initialPosePub = rospy.Publisher('initialpose',
                                              PoseWithCovarianceStamped,
                                              queue_size=10)
        self.diagnosticsPub = rospy.Publisher('/diagnostics',
                                              DiagnosticArray,
                                              queue_size=10)
        self.obstaclePosPub = rospy.Publisher('obstacle_position',
                                              Odometry,
                                              queue_size=10)
        # self.flexbePreemptPub = rospy.Publisher('flexbe/command/preempt',
        #                                         Empty,
        #                                         queue_size=10)
        self.serverRequestPub = rospy.Publisher('agv_info',
                                                String,
                                                queue_size=10)
        self.phbInfoPub = rospy.Publisher('phb_info',
                                          String,
                                          queue_size=10,
                                          latch=True)
        self.pubDischarge = rospy.Publisher('joy_start_0',
                                            JoyFeedbackArray,
                                            queue_size=10)

        self.character_path = rospy.get_param(
            "~character_path", '')
        self.characters = {}
        if self.character_path != '':
            self.character_path = relative_to_absolute_path(
                self.character_path)
            if os.path.isfile(self.character_path):
                rospy.logdebug('[RS] %s already exists' % self.character_path)
            else:
                with open(self.character_path, 'w') as fp:
                    pass
                self.characters = {'create_time': now_time(),
                                   'mileage': 0,
                                   'uptime': 0,
                                   'tasks_number': 0,
                                   'update_time': now_time(),
                                   'work_time': 0
                                   }
                with open(self.character_path, "w") as f:
                    yaml.dump(self.characters, f)
                    rospy.loginfo('[RS] create new file: %s' %
                                  self.character_path)
            with open(self.character_path, 'r') as stream:
                self.characters = yaml.safe_load(stream)
            rospy.Timer(rospy.Duration(60), self.time_cb, oneshot=False)
            self.char_upd_dict = {'d_number': 0,
                                  'c_ts': time.time(),
                                  'm_ts': 0,
                                  'd_mileage': 0,
                                  'd_wt': 0,
                                  'w_ts': 0}

        if USE_FLEXBE:
            BeLib = BehaviorLibrary()
            self._check_num_dict = {}
            self._flexbe_log = {}
            self._log_id = 0
            self._flexbe_state = String()
            self._flexbe_state.data = "behavior_exit"
            self._flexbe_status = {'level': 0, 'behavior': '', 'pause': False}
            for key in BeLib._behavior_lib:
                be_filepath_new = BeLib.get_sourcecode_filepath(key)
                with open(be_filepath_new, "r") as f:
                    be_content_new = f.read()
                    behavior_checksum = self.to_int16(
                        zlib.adler32(be_content_new))
                    self._check_num_dict[
                        behavior_checksum] = BeLib._behavior_lib[key]['name']

            try:
                self._behavior_dict = {}
                self._flexbe_state_pub = rospy.Publisher(
                    'flexbe/behavior_updated',
                    String,
                    queue_size=10,
                    latch=True)
                self._flexbe_log_pub = rospy.Publisher('flexbe/behavior_log',
                                                       String,
                                                       queue_size=10,
                                                       latch=True)
                self._flexbe_list_pub = rospy.Publisher('flexbe/behavior_list',
                                                        String,
                                                        queue_size=1,
                                                        latch=True)
                self._flexbe_site_pub = rospy.Publisher(
                    'flexbe/site_collection', String, queue_size=1, latch=True)
                self._manifest_path = rospy.get_param(
                    "~behavior_manifest_path", '')
                rospy.loginfo('[RS] FlexBE manifest path: %s' %
                              self._manifest_path)
                if self._manifest_path.find(',') < 0:
                    self._manifest_path_list = [
                        relative_to_absolute_path(self._manifest_path)]
                else:
                    self._manifest_path_list = []
                    divide_idx = [-1]
                    for m in re.finditer(',', self._manifest_path):
                        divide_idx.append(m.start())
                    divide_idx.append(len(self._manifest_path))
                    for i in range(len(divide_idx)-1):
                        self._manifest_path_list.append(
                            relative_to_absolute_path(self._manifest_path[divide_idx[i]+1:divide_idx[i+1]]))

                self.flexbeListPub(self._manifest_path_list)

                rospy.Subscriber('flexbe/status',
                                 BEStatus,
                                 self.flexbeStatusCallback,
                                 queue_size=10)
                rospy.Subscriber('flexbe/behavior_updating',
                                 String,
                                 self.flexbeStateCallback,
                                 queue_size=10)
                rospy.Subscriber('flexbe/command/preempt',
                                 Empty,
                                 self.flexbePreemptCallback,
                                 queue_size=1)
                rospy.Subscriber('flexbe/command/pause',
                                 Bool,
                                 self.flexbePauseCallback,
                                 queue_size=2)
                rospy.sleep(0.1)
                self._flexbe_state_pub.publish(self._flexbe_state)
                self.flexbeSitePub()
            except Exception as e:
                rospy.logwarn('[RS] exception: %s' % str(e))
        else:
            rospy.Subscriber('waypoints',
                             WaypointList,
                             self.waypointsCallback,
                             queue_size=10)
            self.task_diag_msg = DiagnosticArray()
            self._last_4_state = False
            rospy.Subscriber('nav_ctrl_status',
                             NavigationControlStatus,
                             self.navStateCallback,
                             queue_size=10)
            rospy.Subscriber('nav_ctrl',
                             NavigationControl,
                             self.navCtrlCmdCallback,
                             queue_size=10)

        rospy.Subscriber('task_switch',
                         Header,
                         self.taskSwitchCallback,
                         queue_size=10)
        rospy.Subscriber('server_info',
                         String,
                         self.serverResponseCb,
                         queue_size=10)
        rospy.Subscriber("battery",
                         BatteryState,
                         self.monitorBatteryCallback,
                         queue_size=10)
        rospy.Subscriber('amcl_pose',
                         PoseWithCovarianceStamped,
                         self.amclCallback,
                         queue_size=10)
        rospy.Subscriber('odom', Odometry, self.odomCallback, queue_size=10)
        rospy.Subscriber('cmd_vel', Twist, self.velCallback, queue_size=10)
        if rospy.get_param("~obstacle_alert", True):
            rospy.Subscriber('obstacle_distance_alert',
                             Float32,
                             self.obstacleDistanceCallback,
                             queue_size=10)

        if rospy.get_param("~goal_reached_alert", True):
            rospy.Subscriber('move_base/result',
                             MoveBaseActionResult,
                             self.moveBaseResultCallback,
                             queue_size=10)

        rospy.Subscriber('joy', Joy, self.joyCallback, queue_size=1)
        rospy.Subscriber('alert/median_control',
                         Header,
                         self.alertControlCallback,
                         queue_size=10)
        rospy.Subscriber('music/median_player_server/feedback',
                         MedianPlayActionFeedback,
                         self.bgmFeedbackCallback,
                         queue_size=1)

        self._estop_alert = rospy.get_param("~estop_alert", True)
        self._devices_alert = rospy.get_param("~devices_alert", True)
        self._estop_mode, self._agg_broken = False, False
        rospy.Subscriber('/diagnostics_agg',
                         DiagnosticArray,
                         self.aggCallback,
                         queue_size=5)
        if if_teb_turn_judge:
            rospy.Subscriber('move_base/TebLocalPlannerROS/teb_poses',
                             PoseArray,
                             self.tebPosesCallback,
                             queue_size=5)
            self._global_plan_msg = Path()
            rospy.Subscriber('move_base/GlobalPlanner/plan',
                             Path,
                             self.globalPlanCallback,
                             queue_size=10)

        self.mid = 0
        self.areasPub = rospy.Publisher('special_areas',
                                        MarkerArray,
                                        queue_size=10,
                                        latch=True)
        self.areas_msg = self.areasVisulization()
        self.areasPub.publish(self.areas_msg)

        rospy.Subscriber('map_istream_path',
                         MapStream,
                         self.mapPathCallback,
                         queue_size=1)
        self.publish_phb_info()

    def bgmFeedbackCallback(self, msg):
        self._bgm_stamp = time.time()

    def time_cb(self, event):
        t_pass = time.time()-self.char_upd_dict['c_ts']
        self.char_upd_dict['c_ts'] = time.time()
        with open(self.character_path, 'r') as stream:
            self.characters = yaml.safe_load(stream)
            self.characters['uptime'] = round(
                self.characters['uptime'] + t_pass/3600.0, 5)
            self.characters['update_time'] = now_time()
            self.characters['mileage'] = round(
                self.characters['mileage'] + self.char_upd_dict['d_mileage']/1000.0, 5)
            self.char_upd_dict['d_mileage'] = 0
            self.characters['tasks_number'] += self.char_upd_dict['d_number']
            self.char_upd_dict['d_number'] = 0
            self.characters['work_time'] = round(
                self.characters['work_time'] + self.char_upd_dict['d_wt']/3600.0, 5)
            self.char_upd_dict['d_wt'] = 0
        with open(self.character_path, "w") as f:
            yaml.dump(self.characters, f)

    def updatePassFile(self, rs=False):
        with open(self.special_area_path, "r") as f:
            list_doc = yaml.safe_load(f)
            list_doc_copy = list_doc.copy()
            if rs:
                list_doc['phb_pass'] = {'alert': 'todo', 'todo': 'todo'}
            else:
                list_doc['phb_pass'] = self._phb_pass
        with open(self.special_area_path, "w") as f:
            try:
                yaml.dump(list_doc, f)
            except:
                yaml.dump(list_doc_copy, f)

    def publish_phb_info(self):
        try:
            msg = String()
            msg.data = json.dumps({'current': self._phb_pass,
                                   'queue': dict(self._phb_pass_list)})
            self.phbInfoPub.publish(msg)
        except:
            pass

    def serverResponseCb(self, msg):
        try:
            response = json.loads(msg.data)
            if 'area' in response and response['area'] == self._phb_pass['area']:
                server_todo = str(response['todo'])
                if self._phb_pass['todo'] == "applying" and server_todo == "applied":
                    self._phb_pass['todo'] = "applied"
                    rospy.loginfo('applied for entering %s' % response['area'])
                    self._phb_pass['state'] = "applied by server"
                elif self._phb_pass['todo'] == "leaving" and server_todo == "leaved":
                    self._phb_pass['todo'] = "leaved"
                    rospy.loginfo('applied for leaving %s' % response['area'])
                    self._phb_pass['state'] = "leaved by server"
                self.publish_phb_info()
        except Exception as e:
            rospy.logwarn('[RS] server response exception: %s' % str(e))

    def to_int16(self, n):
        return np.array([n], 'int16')[0]

    def globalPlanCallback(self, msg):
        self._global_plan_msg = msg
        if not msg.poses:
            return
        time.sleep(0.2)
        global_points = self.path_to_points(msg, '')
        global_line = LineString(global_points)
        prohibition_pass_list = []
        for key in self._phb_areas:
            if global_line.disjoint(self._phb_areas[key]['polygon']) == False:
                ps = self.path_to_points(
                    msg, self._phb_areas[key]['polygon'])
                p1, p2 = ps[0], ps[-1]
                if self._phb_pass['todo'] == "applied" and self._phb_pass['area'] == key and \
                        SPoint(self.amcl_x, self.amcl_y).within(self._phb_areas[key]['polygon']):
                    self._phb_pass['exit'] = self.exit_edge(SPoint(p2), key)
                    if self._phb_pass['exit'] == 'none':
                        rospy.loginfo('update %s exit: none')
                    else:
                        rospy.loginfo('update %s exit: [%s,%s]' % (
                            key, str(self._phb_pass['exit'][0]), str(self._phb_pass['exit'][1])))
                else:
                    temp_phb_pass = {
                        'alert': 'todo',
                        'state': 'register',
                        'todo': "applying",
                        'timeout': False,
                        'area': key,
                        "coords": self._phb_areas[key]['coords'],
                        'wise': self.enterPolygonWiseEum(SPoint(p1), SPoint(
                            p2), self._phb_areas[key]['gate_a'], self._phb_areas[key]['gate_b']),
                        'exit': self.exit_edge(SPoint(p2), key),
                        'robot_id': self._namespace[1:-1]
                    }
                    if (self._flexbe_state.data in self._flexbe_site)\
                            and ('exclusive' in self._flexbe_site[self._flexbe_state.data]) \
                            and (self._flexbe_site[self._flexbe_state.data]['exclusive']):
                        site_xy = self._flexbe_site[self._flexbe_state.data]['pose']['position']
                        site_point = SPoint(site_xy['x'], site_xy['y'])
                        if site_point.within(self._phb_areas[temp_phb_pass['area']]['polygon']):
                            temp_phb_pass['wise'] = 666
                    if temp_phb_pass['exit'] == 'none':
                        rospy.loginfo('%s: global path through area %s with wise: %d ,exit: none' %
                                      (self._flexbe_state.data, key, temp_phb_pass['wise']))
                    else:
                        rospy.loginfo('%s: global path through area %s with wise: %d, exit: [%s,%s]' %
                                      (self._flexbe_state.data, key, temp_phb_pass['wise'],
                                       str(temp_phb_pass['exit'][0]), str(temp_phb_pass['exit'][1])))
                    global_points = map(lambda p: (
                        round(p[0], 3), round(p[1], 3)), global_points)
                    inter_p = (round(p1[0], 3),    round(p1[1], 3))
                    prohibition_pass_list.append(
                        (global_points.index(inter_p),   copy.deepcopy(temp_phb_pass)))
        if prohibition_pass_list:
            prohibition_pass_list.sort()
            self._phb_pass_list = prohibition_pass_list
        if self._phb_pass['todo'] == 'applied' or self._phb_pass['todo'] == 'leaving':
            rospy.loginfo('replanning: unfinished %s, continue' %
                          self._phb_pass['area'])
        else:
            self._phb_pass['todo'] = 'todo'
        self.publish_phb_info()

    def teb_to_line(self, teb_msg, skip=5):
        poses = teb_msg.poses[:-1][::skip] + [teb_msg.poses[-1]]
        points = map(lambda p: (p.position.x, p.position.y), poses)
        return LineString(points)

    def path_to_points(self, plan_msg, polygon, skip=3, dist=-1):
        if plan_msg.poses:
            poses = plan_msg.poses[:-1][::skip] + [plan_msg.poses[-1]]
            points = map(lambda p: (p.pose.position.x,
                                    p.pose.position.y), poses)
            if dist > 0:
                points = filter(lambda p: abs(
                    p[0] - self.amcl_x) < dist and abs(p[1] - self.amcl_y) < dist, points)
            if polygon is not '':
                points = filter(lambda p: SPoint(
                    p[0], p[1]).within(polygon), points)

            return points
        else:
            return []

    def if_on_exit_side(self, phb_p):
        phb = phb_p['area']
        if phb_p['exit'] == 'none':
            return False
        elif abs(self._phb_areas[phb]['robot_dist'] - SPoint(
                self.amcl_x, self.amcl_y).distance(LineString(phb_p['exit']))) < 0.1:
            return True
        else:
            return False

    def mapPathCallback(self, msg):
        try:
            with open(relative_to_absolute_path("~/catkin_ws/dbparam/map_path.yaml")) as f:
                list_doc = yaml.safe_load(f)
                list_doc['image'] = "map_path.png"
                list_doc['occupied_thresh'] = 0.8
                list_doc['free_thresh'] = 0.411764
                list_doc['negate'] = 0
                list_doc['mode'] = "scale"
            with open(relative_to_absolute_path("~/catkin_ws/dbparam/map_path.yaml"),
                      "w") as f:
                yaml.dump(list_doc, f)
                rospy.loginfo("[RS] map_path yaml edited")
        except:
            rospy.logerr('[RS] Failed to edit map_path yaml: %s' % str(e))

    def flexbeTaskRemove(self, behavior_path_list, bahvior_name):
        for behavior_path in reversed(behavior_path_list):
            try:
                xml_path = '%s/%s.xml' % (behavior_path,
                                          self._behavior_dict[bahvior_name][1])
                pkg_e = behavior_path.index('_flexbe_behaviors')
                ss = behavior_path[:pkg_e]
                sm_path = '%ssrc/%s_flexbe_behaviors/%s_sm.py' % (
                    behavior_path[:-8], behavior_path[ss.rindex('/')+1: pkg_e], self._behavior_dict[bahvior_name][1])
                if self.fileRemove(xml_path) and self.fileRemove(sm_path):
                    break
            except Exception as e:
                pass

    def fileRemove(self, file_path):
        if os.path.isfile(file_path):
            os.remove(file_path)
            rospy.loginfo('%s is removed' % file_path)
            return True
        else:
            return False

    def flexbeSitePub(self):
        with open(self.special_area_path, 'r') as stream:
            areas = (yaml.safe_load(stream)).items()
            tag_areas = [
                a for a in areas if a[0][:3] == 'tag']
            self._tag_areas = {}
            for t in tag_areas:
                self._tag_areas[t[0]] = {"position": {"x": round(t[1]['position'][0], 2), "y": round(t[1]['position'][1], 2), "z": round(t[1]['position'][2], 2)},
                                         "tag_id": t[1]['tag_id'], "size": t[1]['size'],
                                         "create_time": t[1]['time']}
            prohibition_areas = [
                a for a in areas if a[0][:11] == 'prohibition']
            phb = {}
            for p in prohibition_areas:
                phb[p[0]] = {"area":  p[0], "coords": p[1]}

            path_areas = [a for a in areas if a[0][:4] == 'path']
            path_dict = {}
            for p in path_areas:
                path_dict[p[0]] = p[1]

        with open(self._flexbe_site_path) as site_f:
            self._flexbe_site = yaml.safe_load(site_f)
            msg = String()
            msg.data = json.dumps(
                {'flexbe_site': self._flexbe_site, 'tag': self._tag_areas, 'phb': phb, 'path': path_dict, 'character': self.characters}, sort_keys=True)
            self._flexbe_site_pub.publish(msg)

    def flexbeListPub(self, behavior_path_list):
        self._behavior_dict = {}
        for behavior_path in behavior_path_list:
            for b in os.listdir(behavior_path):
                f = behavior_path + '/' + b
                b_name = str(ET.parse(f).getroot().attrib['name'])
                self._behavior_dict[b_name] = [
                    ET.parse(f).getroot().find('tagstring').text + ':' +
                    ET.parse(f).getroot().find('description').text, b[:-4]
                ]

        behaviors_ordered = sorted(self._behavior_dict.items())
        json_string = "{"
        for item in behaviors_ordered[: -1]:
            json_string += '"%s": "%s", ' % (item[0], item[1][0])
        if behaviors_ordered:
            json_string += '"%s": "%s"' % (behaviors_ordered[-1][0],
                                           behaviors_ordered[-1][1][0])
        json_string += "}"

        list_msg = String()
        # list_msg.data = json.dumps(json_string)
        list_msg.data = json_string
        self._flexbe_list_pub.publish(list_msg)

    def alertControlCallback(self, msg):
        if msg.frame_id[:12] == "flexbe_start":
            split_index = msg.frame_id.find(':')
            mode = msg.frame_id[13:split_index]
            self.start_alert(msg.seq, mode)
            if mode == 'single':
                rospy.sleep(float(msg.frame_id[split_index + 1:]))
                self.end_alert(cancelgoal=False)
        elif msg.frame_id[:11] == "flexbe_stop":
            self.end_alert()

    def waypointsCallback(self, wp_msg):
        home_pose = filter(lambda wy: wy.name == self._home_point,
                           wp_msg.waypoints)
        if len(home_pose) != 1:
            return
        self._home_pose = home_pose[0].pose

    def aggCallback(self, agg_msg):
        purified_agg = dict([(a.name, a) for a in agg_msg.status])
        if self._estop_alert:
            try:
                if purified_agg[self._namespace + 'STATUS/ESTOP'].level == 1:
                    if not self._estop_mode:
                        if (self._namespace + 'STATUS/ESTOP/AREA' in purified_agg) and purified_agg[self._namespace + 'STATUS/ESTOP/AREA'].level == 1:
                            self.start_alert(42, 'loop')
                        else:
                            self.start_alert(7, 'loop')
                        self._estop_mode = True
                        rospy.logwarn('[RS] estop mode')
                else:
                    if self._estop_mode:
                        self.end_alert()
                        self._estop_mode = False
                        rospy.loginfo('[RS] operable mode')
            except Exception as e:
                rospy.logdebug('[RS] failed to parse agg msg: %s ' % str(e))

        if self._devices_alert and time.time() - RS_START > 40:
            try:
                if purified_agg[self._namespace + 'DEVICES'].level != 0:
                    if not self._agg_broken:
                        self.start_alert(8, 'loop')
                        self._agg_broken = True
                        rospy.logerr('[RS] device in fault status')
                else:
                    if self._agg_broken:
                        self.end_alert()
                        self._agg_broken = False
                        rospy.loginfo('[RS] device is operable again ')
            except Exception as e:
                rospy.logdebug('[RS] failed to parse agg msg: %s ' % str(e))
            # try:
            #     if purified_agg[self._namespace + 'DEVICES/LASER'] == 1:
            #         rospy.logwarn('[RS] scan agg warning')
            #     elif purified_agg[self._namespace + 'DEVICES/LASER'] == 2 \
            #             or purified_agg[self._namespace + 'DEVICES/LASER'] == 3 \
            #             or purified_agg[self._namespace + 'ALGORITHM/AMCL'] == 3:
            #         if self._behavior_state == 'buzzy':
            #             self.flexbePreemptPub.publish(Empty())
            #             rospy.logerr('[RS] scan agg error. Canceling Task')
            #         rospy.logerr('[RS] scan agg error.')
            # except Exception as e:
            #     rospy.logdebug('[RS] scan monitor exception: %s ' % str(e))

        if self._diversion['on_timeout'] > 0 and self._namespace + 'ALGORITHM/OBSTACLE' in purified_agg:
            self.alert_obstacle_agg(
                purified_agg[self._namespace + 'ALGORITHM/OBSTACLE'])
            if purified_agg[self._namespace +
                            'ALGORITHM/OBSTACLE'].level != 2:
                self._ob_ts = time.time()
            elif time.time() - self._ob_ts > self._diversion['on_timeout']:
                self._ob_ts = time.time()
                msg = Odometry()
                msg.header.frame_id = "map"
                msg.pose.pose.position.x = self.amcl_x
                msg.pose.pose.position.y = self.amcl_y
                (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                 msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
                 ) = tf.transformations.quaternion_from_euler(
                     0.0, 0.0, self.amcl_yaw)
                self.obstaclePosPub.publish(msg)

                if self._diversion['mode'] == 'off':
                    self._diversion['mode'] = 'check'
                    rospy.loginfo('[RS] check diversion mode')
        else:
            self._ob_ts = time.time()

    def areasVisulization(self):
        areas_msg = MarkerArray()
        if self._special_areas != None:
            areas_msg.markers += self.areaMakers(self._special_areas,
                                                 'dynamic_parameter')
        if self._door_areas != None:
            areas_msg.markers += self.areaMakers(self._door_areas,
                                                 'safe_doors', (0, 0.7, 0.3))
        if self._alley_areas:
            areas_msg.markers += self.areaMakers(self._alley_areas, 'alley',
                                                 (0.5, 0.2, 0.3))
        if self._phb_areas:
            areas_msg.markers += self.areaMakers(
                self._phb_areas,  'prohibition',  (0, 1.0, 0.7))
        return areas_msg

    def areaMakers(self,
                   areas,
                   ns,
                   rgb1=(0.7, 1, 1),
                   transparency=0.8,
                   frame_id='map'):

        markers = []
        spa_msg = Marker()
        spa_msg.ns = ns
        spa_msg.header.stamp = rospy.Time.now()
        spa_msg.header.frame_id = frame_id
        spa_msg.id = self.mid
        self.mid += 1
        spa_msg.type = 5
        spa_msg.action = 0
        spa_msg.scale.x = 0.08
        spa_msg.pose.orientation.w = 1
        if type(areas) is list:
            for area in areas:
                p1, p2, p3, p4 = Point(), Point(), Point(), Point()
                p1.x, p1.y = area[1]['xmin'], area[1]['ymin']
                p2.x, p2.y = area[1]['xmax'], area[1]['ymin']
                p3.x, p3.y = area[1]['xmax'], area[1]['ymax']
                p4.x, p4.y = area[1]['xmin'], area[1]['ymax']
                spa_msg.points += [p1, p2, p3, p2, p3, p4, p1, p4]

                color1 = ColorRGBA()
                color1.a = transparency
                (color1.r, color1.g, color1.b) = rgb1
                spa_msg.colors += [color1, color1] * 4
                markers.append(spa_msg)
        else:
            points = []
            for key in areas:
                points = areas[key]['polygon'].exterior.coords
                for n in range(len(points)-1):
                    spa_msg.points += self.lineMarker(points[n], points[n+1])
                    spa_msg.colors += self.lineColor(rgb1, transparency)
                markers.append(spa_msg)
                gate_a = areas[key]['gate_a'].coords
                spa_msg.points += self.lineMarker(gate_a[0], gate_a[1])
                spa_msg.colors += self.lineColor((0.92,
                                                  0.455, 0.455), transparency)
                gate_b = areas[key]['gate_b'].coords
                spa_msg.points += self.lineMarker(gate_b[0], gate_b[1])
                spa_msg.colors += self.lineColor((0.545,
                                                  0.753, 0.882), transparency)
        return markers

    def lineMarker(self, x, y):
        p1, p2 = Point(), Point()
        p1.x, p1.y = x
        p2.x, p2.y = y
        return [p1, p2]

    def lineColor(self, c1, transparency):
        color1 = ColorRGBA()
        color1.a = transparency
        (color1.r, color1.g, color1.b) = c1
        return [color1, color1]

    def joyCallback(self, joyMsg):
        door_diagnostics_msg = DiagnosticArray()
        door_diagnostics_msg.header.stamp = rospy.Time.now()
        status_msg = DiagnosticStatus()
        status_msg.name = "estop/door"
        status_msg.level = 0
        try:
            if not (self.safe_door_xmin < self.amcl_x < self.safe_door_xmax and
                    self.safe_door_ymin < self.amcl_y < self.safe_door_ymax) \
                    and joyMsg.buttons[6] == 1:
                status_msg.level = 1
                status_msg.message = "safe door estop mode"
                rospy.logwarn('[RS] safe door estop mode (param-definition)')
            if self._door_areas != None:
                for door in self._door_areas:
                    if not (door[1]['xmin'] < self.amcl_x < door[1]['xmax'] and
                            door[1]['ymin'] < self.amcl_y < door[1]['ymax']) \
                            and joyMsg.buttons[6] == 1:
                        status_msg.level = 1
                        status_msg.message = "safe door estop mode"
                        rospy.logwarn(
                            '[RS] safe door estop mode (yaml-definition)')
            door_diagnostics_msg.status.append(status_msg)
            self.diagnosticsPub.publish(door_diagnostics_msg)
        except:
            rospy.logdebug('[RS] joy Callback Exceptions.')

    def tebPosesCallback(self, tebPosesMsg):
        teb_yaw_list = map(
            lambda f: tf.transformations.euler_from_quaternion([
                f.orientation.x, f.orientation.y, f.orientation.z, f.
                orientation.w
            ])[2], tebPosesMsg.poses)
        yaw_prediction_list = [self.amcl_yaw] + teb_yaw_list
        yaw_diff_prediction_list = np.diff(yaw_prediction_list)
        yaw_diff_filtered = filter(lambda f: abs(f) < 0.5,
                                   yaw_diff_prediction_list)

        yaw_diff_sum = sum(yaw_diff_filtered)
        if yaw_diff_sum > self.teb_turn_yaw:
            self.teb_turn_judge = 'left'
        elif yaw_diff_sum < -self.teb_turn_yaw:
            self.teb_turn_judge = 'right'
        else:
            self.teb_turn_judge = 'none'

        if self._diversion['mode'] == 'check':
            self.start_alert(45, 'single')
            self.setParams(self._diversion['dy_params'])
            time.sleep(2)
            self.end_alert(cancelgoal=False)
            dist_mean, dist_max = self.teb_to_global_path(tebPosesMsg)
            # yaw_max = np.max(np.abs(np.array(teb_yaw_list)-teb_yaw_list[0]))
            if dist_mean < 0.6 and dist_max < self._diversion['max_offtrack']:
                dynamicParameterSet(
                    'cmd_vel_filter', 'filter_enabled', 'False')
                self._diversion['stamp'] = time.time()
                self._diversion['start_pose'] = [
                    self.amcl_x, self.amcl_y, self.amcl_yaw]
                self._diversion['mode'] = 'on'
                rospy.loginfo('[RS] diversion check -> on')
            else:
                self._ob_ts = time.time()
                self.diversion_off()
                rospy.loginfo('[RS] diversion not feasible, check -> off')
        elif self._diversion['mode'] == 'on' and time.time() - self._diversion['stamp'] > 5:
            dist_mean, dist_max = self.teb_to_global_path(tebPosesMsg)
            if (dist_mean < 0.03 and dist_max < self._diversion['fit_dist']):
                self.diversion_off()
                rospy.loginfo('[RS] path fitting, diversion on -> off')
            elif time.time() - self._diversion['stamp'] > 20 and  \
                    np.sqrt((self.amcl_x - self._diversion['start_pose'][0])**2+(self.amcl_y - self._diversion['start_pose'][1])**2) < 2:
                self.diversion_off()
                rospy.loginfo('[RS] move slowly, diversion on -> off')
            elif time.time() - self._diversion['stamp'] > self._diversion['off_timeout']:
                self.diversion_off()
                rospy.loginfo('[RS] timeout, diversion on -> off')

    def diversion_off(self):
        self.start_alert(46, 'single')
        dynamicParameterSet('cmd_vel_filter', 'filter_enabled', 'True')
        self.setParams(self._diversion['pre_params'])
        self._diversion['mode'] = 'off'
        self._ob_ts = time.time()
        self.end_alert(cancelgoal=False)

    def teb_to_global_path(self, tebPosesMsg):
        try:
            teb_xy_list = map(lambda f: (
                f.position.x, f.position.y), tebPosesMsg.poses)
            global_plan_line = LineString(self.path_to_points(
                self._global_plan_msg, '', dist=5))
            poses_dist = map(lambda t: SPoint(t[0], t[1]).distance(
                global_plan_line), teb_xy_list)
            return np.mean(poses_dist), np.max(poses_dist)
        except:
            return 1, 1

    def enterPolygonValue(self, a, b, min_d=0.5):
        if abs(a-b) > min_d:
            return (b-a)/abs(b-a)
        else:
            return 0

    def exit_edge(self, p, phb):
        dist_list = []
        for i in range(len(self._phb_areas[phb]["edges"])):
            dist_list.append((p.distance(self._phb_areas[phb]["edges"][i]), i))
        dist_list.sort()
        if dist_list[0][0] < 0.25:
            edge = self._phb_areas[phb]["edges"][dist_list[0][1]]
            return [list(edge.coords[0]), list(edge.coords[1])]
        else:
            return 'none'

    def enterPolygonWiseEum(self, p1, p2, g1, g2):
        if p1.distance(g1) < 0.25:
            return 1
        elif p1.distance(g2) < 0.25:
            return -1
        elif p2.distance(g1) < 0.25:
            return -1
        elif p2.distance(g2) < 0.25:
            return 1
        else:
            return -666

    def enterPolygonWiseGeom(self, ox, oy, rx, ry):
        print ox, oy, rx, ry
        if abs(ox-rx) <= 1 and abs(oy-ry) <= 1:
            rospy.loginfo('geom-positive-wise')
            return 1
        elif rx == 0 and abs(oy-ry) == 2:
            rospy.loginfo('geom-negative-wise')
            return -1
        elif ry == 0 and abs(ox-rx) == 2:
            rospy.loginfo('geom-negative-wise')
            return -1
        elif abs(ox-rx) >= 1 and abs(oy-ry) >= 1:
            rospy.loginfo('geom-negative-wise')
            return -1
        else:
            rospy.loginfo('geom-unknown-wise')
            return 0

    def cancelTasks(self):
        self.cancelHomingPub.publish(GoalID())
        rospy.loginfo('[RS] Cancel Homing goal')
        self.pubTaskSwitch('shelf', 0)
        self.pubTaskSwitch('charging_pile', 0)
        self.pubTaskSwitch('one', 0)
        self.pubTaskSwitch('triangle', 0)
        self.musicCancelPub.publish(GoalID())
        self.if_bgm_canceled = True
        self.alertCancelPub.publish(GoalID())
        self.is_alerting = False

    def navCtrlCmdCallback(self, navCmdMsg):
        if navCmdMsg.control == 0:
            self.cancelTasks()

    def flexbePreemptCallback(self, data):
        self.cancelMoveBasePub.publish(GoalID())
        self.cancelTasks()
        self.start_alert(22, 'single')
        rospy.sleep(2.5)
        rospy.loginfo('[RS] flexbe preempted')
        self.end_alert(cancelgoal=False)

    def flexbePauseCallback(self, msg):
        if msg.data:
            self._flexbe_status['pause'] = True
            self.start_alert(23, 'single')
            rospy.sleep(2.5)
            rospy.loginfo('[RS] flexbe paused')
            self.end_alert(cancelgoal=False)
        else:
            self._flexbe_status['pause'] = False
            self.start_alert(24, 'single')
            rospy.loginfo('[RS] flexbe resumed')
            rospy.sleep(2.5)
            self.end_alert(cancelgoal=False)

    def moveBaseResultCallback(self, resultMsg):
        if resultMsg.status.status == 2 or resultMsg.status.status == 3:
            amcl_point = SPoint(self.amcl_x, self.amcl_y)
            if self._phb_pass['todo'] == "applying" or self._phb_pass['todo'] == "waiting":
                rospy.loginfo('[RS] applying %s to be reset while movebase canceled' %
                              self._phb_pass['area'])
                self._phb_pass = {'alert': 'todo',
                                  'todo': 'todo', 'state': 'canceled_0', 'timeout': False, 'applied_time': 0}
            elif self._phb_pass['todo'] == "applied" and \
                    amcl_point.disjoint(self._phb_areas[self._phb_pass['area']]['polygon']):
                rospy.logwarn('[RS] robot out of applied %s, reset area while movebase canceled' %
                              self._phb_pass['area'])
                self._phb_pass['todo'] = "leaving"
                self._phb_pass['timeout'] = False
                self._phb_pass['applied_time'] = 0
                self._phb_pass['state'] = "canceled_1"
                rospy.loginfo('informing server')
            elif self._phb_pass['todo'] == "applied" and \
                    amcl_point.within(self._phb_areas[self._phb_pass['area']]['polygon']):
                rospy.logwarn('[RS] robot within applied %s, cannot reset area while movebase canceled' %
                              self._phb_pass['area'])
                self._phb_pass['state'] = "cannot_canceled"
            elif self._phb_pass['todo'] == "leaving":
                rospy.logwarn('[RS] robot leaving %s, continue to reset area while movebase canceled' %
                              self._phb_pass['area'])
                self._phb_pass['state'] = "continue_leaving"
            self._phb_pass_list = []
            self.publish_phb_info()
        self.moveBaseGoalAlert(resultMsg)

    def moveBaseGoalAlert(self, resultMsg):
        if resultMsg.status.status == 3:
            self.start_alert(5, 'single')
            rospy.loginfo('[RS] move base goal reached')
            self.teb_turn_judge = 'none'
            rospy.sleep(2.5)
            self.end_alert(cancelgoal=False)
        elif resultMsg.status.status == 2:
            self.teb_turn_judge = 'none'

    def obstacleDistanceCallback(self, disMsg):
        return
        if len(self.obstacle_distances) <= 5:
            self.obstacle_distances.append(disMsg.data)
        if len(self.obstacle_distances) >= 6:
            self.obstacle_distances.pop(0)
        try:
            self.alert_obstacle(self.obstacle_distances)
        except:
            rospy.logdebug('[RS-Ecp] detect obstacle distance callback')

    def alert_obstacle(self, dis_list):
        mean_dist = np.mean(dis_list)
        enable_flag = (not self._detecting) and self._robot_active
        if mean_dist <= self.obstacle_alert_distance and enable_flag:
            if not self.is_alerting:
                self.start_alert(2, 'loop')
                self.close_obstacle = "true"
        elif mean_dist > self.obstacle_alert_distance and self.close_obstacle == "true":
            if not self._detecting:
                self.end_alert()
            self.close_obstacle = "false"
        elif mean_dist <= self.obstacle_alert_distance and (
                not self._robot_active) and self.close_obstacle == "true":
            self.end_alert()
            self.close_obstacle = "false"

    def alert_obstacle_agg(self, agg_status):
        enable_flag = (not self._detecting) and self._robot_active
        obs_state = filter(lambda v: "cmd_vel_filter" in v.key,
                           agg_status.values)
        if len(obs_state) > 0:
            if obs_state[0].value == "obstacle alert" and enable_flag:
                if not self.is_alerting:
                    self.start_alert(2, 'loop')
                    self.close_obstacle = "true"
            elif obs_state[0].value != "obstacle alert" and self.close_obstacle == "true":
                if not self._detecting:
                    self.end_alert()
                self.close_obstacle = "false"
            elif obs_state[0].value == "obstacle alert" and (
                    not self._robot_active) and self.close_obstacle == "true":
                self.end_alert()
                self.close_obstacle = "false"
            '''
            if obs_state[0].value == "side alert" and enable_flag:
                if not self.is_alerting:
                    self.start_alert(2, 'loop')
                    self.close_obstacle = "side"
            elif obs_state[0].value != "side alert" and self.close_obstacle == "side":
                if not self._detecting:
                    self.end_alert()
                self.close_obstacle = "false"
            elif obs_state[0].value == "side alert" and (
                    not self._robot_active) and self.close_obstacle == "side":
                self.end_alert()
                self.close_obstacle = "false"
            '''

    def velCallback(self, velMsg):
        if self.cmd_vel_music:
            if len(self.cmd_vel_linear) <= self.turn_threshold_num:
                self.cmd_vel_linear.append(velMsg.linear.x)
                self.cmd_vel_angular.append(velMsg.angular.z)
            else:
                self.cmd_vel_linear.pop(0)
                self.cmd_vel_angular.pop(0)
                try:
                    self.judgeTurn(self.cmd_vel_angular)
                    self.judgeBack(self.cmd_vel_linear)
                except:
                    rospy.logdebug('[RS-Ecp] velocity Callback')

    def pause_enforcement(self):
        music_control_msg = Header()
        music_control_msg.frame_id = 'pause'
        self.musicControlPub.publish(music_control_msg)

    def start_alert(self, play_id, play_mode, only_bgm_control=False):
        music_control_msg = Header()
        music_control_msg.frame_id = 'pause'
        self.musicControlPub.publish(music_control_msg)
        self.bgm_pause = True
        rospy.logdebug('[RS] Pause bgm on starting alert')

        if not only_bgm_control:
            alert_goal_msg = MedianPlayActionGoal()
            alert_goal_msg.goal.mode = play_mode
            alert_goal_msg.goal.sound_id = play_id
            self.alertGoalPub.publish(alert_goal_msg)
            rospy.logdebug('play ID {}'.format(play_id) +
                           ' with mode {}'.format(play_mode))
            self.is_alerting = True

    def end_alert(self, cancelgoal=True):
        if cancelgoal:
            self.alertCancelPub.publish(GoalID())
            rospy.logdebug('[RS] Cancel Alert Action goal')

        music_control_msg = Header()
        music_control_msg.frame_id = 'set_pause'
        rospy.logdebug('[RS] BGM set pause to play')
        # self.if_bgm_canceled = False
        self.bgm_pause = False
        rospy.sleep(2)
        self.musicControlPub.publish(music_control_msg)
        self.is_alerting = False

    def judgeTurn(self, vel_list):
        mean_angular = np.mean(vel_list)
        if self._estop_mode:
            return
        if mean_angular > self.turn_threshold_omega or self.teb_turn_judge == 'left':
            self.is_alerting = True
            self.start_alert(4, 'loop', only_bgm_control=True)
            if self.turn_judge != 'left':
                self.start_alert(4, 'loop')
                self.turn_judge = 'left'
                rospy.loginfo_throttle(2, '[RS] Robot turning left')
        elif mean_angular < -self.turn_threshold_omega or self.teb_turn_judge == 'right':
            self.is_alerting = True
            self.start_alert(3, 'loop', only_bgm_control=True)
            if self.turn_judge != 'right':
                self.start_alert(3, 'loop')
                self.turn_judge = 'right'
                rospy.loginfo_throttle(2, '[RS] Robot turning right')
        else:
            if self.turn_judge != 'none':
                self.end_alert()
                self.turn_judge = 'none'

    def judgeBack(self, vel_list):
        mean_vel = np.mean(vel_list)
        if mean_vel <= self.back_alert_vel:
            self.is_alerting = True
            self.start_alert(18, 'loop', only_bgm_control=True)
            if not self.back_judge:
                self.start_alert(18, 'loop')
                self.back_judge = True
                rospy.loginfo_throttle(2, '[RS] Robot doing back')
        else:
            if self.back_judge:
                self.end_alert()
                self.back_judge = False

    def taskSwitchCallback(self, taskMsg):
        try:
            self.task_switch_json_object = json.loads(taskMsg.frame_id)
            rospy.loginfo('[RS] Get json style task switch')
            if self.task_switch_json_object['op'] == 'dbparam':
                self.dbparam_git_task = True

        except Exception as e:
            self.dbparam_git_task = False
            # deal with logic None.
            if taskMsg.frame_id in ['shelf',   'triangle',   'one',   'charging_pile', 'apriltag']:
                self.startDetector = self.switchFlag(taskMsg)
                self.detectorTarget = taskMsg.frame_id

            elif taskMsg.frame_id == 'system_volume':
                system_volume_cmd = 'pactl set-sink-volume @DEFAULT_SINK@ ' + \
                    str(taskMsg.seq)+'%'
                subprocess.call(system_volume_cmd, shell=True)
                rospy.loginfo('[RS] Set system volume to {}%.'.format(
                    taskMsg.seq))

            elif taskMsg.frame_id == 'initialization_ending' and taskMsg.seq == 1:
                self.initialization_completition = True

            elif taskMsg.frame_id == 'update_request' and taskMsg.seq == 1:
                self.updateRequestFlag = True

            elif taskMsg.frame_id[:13] == 'flexbe_delete' and taskMsg.seq == 1:
                self.flexbeTaskRemove(self._manifest_path_list,
                                      taskMsg.frame_id[14:])
                self.flexbeListPub(self._manifest_path_list)
                self.start_alert(35, 'single')
                rospy.sleep(2)

            elif taskMsg.frame_id[:7] == 'launch_' and taskMsg.seq == 1:
                self._switch_str = taskMsg.frame_id

            elif taskMsg.frame_id[:6] == 'shell_' and taskMsg.seq == 1:
                self._switch_str = taskMsg.frame_id

            elif taskMsg.frame_id == 'alert':
                self.start_alert(int(taskMsg.seq), 'single')
                rospy.sleep(3)

            elif taskMsg.frame_id[:4] == 'lock':
                dynamicParameterSet(
                    'move_base/global_costmap/'+taskMsg.frame_id[5:], 'enabled', True)
            elif taskMsg.frame_id[:6] == 'unlock':
                dynamicParameterSet(
                    'move_base/global_costmap/'+taskMsg.frame_id[7:], 'enabled', False)

            elif taskMsg.frame_id == 'slam':
                self.startSLAM = self.switchFlag(taskMsg)
                if taskMsg.seq == 1:
                    self.slamReset = True
                elif taskMsg.seq == 2:
                    self.slamContinue = True

            elif taskMsg.frame_id[:8] == 'save_map':
                self.saveMap = self.switchFlag(taskMsg)
                self._switch_str = taskMsg.frame_id[9:]

            elif taskMsg.frame_id == 'slam_pause':
                self.slamPause = self.switchFlag(taskMsg)

            elif taskMsg.frame_id == 'slam_go_on':
                self.slamGoOn = self.switchFlag(taskMsg)

            elif self._departure_alert and taskMsg.frame_id == 'departure' and taskMsg.seq == 1:
                self.start_alert(11, 'single')
                rospy.loginfo('[RS] Departuring Attention')
                rospy.sleep(3)
                self.end_alert(cancelgoal=False)
            elif taskMsg.frame_id[:10] == 'delete_tag' and taskMsg.seq == 1:
                with open(self.special_area_path, 'r') as f:
                    list_doc = yaml.safe_load(f)
                    list_doc_copy = list_doc
                try:
                    if taskMsg.frame_id[7:] in list_doc:
                        with open(self.special_area_path, 'w') as f:
                            del list_doc[taskMsg.frame_id[7:]]
                            yaml.dump(list_doc, f)
                    else:
                        pass
                    self.start_alert(29, 'single')
                    rospy.loginfo('[RS] %s ' % taskMsg.frame_id)
                    rospy.sleep(2)
                except Exception as e:
                    with open(self.special_area_path, 'w') as f:
                        yaml.dump(list_doc_copy, f)
                    rospy.logwarn("[RS] %s exception: %s" %
                                  (taskMsg.frame_id, str(e)))
                self.flexbeSitePub()
            elif taskMsg.frame_id[:11] == 'delete_site' and taskMsg.seq == 1:
                self.delete_special_area(
                    self._flexbe_site_path, taskMsg.frame_id[12:], 34)
                with open(self._flexbe_site_path, 'r') as f:
                    self._flexbe_site = yaml.safe_load(f)
            elif taskMsg.frame_id[:11] == 'create_site' and taskMsg.seq == 1:
                with open(self._flexbe_site_path, 'r') as f:
                    list_doc = yaml.safe_load(f)
                    list_doc_copy = list_doc
                try:
                    if taskMsg.frame_id[12:] not in list_doc:
                        with open(self._flexbe_site_path, 'w') as f:
                            rot = tf.transformations.quaternion_from_euler(
                                0.0, 0.0, self.amcl_yaw)
                            list_doc[taskMsg.frame_id[12:]] = {
                                'frame_id': 'map',
                                'pose': {
                                    'orientation': {'x': 0, 'y': 0, 'z': round(float(rot[2]), 5), 'w': round(float(rot[3]), 5)},
                                    'position': {'x': round(self.amcl_x, 5), 'y': round(self.amcl_y, 5), 'z': 0}
                                }
                            }
                            yaml.dump(list_doc, f)
                            rospy.loginfo('[RS] %s ' % taskMsg.frame_id)
                            self._flexbe_site = list_doc
                    else:
                        rospy.logwarn('%s already exists' %
                                      taskMsg.frame_id[12:])
                    self.start_alert(33, 'single')
                    rospy.sleep(2)
                except Exception as e:
                    with open(self._flexbe_site_path, 'w') as f:
                        yaml.dump(list_doc_copy, f)
                    rospy.logwarn("[RS] %s exception: %s" %
                                  (taskMsg.frame_id, str(e)))
                self.flexbeSitePub()
            elif taskMsg.frame_id[:10] == 'phb_create' and taskMsg.seq == 1:
                with open(self.special_area_path, 'r') as f:
                    list_doc = yaml.safe_load(f)
                    list_doc_copy = list_doc
                try:
                    phb = json.loads(taskMsg.frame_id[11:])
                    for key in phb:
                        phb[str(key)] = phb.pop(key)
                    with open(self.special_area_path, 'w') as f:
                        yaml.dump(dict(list_doc, **phb), f)
                        rospy.loginfo('[RS] create new phb area')
                    self.start_alert(36, 'single')
                    rospy.sleep(2)
                except Exception as e:
                    with open(self.special_area_path, 'w') as f:
                        yaml.dump(list_doc_copy, f)
                        rospy.logwarn("[RS] %s exception: %s" %
                                      (taskMsg.frame_id, str(e)))
                self.flexbeSitePub()
            elif taskMsg.frame_id[:10] == 'delete_phb' and taskMsg.seq == 1:
                self.delete_special_area(
                    self.special_area_path, taskMsg.frame_id[11:], 37)
            # elif taskMsg.frame_id[:11] == "delete_path" and taskMsg.seq == 1:
            #     self.delete_special_area(
            #         self.special_area_path, taskMsg.frame_id[12:], 37)
            #     self.flexbeSitePub()

    def delete_special_area(self, file_path, k, alert_id):
        with open(file_path, 'r') as f:
            list_doc = yaml.safe_load(f)
            list_doc_copy = list_doc
        try:
            if k in list_doc:
                with open(file_path, 'w') as f:
                    del list_doc[k]
                    yaml.dump(list_doc, f)
            else:
                pass
            self.start_alert(alert_id, 'single')
            rospy.sleep(2)
            rospy.loginfo('[RS] delete %s ' % k)
        except Exception as e:
            with open(file_path, 'w') as f:
                yaml.dump(list_doc_copy, f)
            rospy.logwarn("[RS] %s exception: %s" %
                          (taskMsg.frame_id, str(e)))
        self.flexbeSitePub()

    # switchFlag: used to check for startup or shutdown tasks
    def switchFlag(self, headerMsg):
        return (headerMsg.seq >= 1)

    def monitorBatteryCallback(self, batteryMsg):
        self.battery_percentage = batteryMsg.percentage
        self._charging = (batteryMsg.current > 0)

    def bgm_on(self, bgm_mode, bgm_id):
        start_music_msg = MedianPlayActionGoal()
        start_music_msg.goal.mode = bgm_mode
        start_music_msg.goal.sound_id = bgm_id
        self.musicGoalpub.publish(start_music_msg)
        self.if_bgm_canceled = False

    def navStateCallback(self, navStateMsg):
        # IDLING=0, RUNNING=1, PAUSED=2, COMPLETED=3, CANCELLED=4, ERROR=-1
        if navStateMsg.status == 0 or navStateMsg.status == 4:
            self._behavior_state = 'free'
            self.musicCancelPub.publish(GoalID())
            self.if_bgm_canceled = True
        else:
            self._behavior_state = 'buzzy'

        self._waypoint_name = navStateMsg.waypoint_name
        # self._waypoint_name = navStateMsg.status_desc
        wait_time = 0
        if self._waypoint_name[:4] == 'wait':
            newstr = ''.join((ch if ch in '0123456789' else ' ')
                             for ch in self._waypoint_name)
            listOfNumbers = [float(i) for i in newstr.split()]
            try:
                wait_time = listOfNumbers[0]
            except:
                rospy.logdebug('[RS-Ecp] wait time guess')

        if wait_time > self._trash_wait_time:
            self._trashing = True
            self.start_alert(12, 'loop')
        elif self._trashing:
            # stop play trash
            self.end_alert()
            self._trashing = False
        if navStateMsg.status == 4:
            self._last_4_state = True
        if self._last_4_state and navStateMsg.status == 0:
            self.task_diag_msg = diag_array_gen(
                'task_state', 1, 'task cancled')
            self._last_4_state = False
        else:
            self.task_diag_msg = diag_array_gen('task_state', 0, 'ok')

    def flexbeStatusCallback(self, fStatusMsg):
        if fStatusMsg.code == 1 or fStatusMsg.code == 2 or fStatusMsg.code == 20:
            exit_msg = String()
            if fStatusMsg.args == ['preempted']:
                exit_msg.data = 'behavior_preempted'
            else:
                exit_msg.data = 'behavior_exit'
            self._flexbe_state_pub.publish(exit_msg)
            self._flexbe_state = exit_msg

            self._behavior_state = 'free'
            self._flexbe_status['level'] = 0
            self._flexbe_status['pause'] = False

            time.sleep(2)
            self.musicCancelPub.publish(GoalID())
            self.if_bgm_canceled = True

            if fStatusMsg.code == 1 or fStatusMsg.code == 2:
                try:
                    # self._flexbe_log[self._log_id] = self._check_num_dict[
                    #     self.to_int16(fStatusMsg.behavior_id)] + '->done'
                    self._flexbe_log = {}
                    task_name = self._check_num_dict[self.to_int16(
                        fStatusMsg.behavior_id)]
                    self._flexbe_status['behavior'] = task_name
                    self._flexbe_log[1] = task_name + '->done'
                    if self._behavior_dict[task_name][0][:4] != 'base' and self.character_path != '':
                        self.char_upd_dict['d_number'] += 1
                        self.char_upd_dict['d_wt'] = self.char_upd_dict['d_wt'] + \
                            (time.time() - self.char_upd_dict['w_ts'])
                    log_msg = String()
                    log_msg.data = json.dumps(self._flexbe_log)
                    self._flexbe_log_pub.publish(log_msg)
                except Exception as e:
                    rospy.logwarn('[RS] exception: %s' % str(e))
        elif fStatusMsg.code == 0:
            self._behavior_state = 'buzzy'
            if self._flexbe_status['behavior'] == "autocharge_on" and \
                    self._check_num_dict[self.to_int16(fStatusMsg.behavior_id)] != "autocharge_on":
                rospy.loginfo("other task after autocharge, battery: {:5.3f}".format(
                    self.battery_percentage))
                self.pubJoyFeedback(7, 0, 1)
            self._log_id += 1
            if self.character_path != '':
                self.char_upd_dict['w_ts'] = time.time()
            self._flexbe_status['level'] = 1
            self._flexbe_status['behavior'] = self._check_num_dict[self.to_int16(
                fStatusMsg.behavior_id)]

        elif fStatusMsg.code == 11:
            rospy.logerr('[RS] flexbe status is ERROR')
            self._flexbe_status['level'] = 2

    def flexbeStateCallback(self, fStateMsg):
        if fStateMsg.data == "":
            self._behavior_state = 'free'
        else:
            try:
                self._behavior_state = 'buzzy'
                self._flexbe_state_pub.publish(fStateMsg)
                self._flexbe_state = fStateMsg

                # indx = self._flexbe_log[self._log_id].find('->')
                # self._flexbe_log[self._log_id] = self._flexbe_log[self._log_id][:indx] + \
                #     '->'+fStateMsg.data

                # log_msg = String()
                # log_msg.data = json.dumps(self._flexbe_log)
                # self._flexbe_log_pub.publish(log_msg)

                if fStateMsg.data in ["tag_register", "update_learned_path", "update_collection"]:
                    self.flexbeSitePub()
                    if fStateMsg.data == "update_learned_path":
                        self.start_alert(43, 'single')
                        rospy.sleep(2.5)
            except Exception as e:
                rospy.logwarn('[RS] exception: %s' % str(e))

    def pubJoyFeedback(self, id, intensity, type):
        joy_feedback_msg = JoyFeedbackArray()
        discharge_msg = JoyFeedback()
        joy_feedback_msg.array.append(discharge_msg)
        joy_feedback_msg.array[0].id = 7
        joy_feedback_msg.array[0].intensity = 0
        joy_feedback_msg.array[0].type = 1
        self.pubDischarge.publish(joy_feedback_msg)
        rospy.loginfo('joy feedback published')

    def amclCallback(self, amclMsg):
        self.amcl_x = amclMsg.pose.pose.position.x
        self.amcl_y = amclMsg.pose.pose.position.y
        _, _, self.amcl_yaw = tf.transformations.euler_from_quaternion([
            amclMsg.pose.pose.orientation.x, amclMsg.pose.pose.orientation.y,
            amclMsg.pose.pose.orientation.z, amclMsg.pose.pose.orientation.w
        ])
        if self._special_areas != None:
            try:
                for area in self._special_areas:
                    if (not self._in_special_area)  \
                            and (area[1]['xmin'] <= self.amcl_x <= area[1]['xmax']) \
                            and (area[1]['ymin'] <= self.amcl_y <= area[1]['ymax']):
                        params_dict = area[1].copy()
                        params_dict.pop('xmin')
                        params_dict.pop('xmax')
                        params_dict.pop('ymin')
                        params_dict.pop('ymax')
                        self.__pre_params_dict = self.getParamsDictionary(
                            params_dict.keys())
                        print "\033[0;31;47m--- in ---\033[0m"
                        self.setParams(params_dict)
                        self._in_special_area = True
                        self._special_info = [
                            area[1]['xmin'], area[1]['xmax'], area[1]['ymin'],
                            area[1]['ymax']
                        ]
                        break
                if self._in_special_area:
                    if not (self._special_info[0] <= self.amcl_x <=
                            self._special_info[1] and self._special_info[2] <=
                            self.amcl_y <= self._special_info[3]):
                        rospy.loginfo(
                            '[RS] Out special area, restore parameters')
                        self.setParams(self.__pre_params_dict)
                        self._in_special_area = False
                        print "\033[0;31;47m--- out ---\033[0m"
            except:
                rospy.logwarn('[RS] Exception in special area')

        if self._alley_areas:
            try:
                for area in self._alley_areas:
                    if (not self._in_alley)  \
                            and (area[1]['xmin'] <= self.amcl_x <= area[1]['xmax']) \
                            and (area[1]['ymin'] <= self.amcl_y <= area[1]['ymax']):
                        print "\033[0;32;47m--- in Alley ---\033[0m"
                        self._in_alley = True
                        msg = PointStamped()
                        msg.header.seq = 1
                        msg.header.stamp = rospy.Time.now()
                        msg.header.frame_id = 'i:' + self._namespace + 'map'
                        msg.point.x = area[1]['block'][0]
                        msg.point.y = area[1]['block'][1]
                        self.alleyPub.publish(msg)
                        self._alley_info = [
                            area[1]['xmin'], area[1]['xmax'], area[1]['ymin'],
                            area[1]['ymax']
                        ]
                        break
                if self._in_alley:
                    if not (self._alley_info[0] <= self.amcl_x <=
                            self._alley_info[1] and self._alley_info[2] <=
                            self.amcl_y <= self._alley_info[3]):
                        self._in_alley = False
                        msg = PointStamped()
                        msg.header.seq = 0
                        msg.header.stamp = rospy.Time.now()
                        msg.header.frame_id = 'o:' + self._namespace + 'map'
                        msg.point.x = area[1]['block'][0]
                        msg.point.y = area[1]['block'][1]
                        self.alleyPub.publish(msg)
                        print "\033[0;32;47m--- out Alley ---\033[0m"
            except Exception as e:
                rospy.logwarn('[RS] exception: %s' % str(e))

        amcl_point = SPoint(self.amcl_x, self.amcl_y)
        if 'area' in self._phb_pass:
            key = self._phb_pass['area']
            if (not amcl_point.within(self._phb_areas[key]['polygon'])) and self._phb_areas[key]['if_amcl_in']:
                self._phb_areas[key]['if_amcl_in'] = False
                rospy.loginfo("robot out of %s: pose ({0}, {1}, {2}) ".format(
                    round(self.amcl_x, 2),  round(self.amcl_y, 2), round(self.amcl_yaw, 2)) % key)
                if self._phb_pass['todo'] == "applied":
                    self._phb_pass['todo'] = "leaving"
                elif self._phb_pass['todo'] == "applying":
                    self._phb_pass['todo'] = "leaved"
                self._phb_pass['state'] = "out of area"
                self._phb_pass['applied_time'] = 0
                self.publish_phb_info()
            elif (self._phb_pass['todo'] == "applied" or self._phb_pass['todo'] == "applying") and (not amcl_point.within(self._phb_areas[key]['polygon'])):
                self._phb_areas[key]['if_amcl_in'] = amcl_point.within(
                    self._phb_areas[key]['polygon'])
                self._phb_areas[key]['robot_dist'] = amcl_point.distance(
                    self._phb_areas[key]['polygon'])
                if (self._phb_areas[key]['robot_dist'] > self.area_apply_dist or
                    (self._phb_pass['exit'] != 'none' and
                     abs(self._phb_areas[key]['robot_dist'] - amcl_point.distance(LineString(self._phb_pass['exit']))) < 0.1)):
                    rospy.loginfo("robot out of %s nearby: pose ({0}, {1}, {2}) ".format(
                        round(self.amcl_x, 2),  round(self.amcl_y, 2), round(self.amcl_yaw, 2)) % key)
                    if self._phb_pass['todo'] == "applied":
                        self._phb_pass['todo'] = "leaving"
                    else:
                        self._phb_pass['todo'] = "leaved"
                    self._phb_pass['state'] = "out of area nearby"
                    self._phb_pass['applied_time'] = 0
                    self.publish_phb_info()

            if self._phb_pass['todo'] != "applied" and self._phb_areas[key]['if_amcl_in']:
                rospy.logerr_throttle(5, 'robot in %s, but phb todo is %s' % (
                    key, self._phb_pass['todo']))

        for phb in self._phb_areas:
            self._phb_areas[phb]['if_amcl_in'] = amcl_point.within(
                self._phb_areas[phb]['polygon'])
            self._phb_areas[phb]['robot_dist'] = amcl_point.distance(
                self._phb_areas[phb]['polygon'])

    def getParamsDictionary(self, params_list):
        dictt = {}
        try:
            for param in params_list:
                dictt[param] = rospy.get_param(param)
        except:
            rospy.logdebug(
                '[RS] Exception while getting pre-parameter in special area')
        return dictt

    def setParams(self, params_dict):
        for param in params_dict:
            line_idx = -1 - (param[::-1]).index('/')
            try:
                dynamicParameterSet(param[:line_idx], param[line_idx + 1:],
                                    params_dict[param])
            except:
                rospy.logdebug(
                    '[RS] Exception while setting parameter in special area')

    def odomCallback(self, odomMsg):
        if self.character_path != '':
            if self.char_upd_dict['m_ts'] == 0:
                self.char_upd_dict['m_ts'] = time.time()
            else:
                self.char_upd_dict['d_mileage'] += (time.time()-self.char_upd_dict['m_ts']) * \
                    abs(odomMsg.twist.twist.linear.x)
                self.char_upd_dict['m_ts'] = time.time()

        if self.odom_vel_music:
            if len(self.odom_vel_angular) <= self.turn_threshold_num:
                self.odom_vel_angular.append(odomMsg.twist.twist.angular.z)
                self.odom_vel_linear.append(odomMsg.twist.twist.linear.x)
            else:
                self.odom_vel_angular.pop(0)
                self.odom_vel_linear.pop(0)
                try:
                    self.judgeTurn(self.odom_vel_angular)
                    self.judgeBack(self.odom_vel_linear)

                except:
                    rospy.logdebug('[RS-Ecp] judge turn by odom')
            buzzy_vel = np.mean(np.abs(self.odom_vel_linear))
        else:
            buzzy_vel = abs(odomMsg.twist.twist.linear.x)

        try:
            if not self._battery_alert:
                pass
            elif self.battery_percentage <= self._low_battery \
                    and self._bgm_id != 19 \
                    and self._waypoint_name != "autocharge_on":
                self._bgm_id = 19
                self.bgm_on('loop', self._bgm_id)
                rospy.logwarn('[RS] low battery warning on')
            elif self._bgm_id == 19 and \
                (self._waypoint_name == "autocharge_on"
                    or self.battery_percentage > self._low_battery):
                self._bgm_id = rospy.get_param("~bgm_id", 10)
                self.bgm_on('loop', self._bgm_id)
                rospy.logwarn('[RS] battery warning off')
        except:
            pass

        if self._behavior_state == 'buzzy' or buzzy_vel > self.bgm_vel:
            self._robot_active = True
            shield_bgm = (self.is_alerting or self._trashing or self._charging
                          or self._detecting)
            if self.if_bgm_canceled and (not shield_bgm):
                self.bgm_on('loop', self._bgm_id)
                rospy.logdebug(
                    '[RS] Moving or buzzy state: BGM ON / BGM ACTION GOAL')
            elif (not self.if_bgm_canceled) and shield_bgm:
                self.musicCancelPub.publish(GoalID())
                self.if_bgm_canceled = True
        elif (not self.if_bgm_canceled) or time.time() - self._bgm_stamp < 5:
            self._robot_active = False
            self.musicCancelPub.publish(GoalID())
            rospy.logdebug(
                '[RS] Still or free state: BGM OFF /BGM ACTION CANCEL')
            self.if_bgm_canceled = True
        else:
            self._robot_active = False

        self.odom_x = odomMsg.pose.pose.position.x
        self.odom_y = odomMsg.pose.pose.position.y
        _, _, self.odom_yaw = tf.transformations.euler_from_quaternion([
            odomMsg.pose.pose.orientation.x, odomMsg.pose.pose.orientation.y,
            odomMsg.pose.pose.orientation.z, odomMsg.pose.pose.orientation.w
        ])

        if self._diversion['mode'] == 'on' and time.time() - self._diversion['stamp'] > self._diversion['off_timeout']:
            self.diversion_off()
            rospy.loginfo('[RS] timeout, diversion on -> off')

    def pubTaskSwitch(self, task_name, switch_seq):
        taskPubMsg = Header()
        taskPubMsg.stamp = rospy.Time.now()
        taskPubMsg.frame_id = task_name
        taskPubMsg.seq = switch_seq
        self.taskSwitchPub.publish(taskPubMsg)
        rospy.sleep(0.05)

    def pubAMCLPoseAMCLInitial(self, x, y, quat, time_pub_initial):
        initialMsg = PoseWithCovarianceStamped()
        initialMsg.header.frame_id = "map"
        initialMsg.pose.pose.position.x = x
        initialMsg.pose.pose.position.y = y
        initialMsg.pose.pose.orientation = quat
        initialMsg.pose.covariance[0] = 0.25
        initialMsg.pose.covariance[7] = 0.25
        initialMsg.pose.covariance[-1] = 0.0685

        start_idx = 0
        while start_idx < time_pub_initial / 0.2:
            self.initialPosePub.publish(initialMsg)
            rospy.sleep(0.2)
            start_idx = start_idx + 1

    def mappingDetect(self):
        master = rosgraph.Master("")
        try:
            master.lookupNode(self._namespace + "cartographer")
            carto_diag = diag_array_gen(
                "mapping/cartographer", 0, "cartographer")
            self.diagnosticsPub.publish(carto_diag)
        except:
            pass
        try:
            master.lookupNode(self._namespace + "slam_toolbox_offline")
            karto_diag = diag_array_gen("mapping/karto", 0, "karto")
            self.diagnosticsPub.publish(karto_diag)
        except:
            pass


class pubStates(object):
    def __init__(self):
        self._namespace = rospy.get_namespace()
        self.pub = rospy.Publisher('task_state',
                                   DiagnosticStatus,
                                   queue_size=10)
        self.stateMsg = DiagnosticStatus()
        self.stateMsg.name = "Tasks State Monitor"
        self.stateMsg.message = "value 0/1: running/stopping"
        self.stateMsg.values = list()

        self.addState("cartographer")
        self.addState("slam_toolbox_offline")

    def addState(self, stateName):
        stateNew = KeyValue()
        self.stateMsg.values.append(stateNew)
        self.stateMsg.values[-1].key = stateName
        self.stateMsg.values[-1].value = "0"

    def publishing(self):
        for i in range(len(self.stateMsg.values)):
            try:
                master = rosgraph.Master("")
                master.lookupNode(self._namespace +
                                  self.stateMsg.values[i].key)
                self.stateMsg.values[i].value = "1"
            except:
                self.stateMsg.values[i].value = "0"

        self.pub.publish(self.stateMsg)


def dynamicParameterSet(client_name, parameter, value):
    client_recfg = dynamic_reconfigure.client.Client(client_name)
    params = {parameter: value}
    try:
        client_recfg.update_configuration(params)
        rospy.logdebug('[RS] set %s%s/%s to be %s ' %
                       (rospy.get_namespace(), client_name, parameter, str(value)))
    except:
        pass


def dynamicParameterSetCmd(client_name, parameter, value):
    client_name_ns = rospy.get_namespace() + client_name
    dynamic_param_set_cmd = 'rosrun dynamic_reconfigure dynparam set ' + \
        client_name_ns + ' '+parameter + ' ' + value
    subprocess.call(dynamic_param_set_cmd, shell=True)


def getTFPose(parent_frame, child_frame):
    # todo: Clearing TF buffer.
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.0)
    if parent_frame != 'map':
        parent_frame_ns = parent_frame
    else:
        parent_frame_ns = parent_frame

    if child_frame != 'map':
        child_frame_ns = child_frame
    else:
        child_frame_ns = child_frame
    try:
        tform = buf.lookup_transform(parent_frame_ns,
                                     child_frame_ns,
                                     rospy.Time(0),
                                     timeout=rospy.Duration(4))
        pos_trans = tform.transform.translation
        pose_quat = tform.transform.rotation
    except:
        rospy.logerr('TF Exception')
    return (pos_trans, pose_quat)


def indent(elem, level=0):
    i = "\n" + level * "  "
    if elem:
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level + 1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i


def add_dynamic_obstacles_to_launch_file(file_org, file_gen, initial_x,
                                         initial_y, initial_yaw):
    tree = ET.parse(file_org)
    root = tree.getroot()

    ele_node = ET.SubElement(root, 'node')
    ele_node.attrib['pkg'] = 'robot_localization'
    ele_node.attrib['type'] = 'ekf_localization_node'
    ele_node.attrib['name'] = 'ekf_node_global'
    ele_node.attrib['clear_params'] = 'true'
    ele_cfg = ET.SubElement(ele_node, 'rosparam')
    ele_cfg.attrib['command'] = 'load'
    ele_cfg.attrib['file'] = '$(find bringup)/scripts/pose_ekf_global.yaml'

    ele_initial = ET.SubElement(ele_node, 'rosparam')
    ele_initial.text = 'initial_state: ' + '[' + str(initial_x) + ', ' + str(
        initial_y) + ', 0.0, 0.0, 0.0, ' + str(
            initial_yaw) + ', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'

    ele_remap = ET.SubElement(ele_node, 'remap')
    ele_remap.attrib['from'] = 'odometry/filtered'
    ele_remap.attrib['to'] = 'global_pose'

    indent(root)
    tree.write(file_gen)


def relative_to_absolute_path(relative_path):
    if relative_path[0] == '~':
        absolute_path = '/home/' + getpass.getuser() + relative_path[1:]
    else:
        absolute_path = relative_path
    return absolute_path


def getBranchesList(branches):
    newstr = ''.join(
        (ch if ch in
         '0123456789_-qwertzuiopasdfghjklyxcvbnmQWERTZUIOPASDFGHJKLYXCVBNM'
         else ' ') for ch in branches)
    listOfNumbers = [str(i) for i in newstr.split()]
    return listOfNumbers


def launch_gen(path):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    return roslaunch.parent.ROSLaunchParent(uuid, [path])


def diag_array_gen(d_name, d_level, d_msg):
    da_msg = DiagnosticArray()
    da_msg.header.stamp = rospy.Time.now()
    status_msg = DiagnosticStatus()
    status_msg.name = d_name
    status_msg.level = d_level
    status_msg.message = d_msg
    da_msg.status.append(status_msg)
    return da_msg


def now_time():
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))


def main():
    systemSwitcher = systemTask()
    statePublisher = pubStates()

    map_folder_path = relative_to_absolute_path(
        rospy.get_param("~map_folder_path", "~/catkin_ws/dbparam"))
    agv_repo_path = relative_to_absolute_path(
        rospy.get_param("~agv_repo_path", "~/catkin_ws"))
    rospy.loginfo('[RS] AGV Update Path: %s' % agv_repo_path)
    agv_repo = git.Repo(agv_repo_path)

    dbparam_repo_path = relative_to_absolute_path(
        rospy.get_param("~dbparam_repo_path", "~/catkin_ws/dbparam"))
    rospy.loginfo('[RS] dbparam Path: %s' % dbparam_repo_path)
    dbparam_repo = git.Repo(dbparam_repo_path)
    dbparam_git = git.Git(dbparam_repo_path)

    dbparam_launch_path = rospy.get_param("~dbparam_launch_path",
                                          "/tmp/.sor/quiz.cfg/dbparam.cfg")
    launch_dbparam = launch_gen(dbparam_launch_path)

    slam_reset_launch_path = rospy.get_param("~slam_reset_launch_path",
                                             "/tmp/.sor/quiz.cfg/slam.cfg")
    slam_continue_launch_path = rospy.get_param(
        "~slam_continue_launch_path", "/tmp/.sor/quiz.cfg/slam_load.cfg")

    detect_path_dict = {}
    detect_path_dict['shelf'] = rospy.get_param(
        "~shelf_launch_path", "/tmp/.sor/quiz.cfg/shelf_detector.cfg")
    detect_path_dict['one'] = rospy.get_param("~one_launch_path",
                                              "/tmp/.sor/quiz.cfg/one_detector.cfg")
    detect_path_dict['triangle'] = rospy.get_param(
        "~triangle_launch_path", "/tmp/.sor/quiz.cfg/triangle_detector.cfg")
    detect_path_dict['charging_pile'] = rospy.get_param(
        "~charging_pile_launch_path", "/tmp/.sor/quiz.cfg/charge.cfg")
    detect_path_dict['apriltag'] = rospy.get_param("~tag_launch_path",
                                                   "/tmp/.sor/quiz.cfg/tag_detector.cfg")

    bgm_enabled = rospy.get_param("~bgm_enabled", True)
    bgm_mode = rospy.get_param("~bgm_mode", 'loop')
    bgm_id = rospy.get_param("~bgm_id", 10)

    full_electric_quantity = rospy.get_param("~full_battery", 1.1)

    charge_reset = True
    if USE_FLEXBE:
        behavior_publisher = rospy.Publisher('flexbe/execute_behavior/goal',
                                             BehaviorExecutionActionGoal,
                                             queue_size=5)
    else:
        task_publisher = rospy.Publisher('nav_ctrl',
                                         NavigationControl,
                                         queue_size=5)

    rospy.loginfo('[RS] rosbridge_system is on')

    if bgm_enabled:
        rospy.sleep(0.1)
        systemSwitcher.start_alert(14, 'single')
        rospy.loginfo('[RS] system manager alert is on')
        rospy.sleep(2.5)
        systemSwitcher.end_alert(False)
    else:
        rospy.sleep(1)

    if USE_FLEXBE:
        initial_task = BehaviorExecutionActionGoal()
        initial_task.header.stamp = rospy.Time.now()
        initial_task.goal.behavior_name = rospy.get_param(
            "~initial_task", "initial_task")
        if initial_task.goal.behavior_name == "initial_task":
            initial_task.goal.input_keys.append("open_web")
            initial_task.goal.input_values.append("True")
        behavior_publisher.publish(initial_task)

    while not rospy.is_shutdown():
        if systemSwitcher.initialization_completition:
            rospy.loginfo('[RS] Initialization is completed')
            systemSwitcher.initialization_completition = False
            try:
                systemSwitcher.start_alert(13, 'single')
                rospy.sleep(3)
                systemSwitcher.end_alert(False)
            except:
                rospy.logwarn(
                    '[RS] Failed to alert on the ending of initialization')

        if False and systemSwitcher._initialization_task != ''  \
                and time.time() - RS_START < 15 and systemSwitcher._behavior_state == 'free':
            try:
                rospy.loginfo('[RS] checking initialization')
                rospy.sleep(2)
                dxy = abs(systemSwitcher.amcl_x - systemSwitcher._home_pose.position.x) \
                    + abs(systemSwitcher.amcl_y -
                          systemSwitcher._home_pose.position.y)
                dyaw = abs(
                    systemSwitcher.amcl_yaw -
                    tf.transformations.euler_from_quaternion([
                        systemSwitcher._home_pose.orientation.x, systemSwitcher
                        ._home_pose.orientation.y, systemSwitcher._home_pose.
                        orientation.z, systemSwitcher._home_pose.orientation.w
                    ])[2])
                if dxy > 0.1 or dyaw > 0.1:
                    if not USE_FLEXBE:
                        initialization_msg = NavigationControl()
                        initialization_msg.control = 1
                        initialization_msg.goal_name = systemSwitcher._initialization_task
                        task_publisher.publish(initialization_msg)
                    rospy.loginfo('[RS] resend initialization task: %s' %
                                  initialization_msg.goal_name)
                    systemSwitcher._initialization_task = ''
                else:
                    systemSwitcher._initialization_task = ''
                    rospy.loginfo(
                        '[RS] Initialpose matching. Initialization already complete'
                    )
            except:
                systemSwitcher._initialization_task = ''
                rospy.logwarn("[RS] initialization check exception")

        if systemSwitcher._phb_pass['todo'] == "todo":
            phb_dist = 100
            area_diagnostics_msg = diag_array_gen("estop/area", 0, "todo")
            for phb_info in systemSwitcher._phb_pass_list:
                phb = phb_info[1]['area']
                if_exit_side = systemSwitcher.if_on_exit_side(phb_info[1])
                if systemSwitcher._phb_areas[phb]['if_amcl_in'] or (systemSwitcher._phb_areas[phb]['robot_dist'] < min(
                        systemSwitcher.area_apply_dist, phb_dist) and (not if_exit_side)):
                    systemSwitcher._phb_pass = copy.deepcopy(phb_info[1])
                    systemSwitcher._phb_pass['todo'] = "applying"
                    phb_dist = systemSwitcher._phb_areas[phb]['robot_dist']
                    area_diagnostics_msg = diag_array_gen(
                        "estop/area", 0, "todo-applying %s" % phb)
            if systemSwitcher._phb_pass['todo'] != "todo":
                systemSwitcher.publish_phb_info()
            systemSwitcher.diagnosticsPub.publish(area_diagnostics_msg)

        if systemSwitcher._phb_pass['todo'] == "waiting":
            phb_temp = systemSwitcher._phb_pass['area']
            for phb_info in systemSwitcher._phb_pass_list:
                phb = phb_info[1]['area']
                if_exit_side = systemSwitcher.if_on_exit_side(phb_info[1])
                if systemSwitcher._phb_areas[phb]['if_amcl_in'] or (systemSwitcher._phb_areas[phb]['robot_dist'] < min(
                        systemSwitcher.area_apply_dist, systemSwitcher._phb_areas[phb_temp]['robot_dist']) and (not if_exit_side)):
                    phb_temp = phb
                    break
            if phb_temp != systemSwitcher._phb_pass['area']:
                systemSwitcher._phb_pass['todo'] = "todo"
                log_info = "waiting %s - todo %s" % (
                    systemSwitcher._phb_pass['area'], phb_temp)
                area_diagnostics_msg = diag_array_gen(
                    "estop/area", 0, log_info)
                rospy.logwarn(log_info)
                systemSwitcher.publish_phb_info()
            elif systemSwitcher._phb_areas[phb_temp]['if_amcl_in'] or \
                    (systemSwitcher._phb_areas[phb_temp]['robot_dist'] < systemSwitcher.area_apply_dist and (not systemSwitcher.if_on_exit_side(systemSwitcher._phb_pass))):
                if systemSwitcher._phb_areas[phb_temp]['robot_dist'] < systemSwitcher.area_stop_dist:
                    area_diagnostics_msg = diag_array_gen("estop/area", 1,
                                                          "estop for waiting %s" % phb_temp)
                    rospy.logwarn_throttle(5, '%s estopping: wait to apply %s' % (
                        rospy.get_namespace(), phb_temp))
                else:
                    area_diagnostics_msg = diag_array_gen(
                        "estop/area", 0, "waiting - applying %s" % phb_temp)
                if time.time() - systemSwitcher._phb_pass['applied_time'] > (systemSwitcher.phb_applied_timeout + systemSwitcher.phb_wait_timeout):
                    systemSwitcher._phb_pass['todo'] = "applying"
                    systemSwitcher._phb_pass['applied_time'] = 0
                    systemSwitcher._phb_pass['timeout'] = False
                    systemSwitcher._phb_pass['state'] = 'reapplying after waiting'
                    rospy.loginfo('wait enough, re applying %s' %
                                  systemSwitcher._phb_pass['area'])
                    systemSwitcher.publish_phb_info()
            else:
                systemSwitcher._phb_pass['todo'] = "todo"
                systemSwitcher.publish_phb_info()
                log_info = "%s bacam invalid while waiting." % systemSwitcher._phb_pass['area']
                area_diagnostics_msg = diag_array_gen(
                    "estop/area", 0, log_info)
                rospy.logwarn(log_info)
            systemSwitcher.diagnosticsPub.publish(area_diagnostics_msg)

        elif systemSwitcher._phb_pass['todo'] == "applying":
            if systemSwitcher._phb_areas[systemSwitcher._phb_pass['area']]['robot_dist'] < systemSwitcher.area_stop_dist:
                area_diagnostics_msg = diag_array_gen("estop/area", 1,
                                                      "estop for applying %s" % systemSwitcher._phb_pass['area'])
                rospy.logwarn_throttle(5, '%s estopping: wait to enter %s' % (
                    rospy.get_namespace(), systemSwitcher._phb_pass['area']))
            else:
                area_diagnostics_msg = diag_array_gen("estop/area", 0,
                                                      "applying: %s" % systemSwitcher._phb_pass['area'])
            systemSwitcher.diagnosticsPub.publish(area_diagnostics_msg)

            if systemSwitcher._phb_areas[systemSwitcher._phb_pass['area']]['robot_dist'] < systemSwitcher.area_apply_dist:
                msg = String()
                msg.data = json.dumps(systemSwitcher._phb_pass)
                systemSwitcher.serverRequestPub.publish(msg)
                rospy.loginfo_throttle(
                    10, ('applying to server for entering %s' % systemSwitcher._phb_pass['area']))
        elif systemSwitcher._phb_pass['todo'] == "applied":
            area_diagnostics_msg = diag_array_gen("estop/area",    0,
                                                  "applied %s" % systemSwitcher._phb_pass['area'])
            systemSwitcher.diagnosticsPub.publish(area_diagnostics_msg)
            if systemSwitcher._phb_pass['alert'] != 'applied':
                if systemSwitcher._phb_pass['wise'] == 666:
                    systemSwitcher.start_alert(40, 'single')
                else:
                    systemSwitcher.start_alert(38, 'single')
                systemSwitcher._phb_pass['alert'] = 'applied'
                systemSwitcher.publish_phb_info()
                systemSwitcher.updatePassFile()
                rospy.sleep(2.5)

            if (not systemSwitcher._phb_areas[systemSwitcher._phb_pass['area']]['if_amcl_in']):
                if 'applied_time' not in systemSwitcher._phb_pass or \
                        systemSwitcher._phb_pass['applied_time'] == 0:
                    systemSwitcher._phb_pass['applied_time'] = round(
                        time.time(), 2)
                    systemSwitcher.publish_phb_info()
                elif systemSwitcher._phb_pass['applied_time'] > 0 and \
                        time.time() - systemSwitcher._phb_pass['applied_time'] > systemSwitcher.phb_applied_timeout:
                    rospy.logwarn("applied timeout: %s" %
                                  systemSwitcher._phb_pass['area'])
                    systemSwitcher._phb_pass['todo'] = "leaving"
                    systemSwitcher._phb_pass['state'] = "applied timeout"
                    systemSwitcher._phb_pass['timeout'] = True
                    systemSwitcher.publish_phb_info()
            else:
                systemSwitcher._phb_pass['applied_time'] = -1
                systemSwitcher._phb_pass['timeout'] = False

        elif systemSwitcher._phb_pass['todo'] == "leaving":
            msg = String()
            msg.data = json.dumps(systemSwitcher._phb_pass)
            systemSwitcher.serverRequestPub.publish(msg)
            rospy.loginfo_throttle(
                10, ('applying to server for leaving %s' % systemSwitcher._phb_pass['area']))
            leaving_stop = False
            for phb_info in systemSwitcher._phb_pass_list:
                phb = phb_info[1]['area']
                if_exit_side = systemSwitcher.if_on_exit_side(phb_info[1])
                if systemSwitcher._phb_areas[phb]['if_amcl_in'] or (systemSwitcher._phb_areas[phb]['robot_dist'] < systemSwitcher.area_stop_dist and (not if_exit_side)):
                    leaving_stop = True
                    break
            if systemSwitcher._phb_pass['timeout'] or leaving_stop:
                if systemSwitcher._phb_pass['timeout']:
                    log_info = '%s estopping: leaving %s after timeout' % (
                        rospy.get_namespace(), systemSwitcher._phb_pass['area'])
                    systemSwitcher._phb_pass['state'] = 'timeout leaving'
                else:
                    log_info = '%s estopping: leaving %s before entering %s' % (
                        rospy.get_namespace(), systemSwitcher._phb_pass['area'], phb)
                    systemSwitcher._phb_pass['state'] = 'waiting leaving'
                area_diagnostics_msg = diag_array_gen(
                    "estop/area", 1,  log_info)
                systemSwitcher.publish_phb_info()
                rospy.logwarn_throttle(5, log_info)
            else:
                area_diagnostics_msg = diag_array_gen(
                    "estop/area", 0,  "leaving %s" % systemSwitcher._phb_pass['area'])
            systemSwitcher.diagnosticsPub.publish(area_diagnostics_msg)

        elif systemSwitcher._phb_pass['todo'] == "leaved":
            area_diagnostics_msg = diag_array_gen("estop/area",    0,
                                                  "out %s" % systemSwitcher._phb_pass['area'])
            if systemSwitcher._phb_pass['alert'] == 'applied':
                if systemSwitcher._phb_pass['wise'] == 666:
                    systemSwitcher.start_alert(41, 'single')
                else:
                    systemSwitcher.start_alert(39, 'single')
                systemSwitcher._phb_pass['alert'] = 'leaved'
                systemSwitcher._phb_pass['state'] = 'finished'
                rospy.loginfo('%s finished' %
                              systemSwitcher._phb_pass['area'])
                systemSwitcher.publish_phb_info()
                systemSwitcher.updatePassFile(True)

            if systemSwitcher._phb_pass['timeout']:
                systemSwitcher._phb_pass['state'] = 'timeout leaved'
                systemSwitcher._phb_pass['todo'] = "waiting"
                systemSwitcher.publish_phb_info()
            else:
                systemSwitcher._phb_pass['todo'] = 'todo'
                systemSwitcher.publish_phb_info()
            systemSwitcher.diagnosticsPub.publish(area_diagnostics_msg)
        # launch slam
        if systemSwitcher.startSLAM and (systemSwitcher.excutedSLAM is False):
            trajectory_id = 1
            if systemSwitcher.slamReset:
                slam_launch_path = slam_reset_launch_path
                rospy.loginfo('[RS]: SLAM reset mode')
            elif systemSwitcher.slamContinue:
                slam_launch_path = slam_continue_launch_path
                rospy.loginfo('[RS]: SLAM continue mode')
            launch = launch_gen(slam_launch_path)
            try:
                launch.start()
                rospy.sleep(3)  # enough time for launch to complete
                rospy.loginfo('[RS] SLAM is launched')
                systemSwitcher.start_alert(20, 'single')
                rospy.sleep(2.5)
                systemSwitcher.end_alert(False)
                systemSwitcher.excutedSLAM = True
            except Exception as e:
                rospy.logwarn(
                    '[RS] Failed to launch SLAM: %s. Relaunching...' % str(e))
                systemSwitcher.excutedSLAM = False
            if systemSwitcher.excutedSLAM:
                try:
                    systemSwitcher.pubTaskSwitch('base_laser', 0)
                    rospy.loginfo(
                        '[RS] setting task base_laser False within 2s')

                    # deactivate /amcl node for publish tf between /map and /odom
                    dynamicParameterSetCmd('amcl', 'tf_broadcast', 'False')
                    rospy.loginfo('[RS]: setting AMCL tf False')

                    # double localization. To to: restore tf after SLAM
                    ekf_node_global_ns = rospy.get_namespace(
                    ) + 'ekf_node_global'
                    kill_ekd_global_cmd = 'rosnode kill ' + ekf_node_global_ns
                    subprocess.call(kill_ekd_global_cmd, shell=True)
                except Exception as e:
                    rospy.logwarn('[RS] TF tree exception by SLAM: %s' %
                                  str(e))

        elif systemSwitcher.startSLAM and systemSwitcher.excutedSLAM and systemSwitcher.slamPause:
            finish_trajectory_cmd = "rosservice call " + rospy.get_namespace() + "finish_trajectory " + \
                str(trajectory_id)
            os.system(finish_trajectory_cmd)
            systemSwitcher.slamPause = False

        elif systemSwitcher.startSLAM and systemSwitcher.excutedSLAM and systemSwitcher.slamGoOn:
            start_trajectory_cmd = "rosrun cartographer_ros cartographer_start_trajectory  -configuration_directory /home/" + getpass.getuser() + "/catkin_ws/devel/share/cartographer/configuration_files -configuration_basename hitrobot.lua -initial_pose '{to_trajectory_id = 1, relative_pose = { translation = { " \
                + str(systemSwitcher.amcl_x) \
                + ", " \
                + str(systemSwitcher.amcl_y) \
                + ", 0.0 }, rotation = { 0.0, 0.0,"\
                + str(systemSwitcher.amcl_yaw)\
                + "} } }'"
            os.system(start_trajectory_cmd)
            trajectory_id += 1
            systemSwitcher.slamGoOn = False

        if systemSwitcher.startSLAM and (
                systemSwitcher.excutedSLAM) and systemSwitcher.saveMap:
            try:
                if systemSwitcher._switch_str == '':
                    map_name = 'map'
                else:
                    map_name = systemSwitcher._switch_str
                    systemSwitcher._switch_str = ''
                # todo: ROS API
                save_map_service = "rosservice call " + rospy.get_namespace() + "write_state '{filename: '/home/" + getpass.getuser() + "/catkin_ws/dbparam/" + \
                    map_name + ".pbstream'}'"
                os.system(save_map_service)
                rospy.loginfo('[RS] calling save map service')
                rospy.sleep(2.0)

                save_map_file = "rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=/home/" + getpass.getuser() + "/catkin_ws/dbparam/" + \
                    map_name + " -pbstream_filename=/home/" + getpass.getuser() + "/catkin_ws/dbparam/" + \
                    map_name + ".pbstream -resolution=0.0"
                if ROS_MELODIC:
                    save_map_file += '5'
                else:
                    save_map_file += '5'
                os.system(save_map_file)
                rospy.loginfo(
                    '[RS]: saving map files: %s.pgm, %s.pbstream, %s.yaml ' %
                    (map_name, map_name, map_name))
                rospy.sleep(3.0)

                convert_map_cmd = "convert /home/" + getpass.getuser() + "/catkin_ws/dbparam/" + \
                    map_name + ".pgm /home/" + getpass.getuser() + "/catkin_ws/dbparam/"+map_name+".png"
                os.system(convert_map_cmd)
                rospy.loginfo('[RS]: converting map files to %s.png' %
                              map_name)
                rospy.sleep(0.5)

                map_yaml_path = map_folder_path + "/" + map_name + ".yaml"
                with open(map_yaml_path) as f:
                    list_doc = yaml.safe_load(f)
                    list_doc['image'] = map_name + ".png"
                    list_doc['origin'] = [0.0, 0.0, 0.0]
                    map_origin = list_doc['origin']
                    map_rsl = list_doc['resolution']
                with open(map_yaml_path, "w") as f:
                    yaml.dump(list_doc, f)
                    rospy.loginfo("[RS] map yaml edited")
                systemSwitcher.saveMap = False
                systemSwitcher.start_alert(21, 'single')
                rospy.sleep(2.5)
                systemSwitcher.end_alert(False)

                map_path = map_folder_path + "/" + map_name + ".png"
                map_display_path = map_folder_path + "/" + map_name + "_display.png"
                map_display_yaml_path = map_folder_path + "/" + map_name + "_display.yaml"
                copyfile(map_path, map_display_path)
                copyfile(map_yaml_path, map_display_yaml_path)
                rospy.loginfo("[RS] map display generated")

                with open(map_display_yaml_path) as f:
                    list_doc = yaml.safe_load(f)
                    list_doc['image'] = map_name + "_display.png"
                with open(map_display_yaml_path, "w") as f:
                    yaml.dump(list_doc, f)
                    rospy.loginfo("[RS] map display yaml edited")

                map_path_path = map_folder_path + "/" + map_name + "_path.png"
                map_path_yaml_path = map_folder_path + "/" + map_name + "_path.yaml"
                copyfile(map_path, map_path_path)
                copyfile(map_yaml_path, map_path_yaml_path)
                rospy.loginfo("[RS] map path png generated")

                map_reflection_path = map_folder_path + "/" + map_name + "_reflection.png"
                map_reflection_yaml_path = map_folder_path + "/" + map_name + "_reflection.yaml"
                copyfile(map_path, map_reflection_path)
                copyfile(map_yaml_path, map_reflection_yaml_path)
                rospy.loginfo("[RS] map reflection png generated")
                with open(map_reflection_yaml_path) as f:
                    list_doc = yaml.safe_load(f)
                    list_doc['image'] = map_name + "_reflection.png"
                with open(map_reflection_yaml_path, "w") as f:
                    yaml.dump(list_doc, f)
                    rospy.loginfo("[RS] map reflection yaml edited")
            except Exception as e:
                rospy.logerr('[RS]: Exception to save map: %s' % str(e))

        # shutdown slam: (1)record slam pose                  (2)shutdown
        #                (3)recover tf of amcl and base_laser (4) publish slam pose to /initialpose
        if (systemSwitcher.startSLAM is False) and systemSwitcher.excutedSLAM:
            try:
                slam_laser_trans, slam_laser_quat = getTFPose(
                    'map', 'base_laser')
                slam_laser_x = slam_laser_trans.x
                slam_laser_y = slam_laser_trans.y
                rospy.loginfo('[RS] getting SLAM pose')
                rospy.sleep(2)
                # Todo: continue with AMCL or double localization
                rospy.loginfo('[RS] setting AMCL tf True')
                systemSwitcher.pubTaskSwitch('base_laser', 1)
                rospy.loginfo('[RS] setting task base_laser True within 2s')
            except Exception as e:
                rospy.logdebug('[RS-Ecp] get laser pose: %s' % str(e))
            try:
                launch.shutdown()
                rospy.sleep(3)
                rospy.loginfo('[RS] SLAM is shutdown')
                systemSwitcher.excutedSLAM = False
                systemSwitcher.slamReset = False
                systemSwitcher.slamContinue = False
                trajectory_id = 0

            except Exception as e:
                systemSwitcher.excutedSLAM = True
                rospy.logwarn('[RS] exception: %s' % str(e))

            rospy.loginfo('[RS] pulish AMCL pose to /initialpose within 2s')
            systemSwitcher.pubAMCLPoseAMCLInitial(slam_laser_x, slam_laser_y,
                                                  slam_laser_quat, 2)
            rospy.loginfo('[RS] finish SLAM task')

        elif systemSwitcher.startDetector and (
                systemSwitcher.executedDetector is False):
            launch_detect = launch_gen(
                detect_path_dict[systemSwitcher.detectorTarget])
            try:
                launch_detect.start()
                rospy.sleep(2)
                rospy.loginfo("[RS] %s detection is launched" %
                              systemSwitcher.detectorTarget)
                systemSwitcher.executedDetector = True
            except Exception as e:
                rospy.logwarn(
                    "[RS]: Failed to launch Charging Pile: %s. Retrying..." %
                    str(e))
                systemSwitcher.executedDetector = False
            systemSwitcher._detecting = systemSwitcher.executedDetector
        elif systemSwitcher.startDetector is False and systemSwitcher.executedDetector:
            try:
                launch_detect.shutdown()
                rospy.sleep(2)
                rospy.loginfo('[RS] %s detection is shutdown' %
                              systemSwitcher.detectorTarget)
                systemSwitcher.executedDetector = False
            except Exception as e:
                rospy.logwarn(
                    "[RS]: Failed to shutdown : %s. Retrying..." %
                    str(e))
                systemSwitcher.executedDetector = True
            systemSwitcher._detecting = systemSwitcher.executedDetector

        if systemSwitcher._switch_str[:7] == 'launch_':
            print 'launch ..', relative_to_absolute_path(
                systemSwitcher._switch_str[7:])
            l_file = launch_gen(
                relative_to_absolute_path(systemSwitcher._switch_str[7:]))
            rospy.loginfo('[RS] %s launching' % systemSwitcher._switch_str[7:])
            l_file.start()
            rospy.sleep(1)
            systemSwitcher._switch_str = ''
        elif systemSwitcher._switch_str[:6] == 'shell_':
            rospy.loginfo('[RS] %s' % systemSwitcher._switch_str[6:])
            subprocess.Popen(systemSwitcher._switch_str[6:], shell=True)
            systemSwitcher._switch_str = ''

        if systemSwitcher.updateRequestFlag:
            update_result = 1
            update_detail = 'update_response'
            print ' ======= GitPython ======'
            try:
                rmtree('/home/' + getpass.getuser() + '/workspaces_tree')
            except Exception as e:
                rospy.logwarn('[RS] exception: %s' % str(e))
            try:
                print 'backing up workspaces to workspaces_tree'
                copytree('/home/' + getpass.getuser() + '/workspaces',
                         '/home/' + getpass.getuser() + '/workspaces_tree')
            except Exception as e:
                print 'failed to backup workspaces'
                rospy.logwarn('[RS] exception: %s' % str(e))

            try:
                agv_repo.git.reset('--hard')
                print '\n' + 'Git Resetting'
                # rospy.sleep(2)
            except Exception as e:
                update_result = 0
                print 'Failed to reset'
                rospy.logwarn('[RS] exception: %s' % str(e))
                excepName = type(e).__name__
                update_detail = update_detail + ':' + excepName

            try:
                agv_repo.remotes.origin.pull()
                print '\n' + 'Git Pulling'
                # rospy.sleep(3)
            except Exception as e:
                update_result = 0
                print 'Failed to pull'
                rospy.logwarn('[RS] exception: %s' % str(e))
                excepName = type(e).__name__
                update_detail = update_detail + ':' + excepName

            head_commit = agv_repo.head.commit
            print '\n' + 'Head Commit:' + '\n', head_commit
            status = agv_repo.git.status()
            print '\n' + 'Status in Detail:' + '\n' + status

            print ' ======= GitPython ======'
            systemSwitcher.pubTaskSwitch(update_detail, update_result)
            rospy.loginfo('[RS] Send update feedback')

            systemSwitcher.updateRequestFlag = False

        elif systemSwitcher.dbparam_git_task:
            print ' ======= GitPython ======'
            dbparam_diagnostic_msg = DiagnosticArray()
            dbparam_diagnostic_msg.header.stamp = rospy.Time.now()
            status_msg = DiagnosticStatus()
            status_msg.name = "git:dbparam"
            status_msg.level = 0
            if systemSwitcher.task_switch_json_object["topic"] == "query":
                status_msg.hardware_id = "query"
                try:
                    a = dbparam_git.branch()
                    branches_list = getBranchesList(a)
                    rospy.loginfo('[RS] dbparam active branch: %s' %
                                  dbparam_repo.active_branch)
                    status_msg.message = str(dbparam_repo.active_branch)
                    for i in range(len(branches_list)):
                        status_msg.message = status_msg.message + \
                            ':' + branches_list[i]
                    rospy.loginfo('[RS] dbparam-query succeed')

                except Exception as e:
                    rospy.logwarn('[RS] exception: %s' % str(e))
                    status_msg.message = type(e).__name__
                    status_msg.level = 1
                    rospy.logwarn('[RS] Failed to dbparam-query')
                dbparam_diagnostic_msg.status.append(status_msg)
                systemSwitcher.diagnosticsPub.publish(dbparam_diagnostic_msg)

            elif systemSwitcher.task_switch_json_object["topic"] == "select":
                status_msg.hardware_id = "select"
                try:
                    diff_list = [
                        diff.a_blob.path
                        for diff in dbparam_repo.index.diff(None)
                    ]
                    if len(diff_list) > 0:
                        dbparam_repo.index.add(diff_list)
                        dbparam_repo.index.commit('commit by GitPython')
                        rospy.loginfo(
                            '[RS] commit changes before checkout other branch')
                    dbparam_git.checkout(
                        systemSwitcher.task_switch_json_object["name"])
                    rospy.loginfo(
                        '[RS] dbparam checkout -> %s' %
                        systemSwitcher.task_switch_json_object["name"])
                    rospy.sleep(0.2)
                    status_msg.message = str(dbparam_repo.active_branch)
                    if str(dbparam_repo.active_branch
                           ) != systemSwitcher.task_switch_json_object["name"]:
                        status_msg.level = 1
                        rospy.logwarn(
                            '[RS] Current!= Require. Failed to dbparam-select.'
                        )
                    else:
                        launch_dbparam.start()
                        rospy.sleep(2)
                        rospy.loginfo('[RS] dbparam relaunched')

                except Exception as e:
                    rospy.logwarn('[RS] exception: %s' % str(e))
                    status_msg.message = type(e).__name__
                    status_msg.level = 1
                    rospy.logwarn('[RS] Exception by dbparam-select')
                dbparam_diagnostic_msg.status.append(status_msg)
                systemSwitcher.diagnosticsPub.publish(dbparam_diagnostic_msg)

            elif systemSwitcher.task_switch_json_object["topic"] == "add":
                status_msg.hardware_id = "add"
                try:
                    dbparam_repo.create_head(
                        systemSwitcher.task_switch_json_object["name"])
                    dbparam_git.checkout(
                        systemSwitcher.task_switch_json_object["name"])
                    rospy.loginfo(
                        '[RS] dbparam add new branch + %s and check it out' %
                        systemSwitcher.task_switch_json_object["name"])
                    rospy.sleep(0.2)
                    status_msg.message = str(dbparam_repo.active_branch)
                    if str(dbparam_repo.active_branch
                           ) != systemSwitcher.task_switch_json_object["name"]:
                        status_msg.level = 1
                        rospy.logwarn(
                            '[RS] Current!= Require. Failed to dbparam-add.')
                    else:
                        rospy.loginfo('[RS] dbparam-add succeed')
                except Exception as e:
                    rospy.logwarn('[RS] exception: %s' % str(e))
                    status_msg.message = type(e).__name__
                    status_msg.level = 1
                    rospy.logwarn('[RS] Failed to dbparam-add')
                dbparam_diagnostic_msg.status.append(status_msg)
                systemSwitcher.diagnosticsPub.publish(dbparam_diagnostic_msg)

            elif systemSwitcher.task_switch_json_object["topic"] == "remove":
                status_msg.hardware_id = "remove"
                try:
                    if systemSwitcher.task_switch_json_object["name"] != str(
                            dbparam_repo.active_branch):
                        dbparam_repo.delete_head(
                            systemSwitcher.task_switch_json_object["name"],
                            force=True)
                        rospy.loginfo(
                            '[RS] dbparam delete branch %s' %
                            systemSwitcher.task_switch_json_object["name"])
                    else:
                        rospy.logwarn(
                            '[RS] Cannot delete the branch %s which you are currently on.Check out master branch then delete...'
                            % systemSwitcher.task_switch_json_object["name"])
                        a = dbparam_git.branch()
                        branches_list = getBranchesList(a)
                        dbparam_git.checkout(branches_list[0])
                        dbparam_repo.delete_head(
                            systemSwitcher.task_switch_json_object["name"],
                            force=True)
                    status_msg.message = str(dbparam_repo.active_branch)
                    rospy.loginfo('[RS] dbparam-remove succeed')
                except Exception as e:
                    rospy.logwarn('[RS] exception: %s' % str(e))
                    status_msg.message = type(e).__name__
                    status_msg.level = 1
                    rospy.logwarn('[RS] Failed to dbparam-remove')
                dbparam_diagnostic_msg.status.append(status_msg)
                systemSwitcher.diagnosticsPub.publish(dbparam_diagnostic_msg)

            systemSwitcher.dbparam_git_task = False
            print ' ======= GitPython ======'

        # publish tasks state
        statePublisher.publishing()
        systemSwitcher.mappingDetect()
        if USE_FLEXBE:
            if systemSwitcher.battery_percentage < systemSwitcher._low_battery and \
                time.time()-RS_START > 60 and \
                    (systemSwitcher._flexbe_state.data == "behavior_exit" or systemSwitcher._flexbe_state.data == "behavior_preempted"):
                systemSwitcher._flexbe_state.data = "autocharge_on"
                charge_msg = BehaviorExecutionActionGoal()
                charge_msg.header.stamp = rospy.Time.now()
                charge_msg.goal.behavior_name = "autocharge_on"
                time.sleep(0.1)
                behavior_publisher.publish(charge_msg)
                systemSwitcher._flexbe_status['behavior'] = "autocharge_on"
                rospy.loginfo('low battery {:5.3f}: autocharge on'.format(
                    systemSwitcher.battery_percentage))
                full_electric_quantity = rospy.get_param("~full_battery", 1.1)
            elif systemSwitcher.battery_percentage > full_electric_quantity and systemSwitcher._flexbe_status['behavior'] == "autocharge_on":
                rospy.loginfo('full battery {:5.3f}: shutdown relay'.format(
                    systemSwitcher.battery_percentage))
                systemSwitcher.pubJoyFeedback(7, 0, 1)
                full_electric_quantity = 2

            systemSwitcher._flexbe_state_pub.publish(
                systemSwitcher._flexbe_state)

            state_level = 0
            if systemSwitcher._flexbe_state.data == "behavior_exit":
                behavior_level = 0
            elif systemSwitcher._flexbe_state.data == 'behavior_preempted':
                behavior_level = 2
            else:
                if systemSwitcher._flexbe_status['pause']:
                    state_level = 2
                else:
                    state_level = 1
                behavior_level = 1
            state_diagnostics_msg = diag_array_gen("task/state", state_level,
                                                   "%s - %s" % (systemSwitcher._flexbe_state.data, systemSwitcher._flexbe_status['behavior']))
            beh_diagnostics_msg = diag_array_gen(
                "task/behavior", behavior_level, "%s" % systemSwitcher._flexbe_status['behavior'])
            state_diagnostics_msg.status.append(beh_diagnostics_msg.status[0])
            systemSwitcher.diagnosticsPub.publish(state_diagnostics_msg)
        else:
            systemSwitcher.diagnosticsPub.publish(systemSwitcher.task_diag_msg)

        # to do: Inbound TCP/IP connection

        rospy.sleep(0.2)

    try:
        rospy.spin()
    except:
        print "Shutting down..."


if __name__ == '__main__':
    main()
