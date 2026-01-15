#!/bin/bash
cd ~/XProbot
mkdir install

catkin_make  install -j4  --only-pkg-with-deps robot_state_msgs
catkin_make  install -j4  --only-pkg-with-deps robot_task_msgs
catkin_make  install -j4  --only-pkg-with-deps cmd_vel_mux
catkin_make  install -j4  --only-pkg-with-deps uuid_msgs
catkin_make  install -j4  --only-pkg-with-deps geographic_msgs
catkin_make  install -j4  --only-pkg-with-deps robot_launch
catkin_make  install -j4  --only-pkg-with-deps dummy_robot
catkin_make  install -j4  --only-pkg-with-deps rosauth
catkin_make  install -j4  --only-pkg-with-deps robot_pose_publisher
catkin_make  install -j4  --only-pkg-with-deps pose_init
catkin_make  install -j4  --only-pkg-with-deps pointcloud_to_laserscan
catkin_make  install -j4  --only-pkg-with-deps link_visual
catkin_make  install -j4  --only-pkg-with-deps boxs_obstacle
catkin_make  install -j4  --only-pkg-with-deps obstacle_stop
catkin_make  install -j4  --only-pkg-with-deps point_cloud_merger
catkin_make  install -j4  --only-pkg-with-deps laser_avoidance
catkin_make  install -j4  --only-pkg-with-deps rosbridge_suite
catkin_make  install -j4  --only-pkg-with-deps motion_control
catkin_make  install -j4  --only-pkg-with-deps robot_control
catkin_make  install -j4  --only-pkg-with-deps navigation_plan
catkin_make  install -j4  --only-pkg-with-deps cartographer_ros
catkin_make  install -j4  --only-pkg-with-deps tcp_com
catkin_make  install -j4  --only-pkg-with-deps remote_control
catkin_make  install -j4  --only-pkg-with-deps scan_convert
catkin_make  install -j4  --only-pkg-with-deps amcl
catkin_make  install -j4  --only-pkg-with-deps amcl_updater
catkin_make  install -j4  --only-pkg-with-deps localization
catkin_make  install -j4  --only-pkg-with-deps rosbridge_system
catkin_make  install -j4  --only-pkg-with-deps robot_localization
catkin_make  install -j4  --only-pkg-with-deps ros_canopen
catkin_make  install -j4  --only-pkg-with-deps bluesea2
catkin_make  install -j4  --only-pkg-with-deps hins_he_driver
catkin_make  install -j4  --only-pkg-with-deps base_driver
catkin_make  install -j4  --only-pkg-with-deps relay_control
catkin_make  install -j4  --only-pkg-with-deps cross_control
catkin_make  install -j4  --only-pkg-with-deps serial


