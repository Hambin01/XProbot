/**
 * Copyright 2021 The Kali Authors
 * _         _ _                  
 * | | ____ _| (_) __  ___ __ ___  
 * | |/ / _` | | | \ \/ / '_ ` _ \ 
 * |   < (_| | | |  >  <| | | | | |
 * |_|\_\__,_|_|_| /_/\_\_| |_| |_| 
 *
 */

#ifndef ODOM_H_
#define ODOM_H_

// ros msg API
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
// C++ headers
#include <stdio.h>
#include <boost/thread/pthread/shared_mutex.hpp>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>


// msgs
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
// tf
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <robot_state_msgs/robot_state.h>

namespace odom {

class Odom {
 public:
  Odom(ros::NodeHandle& nh);
  ~Odom();

 protected:
  ros::NodeHandle nh_;
  ros::Publisher odom_publisher_, velocity_publisher_;
  ros::Subscriber robot_state_subscriber_, robot_velocity_subscriber_;

  ros::Time current_time1, last_time1;        //里程计累加
  double vth_scale, vx_scale,vy_scale;
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;                             //里程计数据
  int sendTransform =0;
  tf::TransformBroadcaster odom_broadcaster;  //里程计tf发布

  void StateHandle(const robot_state_msgs::robot_stateConstPtr &msg);
};
}  // namespace odom

#endif
