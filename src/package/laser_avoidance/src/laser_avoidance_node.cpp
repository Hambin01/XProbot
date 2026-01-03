/*
 * @Author: your name
 * @Date: 2021-07-20 11:58:45
 * @LastEditTime: 2021-07-30 16:34:21
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /xm-autosystem/src/package/application/robot_pose/src/robot_pose_node.cpp
 */

#include <../include/laser_avoidance/laser_avoidance.hpp>
#include <ros/ros.h>

int main(int argc, char** argv) {
  // 创建glog 
  //glog_helper::GlogHelper glogInit(argv[0]);
  ros::init(argc, argv, "laser_avoidance_node");
  ros::NodeHandle pHandle("~");
  ros::NodeHandle gHandle;
  laser_avoidance::LaserAvoidance laserAvoidance(gHandle,pHandle);
  ros::spin();
 
  return 0;
}
