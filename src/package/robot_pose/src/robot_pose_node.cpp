
#include <../include/robot_pose/robot_pose.hpp>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_pose_node");
  ros::NodeHandle pHandle("~");
  ros::NodeHandle gHandle;
  robot_pose::RobotPose robotPose(gHandle,pHandle);
  ros::spin();
  return 0;
}
