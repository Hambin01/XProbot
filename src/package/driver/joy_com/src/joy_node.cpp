
#include <../include/joy_com/joy_f710.hpp>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "joy_f710_node");
  ros::NodeHandle pHandle("~");
  ros::NodeHandle gHandle;
  joy_com::Joyf710 Joyf710(gHandle,pHandle);
  ros::spin();
  return 0;
}
