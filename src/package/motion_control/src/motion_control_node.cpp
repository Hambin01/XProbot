

#include "../include/motion_control/motion_control.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "motion_control_node");
  ros::NodeHandle pHandle("~");
  ros::NodeHandle gHandle;

  navigation::Navigation Navigation(gHandle, pHandle);
  ros::AsyncSpinner spinner(2); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
