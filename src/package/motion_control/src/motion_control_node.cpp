

#include "../include/motion_control/motion_control.hpp"
#include <ros/ros.h>
#include <signal.h>
void ShutDown(int signum) {
    ros::shutdown();
    exit(-1);
}

int main(int argc, char** argv) {
  glog_helper::GlogHelper glogInit(argv[0]);

  ros::init(argc, argv, "motion_control_node");
  ros::NodeHandle pHandle("~");
  ros::NodeHandle gHandle;

  navigation::Navigation Navigation(gHandle,pHandle);
  LOG(INFO)<<"motion_control_node start!";
 // signal(SIGINT, ShutDown);
  ros::AsyncSpinner spinner(2); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
