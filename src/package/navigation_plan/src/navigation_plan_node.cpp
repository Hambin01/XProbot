
#include "../include/navigation_plan/navigation_plan.hpp"
#include <ros/ros.h>
#include <signal.h>

void ShutDown(int signum) {
    ros::shutdown();
    exit(-1);
}

int main(int argc, char** argv) {
  glog_helper::GlogHelper glogInit(argv[0]);

  ros::init(argc, argv, "navigation_plan_node");
  ros::NodeHandle pHandle("~");
  ros::NodeHandle gHandle;
  signal(SIGINT, ShutDown);
  navigation::Navigation navigationPlan(gHandle,pHandle);
  LOG(INFO)<<"navigation_plan_node start!";
  ros::AsyncSpinner spinner(2); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
