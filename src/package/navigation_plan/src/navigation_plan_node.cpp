
#include "../include/navigation_plan/navigation_plan.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "navigation_plan_node");
  ros::NodeHandle pHandle("~");
  ros::NodeHandle gHandle;
  navigation::Navigation navigationPlan(gHandle, pHandle);
  ros::AsyncSpinner spinner(2); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
