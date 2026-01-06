
#include <../include/laser_avoidance/laser_avoidance.hpp>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_avoidance_node");
  ros::NodeHandle pHandle("~");
  ros::NodeHandle gHandle;
  laser_avoidance::LaserAvoidance laserAvoidance(gHandle, pHandle);
  ros::spin();

  return 0;
}
