
#include <../include/serial_com/serial_stm32.hpp>
#include <ros/ros.h>
#include <../transform_util/glog_util.hpp>

int main(int argc, char** argv) {
  glog_helper::GlogHelper glogInit(argv[0]);
  ros::init(argc, argv, "serial_stm32_node");
  ros::NodeHandle nodeHandle;
  serial_com::SerialToStm32 SerialToStm32(nodeHandle,"/dev/ttyUSB0");
  ros::AsyncSpinner spinner(2); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
