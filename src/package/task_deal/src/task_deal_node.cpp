
#include <../include/task_deal/task_deal.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "task_deal_node");
  ros::NodeHandle pHandle("~");
  ros::NodeHandle gHandle;
  task_deal::TaskDeal taskDeal(gHandle,pHandle);

  ros::spin();

  return 0;
}
