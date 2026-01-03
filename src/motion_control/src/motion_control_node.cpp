#include "motion_control/motion_control.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  // 初始化ROS 2节点
  rclcpp::init(argc, argv);
  
  // 创建节点选项（支持参数命名空间）
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__ns:=/", "-r", "__node:=motion_control_node"});
  
  // 创建导航节点实例（ROS 2中Node本身包含命名空间和参数管理）
  auto navigation_node = std::make_shared<navigation::Navigation>(options);
  
  // 设置参数回调（确保动态参数生效）
  auto param_callback = navigation_node->add_on_set_parameters_callback(
    std::bind(&navigation::Navigation::CfgCallback, navigation_node.get(), std::placeholders::_1));
  
  // 创建多线程执行器（对应ROS 1的AsyncSpinner）
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(navigation_node);
  
  // 执行节点（阻塞直到关闭）
  executor.spin();
  
  // 关闭ROS 2
  rclcpp::shutdown();
  
  return 0;
}