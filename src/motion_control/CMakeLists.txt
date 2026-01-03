cmake_minimum_required(VERSION 3.8)
project(motion_control)

# 设置C++标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 定义编译宏
add_definitions(-DOPENMP)

# 性能优化编译选项（保留原有）
add_definitions(-O4 -g)

# 查找ROS 2依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

# ROS 2接口生成（替代ROS 1的message_generation）
find_package(rosidl_default_generators REQUIRED)

# 声明消息/动作文件（替代ROS 1的add_message_files/add_action_files）
set(msg_files
  "msg/RobotState.msg"
)

set(action_files
  "action/RobotNav.action"
)

# 生成接口（替代ROS 1的generate_messages）
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    actionlib_msgs
)

# 替代ROS 1的catkin_package
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  nav2_costmap_2d
  rosidl_default_runtime
  Boost
)

# 包含目录
include_directories(
  include
  config
  ${Boost_INCLUDE_DIRS}
  ${glog_INCLUDE_DIRS}
)


# 构建库（替代ROS 1的add_library）
add_library(${PROJECT_NAME}
  src/motion_control.cpp
)

# 链接依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  nav2_costmap_2d
  Boost
)

# 链接ROS接口（ROS 2特有）
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}
  ${cpp_typesupport_target}
  ${Boost_LIBRARIES}
  ${glog_LIBRARIES}
  m
  pthread
  stdc++
)

# 构建可执行文件
add_executable(motion_control_node
  src/motion_control_node.cpp
)

# 链接可执行文件依赖
target_link_libraries(motion_control_node
  ${PROJECT_NAME}
)

ament_target_dependencies(motion_control_node
  rclcpp
  rclcpp_action
)

# 安装规则
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS motion_control_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# 安装launch和config文件
install(DIRECTORY launch/ 
  DESTINATION share/${PROJECT_NAME}/launch/
)
install(DIRECTORY  config/
  DESTINATION share/${PROJECT_NAME}/config/
)

# ROS 2包配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()