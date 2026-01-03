cmake_minimum_required(VERSION 3.0.2)
project(pose_to_odom)

add_compile_options(-std=c++11 -std=c++17)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  dynamic_reconfigure
  eigen_conversions
)

# 生成动态重配置文件
generate_dynamic_reconfigure_options(
  config/PoseToOdom.cfg
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs sensor_msgs tf dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pose_to_odom_node src/pose_to_odom_node.cpp)
add_dependencies(pose_to_odom_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_to_odom_node
  ${catkin_LIBRARIES}
)