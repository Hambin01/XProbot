cmake_minimum_required(VERSION 3.0.2)
project(data_convert )

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  robot_state_msgs
)

## Find PCL libraries
#find_package(PCL 1.8 REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

## Catkin package settings
catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs pcl_ros pcl_conversions
  DEPENDS PCL
)

## Build executable
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(data_convert_node src/data_convert_node.cpp)
target_link_libraries(data_convert_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)