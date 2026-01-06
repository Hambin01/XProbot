cmake_minimum_required(VERSION 3.0.2)
project(obstacle_stop)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
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

add_executable(obstacle_stop_node src/obstacle_stop_node.cpp)
target_link_libraries(obstacle_stop_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS obstacle_stop
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)