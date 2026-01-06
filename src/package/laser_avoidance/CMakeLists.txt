cmake_minimum_required(VERSION 2.8.12)
project(laser_avoidance)

# Set c++11 cmake flags
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
ADD_COMPILE_OPTIONS(-std=c++17 )

add_definitions(-DOPENMP)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(catkin  REQUIRED
  COMPONENTS
    roscpp
    rospy
    nodelet
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    tf
    laser_geometry
    pcl_conversions
    pcl_ros
    robot_state_msgs
)


# 生成对应的devel/lib里面的可执行程序
catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# 设置库链接路径
LINK_DIRECTORIES(
  /usr/local/lib
  # devel目录
  ${CATKIN_DEVEL_PREFIX}
)

add_executable(laser_avoidance_node
  src/laser_avoidance_node.cpp
  src/laser_avoidance.cpp
)

target_link_libraries(laser_avoidance_node
  m
  pthread
  stdc++
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)


install(DIRECTORY
    launch
    config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS laser_avoidance_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS laser_avoidance_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

