cmake_minimum_required(VERSION 2.8.12)
project(navigation_plan)

# Set c++11 cmake flags
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")

add_definitions(-DOPENMP)

set(WORKING_DIR $ENV{HOME}/ROBOT_INDOOR)
message("work_dir = ${WORKING_DIR}")
add_definitions(-DWork_Path="${WORKING_DIR}")

# Find rquired packeges

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
    robot_state_msgs
    costmap_2d
    message_generation
    dynamic_reconfigure
    motion_control
)

add_definitions(-O4 -g)
generate_dynamic_reconfigure_options(
  config/navigation_cfg.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    dynamic_reconfigure
  DEPENDS
    Boost
)



include_directories(
  include
  config
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${WORKING_DIR}/src/util/transform_util
  ${WORKING_DIR}/src/lib/jsoncpp/include
)


link_directories(
  ${WORKING_DIR}/install/lib/jsoncpp/lib
)

add_library(${PROJECT_NAME}
    src/navigation_plan.cpp
)

target_link_libraries(${PROJECT_NAME}
  m
  pthread
  stdc++
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  glog
  libjsoncpp.so
)

add_executable(navigation_plan_node
  src/navigation_plan_node.cpp
)


target_link_libraries(navigation_plan_node
  ${PROJECT_NAME}
)

#add_dependencies(${PROJECT_NAME}_lib
#)

install(DIRECTORY
    launch
    config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS navigation_plan_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
