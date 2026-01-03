cmake_minimum_required(VERSION 2.8.12)
project(joy_com)

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
    robot_state_msgs
)

add_definitions(-O4 -g)

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
  DEPENDS
    Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


link_directories(
  #${WORKING_DIR}/src/lib/yaml-0.5/build
)

add_library(${PROJECT_NAME}
    src/joy_f710.cpp
)

target_link_libraries(${PROJECT_NAME}
  m
  pthread
  stdc++
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(joy_f710_node
  src/joy_node.cpp
)


target_link_libraries(joy_f710_node
  ${PROJECT_NAME}
)

#add_dependencies(${PROJECT_NAME}_lib
#)
install(DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS joy_f710_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
