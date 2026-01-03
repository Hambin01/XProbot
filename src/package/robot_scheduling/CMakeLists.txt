cmake_minimum_required(VERSION 3.0.2)
project(robot_scheduling)

set(CMAKE_CXX_STANDARD 17)
set(WORKING_DIR $ENV{HOME}/ROBOT_INDOOR)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
  geometry_msgs
  std_srvs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_scheduling
  CATKIN_DEPENDS roscpp std_msgs visualization_msgs geometry_msgs std_srvs tf 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${WORKING_DIR}/src/lib/jsoncpp/include
)

add_executable(robot_scheduling_node src/multi_robot_planner_node.cpp)
target_link_libraries(robot_scheduling_node
  ${catkin_LIBRARIES}
  ${WORKING_DIR}/install/lib/jsoncpp/lib/libjsoncpp.so
)