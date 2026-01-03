cmake_minimum_required(VERSION 2.8.12)
project(remote_control)

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
    robot_task_msgs
    visualization_msgs
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
  DEPENDS
    Boost
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${WORKING_DIR}/src/util/transform_util
  ${WORKING_DIR}/src/lib/jsoncpp/include
)
link_directories(
  ${WORKING_DIR}/install/lib/jsoncpp/lib
)


add_executable(remote_control_node src/remote_control.cpp)
add_dependencies(remote_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(remote_control_node
  ${catkin_LIBRARIES}
  libjsoncpp.so
)