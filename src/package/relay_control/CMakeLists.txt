cmake_minimum_required(VERSION 3.0.2)
project(relay_control)

add_compile_options(-std=c++17)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  std_msgs
  socketcan_bridge
  can_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_srvs std_msgs socketcan_bridge can_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(relay_control src/relay_control.cpp)
target_link_libraries(relay_control ${catkin_LIBRARIES})

install(TARGETS relay_control
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)