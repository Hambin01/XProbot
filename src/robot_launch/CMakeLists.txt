cmake_minimum_required(VERSION 2.8.3)
project(robot_launch)
find_package(catkin REQUIRED)

catkin_package(
)

install(DIRECTORY
    launch
    maps
    nodes
    script
    carto_file
    rviz
    urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

