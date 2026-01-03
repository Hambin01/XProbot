cmake_minimum_required(VERSION 2.8.3)
project(link_visual)

## Compile as C++11
add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  visualization_msgs
  geometry_msgs
  dynamic_reconfigure
)

## Find system library: yaml-cpp (separate from catkin components)
find_package(yaml-cpp REQUIRED)

## Generate dynamic reconfigure parameters
generate_dynamic_reconfigure_options(
  cfg/ArmVisualizerConfig.cfg
)

## Declare catkin package (REMOVE yaml-cpp from CATKIN_DEPENDS)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp tf2 tf2_ros visualization_msgs geometry_msgs dynamic_reconfigure

)

## Build
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)

## Declare executable
add_executable(link_visual src/link_visual_node.cpp)

## Add dependencies (dynamic reconfigure)
add_dependencies(link_visual 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

## Link libraries (explicitly link yaml-cpp)
target_link_libraries(link_visual
  ${catkin_LIBRARIES}
  yaml-cpp
)

install(TARGETS link_visual
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
#   # FILES_MATCHING PATTERN "*.h"
#   # PATTERN ".svn" EXCLUDE
# )
# install(DIRECTORY rviz/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz/
#   # FILES_MATCHING PATTERN "*.h"
#   # PATTERN ".svn" EXCLUDE
# )
