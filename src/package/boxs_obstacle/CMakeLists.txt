cmake_minimum_required(VERSION 2.8.3)
project(boxs_obstacle)

add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_ros
  pcl_conversions
  eigen_conversions
)

## Find system libraries
#find_package(PCL REQUIRED)
#set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)
find_package(Eigen3 REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs visualization_msgs tf2_ros tf2_geometry_msgs tf2_eigen pcl_ros pcl_conversions
  DEPENDS PCL EIGEN3
)

###########
## Build ##
###########

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Link directories
link_directories(
  ${PCL_LIBRARY_DIRS}
)

## Define the executable
add_executable(boxs_obstacle src/boxs_obstacle_node.cpp)

## Add dependencies to the executable
add_dependencies(boxs_obstacle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Link libraries
target_link_libraries(boxs_obstacle
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

## Install the executable
install(TARGETS boxs_obstacle
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
