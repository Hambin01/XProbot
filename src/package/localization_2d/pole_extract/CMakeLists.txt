cmake_minimum_required(VERSION 2.8.3)
project(pole_extract)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -fext-numeric-literals")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  tf
  # dynamic_reconfigure
)

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

# generate_messages()
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

# generate_dynamic_reconfigure_options(
#   cfg/pose.cfg
# )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs 
  # dynamic_reconfigure
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)

add_executable(pole_extract  src/pole_extract_main.cpp src/pole_extract.cpp)
target_link_libraries(pole_extract ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pole_extract ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} 
  # ${PROJECT_NAME}_gencfg
  )

install(TARGETS pole_extract
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/include/
   FILES_MATCHING PATTERN "*.h"
   # PATTERN ".svn" EXCLUDE
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
