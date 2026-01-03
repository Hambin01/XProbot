cmake_minimum_required(VERSION 2.8.3)
project(video_publisher)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  
  # msgs
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  message_generation

)
#find_package(CUDA REQUIRED)
find_package(yaml-cpp REQUIRED) 
IF(YamlCpp_FOUND) 
    MESSAGE("yaml-cpp Library FOUND: yaml-cpp related sources will be built.") 
ELSEIF(YamlCpp_FOUND) 
    MESSAGE("yaml-cpp Library NOT FOUND!") 
ENDIF(YamlCpp_FOUND) 

find_package(Boost REQUIRED COMPONENTS timer)

# OpenCV
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV Libs: \n${OpenCV_LIBS}\n")
message(STATUS "OpenCV Libraries: \n${OpenCV_LIBRARIES}\n")
message(STATUS "OpenCV Headers: \n${OpenCV_INCLUDE_DIRS}\n")
# add_message_files(
#   DIRECTORY msg
#   FILES
  
# )

# add_service_files(
#   DIRECTORY srv
#   FILES
  
# )

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  LIBRARIES ${PROJECT_NAME}
  std_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# link directories
# link_directories(
# 	include
#   ${PROJECT_NAME}
# )

# add_library(${PROJECT_NAME}

# )

###########
## Build ##
###########

# Plane Feature Association
add_executable(${PROJECT_NAME}_node  src/video_publisher.cpp)
add_dependencies(${PROJECT_NAME}_node  ${catkin_EXPORTED_TARGETS} )
target_link_libraries(${PROJECT_NAME}_node   ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES})


install(TARGETS ${PROJECT_NAME}_node 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# install(TARGETS ${PROJECT_NAME}
# 	DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )
