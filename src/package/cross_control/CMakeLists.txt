cmake_minimum_required(VERSION 3.0.2)
project(cross_control)

add_compile_options(-std=c++17)

# 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  can_msgs
  actionlib
  actionlib_msgs
)

# 生成Action消息
add_action_files(
  FILES
  CrossBelt.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

# 声明catkin包
catkin_package(
  LIBRARIES cross_control
  CATKIN_DEPENDS roscpp std_msgs can_msgs actionlib actionlib_msgs
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 编译节点
add_executable(cross_control_node src/cross_control.cpp)
add_dependencies(cross_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cross_control_node
  ${catkin_LIBRARIES}
)

install(DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS cross_control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS cross_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

