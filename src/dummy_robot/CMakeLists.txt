cmake_minimum_required(VERSION 3.8)
project(dummy_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 编译可执行文件
add_executable(dummy_robot_node src/dummy_robot.cpp)
ament_target_dependencies(dummy_robot_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 安装可执行文件
install(TARGETS
  dummy_robot_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY  maps/
  DESTINATION share/${PROJECT_NAME}/maps/
)
install(DIRECTORY launch/ 
  DESTINATION share/${PROJECT_NAME}/launch/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()