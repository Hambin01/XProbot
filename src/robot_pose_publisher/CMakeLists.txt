cmake_minimum_required(VERSION 3.8)
project(robot_pose_publisher)

# 强制使用C++17（ROS 2推荐，兼容C++14）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 声明可执行文件
add_executable(${PROJECT_NAME} src/robot_pose_publisher.cpp)

# 设置编译依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 安装可执行文件到lib目录
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch/ 
  DESTINATION share/${PROJECT_NAME}/launch/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 排除不符合ROS 2规范的检查项（如需）
  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 导出包配置
ament_package()