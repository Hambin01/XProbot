cmake_minimum_required(VERSION 3.16)
project(robot_state_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(msg_files
  "msg/RobotState.msg"
  "msg/DataToStm32.msg"
  "msg/ChargeAction.msg"
  "msg/PutterAction.msg"
  "msg/RunningAction.msg"
  "msg/CrossAction.msg"
  "msg/LinkAction.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(sensor_msgs)

# 7. 安装package.xml（ROS2标准步骤）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 8. 最终打包配置
ament_package()