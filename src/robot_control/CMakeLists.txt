cmake_minimum_required(VERSION 3.8)
project(robot_control)

# C++ 标准（必须 C++17 以支持 std::shared_mutex）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 关键：导出所有符号（解决 vtable 缺失）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -rdynamic")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -rdynamic")

# Qt 自动编译
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# 工作目录
set(WORKING_DIR $ENV{HOME}/ROBOT_INDOOR)
message("work_dir = ${WORKING_DIR}")
add_definitions(-DWork_Path="${WORKING_DIR}")

# 查找依赖（移除 boost 相关）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)
find_package(nlohmann_json REQUIRED)

# 消息依赖
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(robot_state_msgs REQUIRED)
find_package(robot_task_msgs REQUIRED)

# TF2 依赖
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 包含目录（移除 boost 包含）
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}  # UI 头文件目录
  ${Qt5_INCLUDE_DIRS}
  ${WORKING_DIR}/install/include
)

# UI 文件处理
file(GLOB UI_FILES ui/*.ui)
qt5_wrap_ui(UI_HEADERS ${UI_FILES})

# 源文件
set(SRC_FILES
  src/robot_control.cpp
  ${UI_HEADERS}
)

# 构建库
add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
)

# 设置库属性
set_target_properties(${PROJECT_NAME} PROPERTIES
  OUTPUT_NAME "robot_control"
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib
)

# 链接依赖（移除 boost，仅保留标准库和 ROS2 依赖）
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  rviz_default_plugins
  pluginlib
  nav_msgs
  geometry_msgs
  visualization_msgs
  robot_state_msgs
  robot_task_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nlohmann_json
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
  ${rviz_common_LIBRARIES}
  nlohmann_json::nlohmann_json
  # 标准库无需显式链接（C++17 共享互斥锁属于 libstdc++）
)
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROBOT_CONTROL_BUILDING_LIBRARY")

# 插件导出
pluginlib_export_plugin_description_file(rviz_common plugin_control.xml)

# 安装
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES  plugin_control.xml
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY ui/
  DESTINATION share/${PROJECT_NAME}/ui
)
install(DIRECTORY  include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake rclcpp rviz_common pluginlib Qt5)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

ament_package()