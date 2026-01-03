cmake_minimum_required(VERSION 3.8)
project(cmd_vel_mux)

# 设置C++标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

# 查找ROS2依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# 查找第三方依赖
find_package(yaml-cpp REQUIRED)
if(YAML_CPP_FOUND)
  message(STATUS "yaml-cpp Library FOUND: yaml-cpp related sources will be built.")
else()
  message(STATUS "yaml-cpp Library NOT FOUND!")
endif()

find_package(Boost REQUIRED COMPONENTS timer)

# 包含头文件目录
include_directories(
  include
  ${YAML_CPP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# 构建库文件
add_library(${PROJECT_NAME}
  src/cmd_node.cpp
  src/joy_to_cmd.cpp
)

# 链接库依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)

target_link_libraries(${PROJECT_NAME}
  ${YAML_CPP_LIBRARIES}
  ${Boost_LIBRARIES}
)

# 构建可执行文件
add_executable(${PROJECT_NAME}_node src/cmd_vel_mux.cpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${YAML_CPP_LIBRARIES}
  ${Boost_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)

add_executable(heart_cmd_node src/heart_cmd_node.cpp)
add_dependencies(heart_cmd_node ${PROJECT_NAME})
target_link_libraries(heart_cmd_node
  ${PROJECT_NAME}
  ${YAML_CPP_LIBRARIES}
  ${Boost_LIBRARIES}
)

ament_target_dependencies(heart_cmd_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# 安装目标
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_node heart_cmd_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# 安装launch和config目录
install(DIRECTORY  config/
  DESTINATION share/${PROJECT_NAME}/config/
)
install(DIRECTORY launch/ 
  DESTINATION share/${PROJECT_NAME}/launch/
)

# 导出依赖
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  tf2
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  yaml-cpp
  Boost
)

ament_package()
