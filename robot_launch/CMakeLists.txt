cmake_minimum_required(VERSION 3.8)
project(robot_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/ 
  DESTINATION share/${PROJECT_NAME}/launch/
)
install(DIRECTORY rviz/ 
  DESTINATION share/${PROJECT_NAME}/rviz/
)
install(DIRECTORY config/ 
  DESTINATION share/${PROJECT_NAME}/config/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 导出包配置
ament_package()