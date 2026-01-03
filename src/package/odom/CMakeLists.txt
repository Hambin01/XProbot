cmake_minimum_required(VERSION 2.8.3)
project(odom)

# 包含子cmake文件，引用cmake环境变量
include(cmake/global_defination.cmake)
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
# 查找对应的软件包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  genmsg
  std_msgs
  robot_state_msgs
  tf
  nav_msgs
)

# 添加Boost库
find_package(Boost REQUIRED COMPONENTS thread)





# 设置库链接路径
LINK_DIRECTORIES(
  /usr/local/lib
  # devel目录
  ${CATKIN_DEVEL_PREFIX}
)

# 生成对应的devel/lib里面的可执行程序
catkin_package(
  CATKIN_DEPENDS
)

# 增加语言编译选项为C++14标准
add_compile_options(-std=c++14)

# 引入头文件
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}/lib
)

# 打印cmake的调试信息
MESSAGE(STATUS "CATKIN_DEVEL_PREFIX = ${CATKIN_DEVEL_PREFIX}")

# 设置COMMON_LIB变量
set(COMMON_LIB ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# 生成可执行程序和链接相应的库，如果依赖外部的库需要更改此处
add_executable(odom_node
  src/odom.cpp
  src/odom_node.cpp 
)
add_dependencies(odom_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  odom_node
  ${catkin_LIBRARIES}
  pthread
)

install(DIRECTORY
    launch
    param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS odom_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############


