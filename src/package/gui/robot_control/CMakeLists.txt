
cmake_minimum_required(VERSION 2.8.3)
project(robot_control)

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
set(WORKING_DIR $ENV{HOME}/ROBOT_INDOOR)
message("work_dir = ${WORKING_DIR}")
add_definitions(-DWork_Path="${WORKING_DIR}")
find_package(catkin REQUIRED COMPONENTS rviz robot_state_msgs)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS}
    ${WORKING_DIR}/install/lib/jsoncpp/include
    )
link_directories(${catkin_LIBRARY_DIRS}
    ${WORKING_DIR}/install/lib/jsoncpp/lib)

set(CMAKE_AUTOMOC ON)

file(GLOB_RECURSE QRC_SOURCE_FILES icon/*.qrc)

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  qt4_wrap_ui(UI_HEADERS
              ui/image_viewer.ui
              )
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
  qt5_wrap_ui(UI_HEADERS
              ui/image_viewer.ui
              )
endif()

add_definitions(-DQT_NO_KEYWORDS)

set(SRC_FILES
  src/robot_control.cpp
)

add_definitions(-DQT_NO_KEYWORDS)

add_library(${PROJECT_NAME}
    ${SRC_FILES}
    )
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES}
    libjsoncpp.so)

## Install rules

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  plugin_control.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY media/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)



