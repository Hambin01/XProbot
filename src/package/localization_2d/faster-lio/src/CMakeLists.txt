add_library(${PROJECT_NAME}
        laser_mapping.cc
        pointcloud_preprocess.cc
        options.cc
        utils.cc
        )

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp livox_ros_driver_gencpp)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${PYTHON_LIBRARIES}
        tbb
        glog
        yaml-cpp
        )

target_include_directories(${PROJECT_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})

install(TARGETS ${PROJECT_NAME}
EXPORT ${PROJECT_NAME}-export
LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib
RUNTIME DESTINATION bin
INCLUDES DESTINATION include
)

install(EXPORT ${PROJECT_NAME}-export
FILE ${PROJECT_NAME}Targets.cmake
DESTINATION lib/cmake/${PROJECT_NAME}
)

install(DIRECTORY include/
DESTINATION include/${PROJECT_NAME}
FILES_MATCHING
PATTERN "*.h"
PATTERN "*.hpp"
PATTERN ".svn" EXCLUDE
)
