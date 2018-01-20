cmake_minimum_required(VERSION 2.8.3)
project(sb_pointcloud_processing)

find_package(catkin REQUIRED COMPONENTS nodelet roscpp std_msgs)
find_package(sb_utils REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sb_vision_pointcloud
   CATKIN_DEPENDS nodelet roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${sb_utils_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ./include
)

add_library(preprocessing_node
                src/PreprocessingNode.cpp
                include/PreprocessingNode.h
                include/ColourspaceConverter.h
                src/ColourspaceConverter.cpp
)

target_link_libraries(preprocessing_node ${catkin_LIBRARIES}
                                         ${sb_utils_LIBRARIES}
                                         ${PCL_LIBRARIES})

if(catkin_EXPORTED_LIBRARIES)
    add_dependencies(preprocessing_node ${catkin_EXPORTED_LIBRARIES})
endif()

install(TARGETS preprocessing_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(preprocessing-filter-test test/preprocess-filter-test.cpp src/ColourspaceConverter.cpp include/ColourspaceConverter.h)
    target_link_libraries(preprocessing-filter-test ${PCL_LIBRARIES})
endif()


