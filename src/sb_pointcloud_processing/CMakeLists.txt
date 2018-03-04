cmake_minimum_required(VERSION 2.8.3)
project(sb_pointcloud_processing)
add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS 
    nodelet 
    roscpp 
    std_msgs 
    pcl_ros 
    pcl_msgs
    sensor_msgs
)
find_package(sb_utils REQUIRED)
find_package(PCL REQUIRED)


include_directories(
    ${catkin_INCLUDE_DIRS}
    ${sb_utils_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    include
)


catkin_package(
#  INCLUDE_DIRS include
   LIBRARIES sb_vision_pointcloud
   CATKIN_DEPENDS nodelet roscpp std_msgs pcl_ros
#  DEPENDS system_lib
)

add_library(sb_pointcloud_processing
    include/rgb_to_hsv.h
    src/rgb_to_hsv.cpp
        include/ColourspaceConverter.h
    src/ColourspaceConverter.cpp
)
target_link_libraries(sb_pointcloud_processing
    ${catkin_LIBRARIES}
    ${sb_utils_LIBRARIES}
    ${PCL_LIBRARIES}
)

## TODO: Needed?
if(catkin_EXPORTED_LIBRARIES)
    add_dependencies(preprocessing_node ${catkin_EXPORTED_LIBRARIES})
endif()

install(
    TARGETS
        sb_pointcloud_processing
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(colourspace-converter-test test/colourspace-converter-test.cpp include/ColourspaceConverter.h src/ColourspaceConverter.cpp)
    target_link_libraries(colourspace-converter-test ${PCL_LIBRARIES})

    find_package(rostest REQUIRED)
    add_rostest_gtest(pointcloud_nodelet_filters_rostest test/pointcloud_nodelet_filters.test test/pointcloud_nodelet_filters_rostest.cpp)
    target_link_libraries(pointcloud_nodelet_filters_rostest ${catkin_LIBRARIES} ${PCL_LIBRARIES})

endif()


