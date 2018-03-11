cmake_minimum_required(VERSION 2.8.3)
project(sb_pointcloud_processing)

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    pcl_conversions
    roscpp
    pcl_ros
    pcl_msgs
    sensor_msgs
)
find_package(sb_utils REQUIRED)
# We need mapping_igvc because it contains the message 'LineObstacle' that this node publishes
find_package(mapping_igvc REQUIRED)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sb_pointcloud_processing
  CATKIN_DEPENDS nodelet roscpp std_msgs pcl_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ./include
  ${catkin_INCLUDE_DIRS}
  ${sb_utils_INCLUDE_DIRS}
  ${mapping_igvc_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(line_extractor_node
  src/line_extractor_node.cpp
  src/LineExtractorNode.cpp
  include/LineExtractorNode.h
  src/DBSCAN.cpp
  include/DBSCAN.h
  src/Regression.cpp
  include/Regression.h
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

add_executable(test_pcl_generator_node
  src/test_pcl_generator_node.cpp
  test/TestUtils.h
  )

## Specify libraries to link a library or executable target against
target_link_libraries(line_extractor_node
    ${catkin_LIBRARIES}
    ${sb_utils_LIBRARIES}
    ${mapping_igvc_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${pcl_conversions_LIBRARIES}
)
add_dependencies(line_extractor_node
    ${mapping_igvc_EXPORTED_TARGETS}
)

install(
    TARGETS
        sb_pointcloud_processing
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

target_link_libraries(test_pcl_generator_node
  ${catkin_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)

    # Adding gtests to the package
    catkin_add_gtest(DBSCAN-test test/DBSCAN-test.cpp src/DBSCAN.cpp test/TestUtils.h)
    target_link_libraries(DBSCAN-test ${catkin_LIBRARIES})

    catkin_add_gtest(Regression-test test/Regression-class-test.cpp src/Regression.cpp test/TestUtils.h)
    target_link_libraries(Regression-test ${catkin_LIBRARIES})

    catkin_add_gtest(colourspace-converter-test test/colourspace-converter-test.cpp include/ColourspaceConverter.h src/ColourspaceConverter.cpp)
    target_link_libraries(colourspace-converter-test ${PCL_LIBRARIES})


    # Adding rostest to the package
    find_package(rostest REQUIRED)
    add_rostest_gtest(line_extractor_rostest test/line_extractor_test.test test/line_extractor_rostest.cpp src/DBSCAN.cpp src/Regression.cpp test/TestUtils.h)
    target_link_libraries(line_extractor_rostest ${catkin_LIBRARIES})

    add_rostest_gtest(pointcloud_nodelet_filters_rostest test/pointcloud_nodelet_filters.test test/pointcloud_nodelet_filters_rostest.cpp)
    target_link_libraries(pointcloud_nodelet_filters_rostest ${catkin_LIBRARIES} ${PCL_LIBRARIES})

endif()
