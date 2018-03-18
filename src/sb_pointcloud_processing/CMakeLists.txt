cmake_minimum_required(VERSION 2.8.3)
project(sb_pointcloud_processing)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs pcl_conversions)
find_package(sb_utils REQUIRED)
# We need mapping_igvc because it contains the message 'LineObstacle' that this node publishes
find_package(mapping_igvc REQUIRED)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)

catkin_package(
  INCLUDE_DIRS include
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

## Specify libraries to link a library or executable target against
 target_link_libraries(line_extractor_node
   ${catkin_LIBRARIES}
   ${sb_utils_LIBRARIES}
   ${mapping_igvc_LIBRARIES}
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

    # Adding rostest to the package
    find_package(rostest REQUIRED)
    add_rostest_gtest(line_extractor_rostest test/line_extractor_test.test test/line_extractor_rostest.cpp src/DBSCAN.cpp src/Regression.cpp test/TestUtils.h)
    target_link_libraries(line_extractor_rostest ${catkin_LIBRARIES})
endif()
