cmake_minimum_required(VERSION 2.8.3)
project(sb_pointcloud_processing)

# Build in "Release" (with lots of compiler optimizations) by default
# (If built in "Debug", some functions can take orders of magnitude longer)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
endif()

add_definitions(-std=c++14)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_conversions
  sb_utils
  pcl_ros
  pcl_msgs
  sensor_msgs
  mapping_igvc
  )
find_package(PCL 1.3 REQUIRED COMPONENTS
  common
  io
  )

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
  ./test
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(line_extractor_node
  include/LineExtractorNode.h
  include/DBSCAN.h
  include/Regression.h
  src/line_extractor_node.cpp
  src/LineExtractorNode.cpp
  src/DBSCAN.cpp
  src/Regression.cpp
  include/Regression.h
)

add_executable(igvc_visualizer 
  src/igvc_visualizer.cpp 
  src/IGVCVisualizerNode.cpp
  src/ColourspaceConverter.cpp
  include/IGVCVisualizerNode.h
  include/ColourspaceConverter.h
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
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
)

add_dependencies(line_extractor_node
    ${mapping_igvc_EXPORTED_TARGETS}
)

target_link_libraries(igvc_visualizer
    ${catkin_LIBRARIES} 
    ${sb_utils_LIBRARIES})

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
  )

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)

    # Adding gtests to the package
    catkin_add_gtest(DBSCAN-test
      test/DBSCAN-test.cpp
      test/TestUtils.h
      src/DBSCAN.cpp
      )
    target_link_libraries(DBSCAN-test ${catkin_LIBRARIES})

    catkin_add_gtest(Regression-test
      test/Regression-class-test.cpp
      test/TestUtils.h
      src/Regression.cpp
      )
    target_link_libraries(Regression-test ${catkin_LIBRARIES})

    catkin_add_gtest(colourspace-converter-test test/colourspace-converter-test.cpp include/ColourspaceConverter.h src/ColourspaceConverter.cpp)
    target_link_libraries(colourspace-converter-test ${PCL_LIBRARIES})


    # Adding rostest to the package
    find_package(rostest REQUIRED)

    add_rostest_gtest(line_extractor_rostest
      test/line_extractor_test.test
      test/line_extractor_rostest.cpp
      test/TestUtils.h
      src/DBSCAN.cpp
      src/Regression.cpp
      )
    target_link_libraries(line_extractor_rostest ${catkin_LIBRARIES})
    add_dependencies(line_extractor_rostest
      ${mapping_igvc_EXPORTED_TARGETS}
      )

    add_rostest_gtest(pointcloud_nodelet_filters_rostest test/pointcloud_nodelet_filters.test test/pointcloud_nodelet_filters_rostest.cpp)
    target_link_libraries(pointcloud_nodelet_filters_rostest ${catkin_LIBRARIES} ${PCL_LIBRARIES})

endif()
