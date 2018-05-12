cmake_minimum_required(VERSION 2.8.3)
project(sb_laserscan_processing)

# Build in "Release" (with lots of compiler optimizations) by default
# (If built in "Debug", some functions can take orders of magnitude longer)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
endif()

add_definitions(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        mapping_igvc #For cone obstacle msg
        sb_utils
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES sb_laserscan_processing
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
)

## Declare a C++ executable
add_executable(cone_extractor_node
        src/cone_extractor_node.cpp
        src/ConeExtractorNode.cpp
        include/ConeExtractorNode.h
        src/ConeIdentification.cpp
        include/ConeIdentification.h
        )


## Specify libraries to link a library or executable target against
# target_link_libraries(sample_package_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(cone_extractor_node
        ${catkin_LIBRARIES}
        )

add_dependencies(cone_extractor_node
        ${catkin_EXPORTED_TARGETS})


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)

    # Adding gtests to the package
    #catkin_add_gtest(laserscan-cone-manager-test test/laserscan-cone-manager-test.cpp src/LaserscanConeManager.cpp)
    #target_link_libraries(laserscan-cone-manager-test ${catkin_LIBRARIES})

    catkin_add_gtest(cone-identification-test test/cone-identification-test.cpp src/ConeIdentification.cpp test/LaserscanBuilder.h)
    target_link_libraries(cone-identification-test ${catkin_LIBRARIES})

    # Adding rostest to the package
    #find_package(rostest REQUIRED)
    # name the test and link it to the .test file and the .cpp file itself, this will allow
    # "catkin_make run_tests" to be able to find and run this rostest
    #add_rostest_gtest(laserscan_cone_manager_rostest test/sb_laserscan_processing_test.test test/laserscan_cone_manager_rostest.cpp)
    #target_link_libraries(laserscan_cone_manager_rostest ${catkin_LIBRARIES})
endif()