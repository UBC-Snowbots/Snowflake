cmake_minimum_required(VERSION 2.8.3)
project(pathfinding_igvc)

# Build in "Release" (with lots of compiler optimizations) by default
# (If built in "Debug", some functions can take orders of magnitude longer)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
endif()


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(sb_utils REQUIRED)

add_definitions(-std=c++14)

catkin_package(
   # INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${sb_utils_INCLUDE_DIRS}
        ./include
)

## Declare a C++ executable
# add_executable(sample_package_node src/sample_package_node.cpp)
add_executable(path_finding
        src/path_finding.cpp
        src/PathFinding.cpp
        include/PathFinding.h
        )


## Specify libraries to link a library or executable target against
# target_link_libraries(sample_package_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(path_finding
        ${catkin_LIBRARIES}
        ${sb_utils_LIBRARIES}
        )


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)

    # Adding gtests to the package
    catkin_add_gtest(path-finding-test test/path-finding-test.cpp src/PathFinding.cpp)
    target_link_libraries(path-finding-test ${catkin_LIBRARIES})

    # Adding rostest to the package
    find_package(rostest REQUIRED)
    # name the test and link it to the .test file and the .cpp file itself, this will allow
    # "catkin_make run_tests" to be able to find and run this rostest
    add_rostest_gtest(path_finding_rostest test/pathfinding_igvc_test.test test/path_finding_rostest.cpp)
    target_link_libraries(path_finding_rostest ${catkin_LIBRARIES})
endif()

