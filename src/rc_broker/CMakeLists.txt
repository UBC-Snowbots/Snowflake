cmake_minimum_required(VERSION 2.8.3)
project(rc_broker)

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
        mapping_msgs_urc
        sb_utils
        )

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp
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
add_executable(rc_broker
        src/rc_broker.cpp
        src/RCBrokerNode.cpp
        include/RCBrokerNode.h)


## Specify libraries to link a library or executable target against
# target_link_libraries(sample_package_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(rc_broker
        ${catkin_LIBRARIES}
        )


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
    # Adding rostest to the package
    find_package(rostest REQUIRED)
    # name the test and link it to the .test file and the .cpp file itself, this will allow
    # "catkin_make run_tests" to be able to find and run this rostest
    add_rostest_gtest(rc_broker_rostest test/rc_broker_test.test test/rc_broker_rostest.cpp)
    target_link_libraries(rc_broker_rostest ${catkin_LIBRARIES})
endif()


