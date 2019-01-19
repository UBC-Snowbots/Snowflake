cmake_minimum_required(VERSION 2.8.3)
project(reactive_system_urc)

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
add_executable(reactive_system_node
        src/reactive_system_node.cpp
        src/ReactiveSystemNode.cpp
        src/ReactiveSystemTwist.cpp
        include/ReactiveSystemNode.h
        include/ReactiveSystemTwist.h )


## Specify libraries to link a library or executable target against
# target_link_libraries(sample_package_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(reactive_system_node
        ${catkin_LIBRARIES}
)

install(
        TARGETS
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)

    catkin_add_gtest(reactive-system-test test/reactive-system-test.cpp src/ReactiveSystemTwist.cpp include/ReactiveSystemTwist.h)
    target_link_libraries(reactive-system-test ${catkin_LIBRARIES})

    # Adding rostest to the package
    #find_package(rostest REQUIRED)
    # name the test and link it to the .test file and the .cpp file itself, this will allow
    # "catkin_make run_tests" to be able to find and run this rostest
    #add_rostest_gtest(laserscan_cone_manager_rostest test/sb_laserscan_processing_test.test test/laserscan_cone_manager_rostest.cpp)
    #target_link_libraries(laserscan_cone_manager_rostest ${catkin_LIBRARIES})
endif()
