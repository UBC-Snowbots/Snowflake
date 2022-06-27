cmake_minimum_required(VERSION 2.8.3)
project(drivers)

# Unfortunately some 3rd party libraries we use aren't support in C++17,
# so we have to stick with C++14 for now
add_definitions(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        sensor_msgs
        sb_utils
)
find_library(libserial_LIBRARIES libserial.so.0)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${libserial_INCLUDE_DIRS}
    ./include
)

add_executable(steering_driver
        src/steering_driver.cpp
        src/SteeringDriver.cpp
        include/SteeringDriver.h
        )
target_link_libraries(steering_driver
        ${catkin_LIBRARIES}
        ${libserial_LIBRARIES}
        )

add_executable(encoder_odometry_node
        src/EncoderOdometryNode.cpp
        src/encoder_odometry_node.cpp
        include/EncoderOdometryNode.h
        )
target_link_libraries(encoder_odometry_node
        ${catkin_LIBRARIES}
        )

add_executable(arm_driver
        src/ArmDriver.cpp
        src/arm_driver.cpp
        include/ArmDriver.h
        )
target_link_libraries(arm_driver
        ${catkin_LIBRARIES}
        ${libserial_LIBRARIES}
        )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)

    # Adding rostest to the package
    find_package(rostest REQUIRED)
    # name the test and link it to the .test file and the .cpp file itself, this will allow
    # "catkin_make run_tests" to be able to find and run this rostest
    add_rostest_gtest(encoder_odometry_node_rostest
        test/encoder_odometry_node.test
        test/encoder_odometry_node_rostest.cpp
        )
    target_link_libraries(encoder_odometry_node_rostest
        ${catkin_LIBRARIES}
        )
endif()
