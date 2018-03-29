cmake_minimum_required(VERSION 2.8.3)
project(drivers)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        sensor_msgs
        sb_utils
)
find_library(libserial_LIBRARIES libserial.so.0)
add_definitions(-std=c++14)

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
target_link_libraries(steering_driver
        ${catkin_LIBRARIES}
        )


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
endif()
