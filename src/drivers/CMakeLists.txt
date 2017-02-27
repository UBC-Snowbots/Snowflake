cmake_minimum_required(VERSION 2.8.3)
project(drivers)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
)
find_package(sb_utils REQUIRED)
#find_package(libserial0 REQUIRED)
find_library(libserial_LIBRARIES libserial.so.0)
add_definitions(-std=c++14)

catkin_package(
#  INCLUDE_DIRS include
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
    ${libserial_INCLUDE_DIRS}
    ./include
)

## Declare a C++ executable
# add_executable(drivers_node src/drivers_node.cpp)
add_executable(steering_driver src/steering_driver.cpp src/SteeringDriver.cpp include/SteeringDriver.h)
add_executable(gps_driver src/gps_driver.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(drivers_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(steering_driver ${catkin_LIBRARIES})
target_link_libraries(gps_driver
    ${catkin_LIBRARIES}
    ${sb_utils_LIBRARIES}
    ${libserial_LIBRARIES}
    )


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(steering-driver-test test/steering-driver-test.cpp src/SteeringDriver.cpp)
    target_link_libraries(steering-driver-test ${catkin_LIBRARIES})
endif()
