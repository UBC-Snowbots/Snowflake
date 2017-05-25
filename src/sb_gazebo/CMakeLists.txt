cmake_minimum_required(VERSION 2.8.3)
project(sb_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  tf2_ros
  tf2_sensor_msgs
)

find_package(sb_utils REQUIRED)

add_definitions(-std=c++14)

catkin_package(
#  INCLUDE_DIRS include
)

catkin_install_python(PROGRAMS
   scripts/teleop_twist_keyboard
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
# add_executable(gazebo_node src/gazebo_node.cpp)
add_executable(zed_transform src/zed_transform.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(gazebo_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(zed_transform ${catkin_LIBRARIES} ${sb_utils_LIBRARIES})


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_gazebo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
