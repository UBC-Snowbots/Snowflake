cmake_minimum_required(VERSION 2.8.3)
project(sb_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

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
)

## Declare a C++ executable
# add_executable(gazebo_node src/gazebo_node.cpp)
add_executable(compass_sim src/compass_sim.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(gazebo_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(compass_sim ${catkin_LIBRARIES})


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_gazebo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
