cmake_minimum_required(VERSION 2.8.3)
project(zed_filter)

find_package(catkin REQUIRED COMPONENTS roscpp rospy)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES zed_filter
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ./include
)


