cmake_minimum_required(VERSION 2.8.3)
project(map)

find_package(catkin REQUIRED COMPONENTS
  grid_map_ros
  grid_map_cv
  grid_map_msgs
  roscpp
  rospy
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES map
#  CATKIN_DEPENDS grid_map roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


