cmake_minimum_required(VERSION 2.8.3)
project(map)

add_definitions(--std=c++11)

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
  ./include
  ${catkin_INCLUDE_DIRS}
)

add_executable(map src/Map.cpp src/map.cpp include/Map.h)
target_link_libraries(map ${catkin_LIBRARIES})


