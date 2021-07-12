cmake_minimum_required(VERSION 2.8.3)
project(cameras)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
endif()

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
        )
find_package(sb_utils REQUIRED)


catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp geometry_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${sb_utils_INCLUDE_DIRS}
        ./include
)