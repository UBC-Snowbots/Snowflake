cmake_minimum_required(VERSION 2.8.3)
project(procontroller_snowbots)

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


add_executable(pro_controller src/pro_controller.cpp
        src/ProController.cpp include/ProController.h)


target_link_libraries(pro_controller evdev)
target_link_libraries(pro_controller ${catkin_LIBRARIES} ${sb_utils_LIBRARIES})

