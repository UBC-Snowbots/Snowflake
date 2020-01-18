cmake_minimum_required(VERSION 2.8.3)
project(procontroller_snowbots)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        )

set(SOURCE_FILES src/ProController.cpp src/pro_controller.cpp)
add_executable(pro_controller ${SOURCE_FILES})
target_link_libraries(pro_controller evdev)
target_link_libraries(pro_controller ${catkin_LIBRARIES})

include_directories(
        ${catkin_INCLUDE_DIRS}
)
