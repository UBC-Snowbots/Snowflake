cmake_minimum_required(VERSION 2.8.3)
project(cameras)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
endif()

add_definitions(-std=c++14)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
        std_msgs
        cv_bridge
        sensor_msgs
        image_transport
        )
find_package(OpenCV REQUIRED)
find_package(sb_utils REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${sb_utils_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ./include
)
catkin_package(
        CATKIN_DEPENDS roscpp geometry_msgs
)

add_executable(ImageSaver src/ImageSaver.cpp src/image_saver.cpp include/ImageSaver.h)
target_link_libraries(ImageSaver ${catkin_LIBRARIES} ${OpenCV_LIBS} ${sb_utils_LIBRARIES})
