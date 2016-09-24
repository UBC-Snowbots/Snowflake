cmake_minimum_required(VERSION 2.8.3)

project(elsa_vision)

add_definitions(--std=c++11)

## Find catkin macros and libraries

## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)

## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS roscpp rospy cv_bridge image_transport )

find_package(OpenCV REQUIRED)


include_directories(

  ${catkin_INCLUDE_DIRS}

  ${OpenCV_INCLUDE_DIRS}


)


catkin_package()

add_executable(ros_vision src/ros_vision.cpp src/filter.h src/filter.cpp src/IPM.h src/IPM.cpp)

add_executable(stitcher src/image_stitcher.cpp)

target_link_libraries(ros_vision ${catkin_LIBRARIES} ${OpenCV_LIBS})

target_link_libraries(stitcher ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_dependencies(ros_vision OpenCV)

add_dependencies(stitcher OpenCV)
