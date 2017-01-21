cmake_minimum_required(VERSION 2.8.3)

project(vision)

add_definitions(--std=c++11)

## Find catkin macros and libraries

## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)

## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS roscpp rospy cv_bridge image_transport )

find_package(OpenCV REQUIRED)


include_directories(

  ${catkin_INCLUDE_DIRS}

  ${OpenCV_INCLUDE_DIRS}

  ./include

    
)


catkin_package()

add_executable(ros_vision src/ros_vision.cpp include/RosVision.h src/RosVision.cpp include/filter.h src/filter.cpp include/IPM.h src/IPM.cpp)

add_executable(camera src/camera_init.cpp)

target_link_libraries(ros_vision ${catkin_LIBRARIES} ${OpenCV_LIBS})

target_link_libraries(camera ${catkin_LIBRARIES} ${OpenCV_LIBS})
