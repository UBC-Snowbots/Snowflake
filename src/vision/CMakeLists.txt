cmake_minimum_required(VERSION 2.8.3)
project(vision)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy cv_bridge image_transport )
find_package(OpenCV REQUIRED)
find_package(sb_utils REQUIRED)

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
    ${OpenCV_INCLUDE_DIRS}
    ${sb_utils_INCLUDE_DIRS}
    ./include
)

## Declare a C++ executable
# add_executable(drivers_node src/drivers_node.cpp)
add_executable(ros_vision
    src/ros_vision.cpp include/RosVision.h src/RosVision.cpp
    include/filter.h src/filter.cpp
    include/IPM.h src/IPM.cpp
    )

add_executable(camera src/camera_init.cpp)

add_executable(circle_detection
        src/circle_detection.cpp
        src/CircleDetection.cpp
        include/CircleDetection.h
        )

## Specify libraries to link a library or executable target against
# target_link_libraries(drivers_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(ros_vision
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${sb_utils_LIBRARIES}
    )

target_link_libraries(circle_detection
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${sb_utils_LIBRARIES}
        )

target_link_libraries(camera ${catkin_LIBRARIES} ${OpenCV_LIBS})


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)

    # GTest
    catkin_add_gtest(green-recognition-test test/green-recognition-test.cpp src/HSVFilter.cpp src/HSVFilterNode.cpp src/CircleDetection.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
    target_link_libraries(green-recognition-test ${catkin_LIBRARIES})

endif()

