cmake_minimum_required(VERSION 2.8.3)
project(sb_vision)

# Build in "Release" (with lots of compiler optimizations) by default
# (If built in "Debug", some functions can take orders of magnitude longer)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
endif()

add_definitions(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    cv_bridge
    image_transport
    sb_utils
    )

find_package(OpenCV REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES sb_vision
    CATKIN_DEPENDS roscpp
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
    ./include
)

## Declare a C++ executable
# add_executable(drivers_node src/drivers_node.cpp)
add_executable(hsv_filter
    src/HSVFilter.cpp
    src/hsv_filter.cpp
    src/HSVFilterNode.cpp
    include/HSVFilter.h
    include/HSVFilterNode.h
    )

add_executable(ipm_filter
    src/IPMFilter.cpp
    src/ipm_filter.cpp
    src/IPMFilterNode.cpp
    src/IPM.cpp
    include/IPMFilter.h
    include/IPMFilterNode.h
    include/IPM.h
    )

add_executable(image_merger
        src/ImageMergerNode.cpp
        src/image_merger.cpp
        include/ImageMergerNode.h
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
target_link_libraries(hsv_filter
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    )

target_link_libraries(ipm_filter
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    )

target_link_libraries(image_merger
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

target_link_libraries(circle_detection
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    )

target_link_libraries(camera
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    )

add_library(sb_vision
    src/IPM.cpp
    include/IPM.h)

target_link_libraries(sb_vision
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    )

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
    # Adding gtests
    catkin_add_gtest(green-recognition-test 
        test/green-recognition-test.cpp 
        src/HSVFilter.cpp 
        src/HSVFilterNode.cpp 
        src/CircleDetection.cpp 
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
        )
    target_link_libraries(green-recognition-test
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        )

    # Checking launch files for common errors
    find_package(roslaunch REQUIRED)
    roslaunch_add_file_check(launch)
endif()

