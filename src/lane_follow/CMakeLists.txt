cmake_minimum_required(VERSION 2.8.3)
project(lane_follow)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    roscpp
    rospy
    image_transport
    cv_bridge
    sb_vision
    sb_utils
    )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_definitions(-std=c++14)

catkin_package(
    INCLUDE_DIRS include
    DEPENDS
    EIGEN3
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
    ${EIGEN3_INCLUDE_DIRS}
    ${EIGEN3_LIBRARIES}
    ./include
    )

## Declare a C++ executable
# add_executable(sample_package_node src/sample_package_node.cpp)
add_executable(lane_follow
    src/lane_follow.cpp
    src/LineDetect.cpp
    src/LaneFollow.cpp
    include/LineDetect.h
    include/LaneFollow.h
    )

## Specify libraries to link a library or executable target against
# target_link_libraries(sample_package_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(lane_follow
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_utils_lib.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
if (CATKIN_ENABLE_TESTING)
    # Adding gtests
    catkin_add_gtest(line-detect-test
        test/line-detect-test.cpp
        src/LineDetect.cpp
        src/LaneFollow.cpp)
    target_link_libraries(line-detect-test
            ${catkin_LIBRARIES})
    catkin_add_gtest(lane-follow-test
        test/lane-follow-test.cpp
        src/LaneFollow.cpp
        src/LineDetect.cpp)
    target_link_libraries(lane-follow-test
        ${catkin_LIBRARIES})

    # Checking launch files for common errors
    find_package(roslaunch REQUIRED)
    roslaunch_add_file_check(launch)
endif ()
