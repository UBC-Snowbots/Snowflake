cmake_minimum_required(VERSION 2.8.3)
project(lane_follow)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        cmake_modules
        roscpp
        rospy
        image_transport
        cv_bridge
        )

find_package(OpenCV REQUIRED)
find_package(sb_utils REQUIRED)
find_package(vision REQUIRED)
find_package(Eigen REQUIRED)

add_definitions(-std=c++14)

catkin_package(
        #  INCLUDE_DIRS include
        DEPENDS
        eigen
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${sb_utils_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
        ${vision_INCLUDE_DIRS}
        ./include
)

## Declare a C++ executable
# add_executable(sample_package_node src/sample_package_node.cpp)
add_executable(lane_follow
        src/LineDetect.cpp
        src/lane_follow.cpp
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
        ${sb_utils_LIBRARIES}
        ${vision_LIBRARIES}
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

    # Adding gtests to the package
    catkin_add_gtest(line-detect-test test/line-detect-test.cpp src/LineDetect.cpp src/LaneFollow.cpp ../vision/src/IPM.cpp)
    catkin_add_gtest(lane-follow-test test/lane-follow-test.cpp src/LaneFollow.cpp src/LineDetect.cpp ../vision/src/IPM.cpp)
    target_link_libraries(lane-follow-test ${catkin_LIBRARIES})
    target_link_libraries(line-detect-test ${catkin_LIBRARIES})

endif ()
