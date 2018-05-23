cmake_minimum_required(VERSION 2.8.3)
project(decision_igvc)

# Build in "Release" (with lots of compiler optimizations) by default
# (If built in "Debug", some functions can take orders of magnitude longer)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
endif()

add_definitions(--std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        cv_bridge
        tf2_ros
        gps_common
        tf2_geometry_msgs
        )
find_package(OpenCV REQUIRED)
find_package(sb_utils REQUIRED)

add_definitions(-std=c++14)

catkin_package(
  INCLUDE_DIRS include
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
  ./include
)

## Declare a C++ executable
# add_executable(sample_package_node src/sample_package_node.cpp)
add_executable(lidar_decision 
    src/lidar_decision.cpp
    src/LidarDecision.cpp
    include/LidarDecision.h
    src/LidarObstacle.cpp
    include/LidarObstacle.h
    )
add_executable(vision_decision 
    src/vision_decision.cpp 
    src/VisionDecision.cpp
    include/VisionDecision.h
    )
add_executable(gps_decision 
    src/gps_decision.cpp 
    src/GpsDecision.cpp
    src/GpsMover.cpp
    include/GpsDecision.h
    include/GpsMover.h
    )
add_executable(final_decision 
    src/final_decision.cpp
    src/FinalDecision.cpp 
    include/FinalDecision.h
    )
add_executable(gps_manager 
    src/gps_manager.cpp
    src/GpsManager.cpp 
    include/GpsManager.h
    )

## Specify libraries to link a library or executable target against
# target_link_libraries(sample_package_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(lidar_decision ${catkin_LIBRARIES} ${sb_utils_LIBRARIES})
target_link_libraries(vision_decision ${catkin_LIBRARIES} ${sb_utils_LIBRARIES})
target_link_libraries(gps_decision ${catkin_LIBRARIES} ${sb_utils_LIBRARIES})
target_link_libraries(final_decision ${catkin_LIBRARIES} ${sb_utils_LIBRARIES})
target_link_libraries(gps_manager ${catkin_LIBRARIES} ${sb_utils_LIBRARIES})


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    # gTest
    catkin_add_gtest(lidar-decision-test test/lidar-decision-test.cpp src/LidarDecision.cpp src/LidarObstacle.cpp)
    target_link_libraries(lidar-decision-test ${catkin_LIBRARIES})

    catkin_add_gtest(lidar-obstacle-test test/lidar-obstacle-test.cpp src/LidarObstacle.cpp)
    target_link_libraries(lidar-obstacle-test ${catkin_LIBRARIES})

    catkin_add_gtest(vision-decision-test test/vision-decision-test.cpp src/VisionDecision.cpp
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
    target_link_libraries(vision-decision-test ${catkin_LIBRARIES})

    catkin_add_gtest(gps-mover-test test/gps-mover-test.cpp src/GpsMover.cpp)
    target_link_libraries(gps-mover-test ${catkin_LIBRARIES})

    catkin_add_gtest(final-decision-test test/final-decision-test.cpp src/FinalDecision.cpp)
    target_link_libraries(final-decision-test ${catkin_LIBRARIES})

    catkin_add_gtest(gps-manager-test test/gps-manager-test.cpp src/GpsManager.cpp)
    target_link_libraries(gps-manager-test ${catkin_LIBRARIES})

    #TODO: gps_decision tests

    # rostest
    find_package(rostest REQUIRED)

    add_rostest_gtest(lidar_decision_rostest test/lidar_decision_rostest.test test/lidar_decision_rostest.cpp)
    target_link_libraries(lidar_decision_rostest ${catkin_LIBRARIES})

endif()
