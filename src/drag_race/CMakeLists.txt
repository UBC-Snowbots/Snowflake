cmake_minimum_required(VERSION 2.8.3)
project(drag_race)

# This exports the compile commands so people using Vim with YouCompleteMe
# can have easy autocompletion
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions(--std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        )
find_package(sb_utils REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# TODO: Wait. Do we need this line if we're not "exporting" the headers from the package
catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${sb_utils_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ./include
    )

## Declare a C++ executable
# add_executable(${PROJECT_NAME}_node src/drag_race_node.cpp)

# TODO: Perhaps rename to something like `drag_race_controller_node`
add_executable(drag_race_node
        src/drag_race_node.cpp
        src/DragRaceNode.cpp
        src/DragRaceController.cpp
        src/LidarObstacleManager.cpp
        src/LidarObstacle.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(drag_race_node
        ${catkin_LIBRARIES}
        ${sb_utils_LIBRARIES}
        )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_drag_race.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
if (CATKIN_ENABLE_TESTING)

    # Adding gtests to the package
    catkin_add_gtest(lidar-obstacle-manager-test
        test/lidar-obstacle-manager-test.cpp
        src/LidarObstacleManager.cpp
        src/LidarObstacle.cpp)
    target_link_libraries(lidar-obstacle-manager-test ${catkin_LIBRARIES})

    catkin_add_gtest(lidar-obstacle-test 
        test/lidar-obstacle-test.cpp
        src/LidarObstacle)
    target_link_libraries(lidar-obstacle-test ${catkin_LIBRARIES})

    catkin_add_gtest(drag-race-controller-test
            test/drag-race-controller-test.cpp
            src/DragRaceController.cpp)
    target_link_libraries(drag-race-controller-test ${catkin_LIBRARIES})

    # TODO: Maybe some rostests?

endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
