cmake_minimum_required(VERSION 2.8.3)
project(mapping_igvc)

add_definitions(--std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    mapping_igvc_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS mapping_igvc_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${catkin_INCLUDE_DIRS}
    ./include
)

## Declare a C++ executable
# TODO

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
# TODO

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    #gTest
        catkin_add_gtest(obstacle-manager-test
            test/obstacle-manager-test.cpp
            include/ObstacleManager.h
            src/ObstacleManager.cpp
            )
        target_link_libraries(obstacle-manager-test ${catkin_LIBRARIES})

    # rostest
    # TODO: Maybe? Not sure if we need rostests for the ObstacleManager

endif()

