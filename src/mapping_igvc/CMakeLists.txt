cmake_minimum_required(VERSION 2.8.3)
project(mapping_igvc)

add_definitions(--std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    std_msgs
    sb_utils
    sb_geom
)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY msg
    FILES ConeObstacle.msg LineObstacle.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES sb_geom_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime std_msgs sb_geom_msgs
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
add_executable(obstacle_manager_node
        src/obstacle_manager_node.cpp
        src/ObstacleManagerNode.cpp
        src/ObstacleManager.cpp
        include/ObstacleManagerNode.h
        )
target_link_libraries(obstacle_manager_node
        ${catkin_LIBRARIES}
        )
add_dependencies(obstacle_manager_node ${catkin_EXPORTED_TARGETS})
add_dependencies(obstacle_manager_node mapping_igvc_generate_messages_cpp)

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
    catkin_add_gtest(obstacle-manager-test
            test/obstacle-manager-test.cpp
            src/ObstacleManager.cpp
            include/ObstacleManager.h
            )
    target_link_libraries(obstacle-manager-test
            ${catkin_LIBRARIES}
            )

    # rostest
    # TODO: Maybe? Not sure if we need rostests for the ObstacleManager

endif()

