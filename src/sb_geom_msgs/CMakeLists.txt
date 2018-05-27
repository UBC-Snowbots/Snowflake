cmake_minimum_required(VERSION 2.8.3)
project(sb_geom_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    std_msgs
    message_generation
    )

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY msg
    FILES Point2D.msg Polynomial.msg SplineLine.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS message_runtime std_msgs
)

