#!/bin/bash

######################################################################
# This script will download and install dependencies for the project #
######################################################################

sudo apt-get install -y\
    ros-kinetic-xacro \
    ros-kinetic-controller-manager \
    ros-kinetic-gazebo-ros-control \
    ros-kinetic-ros-controllers \
    ros-kinetic-ros-control \
    ros-kinetic-effort-controllers
