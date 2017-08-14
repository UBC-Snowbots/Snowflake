#!/bin/bash

#######################################################################
# STOP: If the dependency you want to add is required for the project # 
#       to build, it should be added as a rosdep. This script should  #
#       only contain other dependecies, like those required for gazebo#
#######################################################################

######################################################################
# This script will download and install dependencies for the project #
######################################################################

echo "================================================================"
echo "Installing other ROS dependencies..."
echo "================================================================"

sudo apt-get install -y\
    libserial-dev \
    python-rosinstall \
    python-visual \
    python-wxtools \
    ros-kinetic-controller-manager \
    ros-kinetic-effort-controllers \
    ros-kinetic-gazebo-ros-control \
    ros-kinetic-gps-common \
    ros-kinetic-pointcloud-to-laserscan
    ros-kinetic-hector-gazebo \
    ros-kinetic-robot-pose-ekf \
    ros-kinetic-ros-control \
    ros-kinetic-ros-controllers \
    ros-kinetic-turtlebot-teleop \
    ros-kinetic-xacro

echo "================================================================"
echo "Finished installing other ROS dependencies."
echo "================================================================"
