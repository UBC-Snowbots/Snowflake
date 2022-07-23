#!/bin/bash

echo "Installing dependencies for snowbots arm package"

sudo apt-get update && apt-get install -y build-essential \
	rviz python-rosdep \
        ros-melodic-joy ros-melodic-joint-state-publisher vim \
        ros-melodic-robot-state-publisher \
	ros-melodic-joystick-drivers ros-melodic-joy \
        ros-melodic-ros-control ros-melodic-ros-controllers \
        ros-melodic-urdf-tutorial ros-melodic-rqt

echo "Installing ProController moveit driver"

sudo ./pro_controller_moveit_fix.sh


