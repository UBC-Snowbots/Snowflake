#!/bin/bash

#########################################################################
# STOP: If the dependency you want to add is required for the project   #
#       to build, it should be added as a rosdep (ie. a dependency      #
#       specified in one of the packages `package.xml` files).          #
#       This script should only contain other dependecies, like         #
#       external packages or utilities                                  #
#########################################################################

# The current directory
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "================================================================" 
echo "Installing ROS Kinetic"
echo "================================================================"

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full

echo "================================================================"
echo "Installing other ROS dependencies specified by our packages"
echo "================================================================"

# Update Rosdeps
rosdep update
# Install all required dependencies to build this repo
rosdep install --from-paths $CURR_DIR/../src --ignore-src --rosdistro kinetic --skip-keys=librealsense2 -y 

echo "================================================================"
echo "Installing other dependencies specified by our packages"
echo "================================================================"
cd $CURR_DIR
./setup_realsense.sh

echo "================================================================"
echo "Installing Misc. Utilities"
echo "================================================================"

sudo apt-get install -y\
    clang-format\
    python-rosinstall


echo "================================================================"
echo "Installing Project Dependent ROS packages."
echo "================================================================"

# Setup rosinstall
# Setup directory for installing external pkgs
# Install from merged .rosinstall files
cd $CURR_DIR/..
rosinstall .


echo "================================================================"
echo "Installing Udev rules for phidgets"
echo "================================================================"

# Setup udev rules
sudo cp $CURR_DIR/../src/extended_pkgs/phidgets_api/share/udev/99-phidgets.rules /etc/udev/rules.d
echo "Phidgets udev rules have been copied to /etc/udev/rules.d"
# Phidgets_api pkg does this, but it has to be run in right folder, since we are automating
# we will manually do the copying ourselves

echo "================================================================"
echo "Finished Installing Utilities"
echo "================================================================"

