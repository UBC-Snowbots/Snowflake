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

# Update Rosdeps
rosdep update

# The current directory
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Install all required dependencies to build this repo
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y

echo "================================================================"
echo "Finished installing other ROS dependencies."
echo "================================================================"
echo ""
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
sudo mkdir -p /usr/share/ros/
sudo chmod a+rwx /usr/share/ros
rosinstall .
echo "bash $CURR_DIR/setup.sh" >> ~/.bashrc

echo "================================================================"
echo "Installing Project Dependent ROS packages."
echo "================================================================"

# Setup udev rules
sudo cp /usr/share/ros/phidgets_api/share/udev/99-phidgets.rules /etc/udev/rules.d
echo "Phidgets udev rules have been copied to /etc/udev/rules.d"
# The reason the script isn't used is because stupid bash/sh doesn't know which directory it's in
# The actual script that comes with phidget is under /usr/share/ros/phidgets_api/share/setup-udev.sh

echo "================================================================"
echo "Finished Installing Utilities"
echo "================================================================"

