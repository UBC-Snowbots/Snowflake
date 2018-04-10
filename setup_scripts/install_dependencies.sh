#!/bin/bash

<<<<<<< HEAD:setup_scripts/install_dependencies.sh
#########################################################################
# STOP: If the dependency you want to add is required for the project   #
#       to build, it should be added as a rosdep (ie. a dependency      #
#       specified in one of the packages `package.xml` files).          # 
#       This script should only contain other dependecies, like         # 
#       external packages or utilities                                  # 
#########################################################################
=======
#######################################################################
# STOP: If the dependency you want to add is required for the project #
#       to build, it should be added as a rosdep. This script should  #
#       only contain other dependecies, like those required for gazebo#
#######################################################################
>>>>>>> 0e4a72e1041e44de37ff50c0a0850392ee0a62f7:install_dependencies.sh

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
rosdep install --from-paths $CURR_DIR/../src --ignore-src --rosdistro kinetic -y

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
mkdir -p extended_pkg
rosinstall extended_pkg /opt/ros/kinetic .rosinstall
rosinstall .

echo "================================================================"
echo "Installing Udev rules for phidgets"
echo "================================================================"

# Setup udev rules
sudo cp extended_pkg/phidgets_api/share/udev/99-phidgets.rules /etc/udev/rules.d
echo "Phidgets udev rules have been copied to /etc/udev/rules.d"
# The reason the script isn't used is because stupid bash/sh doesn't know which directory it's in
# The actual script that comes with phidget is under /usr/share/ros/phidgets_api/share/setup-udev.sh

echo "================================================================"
echo "Finished Installing Utilities"
echo "================================================================"

