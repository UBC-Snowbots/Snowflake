#!/bin/bash

##########################################################################
# STOP: If the dependency you want to add is required for the project    #
#       to build, it should be added as a rosdep (ie. a dependency       #
#       specified in one of the packages `package.xml` files).           #
#       Dependencies availble through rosdep are listed at               #
#       "https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml". #
#       This script should only contain other dependecies, like          #
#       external packages or utilities                                   #
##########################################################################

# The current directory
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "================================================================" 
echo "Installing ROS Noetic"
echo "================================================================"

sudo apt-get install -y software-properties-common
sudo add-apt-repository ppa:rock-core/qt4

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -O - | sudo apt-key add -
sudo apt-get update -y

sudo apt-get install python3-catkin-pkg python3-wstool python3-rosdep python3-rosinstall-generator ros-noetic-desktop-full -y
source /opt/ros/noetic/setup.sh

# Prepare resdep to install dependencies
sudo rosdep init
rosdep update

echo "================================================================"
echo "Installing Project Dependent ROS packages."
echo "================================================================"

# Setup the workspace
# Setup directory for pulling external pkgs
# Download packages from merged .rosinstall files
cd $CURR_DIR/..
wstool init
wstool update

echo "================================================================"
echo "Installing other ROS dependencies specified by our packages"
echo "================================================================"

cd $CURR_DIR

# Install all required dependencies to build this repo
# (unfortunately this is not recursive, so we have to manually specify a few
# directories)
rosdep install --from-paths \
    $CURR_DIR/../src \
    $CURR_DIR/../src/external_pkgs \
    --ignore-src --rosdistro noetic --skip-keys=librealsense2 -y

echo "================================================================"
echo "Installing other dependencies specified by our packages"
echo "================================================================"
cd $CURR_DIR
sudo ./setup_realsense.sh

cd $CURR_DIR
sudo ./install_phidgets.sh

echo "================================================================"
echo "Installing Misc. Utilities"
echo "================================================================"

sudo apt-get install -y\
    clang-format\
    python3-rosinstall

echo "================================================================"
echo "Finished Installing Utilities"
echo "================================================================"


