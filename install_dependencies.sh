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
    clang-format

echo "================================================================"
echo "Finished Installing Utilities"
echo "================================================================"

