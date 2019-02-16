#!/bin/bash

# This file installs all the dependencies required by the realsense2_camera
# ROS package from intel's repository. The package supports running the
# D415 and D435 realsense cameras.
echo "================================================================"
echo "Configuring realsense repository and drivers."
echo "================================================================"

CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "================================================================"
echo "Preparing Linux Backend and Dev Environment for realsense."
echo "================================================================"

cd $CURR_DIR/../external_libs/librealsense

# Core packages required to build librealsense binaries
sudo apt-get install -y git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev

# Ubuntu 18 specific packages
sudo apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

echo "================================================================"
echo "Installing Intel Realsense permission scripts."
echo "================================================================"

sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger

echo "================================================================"
echo "Building and applying patched kernel modules."
echo "================================================================"

# Make sure webcam is not running before applying the patch
sudo modprobe -r uvcvideo

# Apply patch
./scripts/patch-realsense-ubuntu-lts.sh

echo "================================================================"
echo "Building librealsense2 SDK"
echo "================================================================"

mkdir build
cd build

# Using -DCMAKE_BUILD_TYPE=Release to build with optimizations.
cmake ../ -DCMAKE_BUILD_TYPE=Release

# Recompile and install librealsense binaries
sudo make uninstall
make clean
make
sudo make install

echo "================================================================"
echo "Finished configuring realsense. "
echo "================================================================"
