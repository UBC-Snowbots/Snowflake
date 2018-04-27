#!/bin/bash

# This file installs all the dependencies required by the realsense2_camera
# ROS package from intel's repository. The package supports running the
# D415 and D435 realsense cameras.
echo "================================================================"
echo "Configuring realsense repository and drivers."
echo "================================================================"

 echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' | sudo tee /etc/apt/sources.list.d/realsense-public.list
sudo apt-key adv --keyserver keys.gnupg.net --recv-key 6F3EFCDE
sudo apt-get update
sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg 


echo "================================================================"
echo "Finished configuring realsense. "
echo "================================================================"
