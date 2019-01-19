#!/bin/bash

# This file installs all the dependencies required by the realsense2_camera
# ROS package from intel's repository. The package supports running the
# D415 and D435 realsense cameras.
echo "================================================================"
echo "Configuring realsense repository and drivers."
echo "================================================================"

sudo apt-get install -y software-properties-common
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
sudo rm -f /etc/apt/sources.list.d/realsense-public.list

sudo apt-get update -y

# TODO (#359): We can (and SHOULD) switch back to using the below line once 
# the ros realsense packages are updated to work with librealsense2 2.16.2
# see https://github.com/intel-ros/realsense/issues/502 for updates.
# (Right now we just peg the version to 2.16.1)
# sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg 
version="2.16.3-0~realsense0.116"
sudo apt-get install librealsense2-dkms -y
sudo apt install librealsense2=${version} -y
sudo apt-get install librealsense2-utils=${version} -y
sudo apt-get install librealsense2-dev=${version} -y
sudo apt-get install librealsense2-dbg=${version} -y

echo "================================================================"
echo "Finished configuring realsense. "
echo "================================================================"
