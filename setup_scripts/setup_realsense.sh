#!/bin/bash

# Directory this file was executed from
echo "================================================================"
echo "Configuring realsense repository and drivers."
echo "================================================================"

# The current directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"


 echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' | sudo tee /etc/apt/sources.list.d/realsense-public.list
sudo apt-key adv --keyserver keys.gnupg.net --recv-key 6F3EFCDE
sudo apt-get update
sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg 


echo "================================================================"
echo "Finished configuring realsense. "
echo "================================================================"
