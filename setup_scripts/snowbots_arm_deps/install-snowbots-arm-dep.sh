#!/bin/bash

echo "Installing dependencies for snowbots arm package"

echo "Installing ProController moveit driver"

sudo apt-get update && apt-get install -y git dkms
sudo git clone https://github.com/nicman23/dkms-hid-nintendo && \
cd ./dkms-hid-nintendo && \
sudo add . && sudo dkms build nintendo -v 3.2 && sudo dkms install nintendo -v 3.2

sudo apt-get install apt-transport-https ca-certificates gnupg software-properties-common wget -y
sudo wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | sudo apt-key add -
sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main'
sudo apt-get update
sudo apt-get install cmake

sudo apt-get install libudev-dev -y
git clone https://github.com/DanielOgorchock/joycond && \
cd ../joycond \
sudo cmake . && make install && systemctl enable --now joycond

sudo ./pro_controller_moveit_fix.sh

sudo cp moveitjoy_module.py /opt/ros/melodic/lib/python2.7/dist-packages/moveit_ros_visualization

echo "ProController is now enabled"

