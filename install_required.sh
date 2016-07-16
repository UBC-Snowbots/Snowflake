#!/bin/bash

###############################
# Install ROS
###############################

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS Environment Variables Automatically
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
echo "source /opt/ros/kinetic/setup.zsh" >> ~/.zshrc

# rosinstall allows downloading of source trees for ROS packages 
# with a single command
sudo apt-get install python-rosinstall


###############################
# Install CLion
###############################
install by executing `./install.sh`.
Raw  install_clion.sh
#!/usr/bin/env bash

# Install dependencies
sudo apt-get install -y openjdk-7-jdk git

# Fetch and extract CLion
echo "Fetching and extracting CLion"
wget https://download.jetbrains.com/cpp/CLion-2016.1.3.tar.gz
tar xzf CLion*.tar.gz -C ~
rm CLion*.tar.gz

# Run CLion Setup
cd ~/clion*
./bin/clion.sh

# Make CLion globally accessible
echo "Linking CLion"
sudo ln -s "$(pwd)/bin/clion.sh" "/usr/local/bin/clion"

# Change the CLion command so that it does not hold up the terminal
echo 'alias clion="clion & disown"' >> ~/.bashrc
echo 'alias clion="clion & disown"' >> ~/.zshrc
