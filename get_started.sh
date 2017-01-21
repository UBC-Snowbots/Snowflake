#!/bin/bash

# Directory this file was executed from
echo "================================================================"
echo "Starting first time installation and setup, please wait"
echo "these downloads can take a while if you're on slow internet."
echo "================================================================"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Shell config files to add aliases to
SHELL_CONFIG_FILES=("$HOME/.bashrc" "$HOME/.zshrc")

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
for file_name in $SHELL_CONFIG_FILES; do
    if ! grep -Fxq "source /opt/ros/kinetic/setup.bash" $file_name
    then
        # Source ROS Environment Variables Automatically
        echo "source /opt/ros/kinetic/setup.bash" >> $file_name
    fi
    if ! grep -Fxq "export GAZEBO_MODEL_PATH=$DIR/src/elsa_gazebo/models:${GAZEBO_MODEL_PATH}" $file_name
    then
        # Source ROS Environment Variables Automatically
        echo "export GAZEBO_MODEL_PATH=$DIR/src/elsa_gazebo/models:${GAZEBO_MODEL_PATH}" >> $file_name
    fi
done

# rosinstall allows downloading of source trees for ROS packages 
# with a single command
sudo apt-get install python-rosinstall


###############################
# Install CLion
###############################
# install by executing `./install.sh`.
#!/usr/bin/env bash

# Install dependencies
sudo apt-get install -y openjdk-8-jdk

# Fetch and extract CLion
echo "Fetching and extracting CLion"
wget https://download.jetbrains.com/cpp/CLion-2016.1.3.tar.gz
tar xzf CLion*.tar.gz -C $DIR
rm CLion*.tar.gz

# Run CLion Setup
cd $DIR/clion*
./bin/clion.sh

# Make CLion globally accessible
echo "Linking CLion"
sudo ln -s $DIR/clion*/bin/clion.sh /usr/local/bin/clion

# Change several commands you have to run from the terminal
# so that they auto-close said terminal
aliases=("clion=\"clion & disown && exit\""\
         "rviz=\"rviz & disown && exit\""\
         "rqt=\"rqt & disown && exit\"")
# Check to make sure the alias doesn't already exist
for file_name in $SHELL_CONFIG_FILES; do
    for ((i = 0; i < ${#aliases[@]}; i++))
    do
       if ! grep -F ${aliases[$i]} $file_name
       then
          echo "alias ${aliases[$i]}" >> $file_name 
       fi
    done
done

###############################
# Install Other Dependencies
###############################
cd $DIR
./install_dependencies.sh

echo "================================================================"
echo "Finished first time installation and setup; you're good to go!"
echo "================================================================"
