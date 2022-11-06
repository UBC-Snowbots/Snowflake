#!/bin/bash

install_clion=true
while getopts 'n' OPTION; do
	case "$OPTION" in
		n)
			install_clion=false
	esac
done

# Directory this file was executed from
echo "================================================================"
echo "Starting first time installation and setup, please wait"
echo "these downloads can take a while if you're on slow internet."
echo "================================================================"

# The current directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"


echo "================================================================"
echo "Setting up your shell config files"
echo "================================================================"
# Shell config files that various shells source when they run.
# This is where we want to add aliases, source ROS environment
# variables, etc.
SHELL_CONFIG_FILES=(
    "$HOME/.bashrc"\
    "$HOME/.zshrc"
)

# All lines listed here will be added to the shell config files
# listed above, if they are not present already
declare -a new_shell_config_lines=(
    # Source the ROS Environment Variables Automatically
    "source /opt/ros/noetic/setup.sh"\
    # Make sure that all shells know where to find our custom gazebo models,
    # plugins, and resources. Make sure to preserve the path that already exists as well
    "export GAZEBO_MODEL_PATH=$DIR/../src/sb_gazebo/models:${GAZEBO_MODEL_PATH}"\
    "export GAZEBO_PLUGIN_PATH=$DIR/../src/sb_gazebo/lib:${GAZEBO_PLUGIN_PATH}"\
    "export GAZEBO_RESOURCE_PATH=$DIR/../src/sb_gazebo/models:/usr/share/gazebo:/usr/share/gazebo-9:${GAZEBO_RESOURCE_PATH}"\
    # Aliases to make development easier
    "alias clion=\"clion & disown && exit\""\
    "alias rviz=\"rviz & disown && exit\""\
    "alias rqt=\"rqt & disown && exit\""
)

# Add all of our new shell config options to all the shell
# config files, but only if they don't already have them
for file_name in "${SHELL_CONFIG_FILES[@]}";
do
    echo "Setting up $file_name"
    for line in "${new_shell_config_lines[@]}";
    do
        if ! grep -Fq "$line" $file_name
        then
            echo "$line" >> $file_name
        fi
    done
done

#################
# Install CLion #
#################
if [ "$install_clion" = true ] ; then
	sudo snap install clion --classic
fi

###################
# SKIPPING Install Arduino #
###################
#cd $DIR
#./install_arduino.sh

##############################
# Install Other Dependencies #
##############################
cd $DIR
./install_dependencies.sh

##############################
# Setup Snowbots Udev Rules  #
##############################
cd $DIR
./setup_udev_rules.sh

echo "================================================================"
echo "Finished first time installation and setup; you're good to go!"
echo "If you're working with arduino, or any usb devices, please log"
echo "out and login again for the required changes to take effect"
echo "================================================================"
