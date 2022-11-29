#!/bin/bash

######################################################################
# This script will add udev rules for the sake of persistent device  #
# naming                                                             #
######################################################################

echo "================================================================"
echo "Adding snowbots udev rules to udev folder..."
echo "================================================================"

# The current directory
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Target file directory
FILE="/etc/udev/rules.d/10-snowbots.rules"
if [ -e "$FILE" ]
then
  echo "Deleting the existing Snowbots udev rules"
  
  # Delete the old snowbots udev rules
  sudo rm /etc/udev/rules.d/10-snowbots.rules
fi

# Copy the new snowbots udev rules to the rule folder
sudo cp $CURR_DIR/10-snowbots.rules $FILE

echo "================================================================"
echo "Finished Installing snowbots udev rules"
echo "================================================================"

echo "================================================================"
echo "Adding phidgets udev rules to udev folder..."
echo "================================================================"


# Target file directory
FILE="/etc/udev/rules.d/99-libphidget22.rules"
if [ -e "$FILE" ]
then
  echo "Deleting the existing Phidgets udev rules"
  
  # Delete the old phidgets udev rules
  sudo rm /etc/udev/rules.d/99-phidgets.rules
fi

# Copy the new phidgets udev rules to the rule folder
# From the installation to the udev folder
# Apt should theoretically do this but it doesn't
sudo cp /home/rowan/libphidget22-1.12.20220912/plat/linux/udev/99-libphidget22.rules $FILE

echo "================================================================"
echo "Finished Installing phidgets udev rules"
echo "================================================================"

