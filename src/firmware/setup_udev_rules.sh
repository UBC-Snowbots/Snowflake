#!/bin/bash

######################################################################
# This script will add udev rules for the sake of persistent device  #
# naming                                                             #
######################################################################

echo "================================================================"
echo "Adding snowbots udev rules to dev folder..."
echo "================================================================"

# The current directory
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Target file directory
file="/etc/udev/rules.d/10-snowbots.rules"
if [ -e "$file" ]
then
  echo "Delete the existing Snowbots udev rules"
  
  # Delete the old snowbots udev rules
  sudo rm /etc/udev/rules.d/10-snowbots.rules
fi

# Copy the new snowbots udev rules to the rule folder
sudo cp $CURR_DIR/src/firmware/10-snowbots.rules /etc/udev/rules.d/10-snowbots.rules

echo "================================================================"
echo "Finished Installing snowbots udev rules"
echo "================================================================"

