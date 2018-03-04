#!/bin/bash

echo "================================================================"
echo "Installing Arduino"
echo "================================================================"
ARDUINO_INSTALL_DIR=/usr/share
ARDUINO_DIR_NAME=arduino
ARDUINO_ARCHIVE_NAME=$ARDUINO_DIR_NAME.tar.xz

echo "Downloading Arduino"
sudo rm -rf /tmp/$ARDUINO_DIR_NAME
sudo rm -rf /tmp/$ARDUINO_ARCHIVE_NAME
wget http://downloads.arduino.cc/arduino-1.8.5-linux64.tar.xz -O /tmp/$ARDUINO_ARCHIVE_NAME

echo "Extracting Arduino"
mkdir /tmp/$ARDUINO_DIR_NAME
tar xpvf /tmp/$ARDUINO_ARCHIVE_NAME -C /tmp/$ARDUINO_DIR_NAME --strip-components 1 > /dev/null
sudo rm -rf $ARDUINO_INSTALL_DIR/$ARDUINO_DIR_NAME
sudo mv /tmp/$ARDUINO_DIR_NAME $ARDUINO_INSTALL_DIR

echo "Running the Arduino install script"
$ARDUINO_INSTALL_DIR/$ARDUINO_DIR_NAME/install.sh

echo "Linking Arduino"
sudo rm /usr/local/bin/arduino
sudo ln -s -f $ARDUINO_INSTALL_DIR/$ARDUINO_DIR_NAME/arduino /usr/local/bin/arduino

echo "Giving user correct permissions"

# Add the user to the dialout so they can do arduino things
sudo adduser $USER dialout

