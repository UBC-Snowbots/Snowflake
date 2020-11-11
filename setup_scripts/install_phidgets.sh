#!/bin/bash

echo "================================================================"
echo "Installing Phidgets library"
echo "================================================================"

# update the package index
sudo apt-get update

# install libusb development library
sudo apt-get -y install libusb-dev

# download and unpack the phidgets library
wget https://www.phidgets.com/downloads/phidget22/libraries/linux/libphidget22.tar.gz
tar xvf libphidget22.tar.gz
rm libphidget22.tar.gz

# install
cd libphidget22*
./configure && make && sudo make install
