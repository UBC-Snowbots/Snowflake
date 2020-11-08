#!/bin/bash

sudo apt-get -y install libusb-dev
wget https://www.phidgets.com/downloads/phidget22/libraries/linux/libphidget22.tar.gz
tar xvf libphidget22.tar.gz
cd libphi*
./configure && make && sudo make install
