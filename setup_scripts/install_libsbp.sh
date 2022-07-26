#!/bin/bash

echo "================================================================"
echo "Installing Siwft Navigation's Binary Protocol library || (GPS)"
echo "================================================================"

cd external_libs/libsbp/c # get to build space
mkdir build
cd build
cmake ../
make
sudo make install   # install headers and libraries into /usr/local
cd ../
