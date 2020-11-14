#!/bin/bash

# Note: This file should be removed once alglib is available through rosdep


echo "================================================================"
echo "installing Alglib library"
echo "================================================================"

# update the package index
sudo apt-get update

# install 
sudo apt-get install libalglib-dev
