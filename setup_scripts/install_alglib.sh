#!/bin/bash

echo "========================="
echo "installing Alglib library"
echo "========================="

# update the package index
sudo apt-get update

# install 
sudo apt-get install libalglib-dev
