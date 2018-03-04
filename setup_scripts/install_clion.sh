#!/bin/bash

echo "================================================================"
echo "Installing CLion"
echo "================================================================"
CLION_VERSION="2017.3.3"

# Install dependencies
sudo apt-get install -y openjdk-8-jdk

# Fetch and extract CLion
echo "Fetching and extracting CLion"
wget https://download.jetbrains.com/cpp/CLion-$CLION_VERSION.tar.gz
sudo rm -rf /usr/share/clion-$CLION_VERSION
sudo tar xzf CLion-$CLION_VERSION.tar.gz -C /usr/share
rm -rf CLion-$CLION_VERSION.tar.gz

# Run CLion Setup
cd /usr/share/clion-$CLION_VERSION
./bin/clion.sh

# Make CLion globally accessible
echo "Linking CLion"
sudo rm -rf /usr/local/bin/clion
sudo ln -s -f /usr/share/clion-$CLION_VERSION/bin/clion.sh /usr/local/bin/clion
