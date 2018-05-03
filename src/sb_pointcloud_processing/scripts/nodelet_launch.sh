#!/bin/bash
# This script launches the nodelet manager then waits for a second 
# before launching the nodelets for pointclouds.
# 
# This is done because sometimes the nodelets are instantiated before
# the manager and would cause registering issues.

roslaunch sb_pointcloud_processing nodelet_manager.launch &
sleep 1
roslaunch sb_pointcloud_processing filter_nodelets.launch &
