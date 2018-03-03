#!/bin/bash

roslaunch sb_pointcloud_processing nm.launch &
sleep 1
roslaunch sb_pointcloud_processing filter_nodelets.launch &
