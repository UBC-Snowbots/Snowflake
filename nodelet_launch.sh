#!/bin/bash

. ./devel/setup.sh
roslaunch sb_pointcloud_processing nm.launch &
sleep 1s
roslaunch sb_pointcloud_processing hsv_nodelet.launch &
sleep 1s
roslaunch sb_pointcloud_processing h.launch &
sleep 1s
roslaunch sb_pointcloud_processing s.launch &
sleep 1s
roslaunch sb_pointcloud_processing v.launch &
sleep 1s
roslaunch sb_pointcloud_processing z.launch &


