#!/bin/bash

roslaunch sb_pointcloud_processing nm.test&
sleep 1
roslaunch sb_pointcloud_processing filter_nodelets.test&
