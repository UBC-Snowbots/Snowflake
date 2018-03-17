#!/bin/bash
# Similar to nodelet_launch.sh, this script launches the testing
# nodelet manager and, after a second, launches the nodelets
# themselves due to early instantiation registration issues.

roslaunch sb_pointcloud_processing nm.test&
sleep 1
roslaunch sb_pointcloud_processing filter_nodelets.test&
