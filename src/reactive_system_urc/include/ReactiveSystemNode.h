/**
 * Created by William Gu on Sept 29 2018
 * Class declaration for Reactive System Node
 */

#ifndef REACTIVE_SYSTEM_NODE_H
#define REACTIVE_SYSTEM_NODE_H

#include <iostream>
#include <ros/ros.h>
#include <sb_utils.h>
#include <sensor_msgs/LaserScan.h>

class ReactiveSystemNode {
public:
    ReactiveSystemNode(int argc, char** argv, std::string node_name);

private:
    ros::Subscriber laser_subscriber;
    ros::Publisher twist_publisher;

    /**
    * Callback function for receiving laser scan msgs. Publishes the cones
    * found in the laserscan to
    * the correct topic one by one. Note that the coordinates for cones are in
    * the base-link (robot frame)
    * @param ptr
    */
    void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& ptr);
};

#endif // REACTIVE_SYSTEM_NODE
