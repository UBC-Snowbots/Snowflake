/**
 * Created by William Gu on Sept 29 2018
 * Implementation for Reactive System Node
 */

#include <ReactiveSystemNode.h>

ReactiveSystemNode::ReactiveSystemNode(int argc,
                                     char** argv,
                                     std::string node_name) {
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    uint32_t queue_size = 1;

    std::string subscribe_topic =
            "/robot/laser/scan"; // Setup subscriber to laserscan (Placeholder)
    laser_subscriber = nh.subscribe(
            subscribe_topic, queue_size, &ReactiveSystemNode::laserCallBack, this);
}

void ReactiveSystemNode::laserCallBack(const sensor_msgs::LaserScan::ConstPtr& ptr) {
    sensor_msgs::LaserScan laser_msg = *ptr;

    return;
}
