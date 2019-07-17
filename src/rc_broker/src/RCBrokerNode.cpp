/**
 * Created by William Gu on June 1 2019
 * Implementation for simple RC Broker Node.
 * It forwards messages between client and server nodes.
 * This node is a wrapper for the Remote Communications Broker
 */

#include <RCBrokerNode.h>

RCBrokerNode::RCBrokerNode(int argc, char** argv, std::string node_name) {
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    uint32_t queue_size = 1;

    /* Setup subscribers and publishers */
    std::string client_sub_top = "/client_sub";
    client_subscriber          = nh.subscribe(
    client_sub_top, queue_size, &RCBrokerNode::clientCallBack, this);

    std::string client_pub_top = "/client_pub";
    client_publisher =
    private_nh.advertise<std_msgs::String>(client_pub_top, queue_size);

    std::string server_sub_top = "/server_sub";
    server_subscriber          = nh.subscribe(
    server_sub_top, queue_size, &RCBrokerNode::serverCallBack, this);

    std::string server_pub_top = "/server_pub";
    server_publisher =
    private_nh.advertise<std_msgs::String>(server_pub_top, queue_size);
}

void RCBrokerNode::clientCallBack(const std_msgs::String::ConstPtr& ptr) {
    server_publisher.publish(*ptr);
}

void RCBrokerNode::serverCallBack(const std_msgs::String::ConstPtr& ptr) {
    client_publisher.publish(*ptr);
}
