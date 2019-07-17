/**
 * Created by William Gu on June 1 2019
 * Class declaration for RC Broker Node
 * This node is a wrapper for a simple Remote Communications Broker.
 * It forwards string messages between client and server nodes.
 */

#ifndef RC_BROKER_NODE_H
#define RC_BROKER_NODE_H

#include <iostream>
#include <ros/ros.h>
#include <sb_utils.h>
#include <std_msgs/String.h>

class RCBrokerNode {
  public:
    RCBrokerNode(int argc, char** argv, std::string node_name);

  private:
    ros::Subscriber client_subscriber;
    ros::Publisher client_publisher;
    ros::Subscriber server_subscriber;
    ros::Publisher server_publisher;

    void clientCallBack(const std_msgs::String::ConstPtr& ptr);

    void serverCallBack(const std_msgs::String::ConstPtr& ptr);
};

#endif // RC_BROKER_NODE
