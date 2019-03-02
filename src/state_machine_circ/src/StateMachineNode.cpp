/**
 * Created by William Gu on Mar 2 2019
 * Implementation for State Machine Node
 */

#include "StateMachineNode.h"

StateMachineNode::StateMachineNode(int argc,
                                       char** argv,
                                       std::string node_name) {
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    uint32_t queue_size = 1;


    /* Setup subscribers and publishers */
}
