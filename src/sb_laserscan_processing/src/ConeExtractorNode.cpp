//
// Created by william on 24/03/18.
//

#include <ConeExtractorNode.h>

LaserscanConeManager::LaserscanConeManager(int argc, char **argv, std::string node_name) {
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");


}