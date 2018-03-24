//
// Created by william on 24/03/18.
//
/**
 * Created by William Gu on 24/03/18
 * Implementation for a Cone Extractor Node that identifies cones from a laser msg
 */

#include <ConeExtractorNode.h>

LaserscanConeManager::LaserscanConeManager(int argc, char **argv, std::string node_name) {
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");


}