//
// Created by william on 24/03/18.
//
/**
 * Created by William Gu on 24/03/18.
 * Class declaration for a Cone Extractor Node that identifies cones from a laser msg
 */

#ifndef LASERSCAN_CONE_MANAGER_H
#define LASERSCAN_CONE_MANAGER_H

#include <iostream>
#include <ros/ros.h>

class LaserscanConeManager {
public:
    LaserscanConeManager(int argc, char** argv, std::string node_name);
};

#endif //LASERSCAN_CONE_MANAGER_H
