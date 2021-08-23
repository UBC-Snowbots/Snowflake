/*
 * Created By: Adam Nguyen
 * Created On: August 21st, 2021
 * Snowbots UI
 */

#include "../include/RosIntegration.h"

RosIntegration::RosIntegration()
{
    n = new ros::NodeHandle();
}
RosIntegration::~RosIntegration()
{
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    delete n;
}


