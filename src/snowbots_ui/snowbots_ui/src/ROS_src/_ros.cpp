/*
 * Created By: Adam Nguyen
 * Created On: August 21st, 2021
 * Snowbot UI
 */

#include "_ros.h"
#include "std_msgs/UInt16.h"

_Ros::_Ros()
{
    n = new ros::NodeHandle();
}
_Ros::~_Ros()
{
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    delete n;
}


