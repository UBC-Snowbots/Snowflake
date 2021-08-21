/*
 * Created By: Adam Nguyen
 * Created On: August 21st, 2021
 * Snowbot UI
 */

#ifndef _ROS_H
#define _ROS_H
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "QDebug"
#include "std_msgs/Int32.h"


static geometry_msgs::Twist twist_message;

class _Ros
{
public:
    _Ros();
    ~_Ros();

    //twist topic

    static void twist_callback(const geometry_msgs::Twist::ConstPtr& twist_msg){
        twist_message = *twist_msg;
    }

    void twist_subscriber()
    {
        ros::Rate loop_rate(5);
        twist_sub = n->subscribe("/cmd_vel",1000,twist_callback);
        loop_rate.sleep();
        ros::spinOnce();

    }

    //cmd_vel topic
    void set_twist(geometry_msgs::Twist twist_input){twist_msg = twist_input;}
    geometry_msgs::Twist get_twist(){return twist_msg;}

private:

    ros::NodeHandle *n;

    //cmd_vel topic
    ros::Publisher turtle_move ;
    geometry_msgs::Twist twist_msg;
    double linear_x;

    //twist topic
    ros::Subscriber twist_sub;
};

#endif // _ROS_H
