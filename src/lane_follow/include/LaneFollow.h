/*
 * Created By: Raad Khan
 * Created On: April 23, 2017
 * Description: Takes in an image feed and uses LineDetect to generate
 *              lane lines and a destination point, then broadcasts a
 *              Twist message to stay within the lanes.
 */

#ifndef LANE_FOLLOW_LANEFOLLOW_H
#define LANE_FOLLOW_LANEFOLLOW_H

// Snowbots
#include "HSVFilterNode.h"
#include "IPMFilterNode.h"
#include "LineDetect.h"

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>

class LaneFollow {
    public:
    // Constructor
      LaneFollow(int argc, char** argv, std::string node_name);

    private:
    // Instantiate LineDetect to generate the lane lines
    LineDetect ld;

    // Angle of destination point to robot
    double destination_angle;

    // Camera subscriber node
    image_transport::Subscriber image_feed_sub;

    // Steering publisher node
    ros::Publisher stay_in_lane_pub;

    void laneFollowCallback(const sensor_msgs::Image::ConstPtr& image);

    // Velocity limits
    double angular_vel_cap;
    double linear_vel_cap;

    // Scalars
    double angular_speed_multiplier;
    double linear_speed_multiplier;
};

#endif