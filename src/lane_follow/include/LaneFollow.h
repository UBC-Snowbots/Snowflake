/*
 * Created By: Raad Khan
 * Created On: April 23, 2017
 * Description: Takes in an image feed and uses LineDetect to generate
 *              lane lines and a destination point, then broadcasts a
 *              Twist message to stay within the lanes.
 */

#ifndef LANE_FOLLOW_LANEFOLLOW_H
#define LANE_FOLLOW_LANEFOLLOW_H

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>

// Image Conversion
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// Snowbots
#include "LineDetect.h"
#include "IPM.h"

using namespace ros;
using namespace cv;

class LaneFollow {
    public:
    // Constructor
      LaneFollow(int argc, char** argv, std::string node_name);

    private:
    /**
     * Callback for the filtered image
     *
     * @param address of filtered image matrix
     */
    void laneFollowCallback(const sensor_msgs::Image::ConstPtr& filteredImage);

    /**
     * Converts Image to Mat
     *
     * @param Image
     *
     * @return Mat
     */
    cv::Mat rosToMat(const sensor_msgs::Image::ConstPtr& image);

    /**
     * Converts filtered lane points to raw lane points
     *
     * @param left and right filtered lane points
     *
     * @return left and right raw lane points
     */
    std::vector<std::vector<Point2d>>
    transformPoints(std::vector<std::vector<cv::Point2d>> filtered_points);

    /**
     * Gets intersect angle from origin to intersection point
     *
     * @param intersection point
     *
     * @return intersect angle
     */
    double getAngleFromOriginToIntersectPoint(cv::Point2d intersect_point);

    // Instantiate LineDetect to generate the lane lines
    LineDetect ld;
  
    // Instantiate IPM to perform IPM on points
    IPM ipm;

    // Filtered image subscriber node
    image_transport::Subscriber filtered_image_sub;

    // Steering driver publisher node
    ros::Publisher stay_in_lane_pub;

    // Image processing Mat pipeline
    cv::Mat filtered_image;
    
    // Recommended steer to follow lane
    geometry_msgs::Twist steering_output;

    // Angle of destination point to robot
    double intersect_angle;

    // Origin of the robot
    int origin_point;

    // Velocity limits
    double angular_vel_cap;
    double linear_vel_cap;

    // Speed scalars
    double angular_speed_multiplier;
    double linear_speed_multiplier;
};

#endif