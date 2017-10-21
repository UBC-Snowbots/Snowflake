/*
 * Created By: Raad Khan
 * Created On: April 23, 2017
 * Description: Gets angle of point of intersection of lane lines
 *              and broadcasts a recommended Twist message.
 */

#ifndef LANE_FOLLOW_H
#define LANE_FOLLOW_H

#include <LineDetect.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>

#include <sb_utils.h>
#include <IPM.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

class LaneFollow {

public:

    /**
     * Constructor
     */
    LaneFollow(int argc, char **argv, std::string node_name);

private:

    /**
     * Callback for the filtered image
     *
     * @param address of filtered image matrix
     */
    void subscriberCallBack(const sensor_msgs::ImageConstPtr &msg);

    // Angle of POI of detected lane lines relative to the robot
    int angle_theta;

    /**
     * Initializator
     *
     * @params image information and where to apply the IPM
     */
    void createFilter(float ipm_base_width, float ipm_top_width,
                      float ipm_base_displacement, float ipm_top_displacement,
                      float image_height, float image_width);

    // Instantiate LineDetect class which generates the lane lines
    LineDetect ld;

    // Velocity limits
    double angular_vel_cap;
    double linear_vel_cap;

    // Scaling
    double angular_speed_multiplier;
    double linear_speed_multiplier;

    // Moving away from line variables
    double target_x_distance;
    double target_y_distance;

    // IPM
    IPM ipm;
    float ipm_base_width, ipm_top_width, ipm_base_displacement, ipm_top_displacement;

    // Whether or not we've received the first image
    bool receivedFirstImage;

    /**
     * Subscribes to the raw camera image node
     */
    image_transport::Subscriber image_sub;

    /**
     * Publishes the filtered image
     */
    image_transport::Publisher filter_pub;

    /**
     * Publishes the recommended twist message
     */
    ros::Publisher twist_pub;

    /**
     * Converts ros::sensor_msgs::Image into a cv::Mat
     *
     * @param message to be converted
     */
    cv::Mat rosToMat(const sensor_msgs::Image::ConstPtr &image);

    std::vector <std::vector<cv::Point2d>> transformPoints(std::vector <std::vector<cv::Point2d>> filteredPoints);

};

#endif