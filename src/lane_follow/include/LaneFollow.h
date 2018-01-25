/*
 * Created By: Raad Khan
 * Created On: April 23, 2017
 * Description: Takes in an image feed and uses LineDetect to generate
 * lane lines and destination point, then broadcasts a recommended
 * Twist message to stay within the lanes.
 */

#ifndef LANE_FOLLOW_H
#define LANE_FOLLOW_H

#include <LineDetect.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <IPM.h>
#include <sb_utils.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

class LaneFollow {
  public:
    // Constructor
    LaneFollow(int argc, char** argv, std::string node_name);

  private:
    /**
     * Callback for the filtered image
     *
     * @param_one pointer to image received from ROS
     */
    void subscriberCallBack(const sensor_msgs::ImageConstPtr& msg);

    /**
     * Initializes filter parameters
     *
     * @params image information and where to apply the IPM
     */
    void createFilter(float ipm_base_width,
                      float ipm_top_width,
                      float ipm_base_displacement,
                      float ipm_top_displacement,
                      float image_height,
                      float image_width);

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
    float ipm_base_width, ipm_top_width, ipm_base_displacement,
    ipm_top_displacement;

    // Whether or not we've received the first image
    bool receivedFirstImage;

    // Angle of POI of lane lines relative to the robot
    int angle_theta;

    // Subscribes to the raw image node
    image_transport::Subscriber image_sub;

    // Publishes the filtered image
    image_transport::Publisher filter_pub;

    // Publishes the recommended twist message
    ros::Publisher twist_pub;

    /**
     * Converts ros::sensor_msgs::Image to cv::Mat
     *
     * @param_one sensor_msgs image to be converted
     *
     * @return converted matrix image
     */
    cv::Mat rosToMat(const sensor_msgs::Image::ConstPtr& image);


    std::vector<std::vector<cv::Point2d>>
    transformPoints(std::vector<std::vector<cv::Point2d>> filteredPoints);
};

#endif