/*
 * Created By: Robyn Castro
 * Created On: July 1, 2017
 * Description: Applies an IPM Filter to input, and publishes it.
 *
 */

#ifndef IPM_FILTER_H
#define IPM_FILTER_H

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/opencv.hpp>

// Image Conversion
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

// ROS
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// STD
#include <string>
#include <vector>

// I/O
#include <fstream>
#include <iostream>
#include <stdio.h>

// Snowbots
#include <IPMFilter.h>
#include <sb_utils.h>

using namespace cv;

class IPMFilterNode {
  public:
    /**
     * Empty Constructor
     */
    IPMFilterNode();

    /**
     * Constructor
     */
    IPMFilterNode(int argc, char** argv, std::string node_name);

  private:
    /**
     * Callback for the filtered image
     *
     * @param address of filtered image matrix
     */
    void filteredImageCallBack(const sensor_msgs::Image::ConstPtr& image);

    /**
     * Converts ros::sensor_msgs::Image into a cv::Mat
     *
     * @param message to be converted
     */
    Mat rosToMat(const sensor_msgs::Image::ConstPtr& image);

    /**
     * Subscribes to the raw camera image node
     */
    image_transport::Subscriber image_sub;

    /**
     * Publishes the filtered image
     */
    image_transport::Publisher ipm_filter_pub;

    // Whether or not we've received the first image
    bool receivedFirstImage;

    // Debug and calibration variables
    int image_width, image_height;

    // IPM Filter Variables
    IPMFilter* ipmFilter;
    float ipm_base_width, ipm_top_width, ipm_base_displacement,
    ipm_top_displacement;
};

#endif