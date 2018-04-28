/*
 * Created By: Robyn Castro
 * Created On: June 15, 2017
 * Description: Filters an image in the HSV colourspace to a binary image
 *
 */

#ifndef HSV_FILTER_HSVFILTER_H
#define HSV_FILTER_HSVFILTER_H

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
#include "HSVFilter.h"
#include "sb_utils.h"

using namespace cv;

class HSVFilterNode {
  public:
    /**
     * Constructor
     */
    HSVFilterNode(int argc, char** argv, std::string node_name);

  private:
    /**
     * Callback for the filtered image
     *
     * @param address of filtered image matrix
     */
    void rawImageCallBack(const sensor_msgs::Image::ConstPtr& image);

    /**
     * Initialization of the filter
     */
    void setUpFilter();

    /**
     * Update filter values
     */
    void updateFilter();

    /**
     * Show the image taken from the camera and the
     * image to be published
     */
    void showRawAndFilteredImageWindow();
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
    image_transport::Publisher hsv_filter_pub;

    // Frequency handling
    ros::Time last_published;
    ros::Duration publish_interval;

    // Image processing Mat pipeline
    cv::Mat imageInput;
    cv::Mat filterOutput;

    // The name and size of the display window
    std::string displayWindowName;

    // Filters and their variables
    HSVFilter filter;
    std::string mfilter_file;
    double frequency;

    // Whether or not we've received the first image
    bool receivedFirstImage;

    // Debug and calibration variables
    int image_width, image_height;
    bool showWindow;
    bool isCalibratingManually;
};

#endif
