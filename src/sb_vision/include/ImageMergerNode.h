/*
 * Created By: Robyn Castro
 * Created On: July 12, 2018
 * Description: Combines two images into one image.
 *
 */

#ifndef IMAGE_MERGER_H
#define IMAGE_MERGER_H

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
#include <HSVFilter.h>
#include <sb_utils.h>

class ImageMergerNode {
public:
    /**
     * Constructor
     */
    ImageMergerNode(int argc, char** argv, std::string node_name);

private:

    /**
     * Callback for the first image to be combined with the second image
     *
     * @param address of image containing the white line
     */
    void firstImageCallBack(const sensor_msgs::Image::ConstPtr &image);

    /**
     * Callback for the second image to be combined with the first image
     *
     * @param address of image containing the yellow line
     */
    void secondImageCallBack(const sensor_msgs::Image::ConstPtr &image);

    /**
     * Converts ros::sensor_msgs::Image into a cv::Mat
     *
     * @param message to be converted
     */
    cv::Mat rosToMat(const sensor_msgs::Image::ConstPtr& image);

    /**
     * Subscribes to the white hsv filter
     */
    image_transport::Subscriber first_image_sub;

    /**
     * Subscribes to the yellow hsv filter
     */
    image_transport::Subscriber second_image_sub;

    /**
     * Publishes the merged image
     */
    image_transport::Publisher merged_pub;

    // The name and size of the display window
    std::string display_window_name;

    // Image processing Mat pipeline
    cv::Mat first_image;
    cv::Mat second_image;

    // Whether or not we've received the first images
    bool received_first_image;
    bool received_second_image;

    cv::Mat merged_image;

    // Image window variables
    bool show_window;
};

#endif
