/*
 * Created by: Valerian Ratu
 * Created On: October 15, 2016
 * Description: Subscribes to image from a camera and publishes a binary image
 *              depending on the filter specification and setting
 * Usage: Press m to calibrate filter values, press m to save it.
 *
 * Subscribes to: /else/camera/image_raw
 * Publishes to: /vision/filtered_image
 *
 */


#ifndef PROJECT_ROSVISION_H_H
#define PROJECT_ROSVISION_H_H

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include "filter.h"
#include "IPM.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/console.h>
#include <utils.h>
#include <ros/time.h>
#include <string>

class RosVision {

public:

    /**
     * Constructor
     */
    RosVision(int argc, char** argv, std::string node_name);

    RosVision();

private:

    /**
     * Callback executes on subscribed images, each callback will analyze and output a binary image
     *
     * @param msg the image message subscribed to
     */
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    /**
     * Creates windows showing the image at various points in the pipeline
     */
    void createWindow();

    /**
     * Destroys windows
     */
    void deleteWindow();

    //Publisher and subscribers
    image_transport::Subscriber sub;
    image_transport::Publisher pub;
    std::string image_topic;
    std::string output_topic;

    //Frequency handling
    ros::Time last_published;
    ros::Duration publish_interval;

    //Image processing Mat pipeline
    cv::Mat inputImage;
    cv::Mat workingImage;
    cv::Mat ipmOutput;
    cv::Mat filterOutput;

    //Display window string names
    std::string inputWindow;
    std::string ipmOutputWindow;
    std::string filterOutputWindow;

    //Filters and their variables
    snowbotsFilter filter;
    std::string mfilter_file;
    IPM ipm;
    int width , height;
    int x1, x2, x3, x4, y1, y2, y3, y4;
    std::vector<cv::Point2f> orig_points;
    std::vector<cv::Point2f> dst_points;

    //Debug and calibration variables
    bool showWindow;
    bool isCalibratingManually;
};



#endif //PROJECT_ROSVISION_H_H
