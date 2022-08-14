/*
 * Created By: Ihsan Olawale, Rowan Zawadski
 * Created On: July 17th, 2022
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#ifndef MARKER_QR_DETECTION_DETECT_MARKER_H
#define MARKER_QR_DETECTION_DETECT_MARKER_H

// OpenCV
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>

// Image Conversion
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <sensor_msgs/image_encodings.h>

// STD Includes
#include <iostream>
#include <vector>

// ROS Includes
#include <ros/ros.h>
#include <std_msgs/String.h>

// Snowbots Includes
#include <sb_utils.h>

class DetectMarker {
  public:
    DetectMarker(int argc, char** argv, std::string node_name);

  private:
    /**
     * Callback function for when a new string is received
     *
     * @param msg the string received in the callback
     */
    void subscriberCallBack(const sensor_msgs::Image::ConstPtr& msg);

    std::vector<int> fetchMarkerIds(const cv::Mat& image);

    cv::Mat rosToMat(const sensor_msgs::Image::ConstPtr& image);

    image_transport::Subscriber my_subscriber;
    ros::Publisher my_publisher;
    image_transport::Publisher bounder;

    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    bool draw_markers = false;
    int camera        = 1;
};
#endif // MARKER_QR_DETECTION_DETECT_MARKER_H
