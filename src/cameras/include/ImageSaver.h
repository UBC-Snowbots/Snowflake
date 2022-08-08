/*
 * Created By: Ihsan Olawale, Rowan Zawadski
 * Created On: July 17th, 2022
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#ifndef MARKER_QR_DETECTION_DETECT_QR_CODE_H
#define MARKER_QR_DETECTION_DETECT_QR_CODE_H

// OpenCV
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// Image Conversion
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_calibration_parsers/parse.h>

// STD Includes
#include <iostream>
#include <std_srvs/Empty.h>

// ROS Includes
#include <std_msgs/String.h>
#include <ros/ros.h>

// Snowbots Includes
#include <sb_utils.h>

class ImageSaver {
public:
   ImageSaver(int argc, char **argv, std::string node_name);

private:
    /**
     * Callback function for when a new string is received
     *
     * @param msg the string received in the callback
     */
    void subscriberCallBack(const sensor_msgs::Image::ConstPtr& msg);

    std::vector<std::string> fetchQRCodes(const cv::Mat& image);

    void drawQRCodes(cv::Mat& image, std::vector<std::string> decoded_info, std::vector<cv::Point> corners, double fps);

    cv::Mat rosToMat(const sensor_msgs::Image::ConstPtr& image);

    image_transport::Subscriber my_subscriber;
    ros::Publisher my_publisher;

    // cv::QRCodeDetector qrcode;
    bool draw_qr_codes = false;
};
#endif //MARKER_QR_DETECTION_DETECT_QR_CODE_H
