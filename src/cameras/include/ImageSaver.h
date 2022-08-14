/*
 * Created By: Ihsan Olawale, Rowan Zawadski
 * Created On: July 17th, 2022
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#ifndef CAMERAS_IMAGE_SAVER_H
#define CAMERAS_IMAGE_SAVER_H

// OpenCV
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

// Image Conversion
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <sensor_msgs/image_encodings.h>

// STD Includes
#include <iostream>
#include <std_srvs/Empty.h>

// ROS Includes
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

// Snowbots Includes
#include <sb_utils.h>

class ImageSaver {
  public:
    ImageSaver(int argc, char** argv, std::string node_name);
    int PicsToTake1 = 0;
    int PicsToTake2 = 0;
    int picCounter1 = 0;
    int picCounter2 = 0;

  private:
    void subscriberCallBack1(const sensor_msgs::Image::ConstPtr& img1);
    void subscriberCallBack2(const sensor_msgs::Image::ConstPtr& img2);
    void subscriberCallBackShutter1(const std_msgs::Int32::ConstPtr& msg);
    void subscriberCallBackShutter2(const std_msgs::Int32::ConstPtr& msg);

    image_transport::Subscriber camera1_subscribe;
    image_transport::Subscriber camera2_subscribe;
    ros::Subscriber shutter1; // subscriber to initiate photo saving
    ros::Subscriber shutter2; // subscriber to initiate photo saving

    int camera;
};
#endif // CAMERAS_IMAGE_SAVER_H
