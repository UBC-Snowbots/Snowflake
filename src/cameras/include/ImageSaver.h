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
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// Image Conversion
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <sensor_msgs/image_encodings.h>


// STD Includes
#include <iostream>
#include <std_srvs/Empty.h>

// ROS Includes
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>

// Snowbots Includes
#include <sb_utils.h>

class ImageSaver {
public:
   ImageSaver(int argc, char **argv, std::string node_name);
   int PicsToTake = 0;

private:
    
    void subscriberCallBack(const sensor_msgs::Image::ConstPtr& image);
    void subscriberCallBack2(const std_msgs::Int32::ConstPtr& msg);

    image_transport::Subscriber camera_subscribe;
    ros::Subscriber shutter; // subscriber to initiate photo saving
   // ros::Publisher test;
  
};
#endif //CAMERAS_IMAGE_SAVER_H
