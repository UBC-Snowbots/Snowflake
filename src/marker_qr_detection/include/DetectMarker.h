/*
 * Created By: Ihsan Olawale, Rowan Zawadski
 * Created On: July 17th, 2022
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#ifndef MARKER_QR_DETECTION_DETECT_MARKER_H
#define MARKER_QR_DETECTION_DETECT_MARKER_H

// STD Includes
#include <iostream>

// ROS Includes
#include <std_msgs/String.h>
#include <ros/ros.h>

// Snowbots Includes
#include <sb_utils.h>

class DetectMarker {
public:
    DetectMarker(int argc, char **argv, std::string node_name);

private:
    /**
     * Callback function for when a new string is received
     *
     * @param msg the string received in the callback
     */
    void subscriberCallBack(const std_msgs::String::ConstPtr& msg);

    ros::Subscriber my_subscriber;
    ros::Publisher my_publisher;
};
#endif //MARKER_QR_DETECTION_DETECT_MARKER_H
