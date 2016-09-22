/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#ifndef DECISION_VISION_DECISION_H
#define DECISION_VISION_DECISION_H

#include <iostream>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>

class VisionDecision {
public:
    VisionDecision(int argc, char **argv, std::string node_name);
private:
    void imageCallBack(const sensor_msgs::Image::ConstPtr& raw_scan);
    void publishTwist(geometry_msgs::Twist twist);

    ros::Subscriber image_subscriber;
    ros::Publisher twist_publisher;
};
#endif //DECISION_VISION_DECISION_H
