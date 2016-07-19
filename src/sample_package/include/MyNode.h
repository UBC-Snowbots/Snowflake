/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#ifndef SAMPLE_PACKAGE_MYNODE_H
#define SAMPLE_PACKAGE_MYNODE_H

#include <iostream>
#include <std_msgs/String.h>
#include <ros/ros.h>

class MyClass {
public:
    MyClass();
    static std::string addExclamationPoint(std::string input_string);
private:
    void subscriberCallBack(const std_msgs::String::ConstPtr& msg);
    void republishMsg(std::string msg_to_publish);

    ros::Subscriber my_subscriber;
    ros::Publisher my_publisher;
};
#endif //SAMPLE_PACKAGE_MYNODE_H
