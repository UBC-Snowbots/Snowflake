/*
 * Created By: William Gu
 * Created On: Jan 20, 2018
 * Description: A path finding node which converts a given path into a Twist message to
 *              send to the robot
 */

#include <PathFinding.h>

PathFinding::PathFinding(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    // *** Change to subscribe to path message
    // Setup Subscriber to path
    std::string topic_to_subscribe_to = "/CHANGE";
    int refresh_rate = 10;
    path_subscriber = nh.subscribe(topic_to_subscribe_to, refresh_rate, &MyClass::processPathMsg, this);

    // Setup Publisher to twist
    std::string topic_to_publish_to = "/cmd_vel";
    int queue_size = 1;
    twist_publisher = nh.advertise<geometry_msgs::Twist>(topic_to_publish_to, queue_size);
}


//Subscriber callback
void PathFinding::pathCallBack(const nav_msgs::Path::ConstPtr& path_ptr) {
    nav_msgs::Path path_msg;
    path_msg = *path_ptr; //Take required information from received message
    geometry_msgs::Twist twist_msg = pathToTwist(path_msg);


}

//Algorithm for processing path message and producing to twist message
geometry_msgs::Twist PathFinding::pathToTwist(nav_msgs::Path path_msg) {
    geometry_msgs::Twist twist_msg; //Initialize velocity message


    return twist_msg;
}