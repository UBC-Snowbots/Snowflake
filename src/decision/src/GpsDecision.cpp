/*
 * Created By: YOUR NAME HERE
 * Created On: September 22, 2016
 * Description: The Decision Node for GPS, takes in a point relative to
 *              the robots location and heading and broadcasts a
 *              recommended twist message
 */

#include <GpsDecision.h>

// The constructor
GpsDecision::GpsDecision(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle public_nh("~");

    // Setup Subscriber(s)
    std::string gps_filtered_topic_name = "/gps_conversion/relative_gps";
    int refresh_rate = 10;
    gps_subscriber = public_nh.subscribe(gps_filtered_topic_name, refresh_rate, &GpsDecision::gpsCallBack, this);

    // Setup Publisher(s)
    std::string twist_topic = public_nh.resolveName("command");
    uint32_t queue_size = 10;
    twist_publisher = nh.advertise<geometry_msgs::Twist>(twist_topic, queue_size);

}

// This is called whenever a new message is received
void GpsDecision::gpsCallBack(const geometry_msgs::Point::ConstPtr& relative_gps) {
    // Deal with new messages here
}

void GpsDecision::publishTwist(geometry_msgs::Twist twist){
    twist_publisher.publish(twist);
}
