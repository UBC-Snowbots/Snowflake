/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#include <LidarDecision.h>

// The constructor
LidarDecision::LidarDecision(int argc, char **argv, std::string node_name) {
    ros::init(argc, argv, node_name);
    // Setup NodeHandles
    ros::NodeHandle nh;
    ros::NodeHandle public_nh("~");

    // Setup Subscriber(s)
    std::string laserscan_topic_name = "/elsa/scan";
    int refresh_rate = 10;
    scan_subscriber = public_nh.subscribe(laserscan_topic_name, refresh_rate, &LidarDecision::scanCallBack, this);

    // Setup Publisher(s)
    std::string twist_topic = public_nh.resolveName("lidar_decision_command");
    uint32_t queue_size = 10;
    twist_publisher = nh.advertise<geometry_msgs::Twist>(twist_topic, queue_size);

}

// This is called whenever a new message is received
void LidarDecision::scanCallBack(const geometry_msgs::Twist::ConstPtr& raw_scan) {
    // Deal with new messages here
}

void LidarDecision::publishTwist(geometry_msgs::Twist twist){
    twist_publisher.publish(twist);
}
