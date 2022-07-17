/*
 * Created By: Ihsan Olawale, Rowan Zawadski
 * Created On: July 17th, 2022
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#include <DetectQRCode.h>

DetectQRCode::DetectQRCode(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup Subscriber(s)
    std::string topic_to_subscribe_to = "subscribe_topic";
    int queue_size                    = 10;
    my_subscriber                     = nh.subscribe(
    topic_to_subscribe_to, queue_size, &DetectQRCode::subscriberCallBack, this);

    // Setup Publisher(s)
    std::string topic = private_nh.resolveName("publish_topic");
    queue_size        = 1;
    my_publisher = private_nh.advertise<std_msgs::String>(topic, queue_size);
}

void DetectQRCode::subscriberCallBack(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received message");
    std::string input_string = msg->data.c_str();
}
