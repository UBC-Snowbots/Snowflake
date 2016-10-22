/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#include <MyNode.h>

MyClass::MyClass(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle public_nh("~");

    // Setup Subscriber(s)
    std::string topic_to_subscribe_to = "subscribe_topic";
    int refresh_rate = 10;
    my_subscriber = nh.subscribe(topic_to_subscribe_to, refresh_rate, &MyClass::subscriberCallBack, this);

    // Setup Publisher(s)
    std::string topic = public_nh.resolveName("publish_topic");
    uint32_t queue_size = 1;
    my_publisher = public_nh.advertise<std_msgs::String>(topic, queue_size);
}

void MyClass::subscriberCallBack(const std_msgs::String::ConstPtr& msg) {
    std::string input_string = msg->data.c_str();
    std::string new_msg = addExclamationPoint(input_string);
    republishMsg(new_msg);
}

std::string MyClass::addExclamationPoint(std::string input_string) {
    return input_string.append("!");
}

void MyClass::republishMsg(std::string msg_to_publish) {
    std_msgs::String string_to_publish;
    string_to_publish.data = msg_to_publish;
    my_publisher.publish(string_to_publish);
}
