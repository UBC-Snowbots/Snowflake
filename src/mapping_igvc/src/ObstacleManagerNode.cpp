/*
 * Created By: Gareth Ellis
 * Created On: April 3, 2018
 * Description: The node wrapper for the `ObstacleManager` that provides us
 *              with a discrete map of our environment and can generate
 *              Occupancy grids for navigation
 */

#include <ObstacleManagerNode.h>

ObstacleManagerNode::ObstacleManagerNode(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Obtains character from the parameter server (or launch file), sets '!' as default
    // TODO: Delete me
//    std::string parameter_name = "my_node/character";
//    std::string default_character = "!";
//    SB_getParam(nh, parameter_name, suffix, default_character);

    // Setup Subscriber(s)
    // TODO: Delete me
//    std::string topic_to_subscribe_to = "subscribe_topic";
//    int refresh_rate = 10;
//    my_subscriber = nh.subscribe(topic_to_subscribe_to, refresh_rate, &ObstacleManagerNode::subscriberCallBack, this);

    // Setup Publisher(s)
    // TODO: Delete me
//    std::string topic = private_nh.resolveName("publish_topic");
//    uint32_t queue_size = 1;
//    my_publisher = private_nh.advertise<std_msgs::String>(topic, queue_size);
}

