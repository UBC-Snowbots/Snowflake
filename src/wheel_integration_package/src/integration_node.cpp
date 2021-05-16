/*
 * Created By: Vijeeth Vijhaipranith
 * Created On: Oct 30th, 2020
 * Description: This node subscribes to a topic publishing Geometry Twist
 * messages that indicate the direction to move the rover in,
 *              and publishes velocity values to the left and right wheels of
 * the rover.
 *              The velocity of the left wheels is published to the topic
 * "/integration_node/lwheels_pub_topic"
 *              The velocity of the right wheels is published to the topic
 * "/integration_node/rwheels_pub_topic"
 */

#include <IntegrationNode.h>

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "integration_node";
    float max_speed       = 1.0;

    // Distance between the left and right wheels
    float dist = 0.930;

    // Create an instance of your class
    MyClass my_class(argc, argv, node_name, dist, max_speed);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}
