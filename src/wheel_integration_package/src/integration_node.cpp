/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#include <IntegrationNode.h>


int main(int argc, char **argv){
    // Setup your ROS node
    std::string node_name = "integration_node";
    float max_speed = 1.0;
    float dist = 0.930;
    // Create an instance of your class
    MyClass my_class(argc, argv, node_name, dist,  max_speed);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}