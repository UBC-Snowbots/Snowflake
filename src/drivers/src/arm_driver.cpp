/*
 * Created By: Ihsan Olawale, Tate Kolton
 * Created On: June 26th, 2021
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#include <ArmDriver.h>


int main(int argc, char **argv){
    // Setup your ROS node
    std::string node_name = "arm_driver";

    // Create an instance of your class
    ArmDriver arm_driver(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}
