/*
 * Created By: Ihsan Olawale, Tate Kolton
 * Created On: July 3rd, 2022
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#include <ArmHardwareInterface.h>


int main(int argc, char **argv){
    // Setup your ROS node
    std::string node_name = "arm_hardware_interface";

    // Create an instance of your class
    ArmHardwareInterface arm_hardware_interface(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}
