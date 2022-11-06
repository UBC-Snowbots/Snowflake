/*
 * Created By: Tate Kolton
 * Created On: August 9, 2022
 * Description: Move groups interface for arm
 */

#include "../include/moveGroup.h"

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "move_group";

    // Create an instance of your class
    MoveGroupArm arm(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}
