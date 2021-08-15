/*
 * Created By: Ihsan Olawale, Kevin Lin
 * Created On: August 15th, 2021
 * Description: A file that abstracts the MoveMotor.cpp code away and closes all
 *              motor controllers when the ROS node is quit by Control C
 *              See MoveMotor.cpp and include/MoveMotor.h for more details
 */
#include <MoveMotor.h>

int main(int argc, char** argv) {
    std::string node_name = "move_motor";

    MoveMotor controller(argc, argv, node_name);

    ros::spin();
    controller.close();
    return 0;
}
