#include <MoveMotor.h>

int main(int argc, char **argv) {
    std::string node_name = "move_motor";

    MoveMotor controller(argc, argv, node_name);

    ros::spin();

    return 0;
}
