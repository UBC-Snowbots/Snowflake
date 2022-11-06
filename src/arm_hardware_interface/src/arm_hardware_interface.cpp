#include <armHardwareInterface.h>
#include <ros/callback_queue.h>

int main(int argc, char** argv) {
    std::string node_name = "arm_hardware_interface";
    ros::init(argc, argv, node_name);
    ros::CallbackQueue ros_queue;
    ros::NodeHandle nh;
    nh.setCallbackQueue(&ros_queue);
    ArmHardwareInterface arm(nh);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin(&ros_queue);
    return 0;
}
