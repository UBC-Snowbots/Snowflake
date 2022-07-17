/*
 * Created By: Ihsan Olawale, Rowan Zawadski
 * Created On: July 17th, 2022
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#include <DetectQRCode.h>


int main(int argc, char **argv){
    // Setup your ROS node
    std::string node_name = "detect_qr_code";

    // Create an instance of your class
    DetectQRCode detect_qr_code(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}
