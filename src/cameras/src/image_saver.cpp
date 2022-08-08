/*
 * Created By: Ihsan Olawale, Rowan Zawadski
 * Created On: July 17th, 2022
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#include <ImageSaver.h>


int main(int argc, char **argv){
    // Setup your ROS node
    std::string node_name = "image_saver";

    // Create an instance of your class
    ImageSaver image_saver(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}
