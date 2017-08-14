/*
 * Created By: Valerian Ratu
 * Created On: October 15, 2016
 * Description: Analyzes an image and transforms it into a binary image
 *              given some sort of colour specification.
 */

#include <RosVision.h>

int main(int argc, char* argv[]){
    std::string node_name = "ros_vision";
    RosVision ros_vision(argc, argv, node_name);
    ros::spin();
    return 0;
}
