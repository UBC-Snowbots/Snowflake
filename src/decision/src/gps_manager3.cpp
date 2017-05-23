/*
 * Created By: Gareth Ellis
 * Created On: May 17, 2016
 * Description: Manages the GPS waypoints we are given
 *              (publishing the next one once we've arrived
 *              at the current one)
 */

#include <GpsManager3.h>


int main(int argc, char **argv){
    // Setup your ROS node
    std::string node_name = "gps_manager";

    // Create an instance of your class
    GpsManager3 gps_manager(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}
