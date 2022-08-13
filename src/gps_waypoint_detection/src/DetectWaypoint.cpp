/* 
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#include <DetectWaypoint.h>

DetectWaypoint::DetectWaypoint(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Obtains character from the parameter server (or launch file), sets '!' as default
    std::string parameter_name    = "pathfind";
    std::string default_character = "!";
    SB_getParam(private_nh, parameter_name, pathfind, false);

    // Setup Subscriber(s)
    std::string topic_to_subscribe_to = "gps/fix";
    int queue_size                    = 10;
    coord_sub                     = nh.subscribe(
    topic_to_subscribe_to, queue_size, &DetectWaypoint::CoordsCallBack, this);

    // Setup Publisher(s)
    std::string topic = private_nh.resolveName("wp_proximity");
    queue_size        = 1;
    proximity_pub = private_nh.advertise<std_msgs::String>(topic, queue_size);
}

void DetectWaypoint::CoordsCallBack(const sensor_msgs::NavSatFix::ConstPtr& coords) {
    ROS_INFO("Received Coords");
    RoverLat = coords->latitude; 
    RoverLong = coords->longitude; 
    ROS_INFO("The coords be lookin like LAT: -- %lf  --, LONG: -- %lf --  ", RoverLat, RoverLong);
    DetectWaypoint::CalcProximity(RoverLat, RoverLong);


}

double DetectWaypoint::CalcProximity(double currLatitude, double currLongitude){
double distX = current_target_lat - currLatitude;
double distY = current_target_long - currLongitude;
double proximity = sqrt((distX*distX) + (distY*distY));
ROS_INFO("proximity: %lf", proximity);

}
