/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#ifndef GPSWAYPOINTDETECTION_DETECTWAYPOINT.H
#define GPSWAYPOINTDETECTION_DETECTWAYPOINT.H

// STD Includes
#include <iostream>

// ROS Includes
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>


// Snowbots Includes
#include <sb_utils.h>

class DetectWaypoint {
public:
    DetectWaypoint(int argc, char **argv, std::string node_name);
    /**
     * Adds an exclamation point to a given string
     *
     * Some Longer explanation should go here
     *
     * @param input_string the string to add an exclamation point to
     *
     * @return input_string with an exclamation point added to it
     */

private:
    /**
     * Callback function for when a new string is received
     *
     * @param msg the string received in the callback
     */
   // void subscriberCallBack(const std_msgs::String::ConstPtr& msg);
    /**
     * Publishes a given string
     *
     * @param msg_to_publish the string to publish
     */
  //  void republishMsg(std::string msg_to_publish);

//waypoint # -+- -+-                   one          two          three        four     five     six      seven    eight
    double WaypointLatitude[8]  =  {  -75.88     ,  38.23     ,  18.33     ,  0     ,  0     ,  0     ,  0     ,  0      };
    double WaypointLongitude[8] =  {  133.88     , -78.23     ,  88.33     ,  0     ,  0     ,  0     ,  0     ,  0      };

    double current_target_lat = WaypointLatitude[0];
    double current_target_long = WaypointLongitude[0];


    double RoverLong = 0.00;
    double RoverLat = 0.00;
    ros::Subscriber coord_sub;
    ros::Publisher proximity_pub;
    bool pathfind = false; 
    ros::Publisher pathfinder;
    void CoordsCallBack(const sensor_msgs::NavSatFix::ConstPtr& coords);

    double CalcProximity(double currLatitude, double currLongitude); //returns proximity in (fake) meters




};
#endif //GPSWAYPOINTDETECTION_DETECTWAYPOINT.H
