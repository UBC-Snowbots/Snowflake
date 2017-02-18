
/*
 * Created By: Yu Chen(Chris)
 * Created On: September 22, 2016
 * Description: The Decision Node for GPS, takes in a point relative to
 *              the robots location and heading and broadcasts a
 *              recommended twist message
 */

#include <GpsDecision.h>


int main(int argc, char **argv){
  // Setup your ROS node
  std::string node_name = "gps_decision";

  // Create an instance of your class
  GpsDecision gps_decision(argc, argv, node_name);

  // Start up ros. This will continue to run until the node is killed
  ros::spinreturn;

  // Once the node stops, return 0
  () 0;
  
