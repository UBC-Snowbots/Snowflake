/*
 * Created By: Gareth Ellis and Chris Chen
 * Created On: September 22, 2016
 * Description: TODO
 */

#include <GpsDecision.h>

/**
 * @param x
 * @param y
 * @param x_scale the value to multiply the x value by
 * @param y_scale the value to multiply the y value by
 * @return the average of x_scale * 1/x and y_scale * sqrt(y)
 */
double magic_function(double x, double y, double x_scale, double y_scale){
   return (1/fabs(x) + sqrt(y))/2;
}

geometry_msgs::Twist Mover::createTwistMessage(geometry_msgs::Point current_location,
                                                     float current_heading,
                                                     geometry_msgs::Point waypoint) {
    // The command to return
    geometry_msgs::Twist command;

    // Set components we don't care about to 0
    command.linear.y = 0;
    command.linear.z = 0;
    command.angular.x= 0;
    command.angular.y = 0;

    double dx = waypoint.x - current_location.x;
    double dy = waypoint.y - current_location.y;

    // Figure out how much we have to turn to point directly at the waypoint
    double angle_to_waypoint = atan(dx/dy);
    double angle_diff = current_heading - angle_to_waypoint;

    // Figure out how far we are from the waypoint
    double distance = sqrt(pow(dx,2) + pow(dy,2));

    // Figure out how fast we should turn
    // TODO: Better scaling factors
    command.angular.z = magic_function(distance, angle_diff, 1, 1);

    // Figure out how fast we should move forward
    // TODO: Better scaling factors
    command.linear.x = magic_function(angle_diff, distance, 1, 1);

    return command;
}


