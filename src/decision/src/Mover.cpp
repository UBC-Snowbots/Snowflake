/*
 * Created By: Gareth Ellis
 * Created On: September 22, 2016
 * Description: TODO
 */

#include <GpsDecision.h>

double Mover::magicFunction(double x, double y, double x_scale, double y_scale){
   return (1/fabs(x)*x_scale + sqrt(fabs(y))*y_scale)/2;
}

Mover::Mover(double distance_factor, double heading_factor) {
    this->distance_factor = distance_factor;
    this->heading_factor = heading_factor;
}

void Mover::setDistanceFactor(double distance_factor) {
    this->distance_factor = distance_factor;
}

void Mover::setHeadingFactor(double heading_factor) {
    this->heading_factor = heading_factor;
}

void Mover::setFactors(double distance_factor, double heading_factor){
    setDistanceFactor(distance_factor);
    setHeadingFactor(heading_factor);
}

geometry_msgs::Twist Mover::createTwistMessage(geometry_msgs::Point current_location,
                                                     double current_heading,
                                                     geometry_msgs::Point waypoint) {
    // The command to return
    geometry_msgs::Twist command;

    // Set components we don't care about to 0
    command.linear.y = 0;
    command.linear.z = 0;
    command.angular.x= 0;
    command.angular.y = 0;


    // Figure out the minimum we have to turn to point directly at the waypoint
    double angle_to_waypoint = angleBetweenPoints(current_location, waypoint);
    double min_turning_angle = minAngularChange(current_heading, angle_to_waypoint);

    // Figure out how far we are from the waypoint
    double dx = waypoint.x - current_location.x;
    double dy = waypoint.y - current_location.y;
    double distance = sqrt(pow(dx,2) + pow(dy,2));

    // Figure out how fast we should turn
    command.angular.z = magicFunction(distance, min_turning_angle, distance_factor, heading_factor);

    // Figure out if we should be turning left or right
    command.angular.z *= (min_turning_angle > 0) ? 1 : -1;

    // Figure out how fast we should move forward
    command.linear.x = magicFunction(min_turning_angle, distance, heading_factor, distance_factor);

    return command;
}

double Mover::minAngularChange(double from_heading, double to_heading) {
    // Find the angular difference (mod 2*Pi)
    double angle_diff = fmod((to_heading - from_heading), 2*M_PI);
    // If obtuse angle, then get the acute angle
    if (fabs(angle_diff) > M_PI)
        return (2.0*M_PI - fabs(angle_diff)) * ((angle_diff > 0) ? -1 : 1);
    else
        return angle_diff;
}

double Mover::angleBetweenPoints(geometry_msgs::Point startPoint, geometry_msgs::Point endPoint) {
    double dx = endPoint.x - startPoint.x;
    double dy = endPoint.y - startPoint.y;
    return atan(dy/dx);
}

