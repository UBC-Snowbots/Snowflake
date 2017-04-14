/*
 * Created By: Gareth Ellis
 * Created On: April 1, 2017
 * Description: The implementation for the GpsMover class. This class
 *              contains all the methods required to decide on the
 *              appropriate twist message to make the robot move smoothly
 *              towards a given point from it's current location
 */

#include <GpsMover.h>

double GpsMover::magicFunction(double x, double y, double x_scale, double y_scale){
   return (1/fabs(x)*x_scale + sqrt(fabs(y))*y_scale)/2;
}

GpsMover::GpsMover(double linear_distance_factor, double linear_heading_factor,
             double angular_distance_factor, double angular_heading_factor) {
    setFactors(linear_distance_factor, linear_heading_factor,
                angular_distance_factor, angular_heading_factor);
}

void GpsMover::setFactors(double linear_distance_factor, double linear_heading_factor,
                    double angular_distance_factor, double angular_heading_factor) {
    this->linear_distance_factor = linear_distance_factor;
    this->angular_distance_factor = angular_distance_factor;
    this->linear_heading_factor = linear_heading_factor;
    this->angular_heading_factor = angular_heading_factor;
}

void GpsMover::setMaxSpeeds(double max_linear_speed, double max_angular_speed) {
    this->max_linear_speed = max_linear_speed;
    this->max_angular_speed = max_angular_speed;
}

geometry_msgs::Twist GpsMover::createTwistMessage(geometry_msgs::Point current_location,
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
    command.angular.z = magicFunction(distance, min_turning_angle,
                                      linear_distance_factor, linear_heading_factor);

    // Figure out if we should be turning left or right
    command.angular.z *= (min_turning_angle > 0) ? 1 : -1;

    // Figure out how fast we should move forward
    command.linear.x = magicFunction(min_turning_angle, distance,
                                     linear_heading_factor, linear_distance_factor);

    // Cap our angular and linear speeds
    capValue(command.linear.x, max_linear_speed);
    capValue(command.angular.z, max_angular_speed);

    return command;
}

double GpsMover::minAngularChange(double from_heading, double to_heading) {
    // Find the angular difference (mod 2*Pi)
    double angle_diff = fmod((to_heading - from_heading), 2*M_PI);
    // If obtuse angle, then get the acute angle
    if (fabs(angle_diff) > M_PI)
        return (2.0*M_PI - fabs(angle_diff)) * ((angle_diff > 0) ? -1 : 1);
    else
        return angle_diff;
}

double GpsMover::angleBetweenPoints(geometry_msgs::Point startPoint, geometry_msgs::Point endPoint) {
    double dx = endPoint.x - startPoint.x;
    double dy = endPoint.y - startPoint.y;
    double angle = atan(dy/dx);
    // If the endpoint is behind and to the left
    if (dx < 0 && dy > 0) angle += M_PI;
    // If the endpoint is behind and to the right
    else if (dx < 0 && dy < 0) angle -= M_PI;
    return angle;
}

void GpsMover::capValue(double& val, double cap) {
    if (fabs(val) > cap)
        val = cap * val/fabs(val);
}
