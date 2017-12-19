/*
 * Created By: Robyn Castro
 * Created On: July 9th, 2017
 * Description: Given a line and a set of parameters, determines
 *              how the car should move in the drag race.
 */
#include "DragRaceController.h"

DragRaceController::DragRaceController(){};

DragRaceController::DragRaceController(double targetDistance,
                                       bool lineToTheRight,
                                       double theta_scaling_multiplier,
                                       double angular_speed_multiplier,
                                       double linear_speed_multiplier,
                                       double angular_vel_cap,
                                       double linear_vel_cap)
  : target_distance(targetDistance),
    line_to_the_right(lineToTheRight),
    theta_scaling_multiplier(theta_scaling_multiplier),
    angular_speed_multiplier(angular_speed_multiplier),
    linear_speed_multiplier(linear_speed_multiplier),
    angular_vel_cap(angular_vel_cap),
    linear_vel_cap(linear_vel_cap) {}

geometry_msgs::Twist
DragRaceController::determineDesiredMotion(LineOfBestFit longestConeLine,
                                           bool no_line_on_expected_side) {
    // Determine angle of line.
    double theta = atan(longestConeLine.getSlope());

    double distanceError;
    double minDistanceFromLine = determineDistanceFromLine(longestConeLine);

    if (no_line_on_expected_side) {
        // Aim for three half-lanes across the farther line if in plan b.
        double planBTargetDistance = target_distance * 3;
        distanceError              = planBTargetDistance - minDistanceFromLine;
    } else
        // Aim for one half-lane across the closer line if in plan a.
        distanceError = target_distance - minDistanceFromLine;

    // Make distance relative to where the line is, turning it into
    // displacement.
    if (!line_to_the_right) distanceError *= -1.0;

    // Account for using the opposite line (Flip displacement).
    if (no_line_on_expected_side) distanceError *= -1.0;

    geometry_msgs::Twist command;

    // Set components we don't care about to 0
    command.linear.y  = 0;
    command.linear.z  = 0;
    command.angular.x = 0;
    command.angular.y = 0;

    // If no line then go straight.
    if (longestConeLine.correlation == 0)
        command.angular.z = 0;
    else
        // Figure out how fast we should be turning
        command.angular.z = (theta_scaling_multiplier * theta + distanceError) *
                            angular_speed_multiplier;

    // Limit the angular velocity
    if (fabs(command.angular.z) > angular_vel_cap)
        command.angular.z =
        angular_vel_cap * command.angular.z / fabs(command.angular.z);

    // Figure out how fast we should be moving forward
    if (command.angular.z != 0)
        command.linear.x = linear_speed_multiplier / fabs(command.angular.z);
    else
        command.linear.x = linear_vel_cap;

    // Limit the linear velocity
    if (command.linear.x > linear_vel_cap) command.linear.x = linear_vel_cap;

    return command;
}

double DragRaceController::determineDistanceFromLine(LineOfBestFit line) {
    double negReciprocal = -1 / line.getSlope();

    /* Find the intersection between the line and its perpendicular line. */

    // Set the two sides equal then isolate x to one side.
    double isolatedXSlope = negReciprocal - line.getSlope();

    // Divide both sides by the isolated slope to get the x point intersection.
    double xIntersection = line.getYIntercept() / isolatedXSlope;

    // Plug in the xIntersection to get the y point intersection.
    double yIntersection = negReciprocal * xIntersection;

    // Return distance found
    return sqrt(pow(xIntersection, 2) + pow(yIntersection, 2));
}