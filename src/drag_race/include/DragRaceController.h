/*
 * Created By: Robyn Castro
 * Created On: July 9th, 2017
 * Description: Given a line and a set of parameters, determines
 *              how the car should move in the drag race.
 */

#ifndef DRAG_RACE_CONTROLLER_H
#define DRAG_RACE_CONTROLLER_H

// ROS Includes
#include <geometry_msgs/Twist.h>

// SB Includes
#include <LidarObstacleManager.h>

class DragRaceController {
  public:
    /**
     * Empty Constructor
     */
    DragRaceController();

    /**
     * Initializes parameters of the robot.
     *
     * @param targetDistance
     * @param lineToTheRight
     * @param theta_scaling_multiplier
     * @param angular_speed_multiplier
     * @param linear_speed_multiplier
     * @param angular_vel_cap
     * @param linear_vel_cap
     */
    DragRaceController(double targetDistance,
                       bool lineToTheRight,
                       double theta_scaling_multiplier,
                       double angular_speed_multiplier,
                       double linear_speed_multiplier,
                       double angular_vel_cap,
                       double linear_vel_cap);

    /**
     * Determines the optimal movement to stay within target distance of
     * the given line.
     *
     * @param longestConeLine
     * @return the optimal angular and linear acceleration.
     */
    geometry_msgs::Twist determineDesiredMotion(LineOfBestFit longestConeLine,
                                                bool no_line_on_expected_side);

  private:
    /**
     * Finds the minimum distance from given line and the origin.
     *
     * @param line
     * @return the minimum distance from given line and the origin.
     */
    static double determineDistanceFromLine(LineOfBestFit line);

    // How far from the target line the robot should be
    double target_distance;

    // Where the target line is
    bool line_to_the_right;

    // Velocity limits
    double angular_vel_cap;
    double linear_vel_cap;

    // Scaling
    double theta_scaling_multiplier;
    double angular_speed_multiplier;
    double linear_speed_multiplier;
};

#endif // DRAG_RACE_CONTROLLER_H