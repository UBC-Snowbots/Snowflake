/**
 * Created by William Gu on Nov 3 2018
 * Declarations for Reactive System Pathfinder
 * This node is responsible for moving the robot towards a given GPS goal while avoiding risky areas
 */


#ifndef PROJECT_REACTIVESYSTEMPATHFINDER_H
#define PROJECT_REACTIVESYSTEMPATHFINDER_H

#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <mapping_msgs_urc/RiskArea.h>
#include <mapping_msgs_urc/RiskAreaArray.h>
#include <mapping_msgs_urc/RiskAreaStamped.h>


class ReactiveSystemPathfinder {
public:

    /**
     * Entrypoint for pathfinding algorithm

     */
    static nav_msgs::Path pathFinder(mapping_msgs_urc::RiskAreaArray risk_areas, geometry_msgs::Point curr_pos, geometry_msgs::Point goal_pos);

    /**
     * Calculates the arc trajectory given some twist command (linear and angular velocity)
     * @return
     */
    static std::vector<geometry_msgs::Point> getArcTrajectory(double linear_vel, double angular_vel, double time_inc, int num_incs);

    /**
     * Calculate the arc point in a given arc trajectory by rotating (0,0) about some center point by angle
     * @return
     */
    static geometry_msgs::Point getArcPoint(geometry_msgs::Point center, double angle);
};

#endif //PROJECT_REACTIVESYSTEMPATHFINDER_H
