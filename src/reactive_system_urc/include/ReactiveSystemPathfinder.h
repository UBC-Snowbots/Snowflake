/**
 * Created by William Gu on Nov 3 2018
 * Declarations for Reactive System Pathfinder
 * This node is responsible for moving the robot towards a given GPS goal while avoiding risky areas
 */


#ifndef PROJECT_REACTIVESYSTEMPATHFINDER_H
#define PROJECT_REACTIVESYSTEMPATHFINDER_H

#include <iostream>
#include <math.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <mapping_msgs_urc/RiskArea.h>
#include <mapping_msgs_urc/RiskAreaArray.h>
#include <mapping_msgs_urc/RiskAreaStamped.h>


class ReactiveSystemPathfinder {
public:

    /**
     * Entrypoint for pathfinding algorithm

     */
    static nav_msgs::Path pathFinder(mapping_msgs_urc::RiskAreaArray risk_areas, geometry_msgs::Point32 curr_pos, geometry_msgs::Point32 goal_pos);



};

#endif //PROJECT_REACTIVESYSTEMPATHFINDER_H
