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
    static nav_msgs::Path pathFinder(mapping_msgs_urc::RiskAreaArray risk_areas, geometry_msgs::Point currPos, geometry_msgs::Point goalPos, std_msgs::Float64 risk_threshold);


    /**
     * Filters risk polygons above some risk value
     */
    static std::vector<geometry_msgs::Polygon> filterRiskPolygons(mapping_msgs_urc::RiskAreaArray& risk_areas, std_msgs::Float64 risk_threshold);


    /**
     * Identify adjacent and overlapping polygons and group them into buckets
     */
    static std::vector<std::vector<geometry_msgs::Polygon>> clusterRiskPolygons(mapping_msgs_urc::RiskAreaArray& risk_areas, std::vector<geometry_msgs::Polygon>);


    /**
     * Converts clusters of polygons into clusters of points, removing redundant points in the process
     */
    static std::vector<std::vector<geometry_msgs::Point>> polygonClustersToPoints(std::vector<std::vector<geometry_msgs::Polygon>>);


    /**
     * Performs convex hull algorithm on each cluster of points, returning a vector of resultant polygons (size of resultant polygons should equal number of clusters entered)
     */
    static std::vector<geometry_msgs::Polygon> massConvexHull(std::vector<std::vector<geometry_msgs::Point>>);


    /**
     * Convex hull algorithm, gives minimum area enclosing convex polygon given a group of points
     */
    static std::vector<geometry_msgs::Polygon> convexHull(std::vector<geometry_msgs::Point>);


    /**
     * Constructs a path from currPos to goalPos while around any polygons in the way
     */
    static nav_msgs::Path constructPath(std::vector<geometry_msgs::Polygon>, geometry_msgs::Point currPos, geometry_msgs::Point goalPos);

};

#endif //PROJECT_REACTIVESYSTEMPATHFINDER_H
