/**
 * Created by William Gu on Nov 3 2018
 * Declarations for Reactive System Pathfinder
 * This node is responsible for moving the robot towards a given GPS goal while avoiding risky areas
 */


#ifndef PROJECT_REACTIVESYSTEMPATHFINDER_H
#define PROJECT_REACTIVESYSTEMPATHFINDER_H

#include <DBScanPoly.h>
#include <ConvexHull.h>
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
     * risk_threshold refers to the minimum risk score for polygons to be placed in a cluster
     * cluster_dist_tol refers to the max distance between two polygons in a cluster
     */
    static nav_msgs::Path pathFinder(mapping_msgs_urc::RiskAreaArray risk_areas, geometry_msgs::Point32 curr_pos, geometry_msgs::Point32 goal_pos, double risk_threshold, double cluster_dist_tol, double point_dist_tol);


    /**
     * Filters risk polygons above some risk value
     */
    static std::vector<geometry_msgs::Polygon> filterRiskPolygons(mapping_msgs_urc::RiskAreaArray& risk_areas, double risk_threshold);


    /**
     * Identify adjacent and overlapping polygons and group them into clusters -> main logic is located in DBScanPoly
     */
    static std::vector<std::vector<geometry_msgs::Polygon>> clusterRiskPolygons(std::vector<geometry_msgs::Polygon> polygons, double cluster_dist_tol);


    /**
     * Converts clusters of polygons into clusters of points, removing redundant points in the process
     */
    static std::vector<std::vector<geometry_msgs::Point32>> polygonClustersToPoints(std::vector<std::vector<geometry_msgs::Polygon>> polygon_clusters, double point_dist_tol);


    /**
     * Performs convex hull algorithm on each cluster of points, returning a vector of resultant polygons (size of resultant polygons should equal number of clusters entered)
     */
    static std::vector<geometry_msgs::Polygon> massConvexHull(std::vector<std::vector<geometry_msgs::Point32>> point_clusters);


    //May need to perform outward polygon offsetting/polygon expansion to account for robot size (doesn't get too close to edges)
    //Could use a convex hull heurestic/approximation like this: http://geomalgorithms.com/a11-_hull-2.html
    //Or could build offsetting into convex hull algorithm somehow


    /**
     * Constructs a path from currPos to goalPos avoiding any polygons in the way
     */
    static nav_msgs::Path constructPath(std::vector<geometry_msgs::Polygon> polygon_obstacles, geometry_msgs::Point32 curr_pos, geometry_msgs::Point32 goal_pos);




    /* HELPER FUNCTIONS */

    /**
     * Returns true if points are within dist_tol (only checks x and y for fast computations)
     */
    static bool closePoints(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2, double dist_tol);
};

#endif //PROJECT_REACTIVESYSTEMPATHFINDER_H
