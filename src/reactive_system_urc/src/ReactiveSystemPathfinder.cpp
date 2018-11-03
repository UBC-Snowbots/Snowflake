/**
 * Created by William Gu on Nov 3 2018
 * Implementation for Reactive System Pathfinder
 */

#include <ReactiveSystemPathfinder.h>

nav_msgs::Path ReactiveSystemPathfinder::pathFinder(mapping_msgs_urc::RiskAreaArray risk_areas, geometry_msgs::Point currPos, geometry_msgs::Point goalPos, std_msgs::Float64 risk_threshold) {

    std::vector<geometry_msgs::Polygon> riskyPolygons = filterRiskPolygons(risk_areas, risk_threshold);


    

}


std::vector<geometry_msgs::Polygon> ReactiveSystemPathfinder::filterRiskPolygons(mapping_msgs_urc::RiskAreaArray& risk_areas, std_msgs::Float64 risk_threshold) {

}


std::vector<std::vector<geometry_msgs::Polygon>> ReactiveSystemPathfinder::clusterRiskPolygons(mapping_msgs_urc::RiskAreaArray& risk_areas, std::vector<geometry_msgs::Polygon>){

}


std::vector<std::vector<geometry_msgs::Point>> ReactiveSystemPathfinder::polygonClustersToPoints(std::vector<std::vector<geometry_msgs::Polygon>>) {

}


std::vector<geometry_msgs::Polygon> ReactiveSystemPathfinder::massConvexHull(std::vector<std::vector<geometry_msgs::Point>>) {

}


std::vector<geometry_msgs::Polygon> ReactiveSystemPathfinder::convexHull(std::vector<geometry_msgs::Point>){

}


nav_msgs::Path ReactiveSystemPathfinder::constructPath(std::vector<geometry_msgs::Polygon>, geometry_msgs::Point currPos, geometry_msgs::Point goalPos){

}


