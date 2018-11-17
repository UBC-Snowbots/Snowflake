/**
 * Created by William Gu on Nov 3 2018
 * Implementation for Reactive System Pathfinder
 */

#include <ReactiveSystemPathfinder.h>

nav_msgs::Path ReactiveSystemPathfinder::pathFinder(mapping_msgs_urc::RiskAreaArray risk_areas, geometry_msgs::Point32 curr_pos, geometry_msgs::Point32 goal_pos, double risk_threshold, double cluster_threshold) {

    //Pipeline for obtaining path
    std::vector<geometry_msgs::Polygon> polygons = filterRiskPolygons(risk_areas, risk_threshold);
    std::vector<std::vector<geometry_msgs::Polygon>> clustered_polygons = clusterRiskPolygons(polygons, cluster_threshold);
    std::vector<std::vector<geometry_msgs::Point32>> clustered_points = polygonClustersToPoints(clustered_polygons);
    std::vector<geometry_msgs::Polygon> polygon_obstacles = massConvexHull(clustered_points);
    return constructPath(polygon_obstacles, curr_pos, goal_pos);

}


std::vector<geometry_msgs::Polygon> ReactiveSystemPathfinder::filterRiskPolygons(mapping_msgs_urc::RiskAreaArray& risk_areas, double risk_threshold) {
    std::vector<geometry_msgs::Polygon> polygons;
    for (mapping_msgs_urc::RiskArea risk_area : risk_areas.areas){
        if (risk_area.score.data > risk_threshold){
            polygons.push_back(risk_area.area);
        }
    }
    return polygons;
}


std::vector<std::vector<geometry_msgs::Polygon>> ReactiveSystemPathfinder::clusterRiskPolygons(std::vector<geometry_msgs::Polygon> polygons, double cluster_threshold){

    return DBScanPoly::findClusters(polygons, cluster_threshold);

}



std::vector<std::vector<geometry_msgs::Point32>> ReactiveSystemPathfinder::polygonClustersToPoints(std::vector<std::vector<geometry_msgs::Polygon>> polygon_clusters) {



}


std::vector<geometry_msgs::Polygon> ReactiveSystemPathfinder::massConvexHull(std::vector<std::vector<geometry_msgs::Point32>> point_clusters) {

}


nav_msgs::Path ReactiveSystemPathfinder::constructPath(std::vector<geometry_msgs::Polygon> polygon_obstacles, geometry_msgs::Point32 curr_pos, geometry_msgs::Point32 goal_pos){

}


std::vector<geometry_msgs::Polygon> ReactiveSystemPathfinder::convexHull(std::vector<geometry_msgs::Point32> points){

}


