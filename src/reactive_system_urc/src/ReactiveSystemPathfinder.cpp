/**
 * Created by William Gu on Nov 3 2018
 * Implementation for Reactive System Pathfinder
 */

#include <ReactiveSystemPathfinder.h>

nav_msgs::Path ReactiveSystemPathfinder::pathFinder(mapping_msgs_urc::RiskAreaArray risk_areas, geometry_msgs::Point32 curr_pos, geometry_msgs::Point32 goal_pos, double risk_threshold, double cluster_dist_tol, double point_dist_tol) {

    //Pipeline for obtaining path
    std::vector<geometry_msgs::Polygon> polygons = filterRiskPolygons(risk_areas, risk_threshold);
    std::vector<std::vector<geometry_msgs::Polygon>> clustered_polygons = clusterRiskPolygons(polygons, cluster_dist_tol);
    std::vector<std::vector<geometry_msgs::Point32>> clustered_points = polygonClustersToPoints(clustered_polygons, point_dist_tol);
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


std::vector<std::vector<geometry_msgs::Polygon>> ReactiveSystemPathfinder::clusterRiskPolygons(std::vector<geometry_msgs::Polygon> polygons, double cluster_dist_tol){

    return DBScanPoly::findClusters(polygons, cluster_dist_tol);

}



std::vector<std::vector<geometry_msgs::Point32>> ReactiveSystemPathfinder::polygonClustersToPoints(std::vector<std::vector<geometry_msgs::Polygon>> polygon_clusters, double point_dist_tol) {

    std::vector<std::vector<geometry_msgs::Point32>> point_clusters;

    for (std::vector<geometry_msgs::Polygon> cluster : polygon_clusters) {

        std::vector<geometry_msgs::Point32> point_cluster;

        for (geometry_msgs::Polygon poly : cluster) {
            for (geometry_msgs::Point32 point: poly.points){

                //Check for close enough duplicates in each cluster
                bool duplicate_point = false;
                for (geometry_msgs::Point32 existing_point : point_cluster){
                    if (closePoints(existing_point, point, point_dist_tol)){
                        duplicate_point = true;
                        break;
                    }
                }
                if (!duplicate_point)
                    point_cluster.push_back(point);

            }
        }

        point_clusters.push_back(point_cluster);
    }

    return point_clusters;

}


std::vector<geometry_msgs::Polygon> ReactiveSystemPathfinder::massConvexHull(std::vector<std::vector<geometry_msgs::Point32>> point_clusters) {

    std::vector<geometry_msgs::Polygon> convex_hulls;

    for (std::vector<geometry_msgs::Point32> point_cluster : point_clusters) {

        geometry_msgs::Polygon convex_hull = ConvexHull::convexHull(point_cluster);
        convex_hulls.push_back(convex_hull);
    }

    return convex_hulls;

}


nav_msgs::Path ReactiveSystemPathfinder::constructPath(std::vector<geometry_msgs::Polygon> polygon_obstacles, geometry_msgs::Point32 curr_pos, geometry_msgs::Point32 goal_pos){

}



bool ReactiveSystemPathfinder::closePoints (geometry_msgs::Point32 p1, geometry_msgs::Point32 p2, double dist_tol){

    return abs(p1.x - p2.x) <= dist_tol && abs(p1.y - p2.y) <= dist_tol;

}