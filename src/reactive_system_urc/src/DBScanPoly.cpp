//
// Created by william on 17/11/18.
//

#include <DBScanPoly.h>


std::vector<std::vector<geometry_msgs::Polygon>> DBScanPoly::findClusters(std::vector<geometry_msgs::Polygon> polygons, double dist_tol){

    //NOTE: for now, assume min_adj_polys == 1

    std::vector<bool> is_clustered(polygons.size(), false); //is_clustered[i] represents if polygons[i] is part of a cluster, initialized to all false

    std::vector<std::vector<geometry_msgs::Polygon>> clusters; //container for final clusters

    //Identify seed polygons to begin clusters
    for (int i=0; i<polygons.size(); i++) {
        if (!is_clustered[i]){

            //Add seed polygon to a cluster
            is_clustered[i] = true;
            std::vector<geometry_msgs::Polygon> cluster;
            geometry_msgs::Polygon seed_poly = polygons[i];
            cluster.push_back(seed_poly);

            expand(polygons, cluster, seed_poly, i, is_clustered, dist_tol);

            clusters.push_back(cluster); //Done recursive search for neighbors, form a complete cluster
        }
    }


    return clusters;
}

//Parameters for helper func (should all be pass by reference):
//Curr cluster
//Index of seed (we do not have to search indices before the seed or at the seed)
//Array of clustered

void DBScanPoly::expand(std::vector<geometry_msgs::Polygon> &polygons, std::vector<geometry_msgs::Polygon> &cur_cluster, geometry_msgs::Polygon target_poly, int seed_index, std::vector<bool> &is_clustered, double dist_tol) {

    //Find neighbors of target_polygon
    std::vector<geometry_msgs::Polygon> neighbors;
    for (int i = seed_index + 1; i < polygons.size(); i++) {
        if (!is_clustered[i] && arePolygonsClose(target_poly, polygons[i], dist_tol)) { //Neighbor found
            is_clustered[i] = true;
            neighbors.push_back(polygons[i]);
            cur_cluster.push_back(polygons[i]);
        }
    }

    //For each neighbor, recursively search for new neighbors:
    for (geometry_msgs::Polygon new_target_poly : neighbors) {
        expand(polygons, cur_cluster, new_target_poly, seed_index, is_clustered, dist_tol);
    }
}



//Helper for determining if two polygons are in close proximity:
bool DBScanPoly::arePolygonsClose(geometry_msgs::Polygon poly1, geometry_msgs::Polygon poly2, double dist_tol){

    //TODO: change this to a true min distance between polygons
    for (geometry_msgs::Point32 point1 : poly1.points){
        for (geometry_msgs::Point32 point2 : poly2.points){

            if (pow((point1.x - point2.x), 2) + pow((point1.y - point2.y), 2) <= pow(dist_tol, 2))
                return true;

        }
    }

    return false;

}