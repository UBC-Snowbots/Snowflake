/**
 * Created by William Gu on Nov 17 2018
 *
 * This node declares functions used in a modified Density Based Clustering algorithm
 * This differs from a traditional DBScan in that the minimum adjacent polygons needed to expand a cluster is only 1,
 * and that proximity of polygons is based on the minimum distance between two
 */

#ifndef PROJECT_DBSCANPOLY_H
#define PROJECT_DBSCANPOLY_H

#include <math.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>

class DBScanPoly{
public:

    //Main entrypoint, note that min_adj_polys should be 1
    static std::vector<std::vector<geometry_msgs::Polygon>> findClusters(std::vector<geometry_msgs::Polygon> polygons, double dist_tol);


    //Helper for expanding clusters
    static void expand(std::vector<geometry_msgs::Polygon> &polygons, std::vector<geometry_msgs::Polygon> &cur_cluster, geometry_msgs::Polygon target_poly, int seed_index, std::vector<bool> &is_clustered, double dist_tol);

    //Helper
    static bool arePolygonsClose(geometry_msgs::Polygon poly1, geometry_msgs::Polygon poly2, double dist_tol);

};




#endif //PROJECT_DBSCANPOLY_H
