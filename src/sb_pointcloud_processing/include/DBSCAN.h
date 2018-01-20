/*
 * Created By: Min Gyo Kim
 * Created On: January 20, 2018
 * Description: Class declaration for DBSCAN (Density-based clustering)
 */

#ifndef LINE_EXTRACTOR_IGVC_DBSCAN_H
#define LINE_EXTRACTOR_IGVC_DBSCAN_H

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <tr1/unordered_map>

using namespace std;
using namespace std::tr1;

class DBSCAN {
    /*
     * This variable stores the PointCloud input that we want to cluster
     */
    pcl::PointCloud<pcl::PointXYZ> _pcl;

    /*
     * This variable stores the PointCloud clusters output
     */
    vector<pcl::PointCloud<pcl::PointXYZ>> _clusters;

    /*
     * Key: index of a point in the PointCloud
     * Value: true if the point has already been clustered, false otherwise
     */
    unordered_map<unsigned int, bool> _clustered;

    /*
     * Key: index of a point in the PointCloud
     * Value: true if the point has already been expanded, false otherwise
     */
    unordered_map<unsigned int, bool> _expanded;

    /*
     * Key: index of a point in the PointCloud
     * Value: a vector containing all of the point's neighbors
     * (A neighbour is a point that is within @_radius of a point of interest)
     */
    unordered_map<unsigned int, vector<unsigned int>> _neighbors;

    // TODO: fine-tune parameters with real data
    int _min_neighbors = 5;
    float _radius      = 5;

  public:
    /*
     * Constructor:
     * Takes in minimum number of neighbours and radius as parameters
     */
    DBSCAN(int min_neighbours = 5, float radius = 5);

    /*
     * Main entry function:
     * Given a PointCloud, clusters the PointCloud into a vector of smaller
     * PointClouds
     */
    vector<pcl::PointCloud<pcl::PointXYZ>>
    findClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr);

    void setMinNeighbours(int new_min_neighour);
    void setRadius(float new_radius);

  private:
    double dist(pcl::PointXYZ p1, pcl::PointXYZ p2);
    bool isPointVisited(unsigned int p_index);
    bool isPointExpanded(unsigned int p_index);

    /*
     * Finds all the neighbours of each point in the PointCloud
     * (A neighbour is a point that is within @_radius of a point of interest)
     */
    void findNeighbors();

    /*
     * Expands a cluster around a given point recursively by:
     * 1. Adding all of the point's neighbors to the same cluster as the point
     * (unless they already belong to a cluster)
     * 2. Expand recursively around each neighbor that is a core point
     * (unless the neighbor has already been expanded)
     */
    void expand(unsigned int center_point_index,
                pcl::PointCloud<pcl::PointXYZ>& cluster);

    /*
     * Given the index of a point in the PointCloud, determines whether the
     * point is a core point
     * A core point is a point that has at least @_min_neighbors within
     * @_radius.
     */
    bool isCore(unsigned int center_point_index);
};

#endif // PROJECT_DBSCAN_H
