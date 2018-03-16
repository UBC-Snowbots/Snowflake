/*
 * Created By: Min Gyo Kim
 * Created On: January 20, 2018
 * Description: Class declaration for Regression, which calculates
 *              line of best fit for each cluster of points
 */

#ifndef LINE_EXTRACTOR_IGVC_REGRESSION_H
#define LINE_EXTRACTOR_IGVC_REGRESSION_H

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// using namespace Eigen;
// using namespace std;
// using namespace pcl;

class Regression {
  public:
    /*
     * Returns a std::vector of Eigen::VectorXf
     * Each Eigen::VectorXf corresponds to the line of best fit of a
     * PointCloud<PointXYZ> cluster
     * The corresponding vector and cluster have the same index within each of
     * their vectors.
     * @poly_degree: Degree of polynomial of the line of best fit
     * @lambda: Regularization parameter (Default: 0)
     */
    static std::vector<Eigen::VectorXf>
    getLinesOfBestFit(std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters,
                      unsigned int poly_degree,
                      float lambda = 0);

  private:
    /*
     * Returns a line of best fit given a cluster
     */
    static Eigen::VectorXf
    getLineOfCluster(pcl::PointCloud<pcl::PointXYZ> cluster,
                     unsigned int poly_degree,
                     float lambda = 0);

    /*
     * Constructs a row in the matrix X given a data point and degree of
     * polynomial
     */
    static Eigen::VectorXf constructRow(float x, unsigned int poly_degree);
};

#endif // PROJECT_REGRESSION_H
