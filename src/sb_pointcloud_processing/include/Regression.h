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

    // TODO: Test me
    /**
     * Gets a line of best fit for each given cluster and the standard error for that line
     *
     * @param clusters the clusters to get lines of best fit for
     * @param poly_degree the degree of polynomial to fit to each cluster
     * @param lambda Regularization parameter (Default: 0)
     *
     * @return a vector of pairs of polynomial lines (represented as a vector
     * of ascending coefficients), and the associated standard error of the fit
     * that the line resulted from
     */
    static std::vector<std::pair<Eigen::VectorXf, double>>
    getLinesOfBestFitWithStandardError(std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters,
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

    // TODO: Test me
    /**
     * Computes the standard error of a given polynomial for a given cluster
     *
     * standard_error = sqrt(sum(p.y - y(x) for p in cluster) / n) where:
     * - n is the number of points in the cluster
     * - y is the polynomial line
     *
     * @param cluster
     * @param poly a polynomial represented as a vector of ascending coefficients
     *
     * @return the standard error of `poly` for `cluster`
     */
    static double getStandardError(pcl::PointCloud<pcl::PointXYZ> cluster,
    Eigen::VectorXf poly);

    /*
     * Constructs a row in the matrix X given a data point and degree of
     * polynomial
     */
    static Eigen::VectorXf constructRow(float x, unsigned int poly_degree);
};

#endif // PROJECT_REGRESSION_H
