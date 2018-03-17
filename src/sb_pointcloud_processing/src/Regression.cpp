/*
 * Created By: Min Gyo Kim
 * Created On: February 3, 2018
 * Description: Calculates line of best fit for each cluster of points
 */

#include <Regression.h>

std::vector<Eigen::VectorXf> Regression::getLinesOfBestFit(
std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters,
unsigned int poly_degree,
float lambda) {
    std::vector<Eigen::VectorXf> lines;

    // Calculate line of best fit for each cluster
    for (unsigned int i = 0; i < clusters.size(); i++) {
        lines.push_back(getLineOfCluster(clusters[i], poly_degree, lambda));
    }

    return lines;
}

Eigen::VectorXf
Regression::getLineOfCluster(pcl::PointCloud<pcl::PointXYZ> cluster,
                             unsigned int poly_degree,
                             float lambda) {
    unsigned int n = cluster.size();

    /*
     * Linear Equation to solve:
     * X' * X + lambda * I = X' * y
     *
     * X is a matrix of size (n,d) where n is the number of points in the
     * cluster,
     * and d is the polynomial degree + 1. (+1 for linear bias)
     * Each row of X represents the x coordinate of a point.
     * Given x for the x coordinate of a point and column for the column index
     * in the matrix X:
     * The first column of X is 1 for linear bias, and the rest of the columns
     * contain the value of pow(x, column-1).
     *
     * lambda is the regularization parameter. The higher this number,
     * The higher the penalty on the size of the coefficients of the
     * mathematical lines.
     *
     * y is a column vector of size (n), where n is the number of points in the
     * cluster.
     * Each row of the vector corresponds to the y coordinate of a point.
     */

    Eigen::MatrixXf X(n, poly_degree + 1);
    Eigen::VectorXf y(n);

    for (unsigned int i = 0; i < cluster.size(); i++) {
        pcl::PointXYZ point = cluster[i];

        X.row(i) = constructRow(point.x, poly_degree);

        y(i) = point.y;
    }

    Eigen::MatrixXf regularization(poly_degree + 1, poly_degree + 1);
    regularization.setIdentity();
    regularization *= lambda;

    Eigen::MatrixXf left(X.transpose() * X + regularization);
    Eigen::MatrixXf right(X.transpose() * y);

    Eigen::VectorXf line(n);
    line = (left).ldlt().solve(right);

    return line;
}

Eigen::VectorXf Regression::constructRow(float x, unsigned int poly_degree) {
    Eigen::VectorXf row(poly_degree + 1);

    // linear bias
    row(0) = 1;

    // non-linear
    for (unsigned int j = 1; j < poly_degree + 1; j++) { row(j) = pow(x, j); }

    return row;
}