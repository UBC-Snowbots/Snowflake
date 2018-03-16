/*
 * Created By: Min Gyo Kim
 * Created On: February 3, 2018
 * Description: Tests calculation of line of best fit
 */

#include "./TestUtils.h"
#include <Regression.h>
#include <gtest/gtest.h>

TEST(Regression, OnePerfectLinearFit) {
    // Setup PointCloud parameters
    unsigned int poly_degree = 1;

    float x_min                     = 0;
    float x_max                     = 99;
    float x_delta                   = 1;
    std::vector<float> coefficients = {100, 1};
    LineExtractor::TestUtils::LineArgs args(
    coefficients, x_min, x_max, x_delta);

    // Generate a single PointCloud
    pcl::PointCloud<pcl::PointXYZ> pcl;
    LineExtractor::TestUtils::addLineToPointCloud(args, pcl);

    // Perform Regression
    std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
    clusters.push_back(pcl);

    std::vector<Eigen::VectorXf> lines =
    Regression::getLinesOfBestFit(clusters, poly_degree);

    // Check results
    ASSERT_EQ(lines.size(), 1);
    ASSERT_EQ(lines[0].size(), coefficients.size());

    for (unsigned int i = 0; i < coefficients.size(); i++) {
        EXPECT_FLOAT_EQ(lines[0](i), coefficients[i]);
    }
}

TEST(Regression, MultiplePerfectLinearFits) {
    // Setup PointCloud parameters
    unsigned int poly_degree = 1;

    float x_min                                           = 0;
    float x_max                                           = 999;
    float x_delta                                         = 0.1;
    std::vector<std::vector<float>> coefficients_per_line = {
    {5, -2}, {10, 7}, {-99, -10}};

    unsigned int num_lines = coefficients_per_line.size();

    // Generate multiple PointClouds
    std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;

    for (unsigned int i = 0; i < num_lines; i++) {
        LineExtractor::TestUtils::LineArgs args(
        coefficients_per_line[i], x_min, x_max, x_delta);
        pcl::PointCloud<pcl::PointXYZ> pcl;
        LineExtractor::TestUtils::addLineToPointCloud(args, pcl);

        clusters.push_back(pcl);
    }

    // Preform Regression
    std::vector<Eigen::VectorXf> lines =
    Regression::getLinesOfBestFit(clusters, poly_degree, 10);

    // Check results
    for (unsigned int i = 0; i < num_lines; i++) {
        Eigen::VectorXf line = lines[i];
        ASSERT_EQ(line.size(), coefficients_per_line[i].size());

        for (unsigned int j = 0; j < line.size(); j++) {
            EXPECT_NEAR(line(j), coefficients_per_line[i][j], 1);
        }
    }
}

TEST(Regression, OnePerfectNonLinearFit) {
    // Setup PointCloud paramters
    unsigned int poly_degree = 3;

    float x_min                     = 0;
    float x_max                     = 99;
    float x_delta                   = 1;
    std::vector<float> coefficients = {100, 7, -0.7, 0.007};
    LineExtractor::TestUtils::LineArgs args(
    coefficients, x_min, x_max, x_delta);

    // Generate PointCloud
    pcl::PointCloud<pcl::PointXYZ> pcl;
    LineExtractor::TestUtils::addLineToPointCloud(args, pcl);

    // Perform Regression
    std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
    clusters.push_back(pcl);

    std::vector<Eigen::VectorXf> lines =
    Regression::getLinesOfBestFit(clusters, poly_degree);

    // Check results
    ASSERT_EQ(lines.size(), 1);
    Eigen::VectorXf line = lines[0];

    ASSERT_EQ(line.size(), coefficients.size());

    for (unsigned int i = 0; i < line.size(); i++) {
        EXPECT_NEAR(line(i), coefficients[i], 1);
    }
}

TEST(Regression, OneNonLinearFitWithNoise) {
    // Setup PointCloud parameters
    unsigned int poly_degree = 3;

    float x_min                     = 0;
    float x_max                     = 99;
    float x_delta                   = 1;
    std::vector<float> coefficients = {1000, 7, -0.7, 0.007};
    LineExtractor::TestUtils::LineArgs args(
    coefficients, x_min, x_max, x_delta);

    float max_noise_x = 1;
    float max_noise_y = 1;

    // Generate a single PointCloud with noise
    pcl::PointCloud<pcl::PointXYZ> pcl;
    LineExtractor::TestUtils::addLineToPointCloud(
    args, pcl, max_noise_x, max_noise_y);

    // Perform Regression
    std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
    clusters.push_back(pcl);

    std::vector<Eigen::VectorXf> lines =
    Regression::getLinesOfBestFit(clusters, poly_degree);

    // Check results
    ASSERT_EQ(lines.size(), 1);
    Eigen::VectorXf line = lines[0];

    ASSERT_EQ(line.size(), coefficients.size());

    for (unsigned int i = 0; i < line.size(); i++) {
        float tol;

        // logic to allow more tolerance for y-intercept
        if (i) {
            tol = 5;
        } else {
            tol = 10;
        }

        EXPECT_NEAR(line(i), coefficients[i], tol);
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}