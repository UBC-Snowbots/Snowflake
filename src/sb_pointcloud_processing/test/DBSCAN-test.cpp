/*
 * Created By: Min Gyo Kim
 * Created On: January 27th 2018
 * Description: GTest for DBSCAN implementation
 */

#include "./TestUtils.h"
#include <DBSCAN.h>
#include <gtest/gtest.h>

TEST(DBSCAN, ClusterTwoNearPoints) {
    float min_neighbours = 1;
    float radius         = 5;
    DBSCAN dbscan(min_neighbours, radius);

    pcl::PointCloud<pcl::PointXYZ> pcl;

    //    Test two near points
    pcl::PointXYZ p1;
    p1.x = 1;
    p1.y = 1;
    pcl.push_back(p1);

    pcl::PointXYZ p2;
    p2.x = 1.1;
    p2.y = 1.1;
    pcl.push_back(p2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr = pcl.makeShared();

    vector<pcl::PointCloud<pcl::PointXYZ>> clusters =
    dbscan.findClusters(pcl_ptr);
    ASSERT_EQ(1, clusters.size());
    ASSERT_EQ(2, clusters[0].size());

    EXPECT_EQ(1, clusters[0][0].x);
    EXPECT_FLOAT_EQ(1.1, clusters[0][1].x);
}

TEST(DBSCAN, TestClusterTwoFarPoints) {
    int min_neighbours = 1;
    int radius         = 5;
    DBSCAN dbscan(min_neighbours, radius);

    pcl::PointCloud<pcl::PointXYZ> pcl;

    //    Test two far points
    pcl::PointXYZ p1;
    p1.x = 1;
    p1.y = 1;
    pcl.push_back(p1);

    pcl::PointXYZ p3;
    p3.x = 10;
    p3.y = 10;
    pcl.push_back(p3);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr = pcl.makeShared();

    vector<pcl::PointCloud<pcl::PointXYZ>> clusters =
    dbscan.findClusters(pcl_ptr);
    EXPECT_EQ(0, clusters.size());
}

TEST(DBSCAN, TestExpandCluster) {
    int min_neighbours = 2;
    int radius         = 5;
    DBSCAN dbscan(min_neighbours, radius);

    pcl::PointCloud<pcl::PointXYZ> pcl;

    pcl::PointXYZ p1;
    p1.x = 1;
    p1.y = 1;
    pcl.push_back(p1);

    pcl::PointXYZ p2;
    p2.x = 2;
    p2.y = 1;
    pcl.push_back(p2);

    pcl::PointXYZ p3;
    p3.x = 3;
    p3.y = 1;
    pcl.push_back(p3);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr = pcl.makeShared();

    vector<pcl::PointCloud<pcl::PointXYZ>> clusters =
    dbscan.findClusters(pcl_ptr);
    ASSERT_EQ(1, clusters.size());
    EXPECT_EQ(3, clusters[0].size());
}

TEST(DBSCAN, TestClusterTwoShortHorizontalLines) {
    int min_neighbours = 2;
    int radius         = 5;
    DBSCAN dbscan(min_neighbours, radius);

    pcl::PointCloud<pcl::PointXYZ> pcl;

    float x_min                = 0;
    float x_max                = 3;
    float x_delta              = 1;
    vector<float> coefficients = {10};
    LineExtractor::TestUtils::LineArgs args(
    coefficients, x_min, x_max, x_delta);

    // Add first line to PointCloud
    LineExtractor::TestUtils::addLineToPointCloud(args, pcl);

    // Add second line to PointCloud
    args.coefficients = {-10};
    LineExtractor::TestUtils::addLineToPointCloud(args, pcl);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr = pcl.makeShared();

    vector<pcl::PointCloud<pcl::PointXYZ>> clusters =
    dbscan.findClusters(pcl_ptr);
    ASSERT_EQ(2, clusters.size());
    EXPECT_EQ(4, clusters[0].size());
    EXPECT_EQ(4, clusters[1].size());
}

TEST(DBSCAN, TestClusterTwoSlopedLines) {
    int min_neighbours = 2;
    int radius         = 5;
    DBSCAN dbscan(min_neighbours, radius);

    pcl::PointCloud<pcl::PointXYZ> pcl;

    float x_min                = 0;
    float x_max                = 99;
    float x_delta              = 1;
    vector<float> coefficients = {100, 1};
    LineExtractor::TestUtils::LineArgs args(
    coefficients, x_min, x_max, x_delta);

    // Add first line to PointCloud
    LineExtractor::TestUtils::addLineToPointCloud(args, pcl);

    // Add second line to PointCloud
    args.coefficients = {-100, 1};
    LineExtractor::TestUtils::addLineToPointCloud(args, pcl);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr = pcl.makeShared();

    vector<pcl::PointCloud<pcl::PointXYZ>> clusters =
    dbscan.findClusters(pcl_ptr);
    ASSERT_EQ(2, clusters.size());
    EXPECT_EQ(LineExtractor::TestUtils::getNumPoints(args), clusters[0].size());
    EXPECT_EQ(LineExtractor::TestUtils::getNumPoints(args), clusters[1].size());
}

TEST(DBSCAN, TestClusterTwoSlopedLinesWithOutliers) {
    int min_neighbours = 2;
    int radius         = 5;
    DBSCAN dbscan(min_neighbours, radius);

    pcl::PointCloud<pcl::PointXYZ> pcl;

    float x_min                = 0;
    float x_max                = 99;
    float x_delta              = 1;
    vector<float> coefficients = {100, 1};
    LineExtractor::TestUtils::LineArgs args(
    coefficients, x_min, x_max, x_delta);

    // Add first line to PointCloud
    LineExtractor::TestUtils::addLineToPointCloud(args, pcl);

    // Add second line to PointCloud
    args.coefficients = {-100, 1};
    LineExtractor::TestUtils::addLineToPointCloud(args, pcl);

    //    outliers
    pcl.push_back(pcl::PointXYZ(999999, 999999, 0));
    pcl.push_back(pcl::PointXYZ(-999999, -999999, 0));

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr = pcl.makeShared();

    vector<pcl::PointCloud<pcl::PointXYZ>> clusters =
    dbscan.findClusters(pcl_ptr);
    ASSERT_EQ(2, clusters.size());
    EXPECT_EQ(LineExtractor::TestUtils::getNumPoints(args), clusters[0].size());
    EXPECT_EQ(LineExtractor::TestUtils::getNumPoints(args), clusters[1].size());
}

TEST(DBSCAN, TestClusterTwoLongHorizontalLines) {
    int min_neighbours = 1;
    int radius         = 5;
    DBSCAN dbscan(min_neighbours, radius);

    pcl::PointCloud<pcl::PointXYZ> pcl;

    float x_min                = -10;
    float x_max                = 489;
    float x_delta              = 1;
    vector<float> coefficients = {3};
    LineExtractor::TestUtils::LineArgs args(
    coefficients, x_min, x_max, x_delta);

    // Add first line to PointCloud
    LineExtractor::TestUtils::addLineToPointCloud(args, pcl);

    // Add second line to PointCloud
    args.coefficients = {-3};
    LineExtractor::TestUtils::addLineToPointCloud(args, pcl);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr = pcl.makeShared();

    std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters =
    dbscan.findClusters(pcl_ptr);
    ASSERT_EQ(2, clusters.size());
    EXPECT_EQ(LineExtractor::TestUtils::getNumPoints(args), clusters[0].size());
    EXPECT_EQ(LineExtractor::TestUtils::getNumPoints(args), clusters[1].size());
}

TEST(DBSCAN, TestClusterBorder) {
    int min_neighbours = 1;
    int radius         = 6;
    DBSCAN dbscan(min_neighbours, radius);

    pcl::PointCloud<pcl::PointXYZ> pcl;

    float x_min                = -10;
    float x_max                = -8;
    float x_delta              = 1;
    vector<float> coefficients = {3.000001};
    LineExtractor::TestUtils::LineArgs args(
    coefficients, x_min, x_max, x_delta);

    // Add first line to PointCloud
    LineExtractor::TestUtils::addLineToPointCloud(args, pcl);

    // Add second line to PointCloud
    args.coefficients = {-3};
    LineExtractor::TestUtils::addLineToPointCloud(args, pcl);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr = pcl.makeShared();

    vector<pcl::PointCloud<pcl::PointXYZ>> clusters =
    dbscan.findClusters(pcl_ptr);
    ASSERT_EQ(2, clusters.size());
    EXPECT_EQ(LineExtractor::TestUtils::getNumPoints(args), clusters[0].size());
    EXPECT_EQ(LineExtractor::TestUtils::getNumPoints(args), clusters[1].size());
}

TEST(DBSCAN, TestClusterTwoPolynomialLines) {
    int min_neighbours = 1;
    int radius         = 3;
    DBSCAN dbscan(min_neighbours, radius);

    pcl::PointCloud<pcl::PointXYZ> pcl;

    float x_min                = -20;
    float x_max                = 19;
    float x_delta              = 1;
    vector<float> coefficients = {10, 0, 0, 0.002};
    LineExtractor::TestUtils::LineArgs args(
    coefficients, x_min, x_max, x_delta);

    // Add first line to PointCloud
    LineExtractor::TestUtils::addLineToPointCloud(args, pcl);

    // Add second line to PointCloud
    args.coefficients = {-10, 0, 0, 0.002};
    LineExtractor::TestUtils::addLineToPointCloud(args, pcl);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr = pcl.makeShared();

    vector<pcl::PointCloud<pcl::PointXYZ>> clusters =
    dbscan.findClusters(pcl_ptr);
    ASSERT_EQ(2, clusters.size());
    EXPECT_EQ(LineExtractor::TestUtils::getNumPoints(args), clusters[0].size());
    EXPECT_EQ(LineExtractor::TestUtils::getNumPoints(args), clusters[1].size());
}

TEST(DBSCAN, TestClusterOnePolynomialLinesWithNoise) {
    int min_neighbours = 1;
    int radius         = 80;
    DBSCAN dbscan(min_neighbours, radius);

    pcl::PointCloud<pcl::PointXYZ> pcl;

    float x_min                = 0;
    float x_max                = 99;
    float x_delta              = 1;
    vector<float> coefficients = {1000, 7, -0.7, 0.007};
    LineExtractor::TestUtils::LineArgs args(
    coefficients, x_min, x_max, x_delta);

    // Add first line to PointCloud
    LineExtractor::TestUtils::addLineToPointCloud(args, pcl);

    float true_min, true_max;
    LineExtractor::TestUtils::getMinAndMaxOfPointCloud(true_min, true_max, pcl);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr = pcl.makeShared();

    vector<pcl::PointCloud<pcl::PointXYZ>> clusters =
    dbscan.findClusters(pcl_ptr);

    ASSERT_EQ(clusters.size(), 1);

    float actual_min, actual_max;
    LineExtractor::TestUtils::getMinAndMaxOfPointCloud(
    actual_min, actual_max, clusters[0]);

    EXPECT_EQ(LineExtractor::TestUtils::getNumPoints(args), clusters[0].size());
    EXPECT_FLOAT_EQ(true_min, actual_min);
    EXPECT_FLOAT_EQ(true_max, actual_max);
}

TEST(DBSCAN, TestClusterTwoPolynomialLinesWithNoise) {
    int min_neighbours = 1;
    int radius         = 80;
    DBSCAN dbscan(min_neighbours, radius);

    pcl::PointCloud<pcl::PointXYZ> pcl;

    float x_min                = 0;
    float x_max                = 99;
    float x_delta              = 1;
    vector<float> coefficients = {1000, 7, -0.7, 0.007};
    LineExtractor::TestUtils::LineArgs args(
    coefficients, x_min, x_max, x_delta);

    // Add first line to PointCloud
    LineExtractor::TestUtils::addLineToPointCloud(args, pcl);

    // Add second line to PointCloud
    args.coefficients = {-1000, 7, -0.7, 0.007};
    LineExtractor::TestUtils::addLineToPointCloud(args, pcl);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr = pcl.makeShared();

    vector<pcl::PointCloud<pcl::PointXYZ>> clusters =
    dbscan.findClusters(pcl_ptr);
    ASSERT_EQ(2, clusters.size());
    EXPECT_EQ(LineExtractor::TestUtils::getNumPoints(args), clusters[0].size());
    EXPECT_EQ(LineExtractor::TestUtils::getNumPoints(args), clusters[1].size());
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}