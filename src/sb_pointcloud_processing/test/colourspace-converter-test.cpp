#include "ColourspaceConverter.h"
#include <gtest/gtest.h>

using namespace pcl;

TEST(ColourspaceConverter, convertPointclouds) {
    ColourspaceConverter c = ColourspaceConverter();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input(
    new pcl::PointCloud<pcl::PointXYZRGB>());
    input->height = 1;
    input->width  = 4;

    input->points.resize(input->height * input->width);

    PointXYZRGB black;
    black.r = black.g = black.b = 0;
    black.x = black.y = black.z = 0;
    input->points[0]            = black;

    PointXYZRGB white;
    white.r = white.g = white.b = 255;
    white.x = white.y = white.z = 1;
    input->points[1]            = white;

    PointXYZRGB green;
    green.r = 45;
    green.g = 174;
    green.b = 45;
    green.x = green.y = green.z = 2;
    input->points[2]            = green;

    PointXYZRGB orange;
    orange.r = 243;
    orange.g = 136;
    orange.b = 35;
    orange.x = orange.y = orange.z = 3;
    input->points[3]               = orange;

    c.setInputCloud(input);

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr output(
    new pcl::PointCloud<pcl::PointXYZHSV>());
    c.convert(*output);

    int comparisons = 0;
    for (auto& point : output->points) {
        if (point.z == 0) {
            // White
            ASSERT_EQ(0, point.h);
            ASSERT_EQ(0, point.s);
            ASSERT_EQ(0, point.v);
            comparisons++;
        } else if (point.z == 1) {
            // Black
            ASSERT_EQ(0, point.h);
            ASSERT_EQ(0, point.s);
            ASSERT_EQ(1, point.v);
            comparisons++;
        } else if (point.z == 2) {
            // Green
            ASSERT_NEAR(120, point.h, 1);
            ASSERT_NEAR(0.741, point.s, 0.001);
            ASSERT_NEAR(0.682, point.v, 0.001);
            comparisons++;
        } else if (point.z == 3) {
            // Orange
            ASSERT_NEAR(29, point.h, 1);
            ASSERT_NEAR(0.856, point.s, 0.001);
            ASSERT_NEAR(0.953, point.v, 0.001);
            comparisons++;
        }
    }
    ASSERT_EQ(4, comparisons);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}