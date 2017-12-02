/*
 * Created By: Robyn Castro
 * Created On: December 1, 2017
 * Description: Tests for RvizUtils
 */


#include <RvizUtils.h>
#include <gtest/gtest.h>
#include <ros/init.h>

/**
 * Creates a geometry_msgs::Point with given x and y coordinates
 *
 * @param x the x coordinate of the point
 * @param y the y coordinate of the point
 * @param z the z coordinate of the point
 * @return a point with given x, y, and z coordinates
 */
geometry_msgs::Point createPoint(double x, double y, double z = 0) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

/**
 * Checks if the actual point is equivalent to the expected point
 *
 * @param point_expected the expected point
 * @param point_actual the actual point
 */
void EXPECT_POINT_EQ(geometry_msgs::Point point_expected, geometry_msgs::Point point_actual) {
    EXPECT_FLOAT_EQ(point_expected.x, point_actual.x);
    EXPECT_FLOAT_EQ(point_expected.y, point_expected.y);
    EXPECT_FLOAT_EQ(point_expected.z, point_expected.z);
}

/**
 * Checks if the actual point is equivalent to the expected point
 *
 * @param point_expected the expected point
 * @param point_actual the actual point
 */
void EXPECT_POINTS_EQ(std::vector<geometry_msgs::Point> points_expected,
                      std::vector<geometry_msgs::Point> points_actual) {
    EXPECT_EQ(points_expected.size(), points_actual.size());
    for (int i = 0; i < points_expected.size(); i++) {
        EXPECT_POINT_EQ(points_expected[i], points_actual[i]);
    }

}

TEST(createMarkerScale, smallCube) {
    float expected_x = 0.1;
    float expected_y = 0.1;
    float expected_z = 0.1;
    visualization_msgs::Marker::_scale_type scale_type = snowbots::rviz_utils::createrMarkerScale(expected_x, expected_y,
                                                                                       expected_z);
    EXPECT_FLOAT_EQ(expected_x, scale_type.x);
    EXPECT_FLOAT_EQ(expected_y, scale_type.y);
    EXPECT_FLOAT_EQ(expected_z, scale_type.z);
}

TEST(createMarkerScale, largeCube) {
    float expected_x = 10;
    float expected_y = 10;
    float expected_z = 10;
    visualization_msgs::Marker::_scale_type scale_type = snowbots::rviz_utils::createrMarkerScale(expected_x, expected_y,
                                                                                       expected_z);
    EXPECT_FLOAT_EQ(expected_x, scale_type.x);
    EXPECT_FLOAT_EQ(expected_y, scale_type.y);
    EXPECT_FLOAT_EQ(expected_z, scale_type.z);
}

TEST(createMarkerScale, zeroSizeShape) {
    float expected_x = 0;
    float expected_y = 0;
    float expected_z = 0;
    visualization_msgs::Marker::_scale_type scale_type = snowbots::rviz_utils::createrMarkerScale(expected_x, expected_y,
                                                                                       expected_z);
    EXPECT_FLOAT_EQ(expected_x, scale_type.x);
    EXPECT_FLOAT_EQ(expected_y, scale_type.y);
    EXPECT_FLOAT_EQ(expected_z, scale_type.z);
}

TEST(createMarkerColor, red) {
    float expected_r = 1.0f;
    float expected_g = 0;
    float expected_b = 0;
    float expected_a = 1.0;
    visualization_msgs::Marker::_color_type color_type = snowbots::rviz_utils::createMarkerColor(expected_r, expected_g,
                                                                                      expected_b, expected_a);
    EXPECT_FLOAT_EQ(expected_r, color_type.r);
    EXPECT_FLOAT_EQ(expected_g, color_type.g);
    EXPECT_FLOAT_EQ(expected_b, color_type.b);
    EXPECT_FLOAT_EQ(expected_a, color_type.a);
}

TEST(createMarkerColor, green) {
    float expected_r = 0;
    float expected_g = 1.0f;
    float expected_b = 0;
    float expected_a = 1.0f;
    visualization_msgs::Marker::_color_type color_type = snowbots::rviz_utils::createMarkerColor(expected_r, expected_g,
                                                                                      expected_b, expected_a);
    EXPECT_FLOAT_EQ(expected_r, color_type.r);
    EXPECT_FLOAT_EQ(expected_g, color_type.g);
    EXPECT_FLOAT_EQ(expected_b, color_type.b);
    EXPECT_FLOAT_EQ(expected_a, color_type.a);
}

TEST(createMarkerColor, blue) {
    float expected_r = 0;
    float expected_g = 0;
    float expected_b = 1.0f;
    float expected_a = 1.0f;
    visualization_msgs::Marker::_color_type color_type = snowbots::rviz_utils::createMarkerColor(expected_r, expected_g,
                                                                                      expected_b, expected_a);
    EXPECT_FLOAT_EQ(expected_r, color_type.r);
    EXPECT_FLOAT_EQ(expected_g, color_type.g);
    EXPECT_FLOAT_EQ(expected_b, color_type.b);
    EXPECT_FLOAT_EQ(expected_a, color_type.a);
}

TEST(createMarkerColor, rgb) {
    float expected_r = 1.0f;
    float expected_g = 1.0f;
    float expected_b = 1.0f;
    float expected_a = 1.0f;
    visualization_msgs::Marker::_color_type color_type = snowbots::rviz_utils::createMarkerColor(expected_r, expected_g,
                                                                                      expected_b, expected_a);
    EXPECT_FLOAT_EQ(expected_r, color_type.r);
    EXPECT_FLOAT_EQ(expected_g, color_type.g);
    EXPECT_FLOAT_EQ(expected_b, color_type.b);
    EXPECT_FLOAT_EQ(expected_a, color_type.a);
}

TEST(displayPoints, singlePositivePoint) {
    // Defaulting already tested parameters.
    visualization_msgs::Marker::_color_type color_type = snowbots::rviz_utils::createMarkerColor(0, 0, 0, 0);
    visualization_msgs::Marker::_scale_type scale_type = snowbots::rviz_utils::createrMarkerScale(0, 0, 0);

    // Setup frame_id and namespace
    std::string frame_id = "test";
    std::string ns = "ns";

    // Setup points
    geometry_msgs::Point positive_point = createPoint(10, 5);
    std::vector<geometry_msgs::Point> expected_points;
    expected_points.push_back(positive_point);

    // Testing displayPoints
    visualization_msgs::Marker test_marker = snowbots::rviz_utils::displayPoints(expected_points, color_type, scale_type, frame_id,
                                                                      ns);
    EXPECT_POINTS_EQ(expected_points, test_marker.points);
    EXPECT_TRUE(frame_id == test_marker.header.frame_id);
    EXPECT_TRUE(ns == test_marker.ns);

}

TEST(displayPoints, singleNegativePoint) {
    // Defaulting already tested parameters.
    visualization_msgs::Marker::_color_type color_type = snowbots::rviz_utils::createMarkerColor(0, 0, 0, 0);
    visualization_msgs::Marker::_scale_type scale_type = snowbots::rviz_utils::createrMarkerScale(0, 0, 0);

    // Setup frame_id and namespace
    std::string frame_id = "test";
    std::string ns = "ns";

    // Setup points
    geometry_msgs::Point positive_point = createPoint(-7, -1);
    std::vector<geometry_msgs::Point> expected_points;
    expected_points.push_back(positive_point);

    // Testing displayPoints
    visualization_msgs::Marker test_marker = snowbots::rviz_utils::displayPoints(expected_points, color_type, scale_type, frame_id,
                                                                      ns);
    EXPECT_POINTS_EQ(expected_points, test_marker.points);
    EXPECT_TRUE(frame_id == test_marker.header.frame_id);
    EXPECT_TRUE(ns == test_marker.ns);
}

TEST(displayPoints, multiplePoints) {
    // Defaulting already tested parameters.
    visualization_msgs::Marker::_color_type color_type = snowbots::rviz_utils::createMarkerColor(0, 0, 0, 0);
    visualization_msgs::Marker::_scale_type scale_type = snowbots::rviz_utils::createrMarkerScale(0, 0, 0);

    // Setup frame_id and namespace
    std::string frame_id = "test";
    std::string ns = "ns";

    // Setup points
    std::vector<geometry_msgs::Point> expected_points;
    expected_points.push_back(createPoint(-7, -1));
    expected_points.push_back(createPoint(5, -2));
    expected_points.push_back(createPoint(3, 7));
    expected_points.push_back(createPoint(-100, 9));

    // Testing displayPoints
    visualization_msgs::Marker test_marker = snowbots::rviz_utils::displayPoints(expected_points, color_type, scale_type, frame_id,
                                                                      ns);
    EXPECT_POINTS_EQ(expected_points, test_marker.points);
    EXPECT_TRUE(frame_id == test_marker.header.frame_id);
    EXPECT_TRUE(ns == test_marker.ns);
}

TEST(displayPoint, positivePoint) {
    // Defaulting already tested parameters.
    visualization_msgs::Marker::_color_type color_type = snowbots::rviz_utils::createMarkerColor(0, 0, 0, 0);
    visualization_msgs::Marker::_scale_type scale_type = snowbots::rviz_utils::createrMarkerScale(0, 0, 0);

    // Setup frame_id and namespace
    std::string frame_id = "test";
    std::string ns = "ns";

    // Setup point
    geometry_msgs::Point expected_point = createPoint(10, 5);
    std::vector<geometry_msgs::Point> expected_points;
    expected_points.push_back(expected_point);

    // Testing displayPoints
    visualization_msgs::Marker test_marker = snowbots::rviz_utils::displayPoint(expected_point, color_type, scale_type, frame_id,
                                                                      ns);
    EXPECT_POINTS_EQ(expected_points, test_marker.points);
    EXPECT_TRUE(frame_id == test_marker.header.frame_id);
    EXPECT_TRUE(ns == test_marker.ns);
}

TEST(displayPoint, negativePoint) {
    // Defaulting already tested parameters.
    visualization_msgs::Marker::_color_type color_type = snowbots::rviz_utils::createMarkerColor(0, 0, 0, 0);
    visualization_msgs::Marker::_scale_type scale_type = snowbots::rviz_utils::createrMarkerScale(0, 0, 0);

    // Setup frame_id and namespace
    std::string frame_id = "asdf";
    std::string ns = "xy";

    // Setup point
    geometry_msgs::Point expected_point = createPoint(-4, -9);
    std::vector<geometry_msgs::Point> expected_points;
    expected_points.push_back(expected_point);

    // Testing displayPoints
    visualization_msgs::Marker test_marker = snowbots::rviz_utils::displayPoint(expected_point, color_type, scale_type, frame_id,
                                                                     ns);
    EXPECT_POINTS_EQ(expected_points, test_marker.points);
    EXPECT_TRUE(frame_id == test_marker.header.frame_id);
    EXPECT_TRUE(ns == test_marker.ns);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rviz_utils_test");
    ros::start();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
