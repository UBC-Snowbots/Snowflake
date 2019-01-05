#include "RvizUtils.h"
#include <gtest/gtest.h>

/**
 * This is the helper class which will publish and subscribe messages which will
 * test the node being instantiated
 * It contains at the minimum:
 *      publisher - publishes the input to the node
 *      subscriber - publishes the output of the node
 *      callback function - the callback function which corresponds to the
 * subscriber
 *      getter function - to provide a way for gtest to check for equality of
 * the message recieved
 */
class RvizUtilsRosTest : public testing::Test {
  protected:
    ros::NodeHandle nh;

    // This class doesn't publish or subscribe currently, so no initialisation
    // necessary.
    ros::Publisher test_publisher;
    ros::Subscriber test_subscriber;
};

template <typename T1, typename T2>
void EXPECT_POINT_EQ(std::vector<T1> points1, std::vector<T2> points2);

geometry_msgs::Point32 initialisePoint32(float x, float y, float z) {
    geometry_msgs::Point32 point;
    point.x = x;
    point.y = y;
    point.z = z;

    return point;
}

geometry_msgs::Point initialisePoint(float x, float y, float z) {
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;

    return point;
}

TEST_F(RvizUtilsRosTest, boxPolygonMarker) {
    std::string frame_id = "camera_color_optical_frame";
    std::string ns       = "debug";

    geometry_msgs::Polygon polygon;
    polygon.points.push_back(initialisePoint32(0, 0, 0));
    polygon.points.push_back(initialisePoint32(1, 0, 0));
    polygon.points.push_back(initialisePoint32(0, 1, 0));
    polygon.points.push_back(initialisePoint32(1, 1, 0));

    visualization_msgs::Marker risk_area_marker =
            snowbots::RvizUtils::createPolygonMarker3D(
                    polygon,
                    snowbots::RvizUtils::createMarkerColor(1.0f, 0, 0, 1.0f),
                    snowbots::RvizUtils::createMarkerScale(0.1, 0, 0),
                    frame_id,
                    ns);

    // Make sure message metadata is correct
    EXPECT_EQ(frame_id, risk_area_marker.header.frame_id);
    EXPECT_EQ(ns, risk_area_marker.ns);

    EXPECT_POINT_EQ(polygon.points, risk_area_marker.points);
}

TEST_F(RvizUtilsRosTest, trianglePolygonMarker) {
    std::string frame_id = "camera_color_optical_frame";
    std::string ns       = "debug";

    geometry_msgs::Polygon polygon;
    polygon.points.push_back(initialisePoint32(0, 0, 0));
    polygon.points.push_back(initialisePoint32(1, 0, 0));
    polygon.points.push_back(initialisePoint32(0, 1, 0));

    visualization_msgs::Marker risk_area_marker =
            snowbots::RvizUtils::createPolygonMarker3D(
                    polygon,
                    snowbots::RvizUtils::createMarkerColor(1.0f, 0, 0, 1.0f),
                    snowbots::RvizUtils::createMarkerScale(0.1, 0, 0),
                    frame_id,
                    ns);

    // Make sure message metadata is correct
    EXPECT_EQ(frame_id, risk_area_marker.header.frame_id);
    EXPECT_EQ(ns, risk_area_marker.ns);

    EXPECT_POINT_EQ(polygon.points, risk_area_marker.points);
}

TEST_F(RvizUtilsRosTest, singlePointMarker) {
    std::string frame_id = "camera_color_optical_frame";
    std::string ns       = "debug";

    geometry_msgs::Point point = initialisePoint(0, 0, 1);

    visualization_msgs::Marker risk_area_marker =
    snowbots::RvizUtils::createMarker(
    initialisePoint(0, 0, 1),
    snowbots::RvizUtils::createMarkerColor(1.0f, 0, 0, 1.0f),
    snowbots::RvizUtils::createMarkerScale(0.1, 0, 0),
    frame_id,
    ns);

    // Make sure message metadata is correct
    EXPECT_EQ(frame_id, risk_area_marker.header.frame_id);
    EXPECT_EQ(ns, risk_area_marker.ns);

    EXPECT_EQ(point.x, risk_area_marker.points[0].x);
    EXPECT_EQ(point.y, risk_area_marker.points[0].y);
    EXPECT_EQ(point.z, risk_area_marker.points[0].z);
}

TEST_F(RvizUtilsRosTest, singlePointMarkerInArray) {
    std::string frame_id = "camera_color_optical_frame";
    std::string ns       = "debug";

    std::vector<geometry_msgs::Point> points;
    points.push_back(initialisePoint(0, 0, 1));

    visualization_msgs::Marker risk_area_marker =
    snowbots::RvizUtils::createMarker(
    points,
    snowbots::RvizUtils::createMarkerColor(1.0f, 0, 0, 1.0f),
    snowbots::RvizUtils::createMarkerScale(0.1, 0, 0),
    frame_id,
    ns);

    // Make sure message metadata is correct
    EXPECT_EQ(frame_id, risk_area_marker.header.frame_id);
    EXPECT_EQ(ns, risk_area_marker.ns);

    EXPECT_POINT_EQ(points, risk_area_marker.points);
}

TEST_F(RvizUtilsRosTest, multiplePointMarkersInArray) {
    std::string frame_id = "camera_color_optical_frame";
    std::string ns       = "debug";

    std::vector<geometry_msgs::Point> points;
    points.push_back(initialisePoint(0, 0, 1));
    points.push_back(initialisePoint(0, 0, 1));
    points.push_back(initialisePoint(0, -8, 1));
    points.push_back(initialisePoint(100, 0, 1));

    visualization_msgs::Marker risk_area_marker =
    snowbots::RvizUtils::createMarker(
    points,
    snowbots::RvizUtils::createMarkerColor(1.0f, 0, 0, 1.0f),
    snowbots::RvizUtils::createMarkerScale(0.1, 0, 0),
    frame_id,
    ns);

    // Make sure message metadata is correct
    EXPECT_EQ(frame_id, risk_area_marker.header.frame_id);
    EXPECT_EQ(ns, risk_area_marker.ns);

    EXPECT_POINT_EQ(points, risk_area_marker.points);
}

TEST_F(RvizUtilsRosTest, createMarkerScale) {
    float x = 0;
    float y = -1;
    float z = 1;

    visualization_msgs::Marker::_scale_type scale;
    scale = snowbots::RvizUtils::createMarkerScale(x, y, z);

    EXPECT_FLOAT_EQ(x, scale.x);
    EXPECT_FLOAT_EQ(y, scale.y);
    EXPECT_FLOAT_EQ(z, scale.z);
}

TEST_F(RvizUtilsRosTest, createMarkerColor) {
    float r = 0;
    float g = 0.5f;
    float b = 1.0f;
    float a = 1.0f;

    visualization_msgs::Marker::_color_type color;
    color = snowbots::RvizUtils::createMarkerColor(r, g, b, a);

    EXPECT_FLOAT_EQ(r, color.r);
    EXPECT_FLOAT_EQ(g, color.g);
    EXPECT_FLOAT_EQ(b, color.b);
    EXPECT_FLOAT_EQ(a, color.a);
}

template <typename T1, typename T2>
void EXPECT_POINT_EQ(std::vector<T1> points1, std::vector<T2> points2) {
    // Check for equal size
    EXPECT_EQ(points1.size(), points2.size());

    if (points1.size() == points2.size()) {
        for (int i = 0; i < points1.size(); i++) {
            EXPECT_EQ(points1[i].x, points2[i].x);
            EXPECT_EQ(points1[i].y, points2[i].y);
            EXPECT_EQ(points1[i].z, points2[i].z);
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rviz_utils_rostest");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}