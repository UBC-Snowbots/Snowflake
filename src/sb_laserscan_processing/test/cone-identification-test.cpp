/*
 * Created By: William Gu
 * Created On: March 31 2018
 * Description: GTest for ConeIdentification Implementation
 */

#include <ConeIdentification.h>
#include <gtest/gtest.h>

// Test 3 points->cone in an edge (forming a semi circle) - points are evenly distributed
TEST(ConeIdentification, edgeToCone3Points) {
    std::vector<mapping_igvc::Point2D> edge_points;
    mapping_igvc::Point2D p1; p1.x = -2; p1.y = 2;
    mapping_igvc::Point2D p2; p2.x = 0; p2.y = 0;
    mapping_igvc::Point2D p3; p3.x = 2; p3.y = 2;
    edge_points.push_back(p1);
    edge_points.push_back(p2);
    edge_points.push_back(p3);

    mapping_igvc::ConeObstacle cone = ConeIdentification::edgeToCone(edge_points);
    EXPECT_FLOAT_EQ(cone.center.x, 0);
    EXPECT_FLOAT_EQ(cone.center.y, 2);
    EXPECT_FLOAT_EQ(cone.radius, 2);
}

// Test 6 points->cone in an edge (forming a semi circle) - points are symmetrical about y axis
TEST(ConeIdentification, edgeToCone6Points) {
    std::vector<mapping_igvc::Point2D> edge_points;
    mapping_igvc::Point2D p1; p1.x = -2; p1.y = 2;
    mapping_igvc::Point2D p2; p2.x = -1.73; p2.y = 1;
    mapping_igvc::Point2D p3; p3.x = -1; p3.y = 0.268;
    mapping_igvc::Point2D p4; p4.x = 1; p4.y = 0.268;
    mapping_igvc::Point2D p5; p5.x = 1.73; p5.y = 1;
    mapping_igvc::Point2D p6; p6.x = 2; p6.y = 2;
    edge_points.push_back(p1);
    edge_points.push_back(p2);
    edge_points.push_back(p3);
    edge_points.push_back(p4);
    edge_points.push_back(p5);
    edge_points.push_back(p6);

    mapping_igvc::ConeObstacle cone = ConeIdentification::edgeToCone(edge_points);
    EXPECT_NEAR(cone.center.x, 0, 0.001);
    EXPECT_NEAR(cone.center.y, 2, 0.001);
    EXPECT_NEAR(cone.radius, 2, 0.001);
}

// Test 4 points->cone in an edge (forming a quarter circle/arc) - points are evenly distributed
TEST(ConeIdentification, edgeToConeArc) {
    std::vector<mapping_igvc::Point2D> edge_points;
    mapping_igvc::Point2D p1; p1.x = 0; p1.y = 0;
    mapping_igvc::Point2D p2; p2.x = 1; p2.y = 0.268;
    mapping_igvc::Point2D p3; p3.x = 1.73; p3.y = 1;
    mapping_igvc::Point2D p4; p4.x = 2; p4.y = 2;
    edge_points.push_back(p1);
    edge_points.push_back(p2);
    edge_points.push_back(p3);
    edge_points.push_back(p4);

    mapping_igvc::ConeObstacle cone = ConeIdentification::edgeToCone(edge_points);
    EXPECT_NEAR(cone.center.x, 0, 0.001);
    EXPECT_NEAR(cone.center.y, 2, 0.001);
    EXPECT_NEAR(cone.radius, 2, 0.001);
}

// Test 3 points->cone forming very short arc
TEST(ConeIdentification, shortArc) {
    std::vector<mapping_igvc::Point2D> edge_points;
    mapping_igvc::Point2D p1; p1.x = 0; p1.y = 0;
    mapping_igvc::Point2D p2; p2.x = 0.07; p2.y = 0.001;
    mapping_igvc::Point2D p3; p3.x = 0.13; p3.y = 0.004;
    edge_points.push_back(p1);
    edge_points.push_back(p2);
    edge_points.push_back(p3);

    mapping_igvc::ConeObstacle cone = ConeIdentification::edgeToCone(edge_points);
    EXPECT_NEAR(cone.center.x, 0, 0.2);
    EXPECT_NEAR(cone.center.y, 2, 0.2);
    EXPECT_NEAR(cone.radius, 2, 0.2);
}

// Test unevenly distributed points->cone in an arc
TEST(ConeIdentification, unevenArc) {
    std::vector<mapping_igvc::Point2D> edge_points;
    mapping_igvc::Point2D p1; p1.x = -2; p1.y = 2;
    mapping_igvc::Point2D p2; p2.x = -1.99; p2.y = 1.8;
    mapping_igvc::Point2D p3; p3.x = -1.936; p3.y = 1.498;
    mapping_igvc::Point2D p4; p4.x = -1.845; p4.y = 1.228;
    mapping_igvc::Point2D p5; p5.x = 0; p5.y = 0;
    mapping_igvc::Point2D p6; p6.x = 1.014; p6.y = 0.276;
    edge_points.push_back(p1);
    edge_points.push_back(p2);
    edge_points.push_back(p3);
    edge_points.push_back(p4);
    edge_points.push_back(p5);
    edge_points.push_back(p6);

    mapping_igvc::ConeObstacle cone = ConeIdentification::edgeToCone(edge_points);
    EXPECT_NEAR(cone.center.x, 0, 0.001);
    EXPECT_NEAR(cone.center.y, 2, 0.001);
    EXPECT_NEAR(cone.radius, 2, 0.001);
}

// Test unordered points->cone (not in order they appear in edge)
TEST(ConeIdentification, unorderedArc) {
    std::vector<mapping_igvc::Point2D> edge_points;
    mapping_igvc::Point2D p1; p1.x = -2; p1.y = 2;
    mapping_igvc::Point2D p2; p2.x = -1.73; p2.y = 1;
    mapping_igvc::Point2D p3; p3.x = -1; p3.y = 0.268;
    mapping_igvc::Point2D p4; p4.x = 1; p4.y = 0.268;
    mapping_igvc::Point2D p5; p5.x = 1.73; p5.y = 1;
    mapping_igvc::Point2D p6; p6.x = 2; p6.y = 2;
    edge_points.push_back(p3);
    edge_points.push_back(p1);
    edge_points.push_back(p6);
    edge_points.push_back(p5);
    edge_points.push_back(p4);
    edge_points.push_back(p2);

    mapping_igvc::ConeObstacle cone = ConeIdentification::edgeToCone(edge_points);
    EXPECT_NEAR(cone.center.x, 0, 0.01);
    EXPECT_NEAR(cone.center.y, 2, 0.01);
    EXPECT_NEAR(cone.radius, 2, 0.01);
}

// Test inaccurate points->cone (all points off slightly)
TEST(ConeIdentification, arcWithNoise) {
    std::vector<mapping_igvc::Point2D> edge_points;
    mapping_igvc::Point2D p1; p1.x = -2.02; p1.y = 2.01;
    mapping_igvc::Point2D p2; p2.x = -1.74; p2.y = 0.98;
    mapping_igvc::Point2D p3; p3.x = -0.99; p3.y = 0.275;
    mapping_igvc::Point2D p4; p4.x = 1.01; p4.y = 0.268;
    mapping_igvc::Point2D p5; p5.x = 1.75; p5.y = 0.97;
    mapping_igvc::Point2D p6; p6.x = 2.04; p6.y = 2.01;
    edge_points.push_back(p1);
    edge_points.push_back(p2);
    edge_points.push_back(p3);
    edge_points.push_back(p4);
    edge_points.push_back(p5);
    edge_points.push_back(p6);

    mapping_igvc::ConeObstacle cone = ConeIdentification::edgeToCone(edge_points);
    EXPECT_NEAR(cone.center.x, 0, 0.1);
    EXPECT_NEAR(cone.center.y, 2, 0.1);
    EXPECT_NEAR(cone.radius, 2, 0.1);
}

// Test identifyCones with no valid cones in image
TEST(ConeIdentification, noValidCones) {
    float dist_tol = 1;

    sensor_msgs::LaserScan laser_msg;
    laser_msg.angle_max = 2.0; //in radians
    laser_msg.angle_min = -2.0;
    laser_msg.angle_increment = 0.5;
    laser_msg.range_max = 10.0;
    laser_msg.range_min = 0.0;
    laser_msg.ranges = {10.0, 0.5, 10.0, 0.5, 10.0, 0.5, 10.0, 0.5, 10.0}; //Should have 9 ranges

    std::vector<mapping_igvc::ConeObstacle> cones = identifyCones(laser_msg, dist_tol);

}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}