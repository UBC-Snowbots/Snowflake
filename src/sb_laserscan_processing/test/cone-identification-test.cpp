/*
 * Created By: William Gu
 * Created On: March 31 2018
 * Description: GTest for ConeIdentification Implementation
 */

#include <ConeIdentification.h>
#include "./LaserscanBuilder.h"
#include <gtest/gtest.h>

/*
TEST(ConeIdentification, testRegressionSlope1) {
    std::vector<mapping_igvc::Point2D>edge_points;
    mapping_igvc::Point2D p1; p1.x = 0; p1.y = 0;
    mapping_igvc::Point2D p2; p2.x = 1; p2.y = 1;
    mapping_igvc::Point2D p3; p3.x = 2; p3.y = 2;
    edge_points.push_back(p1);
    edge_points.push_back(p2);
    edge_points.push_back(p3);

    double slope = ConeIdentification::getRegressionSlope(edge_points);
    EXPECT_NEAR(1.0, slope, 0.001);
}

TEST(ConeIdentification, testRegressionSlope2) {
    std::vector<mapping_igvc::Point2D>edge_points;
    mapping_igvc::Point2D p1; p1.x = 0; p1.y = 0;
    mapping_igvc::Point2D p2; p2.x = 1; p2.y = 1.1;
    mapping_igvc::Point2D p3; p3.x = 2; p3.y = 2.3;
    mapping_igvc::Point2D p4; p4.x = 3; p4.y = 3.5;
    mapping_igvc::Point2D p5; p5.x = 4; p5.y = 4.9;
    mapping_igvc::Point2D p6; p6.x = 5; p6.y = 6.0;
    edge_points.push_back(p1);
    edge_points.push_back(p2);
    edge_points.push_back(p3);
    edge_points.push_back(p4);
    edge_points.push_back(p5);
    edge_points.push_back(p6);

    double slope = ConeIdentification::getRegressionSlope(edge_points);
    EXPECT_NEAR(1.217, slope, 0.02);
}*/

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
    EXPECT_NEAR(cone.center.x, 0, 0.01);
    EXPECT_NEAR(cone.center.y, 2, 0.01);
    EXPECT_NEAR(cone.radius, 2, 0.01);
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
    EXPECT_NEAR(cone.center.x, 0, 0.01);
    EXPECT_NEAR(cone.center.y, 2, 0.01);
    EXPECT_NEAR(cone.radius, 2, 0.01);
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

// Test identifyCones with no valid cones in image (there is one valid cone but out of range)
TEST(ConeIdentification, coneOutRange) {
    float dist_tol = 0.01;
    float radius_exp = 1.0;
    float radius_tol = 0.05;
    int min_points_in_cone = 5;
    double ang_threshold = 2.3; //For splitting

    sensor_msgs::LaserScan laser_msg;
    LaserscanBuilder::LaserscanBuilder builder;
    builder.addCone(7, 0, 1); //x y radius
    laser_msg = builder.getLaserscan();

    std::vector<mapping_igvc::ConeObstacle> cones = ConeIdentification::identifyCones(laser_msg, dist_tol, radius_exp, radius_tol, min_points_in_cone, ang_threshold);

    EXPECT_TRUE(cones.empty());
}

// Test identifyCones with no valid cones in image (there is one object with radius too large, of 1.5m instead of 1m)
TEST(ConeIdentification, coneTooLarge) {
    float dist_tol = 0.01;
    float radius_exp = 1.0;
    float radius_tol = 0.05;
    int min_points_in_cone = 5;
    double ang_threshold = 2.3;

    sensor_msgs::LaserScan laser_msg;
    LaserscanBuilder::LaserscanBuilder builder;
    builder.addCone(3, 0, 1.5); //x y radius
    laser_msg = builder.getLaserscan();

    std::vector<mapping_igvc::ConeObstacle> cones = ConeIdentification::identifyCones(laser_msg, dist_tol, radius_exp, radius_tol, min_points_in_cone, ang_threshold);

    EXPECT_TRUE(cones.empty());
}

//Test onevalidcone partially out of range
TEST(ConeIdentification, oneValidCone) {
    float dist_tol = 0.1;
    float radius_exp = 1.5;
    float radius_tol = 0.05;
    int min_points_in_cone = 3;
    double ang_threshold = 2.3;

    sensor_msgs::LaserScan laser_msg;
    LaserscanBuilder::LaserscanBuilder builder;
    builder.addCone(0, 3, 1.5); //x y radius
    laser_msg = builder.getLaserscan();

    std::vector<mapping_igvc::ConeObstacle> cones = ConeIdentification::identifyCones(laser_msg, dist_tol, radius_exp, radius_tol, min_points_in_cone, ang_threshold);

    std::cout<<cones.size()<<std::endl;

    EXPECT_NEAR(cones[0].radius, 1.5, 0.01);
    EXPECT_NEAR(cones[0].center.x, 0, 0.05);
    EXPECT_NEAR(cones[0].center.y, 3.0, 0.05);
}

//Test identifyCones with 2 cones NOT in a cluster (they are seperate from each other)
TEST(ConeIdentification, twoValidCones){
    float dist_tol = 0.1;
    float radius_exp = 1.0;
    float radius_tol = 0.05;
    int min_points_in_cone = 3;
    double ang_threshold = 2.3;

    sensor_msgs::LaserScan laser_msg;
    LaserscanBuilder::LaserscanBuilder builder;
    builder.addCone(3, 3, 1); //x y radius
    builder.addCone (3, -3, 1);
    laser_msg = builder.getLaserscan();

    std::vector<mapping_igvc::ConeObstacle> cones = ConeIdentification::identifyCones(laser_msg, dist_tol, radius_exp, radius_tol, min_points_in_cone, ang_threshold);

    EXPECT_NEAR(cones[0].radius, 1.0, 0.01);
    EXPECT_NEAR(cones[0].center.x, 3.0, 0.01);
    EXPECT_NEAR(cones[0].center.y, -3.0, 0.01);

    EXPECT_NEAR(cones[1].radius, 1.0, 0.01);
    EXPECT_NEAR(cones[1].center.x, 3.0, 0.01);
    EXPECT_NEAR(cones[1].center.y, 3.0, 0.01);
}

//Test identifyCones with 4 cones NOT in a cluster (they are seperate from each other)
TEST(ConeIdentification, fourValidCones){
    float dist_tol = 0.1;
    float radius_exp = 0.5;
    float radius_tol = 0.1;
    int min_points_in_cone = 5;
    double ang_threshold = 2.3;

    sensor_msgs::LaserScan laser_msg;
    LaserscanBuilder::LaserscanBuilder builder;

    builder.addCone(3, 1, 0.5); //x y radius
    builder.addCone(3, -1, 0.5);
    builder.addCone(0, 3, 0.5);
    builder.addCone (0, -3, 0.5); //ISSUE WITH THIS (Appears to be bug with detecting cones along the negative y line)
    laser_msg = builder.getLaserscan();

    std::vector<mapping_igvc::ConeObstacle> cones = ConeIdentification::identifyCones(laser_msg, dist_tol, radius_exp, radius_tol, min_points_in_cone, ang_threshold);

    EXPECT_NEAR(cones[0].radius, 0.5, 0.01);
    EXPECT_NEAR(cones[0].center.x, 0, 0.1);
    EXPECT_NEAR(cones[0].center.y, -3.0, 0.1);

    EXPECT_NEAR(cones[1].radius, 0.5, 0.01);
    EXPECT_NEAR(cones[1].center.x, 3.0, 0.1);
    EXPECT_NEAR(cones[1].center.y, -1.0, 0.1);

    EXPECT_NEAR(cones[2].radius, 0.5, 0.01);
    EXPECT_NEAR(cones[2].center.x, 3.0, 0.1);
    EXPECT_NEAR(cones[2].center.y, 1.0, 0.1);

    EXPECT_NEAR(cones[3].radius, 0.5, 0.01);
    EXPECT_NEAR(cones[3].center.x, 0.0, 0.1);
    EXPECT_NEAR(cones[3].center.y, 3.0, 0.1);
}

//Test identifyCones with 2 cones in a cluster (their laserscan points overlap)
TEST(ConeIdentification, twoOverlappingCones){
    float dist_tol = 1;
    float radius_exp = 1.0;
    float radius_tol = 0.05;
    int min_points_in_cone = 5;
    double ang_threshold = 2.3; //Low as possible

    sensor_msgs::LaserScan laser_msg;
    LaserscanBuilder::LaserscanBuilder builder;
    builder.addCone(3, 0.5, 1); //x y radius
    builder.addCone (3, -0.5, 1);
    laser_msg = builder.getLaserscan();

    std::vector<mapping_igvc::ConeObstacle> cones = ConeIdentification::identifyCones(laser_msg, dist_tol, radius_exp, radius_tol, min_points_in_cone, ang_threshold);

    EXPECT_NEAR(cones[0].radius, 1.0, 0.01);
    EXPECT_NEAR(cones[0].center.x, 3.0, 0.01);
    EXPECT_NEAR(cones[0].center.y, -0.5, 0.01);

    EXPECT_NEAR(cones[1].radius, 1.0, 0.01);
    EXPECT_NEAR(cones[1].center.x, 3.0, 0.01);
    EXPECT_NEAR(cones[1].center.y, 0.5, 0.01);
}

//Test identifyCones with 3 cones in a cluster (their laserscan points overlap)
TEST(ConeIdentification, threeOverlappingCones){
    float dist_tol = 0.1;
    float radius_exp = 1.0;
    float radius_tol = 0.15;
    int min_points_in_cone = 5;
    double ang_threshold = 2.3;

    sensor_msgs::LaserScan laser_msg;
    LaserscanBuilder::LaserscanBuilder builder;
    builder.addCone(3.5, 0, 1); //x y radius
    builder.addCone(2, -1.3, 1); //x y radius
    builder.addCone (2, 1.3, 1);
    laser_msg = builder.getLaserscan();

    std::vector<mapping_igvc::ConeObstacle> cones = ConeIdentification::identifyCones(laser_msg, dist_tol, radius_exp, radius_tol, min_points_in_cone, ang_threshold);

    EXPECT_NEAR(cones[0].radius, 1.0, 0.01);
    EXPECT_NEAR(cones[0].center.x, 2.0, 0.03);
    EXPECT_NEAR(cones[0].center.y, -1.3, 0.03);

    EXPECT_NEAR(cones[1].radius, 1.0, 0.01);
    EXPECT_NEAR(cones[1].center.x, 3.5, 0.03);
    EXPECT_NEAR(cones[1].center.y, 0, 0.03);

    EXPECT_NEAR(cones[2].radius, 1.0, 0.01);
    EXPECT_NEAR(cones[2].center.x, 2.0, 0.03);
    EXPECT_NEAR(cones[2].center.y, 1.3, 0.03);
}

//Test identifyCones with 2 cones in a cluster (their laserscan points "connect", horizontal relative to the robot)
TEST(ConeIdentification, twoConnectedConesHorizontal){
    float dist_tol = 0.1;
    float radius_exp = 1.0;
    float radius_tol = 0.05;
    int min_points_in_cone = 5;
    double ang_threshold = 2.3; //Low as possible

    sensor_msgs::LaserScan laser_msg;
    LaserscanBuilder::LaserscanBuilder builder;
    builder.addCone(3, 1, 1); //x y radius
    builder.addCone (3, -1, 1);
    laser_msg = builder.getLaserscan();

    std::vector<mapping_igvc::ConeObstacle> cones = ConeIdentification::identifyCones(laser_msg, dist_tol, radius_exp, radius_tol, min_points_in_cone, ang_threshold);

    EXPECT_NEAR(cones[0].radius, 1.0, 0.01);
    EXPECT_NEAR(cones[0].center.x, 3.0, 0.03);
    EXPECT_NEAR(cones[0].center.y, -1, 0.03);

    EXPECT_NEAR(cones[1].radius, 1.0, 0.01);
    EXPECT_NEAR(cones[1].center.x, 3.0, 0.03);
    EXPECT_NEAR(cones[1].center.y, 1, 0.03);
}

//Test identifyCones with 2 cones in a diagonal orientation (though not physically connected)
TEST(ConeIdentification, twoConesDiagonal){
    float dist_tol = 0.1;
    float radius_exp = 1.0;
    float radius_tol = 0.2;
    int min_points_in_cone = 5;
    double ang_threshold = 2.3; //Low as possible

    sensor_msgs::LaserScan laser_msg;
    LaserscanBuilder::LaserscanBuilder builder;
    builder.addCone(2, 1, 1); //x y radius
    builder.addCone (3, -1, 1);
    laser_msg = builder.getLaserscan();

    std::vector<mapping_igvc::ConeObstacle> cones = ConeIdentification::identifyCones(laser_msg, dist_tol, radius_exp, radius_tol, min_points_in_cone, ang_threshold);

    EXPECT_NEAR(cones[0].radius, 1.0, 0.01);
    EXPECT_NEAR(cones[0].center.x, 3.0, 0.03);
    EXPECT_NEAR(cones[0].center.y, -1, 0.03);

    EXPECT_NEAR(cones[1].radius, 1.0, 0.01);
    EXPECT_NEAR(cones[1].center.x, 2.0, 0.08); //large-ish error here
    EXPECT_NEAR(cones[1].center.y, 1, 0.05);
}

int main(int argc, char** argv) {
    ros::Time::init();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
