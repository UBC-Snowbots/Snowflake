/*
 * Created By: Gareth Ellis
 * Created On: September 22, 2016
 * Description: Tests for LidarDecision and LidarObstacle
 */

#include <LidarObstacle.h>
#include <gtest/gtest.h>

class LidarObstacleTest : public testing::Test {
  protected:
    virtual void SetUp() {
        readings1 = {{4, 44}, {3, 99}, {6, 2}};
        readings2 = {{-1, 2}, {10, 4}, {-15, 6}};

        obstacle1 = LidarObstacle(0.1, 10);
        obstacle2 = LidarObstacle(0.2, 20);
        obstacle3 = LidarObstacle(0.15, 15);
        obstacle4 = LidarObstacle(readings1);
        obstacle5 = LidarObstacle(readings2);
    }

    std::vector<Reading> readings1, readings2;
    LidarObstacle obstacle1, obstacle2, obstacle3, obstacle4, obstacle5;
};

// Test all constructors to see if variables are set
// and default correctly
TEST_F(LidarObstacleTest, ContstructorMinWallLengthDefaultTest) {
    LidarObstacle empty_constructor;
    LidarObstacle min_length_constructor(50);
    LidarObstacle angle_distance_constructor(10, 20);
    LidarObstacle length_angle_constructor(51, 10, 20);
    LidarObstacle vector_constructor({Reading{10, 20}, Reading{30, 40}});
    LidarObstacle length_vector_constructor(52,
                                            {Reading{10, 20}, Reading{30, 40}});

    int default_min_wall_length = 1;
    EXPECT_EQ(default_min_wall_length, empty_constructor.getMinWallLength());
    EXPECT_EQ(50, min_length_constructor.getMinWallLength());
    EXPECT_EQ(default_min_wall_length,
              angle_distance_constructor.getMinWallLength());
    EXPECT_EQ(51, length_angle_constructor.getMinWallLength());
    EXPECT_EQ(default_min_wall_length, vector_constructor.getMinWallLength());
    EXPECT_EQ(52, length_vector_constructor.getMinWallLength());

    // CLion doesn't like these lines, but it compiles fine.
    EXPECT_EQ(NONE, empty_constructor.getObstacleType());
    EXPECT_EQ(NONE, min_length_constructor.getObstacleType());
}

TEST_F(LidarObstacleTest, ConstructorTest1) {
    auto readings = obstacle1.getAllLaserReadings();
    EXPECT_EQ(1, readings.size());
    EXPECT_EQ(10, readings[0].range);
    EXPECT_NEAR(0.1, readings[0].angle, 0.000001);
}

TEST_F(LidarObstacleTest, ConstructorTest2) {
    auto readings = obstacle4.getAllLaserReadings();
    // Check that the correct number of readings were input
    EXPECT_EQ(3, readings.size());
    // Check that they are in the correct order
    // (sorted in ascending order by angle)
    EXPECT_EQ(3, readings[0].angle);
    EXPECT_EQ(4, readings[1].angle);
    EXPECT_EQ(6, readings[2].angle);
}

TEST_F(LidarObstacleTest, getAvgAngleTest) {
    EXPECT_NEAR(0.1, obstacle1.getAvgAngle(), 0.00001);
    EXPECT_NEAR(4.333333333, obstacle4.getAvgAngle(), 0.00001);
    EXPECT_NEAR(-2, obstacle5.getAvgAngle(), 0.00001);
}

TEST_F(LidarObstacleTest, mergeInReadingsTest) {
    obstacle1.mergeInLidarObstacle(obstacle2);
    obstacle1.mergeInLidarObstacle(obstacle3);
    auto readings = obstacle1.getAllLaserReadings();
    // Check that the merge obstacle has the correct number of readings
    EXPECT_EQ(3, readings.size());
    // Check that they are in the correct order
    // (sorted in ascending order by angle)
    EXPECT_NEAR(0.1, readings[0].angle, 0.000001);
    EXPECT_NEAR(0.15, readings[1].angle, 0.000001);
    EXPECT_NEAR(0.2, readings[2].angle, 0.000001);
}

TEST_F(LidarObstacleTest, determineObstacleTypeTest) {
    LidarObstacle empty_obstacle = LidarObstacle();
    LidarObstacle cone_obstacle  = LidarObstacle(1, {{0, 1}, {M_PI / 6, 1}});
    LidarObstacle wall_obstacle  = LidarObstacle(1, {{0, 1}, {M_PI / 2, 1}});

    // CLion doesn't like this line, but it compiles fine.
    EXPECT_EQ(NONE, empty_obstacle.getObstacleType());
    EXPECT_EQ(CONE, cone_obstacle.getObstacleType());
    EXPECT_EQ(WALL, wall_obstacle.getObstacleType());

    // Change the NONE to a CONE
    empty_obstacle.mergeInLidarObstacle(cone_obstacle);
    EXPECT_EQ(CONE, empty_obstacle.getObstacleType());
}

TEST_F(LidarObstacleTest, getLengthTest) {
    LidarObstacle long_wall({{-M_PI / 4, 1}, {-M_PI / 6, 20}, {M_PI / 4, 1}});
    LidarObstacle cone({{0, 1}, {M_PI / 3, 1}});

    EXPECT_DOUBLE_EQ(std::sqrt(2), long_wall.getLength());
    EXPECT_DOUBLE_EQ(1, cone.getLength());
}

TEST_F(LidarObstacleTest, getReadingsAsPoints) {
    LidarObstacle test_obstacle({{-M_PI / 4, 0.5}, {M_PI * 0.7, 0.9}});
    std::vector<Point> test_points = {
    {0.3535533905932738, -0.35355339059327373},
    {-0.5290067270632258, 0.7281152949374528}};
    std::vector<Point> test_obstacle_points =
    test_obstacle.getReadingsAsPoints();
    EXPECT_DOUBLE_EQ(test_points[0].x, test_obstacle_points[0].x);
    EXPECT_DOUBLE_EQ(test_points[0].y, test_obstacle_points[0].y);
    EXPECT_DOUBLE_EQ(test_points[1].x, test_obstacle_points[1].x);
    EXPECT_DOUBLE_EQ(test_points[1].y, test_obstacle_points[1].y);
}

TEST_F(LidarObstacleTest, getCenterTest) {
    LidarObstacle test_obstacle = LidarObstacle(0, 1);
    Point center                = test_obstacle.getCenter();
    EXPECT_DOUBLE_EQ(1.0, center.x);
    EXPECT_DOUBLE_EQ(0.0, center.y);
    LidarObstacle merge_me = LidarObstacle(M_PI / 2, 1);
    test_obstacle.mergeInLidarObstacle(merge_me);
    center = test_obstacle.getCenter();
    EXPECT_DOUBLE_EQ(0.5, center.x);
    EXPECT_DOUBLE_EQ(0.5, center.y);
}

TEST(PointTest, distanceBetweenPointsTest) {
    Point p1{-0.35312, 341.1231251134}, p2{345345.242, -323423.23};

    EXPECT_DOUBLE_EQ(473378.21709845297, distanceBetweenPoints(p1, p2));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
