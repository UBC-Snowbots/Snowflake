/*
 * Created By: Gareth Ellis
 * Created On: September 22, 2016
 * Description: Tests for LidarDecision and LidarObstacle
 */

#include <LidarDecision.h>
#include <gtest/gtest.h>

class LidarObstacleTest : public testing::Test {
protected:
    virtual void SetUp(){
        readings1 = {{4,44}, {3,99}, {6,2}};
        readings2 = {{-1,2}, {10,4}, {-15,6}};

        obstacle1 = LidarObstacle(0.1, 10);
        obstacle2 = LidarObstacle(0.2, 20);
        obstacle3 = LidarObstacle(0.15, 15);
        obstacle4 = LidarObstacle(readings1);
        obstacle5 = LidarObstacle(readings2);
    }

    std::vector<Reading> readings1, readings2;
    LidarObstacle obstacle1, obstacle2, obstacle3, obstacle4, obstacle5;
};

TEST_F(LidarObstacleTest, ConstructorTest1){
    auto readings = obstacle1.getAllLaserReadings();
    EXPECT_EQ(1, readings.size());
    EXPECT_EQ(10, readings[0].range);
    EXPECT_NEAR(0.1, readings[0].angle, 0.000001);
}

TEST_F(LidarObstacleTest, ConstructorTest2){
    auto readings = obstacle4.getAllLaserReadings();
    // Check that the correct number of readings were input
    EXPECT_EQ(3, readings.size());
    // Check that they are in the correct order
    // (sorted in ascending order by angle)
    EXPECT_EQ(3, readings[0].angle);
    EXPECT_EQ(4, readings[1].angle);
    EXPECT_EQ(6, readings[2].angle);
}

TEST_F(LidarObstacleTest, getAvgAngleTest){
    EXPECT_NEAR(0.1, obstacle1.getAvgAngle(), 0.00001);
    EXPECT_NEAR(4.333333333, obstacle4.getAvgAngle(), 0.00001);
    EXPECT_NEAR(-2, obstacle5.getAvgAngle(), 0.00001);
}

TEST_F(LidarObstacleTest, mergeInReadingsTest){
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

TEST_F(LidarObstacleTest, dangerScoreTest){
    EXPECT_NEAR((cos(0.1) + 1.0/10), obstacle1.dangerScore(), 0.000001);
    EXPECT_NEAR((cos(0.2) + 1.0/20), obstacle2.dangerScore(), 0.000001);
    EXPECT_NEAR((cos(4.333333333) + 1.0/2), obstacle4.dangerScore(), 0.000001);
}

class LidarDecisionTest : public testing::Test {
protected:
    virtual void SetUp(){
        // Create our fake lidar scan1
        ulong num_rays = 300;
        scan1.angle_min = 0;
        scan1.angle_max = (float)M_PI;
        scan1.angle_increment = (scan1.angle_max - scan1.angle_min)/num_rays;
        // Set all the ranges to 0 initially
        scan1.ranges = std::vector<float>(num_rays, 0);
        scan1.range_min = 20;
        scan1.range_max = 40;
        // Add the obstacles
        std::fill(scan1.ranges.begin(), scan1.ranges.begin()+10, 36.123);       //obstacle 1
        std::fill(scan1.ranges.begin()+10, scan1.ranges.begin()+20, 38.123);    //obstacle 2
        std::fill(scan1.ranges.begin()+100, scan1.ranges.begin()+105, 24.123);  //obstacle 3
        std::fill(scan1.ranges.begin()+105, scan1.ranges.begin()+110, 26.123);  //obstacle 4
        std::fill(scan1.ranges.begin()+125, scan1.ranges.begin()+130, 25.123);  //obstacle 5
        std::fill(scan1.ranges.begin()+200, scan1.ranges.begin()+240, 32);      //obstacle 6
        // Make some ranges outside the min and max of the scan
        std::fill(scan1.ranges.begin()+110, scan1.ranges.begin()+125, 45.123);
        std::fill(scan1.ranges.begin()+50, scan1.ranges.begin()+70, 4.123);

    // Create our fake lidar scan2
        scan2.angle_min = 0;
        scan2.angle_max = (float)M_PI;
        scan2.angle_increment = (scan2.angle_max - scan2.angle_min)/num_rays;
        // Set all the ranges to 0 initially
        scan2.ranges = std::vector<float>(num_rays, 0);
        scan2.range_min = 20;
        scan2.range_max = 40;
        // Add the obstacles
        std::fill(scan2.ranges.begin(), scan2.ranges.begin()+10, 37.123);       //obstacle 1
        std::fill(scan2.ranges.begin()+15, scan2.ranges.begin()+20, 37.123);    //obstacle 2
        std::fill(scan2.ranges.begin()+100, scan2.ranges.begin()+105, 24.123);  //obstacle 3
        std::fill(scan2.ranges.begin()+105, scan2.ranges.begin()+110, 26.123);  //obstacle 4
        std::fill(scan2.ranges.begin()+125, scan2.ranges.begin()+130, 25.123);  //obstacle 5
        std::fill(scan2.ranges.begin()+200, scan2.ranges.begin()+240, 32);      //obstacle 6
        // Make some ranges outside the min and max of the scan
        std::fill(scan2.ranges.begin()+110, scan2.ranges.begin()+125, 45.123);
        std::fill(scan2.ranges.begin()+50, scan2.ranges.begin()+70, 4.123);

        // Create some fake sets of obstacles
        // Obstacles sorted in order of ascending angle
        sorted_obstacles = {
                LidarObstacle(0.0, 10),
                LidarObstacle(0.1, 10),
                LidarObstacle(0.2, 10),
                LidarObstacle(0.5, 10),
                LidarObstacle(1.1, 10),
                LidarObstacle(1.2, 10),
                LidarObstacle(1.6, 10),
        };
        // Obstacles not sorted at all (in terms of angle)
        unsorted_obstacles = {
                LidarObstacle(0.1, 10),
                LidarObstacle(0.5, 10),
                LidarObstacle(1.6, 10),
                LidarObstacle(0.2, 10),
                LidarObstacle(0.0, 10),
                LidarObstacle(1.2, 10),
                LidarObstacle(1.1, 10),
        };
        // A more realistic set of obstacles
//        realistic_obstacles = {
//                LidarObstacle()
//        };
    }

    sensor_msgs::LaserScan scan1, scan2;
    std::vector<LidarObstacle> sorted_obstacles, unsorted_obstacles, realistic_obstacles;
};

TEST_F(LidarDecisionTest, mergeSimilarObstaclesPreSortedTest){

    LidarDecision::mergeSimilarObstacles(sorted_obstacles, 0.11, 1.0);
    EXPECT_EQ(4, sorted_obstacles.size());
}

TEST_F(LidarDecisionTest, mergeSimilarObstaclesUnSortedTest){
    // Create some test obstacles in random order
    // (in terms of angle)
    LidarDecision::mergeSimilarObstacles(unsorted_obstacles, 0.11, 1.0);
    EXPECT_EQ(4, unsorted_obstacles.size());
}

// test with angles close enough but not distance
TEST_F(LidarDecisionTest, findObstaclesDistanceTest){
    std::vector<LidarObstacle> found_obstacles = LidarDecision::findObstacles(scan1, 0.02, 1.0);
    // The distance separates obstacle1 and obstacle2 making the size to be 6 instead of 5
    EXPECT_EQ(6, found_obstacles.size());
    // Check that all obstacles are at acceptable ranges
    for (LidarObstacle obstacle : found_obstacles){
        EXPECT_GE(obstacle.getAvgDistance(), 20);
        EXPECT_LE(obstacle.getAvgDistance(), 40);
    }
}

// test with distance close enough but not angles
TEST_F(LidarDecisionTest, findObstaclesAngleTest){
    std::vector<LidarObstacle> found_obstacles = LidarDecision::findObstacles(scan2, 0.02, 1.0);
    // The angle separates obstacle1 and obstacle2 making the size to be 6 instead of 5
    EXPECT_EQ(6, found_obstacles.size());
    // Check that all obstacles are at acceptable ranges
    for (LidarObstacle obstacle : found_obstacles){
        EXPECT_GE(obstacle.getAvgDistance(), 20);
        EXPECT_LE(obstacle.getAvgDistance(), 40);
    }
}

TEST_F(LidarDecisionTest, angular_twist_message_from_obstacleTest){
    // Create some test twist messages with correct twist.angular.z values
    geometry_msgs::Twist test1;
    geometry_msgs::Twist test2;
    geometry_msgs::Twist test3;

    // Check that the correct twist.angular.z values are calculated
    // based on test obstacle's parameters
    test1.angular.z = 1.0606602;
    EXPECT_NEAR(test1.angular.z,
                LidarDecision::twist_message_from_obstacle(LidarObstacle(-M_PI/4, 3.0),
                                                           4.0, M_PI/2, 1.5, 1.5).angular.z,
                0.000001);
    test2.angular.z = 1.5000000;
    EXPECT_NEAR(test2.angular.z,
                LidarDecision::twist_message_from_obstacle(LidarObstacle(0.0, 4.7),
                                                           5.0, M_PI/2, 1.5, 1.5).angular.z,
                0.000001);
    test3.angular.z = -1.5000000;
    EXPECT_NEAR(test3.angular.z,
                LidarDecision::twist_message_from_obstacle(LidarObstacle(M_PI/2, 0.9),
                                                           2.0, M_PI/2, 1.5, 1.5).angular.z,
                0.000001);
}

TEST_F(LidarDecisionTest, linear_twist_message_from_obstacleTest){
    // Create some test twist messages with correct twist.linear.x values
    geometry_msgs::Twist test1;
    geometry_msgs::Twist test2;
    geometry_msgs::Twist test3;

    // Check that the correct twist.linear.x values are calculated
    // based on test obstacle's parameters
    test3.linear.x = 0.0000000;
    EXPECT_NEAR(test3.linear.x,
                LidarDecision::twist_message_from_obstacle(LidarObstacle(3*M_PI/4, 2.1),
                                                           3.0, M_PI/2, 1.5, 1.5).linear.x,
                0.000001);
    test1.linear.x = 1.7320508;
    EXPECT_NEAR(test1.linear.x,
                LidarDecision::twist_message_from_obstacle(LidarObstacle(-M_PI/3, 3.0),
                                                           4.0, M_PI/2, 1.5, 1.5).linear.x,
                0.000001);
    test2.linear.x = 0.6324555;
    EXPECT_NEAR(test2.linear.x,
                LidarDecision::twist_message_from_obstacle(LidarObstacle(0.0, 0.4),
                                                           5.0, M_PI/2, 1.5, 1.5).linear.x,
                0.000001);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    return 0;
}
