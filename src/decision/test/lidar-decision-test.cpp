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

        obstacle1 = LidarObstacle(0.1, 10);
        obstacle2 = LidarObstacle(0.2, 20);
        obstacle3 = LidarObstacle(0.15, 15);
        obstacle4 = LidarObstacle(readings1);
    }

    LidarObstacle obstacle1, obstacle2, obstacle3, obstacle4;
    std::vector<reading> readings1;
};

TEST_F(LidarObstacleTest, ConstructorTest1){
    auto readings = obstacle1.getAllLaserReadings();
    EXPECT_EQ(1, readings.size());
    EXPECT_EQ(10, readings[0].second);
    EXPECT_EQ(0.1, readings[0].first);
}

TEST_F(LidarObstacleTest, ConstructorTest2){
    auto readings = obstacle4.getAllLaserReadings();
    // Check that the correct number of readings were input
    EXPECT_EQ(3, readings.size());
    // Check that they are in the correct order
    // (sorted in ascending order by angle)
    EXPECT_EQ(3, readings[0].first);
    EXPECT_EQ(4, readings[1].first);
    EXPECT_EQ(6, readings[2].first);
}

TEST_F(LidarObstacleTest, mergeInReadingsTest){
    obstacle1.mergeInLidarObstacle(obstacle2);
    obstacle1.mergeInLidarObstacle(obstacle3);
    auto readings = obstacle1.getAllLaserReadings();
    // Check that the merge obstacle has the correct number of readings
    EXPECT_EQ(3, readings.size());
    // Check that they are in the correct order
    // (sorted in ascending order by angle)
    EXPECT_EQ(0.1, readings[0].first);
    EXPECT_EQ(0.15, readings[1].first);
    EXPECT_EQ(0.2, readings[2].first);
}


class LidarDecisionTest : public testing::Test {
protected:
    virtual void SetUp(){
        sensor_msgs::LaserScan scan1;
        scan1.angle_min = 0;
        scan1.angle_max = (float)M_PI;
        scan1.angle_increment = (scan1.angle_max - scan1.angle_min)/300;
        // TODO: YOU ARE HERE, NEED TO FINISH WRITING THE TESTS FOR LIDARDECISION CLASS
    }
};


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    return 0;
}
