/*
 * Created By: Gareth Ellis
 * Created On: July 22, 2016
 * Description: Tests for Final Decision
 */

#include <FinalDecision.h>
#include <gtest/gtest.h>

TEST(FinalDecision, arbitrator) {
    geometry_msgs::Twist lidar1;
    geometry_msgs::Twist lidar2;
    geometry_msgs::Twist lidar3;
    geometry_msgs::Twist vision1;
    geometry_msgs::Twist vision2;
    geometry_msgs::Twist vision3;
    geometry_msgs::Twist gps1;
    geometry_msgs::Twist gps2;
    geometry_msgs::Twist gps3;

    lidar1.angular.z  = 5;
    lidar2.angular.z  = 0;
    lidar3.angular.z  = -5;
    vision1.angular.z = 5;
    vision2.angular.z = 0;
    vision3.angular.z = -5;
    gps1.angular.z    = 5;
    gps2.angular.z    = 0;
    gps3.angular.z    = -5;

    EXPECT_EQ(lidar1.angular.z,
              FinalDecision::arbitrator(lidar1, vision1, gps1).angular.z);
    EXPECT_EQ(lidar1.angular.z,
              FinalDecision::arbitrator(lidar1, vision1, gps2).angular.z);
    EXPECT_EQ(lidar1.angular.z,
              FinalDecision::arbitrator(lidar1, vision1, gps3).angular.z);
    EXPECT_EQ(lidar1.angular.z,
              FinalDecision::arbitrator(lidar1, vision2, gps1).angular.z);
    EXPECT_EQ(lidar1.angular.z,
              FinalDecision::arbitrator(lidar1, vision2, gps2).angular.z);
    EXPECT_EQ(lidar1.angular.z,
              FinalDecision::arbitrator(lidar1, vision2, gps3).angular.z);
    EXPECT_EQ(lidar1.angular.z,
              FinalDecision::arbitrator(lidar1, vision3, gps1).angular.z);
    EXPECT_EQ(lidar1.angular.z,
              FinalDecision::arbitrator(lidar1, vision3, gps2).angular.z);
    EXPECT_EQ(lidar1.angular.z,
              FinalDecision::arbitrator(lidar1, vision3, gps3).angular.z);
    EXPECT_EQ(vision1.angular.z,
              FinalDecision::arbitrator(lidar2, vision1, gps1).angular.z);
    EXPECT_EQ(vision1.angular.z,
              FinalDecision::arbitrator(lidar2, vision1, gps2).angular.z);
    EXPECT_EQ(vision1.angular.z,
              FinalDecision::arbitrator(lidar2, vision1, gps3).angular.z);
    EXPECT_EQ(gps1.angular.z,
              FinalDecision::arbitrator(lidar2, vision2, gps1).angular.z);
    EXPECT_EQ(gps2.angular.z,
              FinalDecision::arbitrator(lidar2, vision2, gps2).angular.z);
    EXPECT_EQ(gps3.angular.z,
              FinalDecision::arbitrator(lidar2, vision2, gps3).angular.z);
    EXPECT_EQ(vision3.angular.z,
              FinalDecision::arbitrator(lidar2, vision3, gps1).angular.z);
    EXPECT_EQ(vision3.angular.z,
              FinalDecision::arbitrator(lidar2, vision3, gps2).angular.z);
    EXPECT_EQ(vision3.angular.z,
              FinalDecision::arbitrator(lidar2, vision3, gps3).angular.z);
    EXPECT_EQ(lidar3.angular.z,
              FinalDecision::arbitrator(lidar3, vision1, gps1).angular.z);
    EXPECT_EQ(lidar3.angular.z,
              FinalDecision::arbitrator(lidar3, vision1, gps2).angular.z);
    EXPECT_EQ(lidar3.angular.z,
              FinalDecision::arbitrator(lidar3, vision1, gps3).angular.z);
    EXPECT_EQ(lidar3.angular.z,
              FinalDecision::arbitrator(lidar3, vision2, gps1).angular.z);
    EXPECT_EQ(lidar3.angular.z,
              FinalDecision::arbitrator(lidar3, vision2, gps2).angular.z);
    EXPECT_EQ(lidar3.angular.z,
              FinalDecision::arbitrator(lidar3, vision2, gps3).angular.z);
    EXPECT_EQ(lidar3.angular.z,
              FinalDecision::arbitrator(lidar3, vision3, gps1).angular.z);
    EXPECT_EQ(lidar3.angular.z,
              FinalDecision::arbitrator(lidar3, vision3, gps2).angular.z);
    EXPECT_EQ(lidar3.angular.z,
              FinalDecision::arbitrator(lidar3, vision3, gps3).angular.z);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
