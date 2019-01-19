/*
 * Created By: William Gu
 * Created On: Jan 19 2019
 * Description: G-unit tests for Reactive System
 */

#include <ReactiveSystemTwist.h>
#include <RiskAreaBuilder.h>
#include <gtest/gtest.h>

static void printPoints(std::vector<sb_geom_msgs::Point2D> points){
    for (int i=0; i<points.size(); i++){
        std::cout<<points[i].x<<","<<points[i].y<<std::endl;
    }
}


TEST(ReactiveSystem, straightTraj) {

    float linear_vel = 1;
    float angular_vel = 0;
    float time_inc = 0.5;
    int num_incs = 3;

    std::vector<sb_geom_msgs::Point2D> traj = ReactiveSystemTwist::getArcTrajectory(linear_vel, angular_vel, time_inc, num_incs);

    EXPECT_FLOAT_EQ(traj[0].x, 0.5);
    EXPECT_FLOAT_EQ(traj[0].y, 0);
    EXPECT_FLOAT_EQ(traj[1].x, 1);
    EXPECT_FLOAT_EQ(traj[1].y, 0);
    EXPECT_FLOAT_EQ(traj[2].x, 1.5);
    EXPECT_FLOAT_EQ(traj[2].y, 0);
}


TEST(ReactiveSystem, curvedTraj) {

    float linear_vel = 1;
    float angular_vel = 1;
    float time_inc = 0.2;
    int num_incs = 5;

    std::vector<sb_geom_msgs::Point2D> traj = ReactiveSystemTwist::getArcTrajectory(linear_vel, angular_vel, time_inc, num_incs);

    EXPECT_NEAR(traj[0].x, 0.199, 0.001);
    EXPECT_NEAR(traj[0].y, 0.020, 0.001);
    EXPECT_NEAR(traj[1].x, 0.389, 0.001);
    EXPECT_NEAR(traj[1].y, 0.079, 0.001);
    EXPECT_NEAR(traj[2].x, 0.565, 0.001);
    EXPECT_NEAR(traj[2].y, 0.175, 0.001);
    EXPECT_NEAR(traj[3].x, 0.717, 0.001);
    EXPECT_NEAR(traj[3].y, 0.303, 0.001);
    EXPECT_NEAR(traj[4].x, 0.841, 0.001);
    EXPECT_NEAR(traj[4].y, 0.460, 0.001);

}


TEST(ReactiveSystem, curvedTrajOpposite) {

    float linear_vel = 1;
    float angular_vel = -1;
    float time_inc = 0.2;
    int num_incs = 5;

    std::vector<sb_geom_msgs::Point2D> traj = ReactiveSystemTwist::getArcTrajectory(linear_vel, angular_vel, time_inc, num_incs);

    EXPECT_NEAR(traj[0].x, 0.199, 0.001);
    EXPECT_NEAR(traj[0].y, -0.020, 0.001);
    EXPECT_NEAR(traj[1].x, 0.389, 0.001);
    EXPECT_NEAR(traj[1].y, -0.079, 0.001);
    EXPECT_NEAR(traj[2].x, 0.565, 0.001);
    EXPECT_NEAR(traj[2].y, -0.175, 0.001);
    EXPECT_NEAR(traj[3].x, 0.717, 0.001);
    EXPECT_NEAR(traj[3].y, -0.303, 0.001);
    EXPECT_NEAR(traj[4].x, 0.841, 0.001);
    EXPECT_NEAR(traj[4].y, -0.460, 0.001);

}


TEST(ReactiveSystem, riskAreaBuilder) {

    sb_geom_msgs::Point2D corner1;
    corner1.x = 0;
    corner1.y = -2;

    sb_geom_msgs::Point2D corner2;
    corner2.x = 2;
    corner2.y = 2;

    float risk_length = 0.01;
    RiskAreaBuilder::RiskAreaBuilder builder(corner1, corner2, risk_length);

}








int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}