/*
 * Created By: William Gu
 * Created On: Jan 19 2019
 * Description: G-unit tests for Reactive System
 */

#include <ReactiveSystemPath.h>
#include <RiskAreaBuilder.h>
#include <gtest/gtest.h>

static void printPoints(std::vector<sb_geom_msgs::Point2D> points) {
    for (int i = 0; i < points.size(); i++) {
        std::cout << points[i].x << "," << points[i].y << std::endl;
    }
}

/*
 * Get the average angle given some path
 */
static float getAvgAngle(nav_msgs::Path path) {
    float sum_x = 0;
    float sum_y = 0;

    for (int i = 0; i < path.poses.size(); i++) {
        sum_x += path.poses[i].pose.position.x;
        sum_y += path.poses[i].pose.position.y;
    }

    return atan2(sum_y, sum_x);
}

TEST(ReactiveSystem, straightTraj) {
    float linear_vel  = 1;
    float angular_vel = 0;
    float time_inc    = 0.5;
    int num_incs      = 3;

    std::vector<sb_geom_msgs::Point2D> traj =
    ReactiveSystemPath::getArcTrajectory(
    linear_vel, angular_vel, time_inc, num_incs);

    EXPECT_FLOAT_EQ(traj[0].x, 0.5);
    EXPECT_FLOAT_EQ(traj[0].y, 0);
    EXPECT_FLOAT_EQ(traj[1].x, 1);
    EXPECT_FLOAT_EQ(traj[1].y, 0);
    EXPECT_FLOAT_EQ(traj[2].x, 1.5);
    EXPECT_FLOAT_EQ(traj[2].y, 0);
}

TEST(ReactiveSystem, curvedTraj) {
    float linear_vel  = 1;
    float angular_vel = 1;
    float time_inc    = 0.2;
    int num_incs      = 5;

    std::vector<sb_geom_msgs::Point2D> traj =
    ReactiveSystemPath::getArcTrajectory(
    linear_vel, angular_vel, time_inc, num_incs);

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
    float linear_vel  = 1;
    float angular_vel = -1;
    float time_inc    = 0.2;
    int num_incs      = 5;

    std::vector<sb_geom_msgs::Point2D> traj =
    ReactiveSystemPath::getArcTrajectory(
    linear_vel, angular_vel, time_inc, num_incs);

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
    float risk_score  = 5;
    float risk_radius = 1;
    sb_geom_msgs::Point2D risk_center;
    risk_center.x = 2;
    risk_center.y = 0;

    sb_geom_msgs::Point2D corner1;
    corner1.x = 0;
    corner1.y = -2;

    sb_geom_msgs::Point2D corner2;
    corner2.x = 4;
    corner2.y = 2;

    float risk_length = 0.05;
    RiskAreaBuilder::RiskAreaBuilder builder(corner1, corner2, risk_length);

    builder.addRiskZone(
    risk_center, risk_radius, risk_score); // rad = 1, score = 5

    mapping_msgs_urc::RiskAreaArray risks = builder.getRiskArray();

    // For testing that riskareabuilder is correct
    for (mapping_msgs_urc::RiskArea area : risks.areas) {
        if (area.score.data == risk_score) { // confirm that risk scores were
                                             // added in correct positions
            EXPECT_NEAR(pow(area.area.points[0].x - risk_center.x, 2) +
                        pow(area.area.points[0].y - risk_center.y, 2),
                        0,
                        pow(risk_radius, 2) + 0.15);
        }
    }
}

// Test the given twist, for a scenario where an object is directly between our
// position and the goal
TEST(ReactiveSystem, objectInDirectPath) {
    // object parameters
    float risk_score  = MAX_RISK;
    float risk_radius = 1;
    sb_geom_msgs::Point2D risk_center;
    risk_center.x = 2;
    risk_center.y = 0;

    sb_geom_msgs::Point2D corner1;
    corner1.x = 0;
    corner1.y = -2;

    sb_geom_msgs::Point2D corner2;
    corner2.x = 4;
    corner2.y = 2;

    float risk_length = 0.05;
    RiskAreaBuilder::RiskAreaBuilder builder(corner1, corner2, risk_length);

    builder.addRiskZone(risk_center, risk_radius, risk_score);

    mapping_msgs_urc::RiskAreaArray risk_areas = builder.getRiskArray();

    sb_geom_msgs::Point2D goal;
    goal.x = 3;
    goal.y = 0;

    float linear_vel      = 1;
    float max_angular_vel = 1.5;
    int num_angular_vel   = 7;
    float time_inc        = 0.2;
    int num_incs          = 10;

    float risk_dist_tol_sq = 0.4;

    nav_msgs::Path path = ReactiveSystemPath::getPath(risk_areas,
                                                      goal,
                                                      time_inc,
                                                      num_incs,
                                                      linear_vel,
                                                      max_angular_vel,
                                                      num_angular_vel,
                                                      risk_dist_tol_sq);

    float path_ang = getAvgAngle(path);
    EXPECT_GE(fabs(path_ang),
              fabs(atan2(risk_center.y + risk_radius, risk_center.x)));
}

// Test the given twist, for a scenario where 3 objects are directly between our
// position and the goal (covering a greater "width)
TEST(ReactiveSystem, threeObjectsInDirectPath) {
    // object parameters
    float risk_score  = MAX_RISK;
    float risk_radius = 1;

    sb_geom_msgs::Point2D risk_center1;
    risk_center1.x = 2;
    risk_center1.y = 0;

    sb_geom_msgs::Point2D risk_center2;
    risk_center2.x = 2;
    risk_center2.y = 2;

    sb_geom_msgs::Point2D risk_center3;
    risk_center3.x = 2;
    risk_center3.y = -2;

    sb_geom_msgs::Point2D corner1;
    corner1.x = 0;
    corner1.y = -2;

    sb_geom_msgs::Point2D corner2;
    corner2.x = 4;
    corner2.y = 2;

    float risk_length = 0.05;
    RiskAreaBuilder::RiskAreaBuilder builder(corner1, corner2, risk_length);

    builder.addRiskZone(risk_center1, risk_radius, risk_score);
    builder.addRiskZone(risk_center2, risk_radius, risk_score);
    builder.addRiskZone(risk_center3, risk_radius, risk_score);

    mapping_msgs_urc::RiskAreaArray risk_areas = builder.getRiskArray();

    sb_geom_msgs::Point2D goal;
    goal.x = 3;
    goal.y = 0;

    float linear_vel      = 1;
    float max_angular_vel = 2;
    int num_angular_vel   = 10;
    float time_inc        = 0.2;
    int num_incs          = 10;

    float risk_dist_tol_sq = 0.4;

    nav_msgs::Path path = ReactiveSystemPath::getPath(risk_areas,
                                                      goal,
                                                      time_inc,
                                                      num_incs,
                                                      linear_vel,
                                                      max_angular_vel,
                                                      num_angular_vel,
                                                      risk_dist_tol_sq);

    float path_ang = getAvgAngle(path);
    EXPECT_GE(fabs(path_ang),
              fabs(atan2(risk_center2.y + risk_radius, risk_center2.x)));
}

// Test the given twist, for a scenario where no objects are in the way at all
TEST(ReactiveSystem, noObjectsInDirectPath) {
    sb_geom_msgs::Point2D corner1;
    corner1.x = 0;
    corner1.y = -2;

    sb_geom_msgs::Point2D corner2;
    corner2.x = 4;
    corner2.y = 2;

    float risk_length = 0.05;
    RiskAreaBuilder::RiskAreaBuilder builder(corner1, corner2, risk_length);

    mapping_msgs_urc::RiskAreaArray risk_areas = builder.getRiskArray();

    sb_geom_msgs::Point2D goal;
    goal.x = 3;
    goal.y = 0;

    float linear_vel      = 1;
    float max_angular_vel = 2;
    int num_angular_vel   = 10;
    float time_inc        = 0.2;
    int num_incs          = 10;

    float risk_dist_tol_sq = 0.4;

    nav_msgs::Path path = ReactiveSystemPath::getPath(risk_areas,
                                                      goal,
                                                      time_inc,
                                                      num_incs,
                                                      linear_vel,
                                                      max_angular_vel,
                                                      num_angular_vel,
                                                      risk_dist_tol_sq);

    float path_ang = getAvgAngle(path);
    EXPECT_EQ(path_ang, 0);
}

// Test the given twist, for a scenario where there are two objects but a gap
// exists in the center
TEST(ReactiveSystem, twoObjectsOnSides) {
    // object parameters
    float risk_score  = MAX_RISK;
    float risk_radius = 1;

    sb_geom_msgs::Point2D risk_center1;
    risk_center1.x = 2;
    risk_center1.y = -2;

    sb_geom_msgs::Point2D risk_center2;
    risk_center2.x = 2;
    risk_center2.y = 2;

    sb_geom_msgs::Point2D corner1;
    corner1.x = 0;
    corner1.y = -2;

    sb_geom_msgs::Point2D corner2;
    corner2.x = 4;
    corner2.y = 2;

    float risk_length = 0.05;
    RiskAreaBuilder::RiskAreaBuilder builder(corner1, corner2, risk_length);

    builder.addRiskZone(risk_center1, risk_radius, risk_score);
    builder.addRiskZone(risk_center2, risk_radius, risk_score);

    mapping_msgs_urc::RiskAreaArray risk_areas = builder.getRiskArray();

    sb_geom_msgs::Point2D goal;
    goal.x = 3;
    goal.y = 0;

    float linear_vel      = 1;
    float max_angular_vel = 2;
    int num_angular_vel   = 10;
    float time_inc        = 0.2;
    int num_incs          = 10;

    float risk_dist_tol_sq = 0.4;

    nav_msgs::Path path = ReactiveSystemPath::getPath(risk_areas,
                                                      goal,
                                                      time_inc,
                                                      num_incs,
                                                      linear_vel,
                                                      max_angular_vel,
                                                      num_angular_vel,
                                                      risk_dist_tol_sq);

    float path_ang = getAvgAngle(path);
    EXPECT_EQ(path_ang, 0);
}

// Test the given twist, for a scenario where there are two objects but a space
// exists to the "right"
TEST(ReactiveSystem, openingToRight) {
    // object parameters
    float risk_score  = MAX_RISK;
    float risk_radius = 1;

    sb_geom_msgs::Point2D risk_center1;
    risk_center1.x = 2;
    risk_center1.y = 0;

    sb_geom_msgs::Point2D risk_center2;
    risk_center2.x = 2;
    risk_center2.y = 2;

    sb_geom_msgs::Point2D corner1;
    corner1.x = 0;
    corner1.y = -2;

    sb_geom_msgs::Point2D corner2;
    corner2.x = 4;
    corner2.y = 2;

    float risk_length = 0.05;
    RiskAreaBuilder::RiskAreaBuilder builder(corner1, corner2, risk_length);

    builder.addRiskZone(risk_center1, risk_radius, risk_score);
    builder.addRiskZone(risk_center2, risk_radius, risk_score);

    mapping_msgs_urc::RiskAreaArray risk_areas = builder.getRiskArray();

    sb_geom_msgs::Point2D goal;
    goal.x = 3;
    goal.y = 0;

    float linear_vel      = 1;
    float max_angular_vel = 2;
    int num_angular_vel   = 10;
    float time_inc        = 0.2;
    int num_incs          = 10;

    float risk_dist_tol_sq = 0.4;

    nav_msgs::Path path = ReactiveSystemPath::getPath(risk_areas,
                                                      goal,
                                                      time_inc,
                                                      num_incs,
                                                      linear_vel,
                                                      max_angular_vel,
                                                      num_angular_vel,
                                                      risk_dist_tol_sq);

    float path_ang = getAvgAngle(path);
    EXPECT_LE(path_ang, atan2(risk_center1.y - risk_radius, risk_center1.x));
}

// Test the given twist, for a scenario where there are three objects but a gap
// exists between two objects to the "right"
TEST(ReactiveSystem, gapToRight) {
    // object parameters
    float risk_score  = MAX_RISK;
    float risk_radius = 0.5;

    sb_geom_msgs::Point2D risk_center1;
    risk_center1.x = 3;
    risk_center1.y = 0;

    sb_geom_msgs::Point2D risk_center2;
    risk_center2.x = 3;
    risk_center2.y = 1;

    sb_geom_msgs::Point2D risk_center3;
    risk_center3.x = 3;
    risk_center3.y = -2;

    sb_geom_msgs::Point2D corner1;
    corner1.x = 0;
    corner1.y = -2;

    sb_geom_msgs::Point2D corner2;
    corner2.x = 4;
    corner2.y = 2;

    float risk_length = 0.05;
    RiskAreaBuilder::RiskAreaBuilder builder(corner1, corner2, risk_length);

    builder.addRiskZone(risk_center1, risk_radius, risk_score);
    builder.addRiskZone(risk_center2, risk_radius, risk_score);
    builder.addRiskZone(risk_center3, risk_radius, risk_score);

    mapping_msgs_urc::RiskAreaArray risk_areas = builder.getRiskArray();

    sb_geom_msgs::Point2D goal;
    goal.x = 5;
    goal.y = 0;

    float linear_vel      = 1;
    float max_angular_vel = 2;
    int num_angular_vel   = 10;
    float time_inc        = 0.2;
    int num_incs          = 10;

    float risk_dist_tol_sq = 0.4;

    nav_msgs::Path path = ReactiveSystemPath::getPath(risk_areas,
                                                      goal,
                                                      time_inc,
                                                      num_incs,
                                                      linear_vel,
                                                      max_angular_vel,
                                                      num_angular_vel,
                                                      risk_dist_tol_sq);

    float path_ang = getAvgAngle(path);
    EXPECT_LE(path_ang, atan2(risk_center1.y - risk_radius, risk_center1.x));
    EXPECT_GE(path_ang, atan2(risk_center3.y + risk_radius, risk_center3.x));
}

// Test the given twist, for a scenario where no objects are in the way at all,
// but goal is not directly straight
TEST(ReactiveSystem, goalToLeft) {
    sb_geom_msgs::Point2D corner1;
    corner1.x = 0;
    corner1.y = -2;

    sb_geom_msgs::Point2D corner2;
    corner2.x = 4;
    corner2.y = 2;

    float risk_length = 0.05;
    RiskAreaBuilder::RiskAreaBuilder builder(corner1, corner2, risk_length);

    mapping_msgs_urc::RiskAreaArray risk_areas = builder.getRiskArray();

    sb_geom_msgs::Point2D goal;
    goal.x = 4;
    goal.y = 2;

    float linear_vel      = 1;
    float max_angular_vel = 2;
    int num_angular_vel   = 10;
    float time_inc        = 0.2;
    int num_incs          = 10;

    float risk_dist_tol_sq = 0.4;

    nav_msgs::Path path = ReactiveSystemPath::getPath(risk_areas,
                                                      goal,
                                                      time_inc,
                                                      num_incs,
                                                      linear_vel,
                                                      max_angular_vel,
                                                      num_angular_vel,
                                                      risk_dist_tol_sq);

    float path_ang = getAvgAngle(path);
    EXPECT_NEAR(path_ang, atan2(goal.y, goal.x), 0.5);
}

// Test the given twist, for a scenario where there are three objects but a gap
// exists between two objects to the "right", BUT goal is to the left behind the
// objects
TEST(ReactiveSystem, goalToLeftAndGapToRight) {
    // object parameters
    float risk_score  = MAX_RISK;
    float risk_radius = 0.5;

    sb_geom_msgs::Point2D risk_center1;
    risk_center1.x = 3;
    risk_center1.y = 0;

    sb_geom_msgs::Point2D risk_center2;
    risk_center2.x = 3;
    risk_center2.y = 1;

    sb_geom_msgs::Point2D risk_center3;
    risk_center3.x = 3;
    risk_center3.y = -2;

    sb_geom_msgs::Point2D risk_center4;
    risk_center4.x = 2;
    risk_center4.y = 1;

    sb_geom_msgs::Point2D risk_center5;
    risk_center5.x = 2.5;
    risk_center5.y = 3;

    sb_geom_msgs::Point2D risk_center6;
    risk_center6.x = 2;
    risk_center6.y = 4;

    sb_geom_msgs::Point2D risk_center7;
    risk_center6.x = 1;
    risk_center6.y = 1;

    sb_geom_msgs::Point2D corner1;
    corner1.x = 0;
    corner1.y = -2;

    sb_geom_msgs::Point2D corner2;
    corner2.x = 4;
    corner2.y = 2;

    float risk_length = 0.05;
    RiskAreaBuilder::RiskAreaBuilder builder(corner1, corner2, risk_length);

    builder.addRiskZone(risk_center1, risk_radius, risk_score);
    builder.addRiskZone(risk_center2, risk_radius, risk_score);
    builder.addRiskZone(risk_center3, risk_radius, risk_score);
    builder.addRiskZone(risk_center4, risk_radius, risk_score);
    builder.addRiskZone(risk_center5, risk_radius, risk_score);
    builder.addRiskZone(risk_center6, risk_radius, risk_score);

    mapping_msgs_urc::RiskAreaArray risk_areas = builder.getRiskArray();

    sb_geom_msgs::Point2D goal;
    goal.x = 5;
    goal.y = 2;

    float linear_vel      = 1;
    float max_angular_vel = 1.5;
    int num_angular_vel   = 10;
    float time_inc        = 0.3;
    int num_incs          = 10;

    float risk_dist_tol_sq = 0.4;

    nav_msgs::Path path = ReactiveSystemPath::getPath(risk_areas,
                                                      goal,
                                                      time_inc,
                                                      num_incs,
                                                      linear_vel,
                                                      max_angular_vel,
                                                      num_angular_vel,
                                                      risk_dist_tol_sq);

    float path_ang = getAvgAngle(path);
    EXPECT_NEAR(path_ang, atan2(-1, 3), 0.5);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}