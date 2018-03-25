/*
 * Created By: Gareth Ellis
 * Created On: January 9, 2018
 * Description: TODO
 */

// Snowbots Includes
#include "ObstacleManager.h"
#include "sb_geom/Point2D.h"

// GTest Includes
#include <gtest/gtest.h>

using namespace sb_geom;

class ObstacleManagerTest : public testing::Test {
protected:
    ObstacleManagerTest():
            obstacle_manager_with_one_cone(0.5, 0)
            {}

    virtual void SetUp() {
        obstacle_manager_with_one_cone.addObstacle(Cone(0,0,0.2));
    }

    ObstacleManager obstacle_manager_with_one_cone;
};

TEST_F(ObstacleManagerTest, add_single_cone){
    std::vector<Cone> actual = obstacle_manager_with_one_cone.getConeObstacles();
    EXPECT_EQ(1, actual.size());
}

TEST_F(ObstacleManagerTest, add_cone_outside_of_merging_tolerance_positive_coordinates){
    // Expected obstacles are the current obstacles with our new one appended
    Cone cone2(0.4,0.31,0.4);
    std::vector<Cone> expected = obstacle_manager_with_one_cone.getConeObstacles();
    expected.push_back(cone2);

    // Add a second cone just outside of the merging tolerance
    obstacle_manager_with_one_cone.addObstacle(cone2);
    std::vector<Cone> actual = obstacle_manager_with_one_cone.getConeObstacles();

    EXPECT_EQ(2, actual.size());
    EXPECT_EQ(expected, actual);
}

TEST_F(ObstacleManagerTest, add_cone_outside_of_merging_tolerance_negative_coordinates){
    // Expected obstacles are the current obstacles with our new one appended
    Cone cone2(0.4,-0.31,0.4);
    std::vector<Cone> expected = obstacle_manager_with_one_cone.getConeObstacles();
    expected.push_back(cone2);

    // Add a second cone just outside of the merging tolerance
    obstacle_manager_with_one_cone.addObstacle(cone2);
    std::vector<Cone> actual = obstacle_manager_with_one_cone.getConeObstacles();

    EXPECT_EQ(2, actual.size());
    EXPECT_EQ(expected, actual);
}

TEST_F(ObstacleManagerTest, add_cone_within_of_merging_tolerance_positive_coordinates){
    // Since this cone is within merging tolerance, we expect that it will be
    // merged (which will overwrite the coordinates of the already present
    // cone with cone2)
    Cone cone2(0.4,0.29,0.4);
    std::vector<Cone> expected = {cone2};

    // Add a second cone just outside of the merging tolerance
    obstacle_manager_with_one_cone.addObstacle(cone2);
    std::vector<Cone> actual = obstacle_manager_with_one_cone.getConeObstacles();

    EXPECT_EQ(1, actual.size());
    EXPECT_EQ(expected, actual);
}

TEST_F(ObstacleManagerTest, add_cone_within_of_merging_tolerance_negative_coordinates){
    // Since this cone is within merging tolerance, we expect that it will be
    // merged (which will overwrite the coordinates of the already present
    // cone with cone2)
    Cone cone2(-0.4,-0.29,0.4);
    std::vector<Cone> expected = {cone2};

    // Add a second cone just outside of the merging tolerance
    obstacle_manager_with_one_cone.addObstacle(cone2);
    std::vector<Cone> actual = obstacle_manager_with_one_cone.getConeObstacles();

    EXPECT_EQ(1, actual.size());
    EXPECT_EQ(expected, actual);
}

TEST_F(ObstacleManagerTest, add_several_cones){
    ObstacleManager obstacle_manager(1,0);

    // A list of a cones, all within merging tolerance of each other
    std::vector<Cone> cones_within_merging_tolerance = {
            Cone(5,5,0.2),
            Cone(5.5,5,0.2),
            Cone(5.2,5.1,0.2),
            Cone(4.9,5.3,0.2)
    };

    // A list of cones, all far enough from each other to not be merged
    std::vector<Cone> cones_outside_merging_tolerance = {
            Cone(0,0,0.3),
            Cone(-10,-5,0.4),
            Cone(-200,400000,0.4),
            Cone(-10,100,0.4)
    };

    // Add all the cones, alternating between the two lists
    for (int i = 0; i < 4; i++){
        obstacle_manager.addObstacle(cones_within_merging_tolerance[i]);
        obstacle_manager.addObstacle(cones_outside_merging_tolerance[i]);
    }

    std::vector<Cone> expected;

    // We expect that all the cones within merging tolerance were merged,
    // with the final cone that was added taking priority over all prior
    expected.push_back(cones_within_merging_tolerance[3]);

    // We expect that all the cones that were greater then the merging tolerance
    // were not merged, and hence should all be present
    expected.insert(expected.end(),
                    cones_outside_merging_tolerance.begin(),
                    cones_outside_merging_tolerance.end());

    std::vector<Cone> actual = obstacle_manager.getConeObstacles();
    // TODO: Use googleMock to compare lists without ordering (gMock supported by catkin in 0.7.9 but not in ubuntu PPA yet)
    EXPECT_EQ(expected, actual);
}

// TODO: Delete me, not a real test
TEST_F(ObstacleManagerTest, messing_about){
    ObstacleManager obstacle_manager(10, 10);

    std::vector<Point2D> points;

    points = {
            {0,1},
            {10,1}
    };
    Spline spline1(points);
    points = {
            {3,3},
            {7,3}
    };
    Spline spline2(points);

    obstacle_manager.addObstacle(spline1);
    obstacle_manager.addObstacle(spline2);


    std::vector<sb_geom::Spline> lines = obstacle_manager.getLineObstacles();

    for (sb_geom::Spline& spline : lines){
        std::cout << "~~~~~~~~~~~~~" << std::endl;
        int num_points = 100;
        for (int i = 0; i < num_points; i++){
            double u = i * 1/num_points;
            sb_geom::Point2D p = spline(u);
            std::cout << u << ", " << p.x() << ", " << p.y() << std::endl;
        }
    }
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}