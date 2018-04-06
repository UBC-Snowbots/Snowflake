/*
 * Created By: Gareth Ellis
 * Created On: January 9, 2018
 * Description: Tests for the `ObstacleManager` class
 */

// Snowbots Includes
#include "ObstacleManager.h"
#include "TestUtils.h"

// GTest Includes
#include <gtest/gtest.h>
#include <tf/transform_datatypes.h>

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

    /**
     * Generates an empty Occupancy Grid
     *
     * @param width the width of the grid
     * @param height the height of the height
     * @param origin_x the x coordinate of the origin
     * @param origin_y the y coordinate of the origin
     * @param origin_rot the rotation of the origin (which rotates the entire graph
     * @param resolution the length/width of each cell in the grid
     * @return
     */
    nav_msgs::OccupancyGrid generateEmptyOccGrid(int width, int height, double origin_x,
                                                 double origin_y, double origin_rot, double resolution){
        nav_msgs::OccupancyGrid occ_grid;
        occ_grid.info.resolution = resolution;

        occ_grid.info.width = width;
        occ_grid.info.height = height;

        occ_grid.info.origin.position.x = origin_x;
        occ_grid.info.origin.position.y = origin_y;
        occ_grid.info.origin.orientation = tf::createQuaternionMsgFromYaw(origin_rot);

        occ_grid.data.reserve(occ_grid.info.width * occ_grid.info.height);
        std::fill(occ_grid.data.begin(), occ_grid.data.end(), 0);

        return occ_grid;
    }
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

TEST_F(ObstacleManagerTest, add_single_line){
    ObstacleManager obstacle_manager(10, 10);

    Spline spline({{29,20}, {50,20}, {55, 20}});

    obstacle_manager.addObstacle(spline);

    std::vector<Spline> expected = {spline};
    EXPECT_EQ(expected, obstacle_manager.getLineObstacles());
}

// Test adding two lines to an ObstacleManager just outside of the tolerance
// needed to merge the two lines
TEST_F(ObstacleManagerTest, add_two_lines_just_outside_merging_tolerance){
    ObstacleManager obstacle_manager(1, 4.9);

    // These splines are basically two straight lines,
    // one at y=0, and one at y=5
    Spline spline1({{0,0}, {10,0}});
    Spline spline2({{2,5}, {7,5}});

    obstacle_manager.addObstacle(spline1);
    obstacle_manager.addObstacle(spline2);

    std::vector<Spline> known_lines = obstacle_manager.getLineObstacles();
    std::vector<Spline> expected_lines = {spline1, spline2};
    EXPECT_EQ(expected_lines.size(), known_lines.size());
    EXPECT_EQ(expected_lines, known_lines);
}

// Test adding two lines to an ObstacleManager just within of the tolerance
// needed to merge the two lines
TEST_F(ObstacleManagerTest, add_two_lines_just_within_merging_tolerance){
    ObstacleManager obstacle_manager(1, 5.1);

    Spline spline1({{0,0}, {10,0}});
    Spline spline2({{2,5}, {7,5}});

    obstacle_manager.addObstacle(spline1);
    obstacle_manager.addObstacle(spline2);

    // We expect that the second spline will be merged into the first. Since the closest
    // points to the endpoints of the second Spline on the first are in the middle section
    // of the first spline, we expect no interpolation points to be "cut" from the first
    // spline, and so the resulting spline should interpolate through all interpolation
    // points on both splines
    std::vector<Spline> expected_lines = { Spline({{0,0}, {2,5}, {7,5}, {10,0}}) };
    std::vector<Spline> known_lines = obstacle_manager.getLineObstacles();

    EXPECT_EQ(expected_lines.size(), known_lines.size());
    EXPECT_EQ(expected_lines, known_lines);
}

// Test merging two lines where the new line overlaps last point of the known line
TEST_F(ObstacleManagerTest, add_two_lines_second_line_overlapping_last_point_of_first_line){
    ObstacleManager obstacle_manager(1, 5);

    Spline spline1({{0,0}, {5,0}, {10,0}});
    Spline spline2({{7, 3}, {15, 4}});

    obstacle_manager.addObstacle(spline1);
    obstacle_manager.addObstacle(spline2);

    // We expect a single spline which is basically `spline1` with it's last interpolation
    // point replaced by the interpolation points of `spline2`
    std::vector<Spline> expected_splines = {
            Spline({{0,0}, {5,0}, {7,3}, {15,4}}),
    };

    EXPECT_EQ(expected_splines, obstacle_manager.getLineObstacles());
}

// Test merging two lines where the new line overlaps first point of the known line
TEST_F(ObstacleManagerTest, add_two_lines_second_line_overlapping_first_point_of_first_line){
    ObstacleManager obstacle_manager(1, 5);

    Spline spline1({{0,0}, {5,0}, {10,0}});
    Spline spline2({{-5, 4}, {3, 4}});

    obstacle_manager.addObstacle(spline1);
    obstacle_manager.addObstacle(spline2);

    // We expect a single spline which is basically `spline1` with it's last interpolation
    // point replaced by the interpolation points of `spline2`
    std::vector<Spline> expected_splines = {
            Spline({{-5,4}, {3,4}, {5,0}, {10,0}}),
    };

    EXPECT_EQ(expected_splines, obstacle_manager.getLineObstacles());
}

// Test merging two lines where the second totally overlaps the first
TEST_F(ObstacleManagerTest, add_two_lines_second_totally_overlaps_first){
    ObstacleManager obstacle_manager(1,4);

    Spline spline1({{0,0}, {5,3}, {10,0}});
    Spline spline2({{-1,0.5}, {5,3}, {11,0.5}});

    obstacle_manager.addObstacle(spline1);
    obstacle_manager.addObstacle(spline2);

    std::vector<Spline> expected = {spline2};

    EXPECT_EQ(expected, obstacle_manager.getLineObstacles());
}

// Test merging a small line into a much longer and more complex one
TEST_F(ObstacleManagerTest, merge_simple_and_small_line_into_large_and_complex_one){
    ObstacleManager obstacle_manager(1,4);

    Spline spline1({{0,0}, {2,2}, {8,8}, {6,0}, {10,-6}});
    Spline spline2({{7, 9}, {10, 10}, {8,6.5}});

    obstacle_manager.addObstacle(spline1);
    obstacle_manager.addObstacle(spline2);

    // We expect the 2nd spline to overwrite the point {8,8} on the first spline
    std::vector<Spline> expected = {
            Spline({{0,0}, {2,2}, {7, 9}, {10, 10}, {8,6.5}, {6,0}, {10,-6}})
    };

    EXPECT_EQ(expected, obstacle_manager.getLineObstacles());
}

// Add several overlapping lines, and a few ones that don't overlap
TEST_F(ObstacleManagerTest, add_variety_of_overlapping_and_not_overlapping_lines){
    ObstacleManager obstacle_manager(1,2);

    // A simple straight line from (0,0) to (10,0)
    Spline spline1({{0,0}, {5,0}, {10,0}});
    // An arc around (5,0) that should merge with `spline1`
    Spline spline2({{4, 1}, {5, 10}, {6,1}});
    // A horizontal line that should merge into the right side of the arc
    // from `spline2`
    Spline spline3({{7, 3}, {7, 8}});
    // A line outside of the merging tolerance of all known lines
    Spline spline4({{10,10}, {11,11}});

    obstacle_manager.addObstacle(spline1);
    obstacle_manager.addObstacle(spline2);
    obstacle_manager.addObstacle(spline3);
    obstacle_manager.addObstacle(spline4);

    std::vector<Spline> expected = {
            Spline({{0,0}, {4,1}, {5,10}, {7,8}, {7,3}, {6,1}, {10,0}}),
            spline4,
    };

    EXPECT_EQ(expected, obstacle_manager.getLineObstacles());
}

// Test adding a line to an obstacle manager with lots of known lines
// (but the new line is close to only one)
TEST_F(ObstacleManagerTest, adding_single_line_to_lots_of_lines){
    ObstacleManager obstacle_manager(1,1);

    // A bunch of lines outside merging tolerance of each other
    std::vector<Spline> known_lines ={
            Spline({{5,5}, {10,10}}),
            Spline({{0,0}, {1,0}, {3,3}}),
            Spline({{-1,-10}, {-13,-14}}),
            Spline({{100,100}, {150,150}, {175, 125}, {200,200}}),
            Spline({{1024,3423}, {1029, 3429}, {1045, 3450}}),
            Spline({{-100,22}, {-102,25}, {-100,27}}),
            Spline({{100, -10}, {105, -20}, {84, -100}}),
    };

    for (auto& line : known_lines){
        obstacle_manager.addObstacle(line);
    }

    // Check that none of the lines merged together
    EXPECT_EQ(known_lines.size(), obstacle_manager.getLineObstacles().size());

    // This spline should merge with the first in `known_lines`
    Spline new_line({{10, 10.2}, {15,15}});
    obstacle_manager.addObstacle(new_line);

    // Check that it was successfully merged

    // If it was merged, the number of lines should be the same
    EXPECT_EQ(known_lines.size(), obstacle_manager.getLineObstacles().size());

    // Search through all the lines for the expected merged line
    std::vector<Spline> curr_lines = obstacle_manager.getLineObstacles();
    Spline expected_merged_line({{5,5}, {10,10}, {10,10.2}, {15,15}});

    EXPECT_TRUE((std::find(curr_lines.begin(), curr_lines.end(), expected_merged_line) != curr_lines.end()));
}

// Test inflating a single point where the inflation radius does not extend beyound the edge
// of the occupancy grid
TEST_F(ObstacleManagerTest, inflate_single_point_inflate_radius_within_occ_grid){
    nav_msgs::OccupancyGrid occ_grid = generateEmptyOccGrid(20, 20, 0, 0, 0, 1);

    ObstacleManager::inflatePoint(occ_grid, Point2D(10,10), 1);

    // all the cells we expect to be marked as "occupied" in pairs: (x,y)
    std::vector<std::pair<int,int>> expected_occupied_cells = {
            std::make_pair(9,9),
            std::make_pair(9,10),
            std::make_pair(10,9),
            std::make_pair(10,10),
            std::make_pair(10,11),
            std::make_pair(11,10),
            std::make_pair(11,11),
    };

    // Figure out the extents of the graph
    int x_min = occ_grid.info.origin.position.x;
    int y_min = occ_grid.info.origin.position.y;
    int x_max = occ_grid.info.origin.position.x + occ_grid.info.width;
    int y_max = occ_grid.info.origin.position.y + occ_grid.info.height;

    // Check that all the cells are set correctly
    for (int x = 0; x < occ_grid.info.width; x++){
        for (int y = 0; y < occ_grid.info.height; y++){
            // Check if this is one of the inflated cells
            if (std::find(expected_occupied_cells.begin(),
                          expected_occupied_cells.end(), std::make_pair(x,y))
                != expected_occupied_cells.end()) {
                EXPECT_EQ(100, (int)occ_grid.data[y * occ_grid.info.width + x]);
            } else {
                EXPECT_EQ(0, (int)occ_grid.data[y * occ_grid.info.width + x]);
            }
        }
    }
}


// TODO: Delete me, not a real test
//TEST_F(ObstacleManagerTest, messing_about){
//    ObstacleManager obstacle_manager(10, 30);
//
//    //Spline spline1({{0,1}, {20,1}, {30,1}, {40,1}, {50,1}, {75,1}, {100,1}});
//    //Spline spline2({{29,20}, {50,20}, {55, 20}});
//    Spline spline1({{0,0}, {100,0}});
//    Spline spline2({{30,20}, {70,20}});
//
//    // TODO: YOU ARE HERE - seems like checking the distance between two splines is fast,
//    // TODO: but when we go to actually merge the splines and check point to spline distance, it's slow
//    //Spline spline1({{0,0}, {10, -10}, {10, 20}, {100,-100}});
//    //Spline spline2({{0,10}, {10, 20}, {10, 30}, {100,100}});
//
//    obstacle_manager.addObstacle(spline1);
//    obstacle_manager.addObstacle(spline2);
//
//    std::vector<sb_geom::Spline> lines = obstacle_manager.getLineObstacles();
//
//    for (sb_geom::Spline& spline : lines){
//        std::cout << "~~~~~~~~~~~~~" << std::endl;
//        int num_points = 100;
//        for (int i = 0; i < num_points; i++){
//            double u = i * 1.0/(double)num_points;
//            sb_geom::Point2D p = spline(u);
//            std::cout << u << ", " << p.x() << ", " << p.y() << std::endl;
//        }
//    }
//}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}