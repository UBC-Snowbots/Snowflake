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
using namespace mapping_igvc;

namespace mapping_igvc {
    // Implementation of the `==` operator for ConeObstacle
    inline bool operator==(const mapping_igvc::ConeObstacle& lhs, const mapping_igvc::ConeObstacle& rhs){
        return (
                lhs.center.x == rhs.center.x &&
                lhs.center.y == rhs.center.y &&
                lhs.radius == rhs.radius
        );
    }
}


class ObstacleManagerTest : public testing::Test {
protected:
    ObstacleManagerTest():
            obstacle_manager_with_one_cone(0.5, 0)
            {}

    virtual void SetUp() {
        obstacle_manager_with_one_cone.addObstacle(generateConeObstacle(0,0,0.2));
    }

    ObstacleManager obstacle_manager_with_one_cone;

    /**
     * Creates a ConeObstacle msg with given values
     *
     * @param x
     * @param y
     * @param radius
     *
     * @return the ConeObstacle msg with the given values
     */
    static ConeObstacle generateConeObstacle(double x, double y, double radius){
        ConeObstacle cone_obstacle;
        cone_obstacle.center.x = x;
        cone_obstacle.center.y = y;
        cone_obstacle.radius = radius;

        return cone_obstacle;
    }

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
        occ_grid.info.resolution = static_cast<float>(resolution);

        occ_grid.info.width = static_cast<unsigned int>(width);
        occ_grid.info.height = static_cast<unsigned int>(height);

        occ_grid.info.origin.position.x = origin_x;
        occ_grid.info.origin.position.y = origin_y;
        occ_grid.info.origin.orientation = tf::createQuaternionMsgFromYaw(origin_rot);

        occ_grid.data = std::vector<int8_t>(occ_grid.info.width * occ_grid.info.height);
        std::fill(occ_grid.data.begin(), occ_grid.data.end(), 0);

        return occ_grid;
    }

    /**
     * Represents if a given cell is occupied
     *
     * Used to allow better visual representations of expected occupancy grids
     * `X` - occupied
     * `_` - not occupied
     */
    enum OccupiedOrNot {
        _, X
    };
    /**
     * Checks that a given occupancy grid has the expected cells marked as "occupied"
     *
     * @param occ_grid the occupancy grid we're validating
     * @param expected_grid the expected grid, where `true` means occupied
     * (a value of 100 in the cell), and `false` means unoccupied
     * (a value of 0 in the cell)
     */
    void checkOccupiedCells(nav_msgs::OccupancyGrid occ_grid,
                            std::vector<std::vector<OccupiedOrNot >> expected_grid){
        // (x,y) pairs representing cells we expect to be occupied
        std::vector<std::pair<int,int>> expected_occupied_cells;
        for (int y = 0; y < expected_grid.size(); y++){
            for (int x = 0; x < expected_grid[y].size(); x++){
                if (expected_grid[y][x] == X){
                    expected_occupied_cells.emplace_back(std::make_pair(x,y));
                }
            }
        }

        checkOccupiedCells(occ_grid, expected_occupied_cells);
    }

    /**
     * Prints out a given occupancy grid in human readable format
     * @param occ_grid the occupancy grid to print
     */
    void printOccGrid(nav_msgs::OccupancyGrid occ_grid){
        for (int y = 0; y < occ_grid.info.height; y++){
            std::cout << std::endl;
            for (int x = 0; x < occ_grid.info.width; x++){
                if (occ_grid.data[y * occ_grid.info.width + x] != 0){
                    std::cout << "X";
                } else {
                    std::cout << ".";
                }
            }
        }
        std::cout << std::endl;
    };

    /**
     * Checks whether a given set of cells are occupied
     *
     * "occupied" is assumed to mean a value of 100 in the cell, all other cells are presumed
     * to be unoccupied
     *
     * @param occ_grid the occupancy grid to check for occupied and unoccupied cells
     * @param expected_occupied_cells the cells that we expect to be occupied
     * (represented as (x,y) pairs)
     *
     * @return if all the points we expect to be occupied were occupied
     */
    bool checkOccupiedCells(nav_msgs::OccupancyGrid occ_grid,
                            std::vector<std::pair<int,int>> expected_occupied_cells){

        // We use this to determine whether to print out the given occupancy grid on failure
        bool print_occ_grid = false;

        // Iterate over all the cells
        for (int x = 0; x < occ_grid.info.width; x++){
            for (int y = 0; y < occ_grid.info.height; y++){
                int expected_cell_val = 0;
                // Check if this is one of the inflated cells
                if (std::find(expected_occupied_cells.begin(),
                              expected_occupied_cells.end(), std::make_pair(x,y))
                    != expected_occupied_cells.end()) {
                    expected_cell_val = 100;
                }
                // Check that the cell contains the value we expect
                EXPECT_EQ(expected_cell_val, (int)occ_grid.data[y * occ_grid.info.width + x])
                                    << "Failed on cell: (" << x << "," << y << ")" << std::endl;
                if (expected_cell_val != (int)occ_grid.data[y * occ_grid.info.width + x]){
                    print_occ_grid = true;
                }
            }
        }

        // Check that every cell we expected to be occupied is within the
        // bounds of the graph
        for (auto& cell : expected_occupied_cells){
            EXPECT_TRUE(
                    cell.first >= 0 &&
                    cell.first < occ_grid.info.width &&
                    cell.second >= 0 &&
                    cell.second < occ_grid.info.height
            ) << "Cell: (" << cell.first << "," << cell.second
              << ") not in graph bounds: (0,0)" << " to ("
              << occ_grid.info.width << "," << occ_grid.info.height << ")";
        }

        if (print_occ_grid){
            std::cout << "ACTUAL OCCUPANCY GRID:";
            printOccGrid(occ_grid);
        }
    }
};

TEST_F(ObstacleManagerTest, add_single_cone){
    std::vector<ConeObstacle> actual = obstacle_manager_with_one_cone.getConeObstacles();
    EXPECT_EQ(1, actual.size());
}

TEST_F(ObstacleManagerTest, add_cone_outside_of_merging_tolerance_positive_coordinates){
    // Expected obstacles are the current obstacles with our new one appended
    ConeObstacle cone2 = generateConeObstacle(0.4,0.31,0.4);
    std::vector<ConeObstacle> expected = obstacle_manager_with_one_cone.getConeObstacles();
    expected.push_back(cone2);

    // Add a second cone just outside of the merging tolerance
    obstacle_manager_with_one_cone.addObstacle(cone2);
    std::vector<ConeObstacle> actual = obstacle_manager_with_one_cone.getConeObstacles();

    EXPECT_EQ(2, actual.size());
    EXPECT_EQ(expected, actual);
}

TEST_F(ObstacleManagerTest, add_cone_outside_of_merging_tolerance_negative_coordinates){
    // Expected obstacles are the current obstacles with our new one appended
    ConeObstacle cone2 = generateConeObstacle(0.4,-0.31,0.4);
    std::vector<ConeObstacle> expected = obstacle_manager_with_one_cone.getConeObstacles();
    expected.push_back(cone2);

    // Add a second cone just outside of the merging tolerance
    obstacle_manager_with_one_cone.addObstacle(cone2);
    std::vector<ConeObstacle> actual = obstacle_manager_with_one_cone.getConeObstacles();

    EXPECT_EQ(2, actual.size());
    EXPECT_EQ(expected, actual);
}

TEST_F(ObstacleManagerTest, add_cone_within_of_merging_tolerance_positive_coordinates){
    // Since this cone is within merging tolerance, we expect that it will be
    // merged (which will overwrite the coordinates of the already present
    // cone with cone2)
    ConeObstacle cone2 = generateConeObstacle(0.4,0.29,0.4);
    std::vector<ConeObstacle> expected = {cone2};

    // Add a second cone just outside of the merging tolerance
    obstacle_manager_with_one_cone.addObstacle(cone2);
    std::vector<ConeObstacle> actual = obstacle_manager_with_one_cone.getConeObstacles();

    EXPECT_EQ(1, actual.size());
    EXPECT_EQ(expected, actual);
}

TEST_F(ObstacleManagerTest, add_cone_within_of_merging_tolerance_negative_coordinates){
    // Since this cone is within merging tolerance, we expect that it will be
    // merged (which will overwrite the coordinates of the already present
    // cone with cone2)
    ConeObstacle cone2 = generateConeObstacle(-0.4,-0.29,0.4);
    std::vector<ConeObstacle> expected = {cone2};

    // Add a second cone just outside of the merging tolerance
    obstacle_manager_with_one_cone.addObstacle(cone2);
    std::vector<ConeObstacle> actual = obstacle_manager_with_one_cone.getConeObstacles();

    EXPECT_EQ(1, actual.size());
    EXPECT_EQ(expected, actual);
}

TEST_F(ObstacleManagerTest, add_several_cones){
    ObstacleManager obstacle_manager(1,0);

    // A list of a cones, all within merging tolerance of each other
    std::vector<ConeObstacle> cones_within_merging_tolerance = {
            generateConeObstacle(5,5,0.2),
            generateConeObstacle(5.5,5,0.2),
            generateConeObstacle(5.2,5.1,0.2),
            generateConeObstacle(4.9,5.3,0.2)
    };

    // A list of cones, all far enough from each other to not be merged
    std::vector<ConeObstacle> cones_outside_merging_tolerance = {
            generateConeObstacle(0,0,0.3),
            generateConeObstacle(-10,-5,0.4),
            generateConeObstacle(-200,400000,0.4),
            generateConeObstacle(-10,100,0.4)
    };

    // Add all the cones, alternating between the two lists
    for (int i = 0; i < 4; i++){
        obstacle_manager.addObstacle(cones_within_merging_tolerance[i]);
        obstacle_manager.addObstacle(cones_outside_merging_tolerance[i]);
    }

    std::vector<ConeObstacle> expected;

    // We expect that all the cones within merging tolerance were merged,
    // with the final cone that was added taking priority over all prior
    expected.push_back(cones_within_merging_tolerance[3]);

    // We expect that all the cones that were greater then the merging tolerance
    // were not merged, and hence should all be present
    expected.insert(expected.end(),
                    cones_outside_merging_tolerance.begin(),
                    cones_outside_merging_tolerance.end());

    std::vector<ConeObstacle> actual = obstacle_manager.getConeObstacles();
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

// Test inflating a single point by 1 where the inflation radius does not extend beyound the edge
// of the occupancy grid
TEST_F(ObstacleManagerTest, inflate_single_point_inflate_radius_1_within_occ_grid){
    nav_msgs::OccupancyGrid occ_grid = generateEmptyOccGrid(5, 5, 0, 0, 0, 1);

    ObstacleManager::inflatePoint(occ_grid, Point2D(2,2), 1);

    std::vector<std::vector<OccupiedOrNot>> expected_occ_grid = {
            {_,_,_,_,_},
            {_,_,X,_,_},
            {_,X,X,X,_},
            {_,_,X,_,_},
            {_,_,_,_,_},
    };

    checkOccupiedCells(occ_grid, expected_occ_grid);
}

// Test inflating a single point by 2 where the inflation radius does not extend beyound the edge
// of the occupancy grid
TEST_F(ObstacleManagerTest, inflate_single_point_inflate_radius_2_within_occ_grid){
    nav_msgs::OccupancyGrid occ_grid = generateEmptyOccGrid(10, 10, 0, 0, 0, 1);

    ObstacleManager::inflatePoint(occ_grid, Point2D(5,5), 2);

    std::vector<std::vector<OccupiedOrNot>> expected_occ_grid = {
            {_,_,_,_,_,_,_,_,_,_},
            {_,_,_,_,_,_,_,_,_,_},
            {_,_,_,_,_,_,_,_,_,_},
            {_,_,_,_,_,X,_,_,_,_},
            {_,_,_,_,X,X,X,_,_,_},
            {_,_,_,X,X,X,X,X,_,_},
            {_,_,_,_,X,X,X,_,_,_},
            {_,_,_,_,_,X,_,_,_,_},
            {_,_,_,_,_,_,_,_,_,_},
            {_,_,_,_,_,_,_,_,_,_},
    };

    checkOccupiedCells(occ_grid, expected_occ_grid);
}

// Test inflating a single point by 3 where the inflation radius does not extend beyound the edge
// of the occupancy grid
TEST_F(ObstacleManagerTest, inflate_single_point_inflate_radius_3_within_occ_grid){
    nav_msgs::OccupancyGrid occ_grid = generateEmptyOccGrid(10, 10, 0, 0, 0, 1);

    ObstacleManager::inflatePoint(occ_grid, Point2D(5,5), 3);

    std::vector<std::vector<OccupiedOrNot>> expected_occ_grid = {
            {_,_,_,_,_,_,_,_,_,_},
            {_,_,_,_,_,_,_,_,_,_},
            {_,_,_,_,_,X,_,_,_,_},
            {_,_,_,X,X,X,X,X,_,_},
            {_,_,_,X,X,X,X,X,_,_},
            {_,_,X,X,X,X,X,X,X,_},
            {_,_,_,X,X,X,X,X,_,_},
            {_,_,_,X,X,X,X,X,_,_},
            {_,_,_,_,_,X,_,_,_,_},
            {_,_,_,_,_,_,_,_,_,_},
    };

    checkOccupiedCells(occ_grid, expected_occ_grid);
}

// Test inflating a single point by 1 where the inflation radius extends beyound the edge
// of the occupancy grid
TEST_F(ObstacleManagerTest, inflate_single_point_inflate_radius_1_partly_outside_occ_grid){
    nav_msgs::OccupancyGrid occ_grid = generateEmptyOccGrid(10, 5, 0, 0, 0, 1);

    ObstacleManager::inflatePoint(occ_grid, Point2D(5,4), 1);

    std::vector<std::vector<OccupiedOrNot>> expected_occ_grid = {
            {_,_,_,_,_,_,_,_,_,_},
            {_,_,_,_,_,_,_,_,_,_},
            {_,_,_,_,_,_,_,_,_,_},
            {_,_,_,_,_,X,_,_,_,_},
            {_,_,_,_,X,X,X,_,_,_},
    };

    checkOccupiedCells(occ_grid, expected_occ_grid);
}

// Test inflating a point on a rotated and offset grid
TEST_F(ObstacleManagerTest, inflate_single_point_offset_and_rotated_grid){
    nav_msgs::OccupancyGrid occ_grid = generateEmptyOccGrid(10, 20, 2, 3, M_PI/6, 1);

    ObstacleManager::inflatePoint(occ_grid, Point2D(10,5), 1);

    // The point (after being translated and rotated into the frame of reference
    // of the occupancy grid) is:
    // (5.92820323027551, 5.732050807568877)
    // We expect that this will be rounded to the closest cell: (6,6)

    std::vector<std::pair<int,int>> expected_occupied_cells = {
            std::make_pair(5,6),
            std::make_pair(6,5),
            std::make_pair(6,6),
            std::make_pair(6,7),
            std::make_pair(7,6),
    };

    checkOccupiedCells(occ_grid, expected_occupied_cells);

    std::vector<std::vector<OccupiedOrNot>> expected_occ_grid = {
            {_,_,_,_,_,_,_,_,_,_},
            {_,_,_,_,_,_,_,_,_,_},
            {_,_,_,_,_,_,_,_,_,_},
            {_,_,_,_,_,_,_,_,_,_},
            {_,_,_,_,_,_,_,_,_,_},
            {_,_,_,_,_,_,X,_,_,_},
            {_,_,_,_,_,X,X,X,_,_},
            {_,_,_,_,_,_,X,_,_,_},
            {_,_,_,_,_,_,_,_,_,_},
            {_,_,_,_,_,_,_,_,_,_},
            {_,_,_,_,_,_,_,_,_,_},
            {_,_,_,_,_,_,_,_,_,_},
            {_,_,_,_,_,_,_,_,_,_},
            {_,_,_,_,_,_,_,_,_,_},
            {_,_,_,_,_,_,_,_,_,_},
            {_,_,_,_,_,_,_,_,_,_},
            {_,_,_,_,_,_,_,_,_,_},
            {_,_,_,_,_,_,_,_,_,_},
            {_,_,_,_,_,_,_,_,_,_},
            {_,_,_,_,_,_,_,_,_,_},
    };

    checkOccupiedCells(occ_grid, expected_occ_grid);
}

// Test inflating a point on a grid with a floating point scale not equal to 1
TEST_F(ObstacleManagerTest, inflate_single_point_on_grid_with_floating_point_scale){
    double occ_grid_scale = 0.385;
    nav_msgs::OccupancyGrid occ_grid = generateEmptyOccGrid(20, 20, 0, 0, 0, occ_grid_scale);

    // Inflate a point by just below the size of a single cell
    // (so it should inflate to directly adjacent cells)
    Point2D inflated_point(2,2);
    ObstacleManager::inflatePoint(occ_grid, inflated_point, occ_grid_scale - 0.0001);

    // Calculate what the closest cell to the point we inflated is
    int expected_center_cell_x = (int)std::round(inflated_point.x() / occ_grid_scale);
    int expected_center_cell_y = (int)std::round(inflated_point.y() / occ_grid_scale);

    std::vector<std::pair<int,int>> expected_occupied_cells = {
            std::make_pair(expected_center_cell_x - 1, expected_center_cell_y),
            std::make_pair(expected_center_cell_x, expected_center_cell_y-1),
            std::make_pair(expected_center_cell_x, expected_center_cell_y),
            std::make_pair(expected_center_cell_x, expected_center_cell_y+1),
            std::make_pair(expected_center_cell_x + 1, expected_center_cell_y),
    };

    checkOccupiedCells(occ_grid, expected_occupied_cells);
}

// Test generating an occupancy grid with a single cone
TEST_F(ObstacleManagerTest, generate_occ_grid_with_single_cone){
    ObstacleManager obstacle_manager(1, 1, 0.1, 0.1);

    obstacle_manager.addObstacle(generateConeObstacle(10, 13, 0.099));

    nav_msgs::OccupancyGrid occ_grid = obstacle_manager.generateOccupancyGrid();

    // What we expect the occupancy grid to be
    std::vector<std::vector<OccupiedOrNot>> expected_occ_grid = {
            {_,_,X,_,_,},
            {_,X,X,X,_,},
            {X,X,X,X,X,},
            {_,X,X,X,_,},
            {_,_,X,_,_,},
    };

    checkOccupiedCells(occ_grid, expected_occ_grid);
}

// Test generating an occupancy grid with a single line
TEST_F(ObstacleManagerTest, generate_occ_grid_with_single_line){
    ObstacleManager obstacle_manager(1, 1, 0.5, 0.5);

    obstacle_manager.addObstacle(Spline({{1,1}, {3,1}, {4,3}, {7,0}}));

    nav_msgs::OccupancyGrid occ_grid = obstacle_manager.generateOccupancyGrid();

    std::vector<std::vector<OccupiedOrNot>> expected_occ_grid = {
            {_,_,_,_,_,_,_,_,_,_,_,_,_,X,_,},
            {_,_,X,X,X,X,_,_,_,_,_,_,X,X,X,},
            {_,X,X,X,X,X,X,_,_,_,_,X,X,X,_,},
            {X,X,X,X,X,X,X,_,_,_,_,X,X,X,_,},
            {_,X,_,_,X,X,X,_,_,_,X,X,X,_,_,},
            {_,_,_,_,_,X,X,X,X,X,X,X,_,_,_,},
            {_,_,_,_,_,X,X,X,X,X,X,_,_,_,_,},
            {_,_,_,_,_,_,X,X,X,X,_,_,_,_,_,},
            {_,_,_,_,_,_,_,X,X,_,_,_,_,_,_,},
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
    };

    checkOccupiedCells(occ_grid, expected_occ_grid);
}

// Test generating an occupancy grid with a line and a cone that are seperate
TEST_F(ObstacleManagerTest, generate_occ_grid_with_seperate_line_and_cone){
    ObstacleManager obstacle_manager(1, 1, 0, 0.5);

    obstacle_manager.addObstacle(Spline({{1,3}, {9,3}}));
    obstacle_manager.addObstacle(generateConeObstacle(4, 10, 1.49));

    nav_msgs::OccupancyGrid occ_grid = obstacle_manager.generateOccupancyGrid();

    std::vector<std::vector<OccupiedOrNot>> expected_occ_grid = {
            {X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,_,}, /* 0 */
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,}, /* 1 */
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,}, /* 2 */
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,}, /* 3 */
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,}, /* 4 */
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,}, /* 5 */
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,}, /* 6 */
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,}, /* 7 */
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,}, /* 8 */
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,}, /* 9 */
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,}, /* 10 */
            {_,_,_,_,_,_,X,_,_,_,_,_,_,_,_,_,_,_,}, /* 11 */
            {_,_,_,_,X,X,X,X,X,_,_,_,_,_,_,_,_,_,}, /* 12 */
            {_,_,_,_,X,X,X,X,X,_,_,_,_,_,_,_,_,_,}, /* 13 */
            {_,_,_,X,X,X,X,X,X,X,_,_,_,_,_,_,_,_,}, /* 14 */
            {_,_,_,_,X,X,X,X,X,_,_,_,_,_,_,_,_,_,}, /* 15 */
            {_,_,_,_,X,X,X,X,X,_,_,_,_,_,_,_,_,_,}, /* 16 */
            {_,_,_,_,_,_,X,_,_,_,_,_,_,_,_,_,_,_,}, /* 17 */
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,}, /* 18 */
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,}, /* 19 */
    };

    checkOccupiedCells(occ_grid, expected_occ_grid);
}

// Test generating an occupancy grid with a line and a cone that are overlapping
TEST_F(ObstacleManagerTest, generate_occ_grid_with_overlapping_line_and_cone){
    ObstacleManager obstacle_manager(1, 1, 0, 0.5);

    obstacle_manager.addObstacle(Spline({{1,3}, {9,3}}));
    obstacle_manager.addObstacle(generateConeObstacle(4, 3, 1.49));

    nav_msgs::OccupancyGrid occ_grid = obstacle_manager.generateOccupancyGrid();

    std::vector<std::vector<OccupiedOrNot>> expected_occ_grid = {
            {_,_,_,_,_,_,X,_,_,_,_,_,_,_,_,_,_,_,},
            {_,_,_,_,X,X,X,X,X,_,_,_,_,_,_,_,_,_,},
            {_,_,_,_,X,X,X,X,X,_,_,_,_,_,_,_,_,_,},
            {X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,_,},
            {_,_,_,_,X,X,X,X,X,_,_,_,_,_,_,_,_,_,},
            {_,_,_,_,X,X,X,X,X,_,_,_,_,_,_,_,_,_,},
            {_,_,_,_,_,_,X,_,_,_,_,_,_,_,_,_,_,_,},
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
    };

    checkOccupiedCells(occ_grid, expected_occ_grid);
}

// Test adding a cone and then adding a cone within "equality" distance of the first
// the obstacle manager should just treat the second cone as the "true" cone and
// the occupancy grid should reflect this
TEST_F(ObstacleManagerTest, generate_occ_grid_slightly_move_cone){
    ObstacleManager obstacle_manager(1, 1, 0, 0.5);

    // This spline is here for force the obstacle manager to generate a larger grid
    obstacle_manager.addObstacle(Spline({{1,3}, {9,3}}));


    obstacle_manager.addObstacle(generateConeObstacle(3.1, 10, 1.49));
    obstacle_manager.addObstacle(generateConeObstacle(4, 10, 1.49));

    nav_msgs::OccupancyGrid occ_grid = obstacle_manager.generateOccupancyGrid();

    std::vector<std::vector<OccupiedOrNot>> expected_occ_grid = {
            {X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,_,}, /* 0 */
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,}, /* 1 */
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,}, /* 2 */
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,}, /* 3 */
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,}, /* 4 */
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,}, /* 5 */
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,}, /* 6 */
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,}, /* 7 */
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,}, /* 8 */
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,}, /* 9 */
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,}, /* 10 */
            {_,_,_,_,_,_,X,_,_,_,_,_,_,_,_,_,_,_,}, /* 11 */
            {_,_,_,_,X,X,X,X,X,_,_,_,_,_,_,_,_,_,}, /* 12 */
            {_,_,_,_,X,X,X,X,X,_,_,_,_,_,_,_,_,_,}, /* 13 */
            {_,_,_,X,X,X,X,X,X,X,_,_,_,_,_,_,_,_,}, /* 14 */
            {_,_,_,_,X,X,X,X,X,_,_,_,_,_,_,_,_,_,}, /* 15 */
            {_,_,_,_,X,X,X,X,X,_,_,_,_,_,_,_,_,_,}, /* 16 */
            {_,_,_,_,_,_,X,_,_,_,_,_,_,_,_,_,_,_,}, /* 17 */
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,}, /* 18 */
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,}, /* 19 */
    };

    checkOccupiedCells(occ_grid, expected_occ_grid);
}

// Test adding two lines within merging distance and checking that they're
// merged properly in the output occ_grid
TEST_F(ObstacleManagerTest, generate_occ_grid_merged_lines){
    ObstacleManager obstacle_manager(1, 1, 0.5, 0.5);

    // Add the first line
    obstacle_manager.addObstacle(Spline({{1,1}, {3,1}, {10, 1}, {13,1}}));
    nav_msgs::OccupancyGrid occ_grid = obstacle_manager.generateOccupancyGrid();

    std::vector<std::vector<OccupiedOrNot>> expected_occ_grid = {
            {_,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,_,_,},
            {X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,_,},
            {_,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,_,_,},
    };

    checkOccupiedCells(occ_grid, expected_occ_grid);

    // Add the second line
    obstacle_manager.addObstacle(Spline({{1,1}, {3,1}, {4,3}, {7,0}}));
    occ_grid = obstacle_manager.generateOccupancyGrid();

    expected_occ_grid = {
            {_,_,_,_,_,_,_,_,_,_,_,_,X,X,X,X,_,_,_,_,_,_,_,_,_,_,_,_,},
            {_,_,_,_,_,_,_,_,_,_,_,X,X,X,X,X,X,X,_,_,_,_,_,_,_,_,_,_,},
            {_,X,X,X,X,X,_,_,_,_,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,X,_,_,},
            {X,X,X,X,X,X,X,_,_,X,X,X,X,_,_,X,X,X,X,X,X,X,X,X,X,X,X,_,},
            {_,X,X,X,X,X,X,X,_,X,X,X,_,_,_,_,_,_,X,X,X,X,X,X,X,X,_,_,},
            {_,_,_,_,_,X,X,X,X,X,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
            {_,_,_,_,_,X,X,X,X,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
            {_,_,_,_,_,X,X,X,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
            {_,_,_,_,_,_,X,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
            {_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,},
    };

    checkOccupiedCells(occ_grid, expected_occ_grid);
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