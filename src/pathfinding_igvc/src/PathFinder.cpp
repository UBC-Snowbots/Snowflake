/*
 * Created By: Min Gyo Kim
 * Created On: May 5th 2018
 * Description: Class that acts as an interface between PathFinderNode and
 * AStar.
 *              Deals with a lot of frame transformations and data type
 * conversions between ROS types and native types.
 *              Does not actually calculate the path itself - depends on AStar
 * to get the path.
 */

#include <PathFinder.h>

nav_msgs::Path PathFinder::calculatePath(geometry_msgs::Point start,
                                   geometry_msgs::Point goal,
                                   nav_msgs::OccupancyGrid grid) {
    AStar::GridPoint goal_on_grid;
    AStar::GridPoint start_on_grid;
    processGridAndGetStartAndGoalOnGrid(grid, start, goal, start_on_grid, goal_on_grid);

    std::stack<AStar::GridPoint> points =
    AStar().run(grid, start_on_grid, goal_on_grid);
    return PathConstructor(
           OccupancyGridAdapter(grid.info))
    .constructPath(points);
}

void PathFinder::processGridAndGetStartAndGoalOnGrid(nav_msgs::OccupancyGrid &grid, geometry_msgs::Point start, geometry_msgs::Point goal, AStar::GridPoint &start_on_grid, AStar::GridPoint &goal_on_grid) {
    AStar::GridPoint initial_goal_on_grid = OccupancyGridAdapter(grid.info).convertFromMapToGridPoint(goal);
    bool grid_needs_resizing = !PathFinderUtils::isPointInsideGrid(grid.info, initial_goal_on_grid);
    if (grid_needs_resizing) {
        OccupancyGridResizer::addSpaceAroundGrid(grid);
    }

    OccupancyGridAdapter occupancy_grid_adapter = OccupancyGridAdapter(grid.info);
    start_on_grid = occupancy_grid_adapter.convertFromMapToGridPoint(start);
    goal_on_grid =
            occupancy_grid_adapter.convertFromMapToGridPoint(goal);

    bool point_needs_fitting = !PathFinderUtils::isPointInsideGrid(grid.info, goal_on_grid);
    if (point_needs_fitting) {
        PathFinderUtils::fitPointInsideGrid(grid.info, goal_on_grid);
    }
}
