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

nav_msgs::Path PathFinder::perform(geometry_msgs::Point start,
                                   geometry_msgs::Point goal,
                                   nav_msgs::OccupancyGrid grid) {
    OccupancyGridConversionService occupancy_grid_conversion_service =
    OccupancyGridConversionService(grid.info);

    AStar::GridPoint start_on_grid =
    occupancy_grid_conversion_service.convertToGridPoint(start);
    AStar::GridPoint goal_on_grid =
    occupancy_grid_conversion_service.convertToGridPoint(goal);
    OccupancyGridResizeService::resizeOccupancyGridToFitGoal(grid,
                                                             goal_on_grid);

    std::stack<AStar::GridPoint> points =
    AStar().run(grid, start_on_grid, goal_on_grid);
    return PathConstructionService::buildService(
           occupancy_grid_conversion_service)
    .constructPath(points);
}
