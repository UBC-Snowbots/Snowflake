/*
 * Created By: Min Gyo Kim
 * Created On: May 14th 2018
 * Description: A service that takes in the output of AStar, represented as a
 * stack of cell locations, and produces the actual path
 *              containing poses in map frame
 */

#ifndef PATHFINDING_IGVC_PATHCONSTRUCTIONSERVICE_H
#define PATHFINDING_IGVC_PATHCONSTRUCTIONSERVICE_H

#include <AStar.h>
#include <OccupancyGridConversionService.h>

class PathConstructionService {
    OccupancyGridConversionService *_occupancy_grid_conversion_service;

  public:
    /**
     * Takes in an OccupancyGridConversionService and returns a
     * PathConstructionService.
     * The occupancy grid conversion service is used to convert the cell
     * location of the points to
     * an actual point in the map frame.
     *
     * @param occupancy_grid_conversion_service
     * @return PathConstructionService
     */
    static PathConstructionService buildService(
    OccupancyGridConversionService occupancy_grid_conversion_service);

    /**
     * Takes in a stack of GridPoints and returns a path
     *
     * @param points a stack of GridPoints where the top contains the starting
     * GridPoint and the bottom contains the goal GridPoint
     * @return path
     */
    nav_msgs::Path constructPath(std::stack<AStar::GridPoint> points);
};

#endif // PATHFINDING_IGVC_PATHCONSTRUCTIONSERVICE_H
