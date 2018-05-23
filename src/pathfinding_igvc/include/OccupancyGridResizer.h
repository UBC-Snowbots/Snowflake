/*
 * Created By: Min Gyo Kim
 * Created On: May 14th 2018
 * Description: A class that adds space around the grid, i.e.
 * an additional row below the grid and above the grid, and
 * an additional column to the left and right of the grid.
 */

#ifndef PATHFINDING_IGVC_OCCUPANCYGRIDRESIZER_H
#define PATHFINDING_IGVC_OCCUPANCYGRIDRESIZER_H

#include "AStar.h"
#include <OccupancyGridAdapter.h>

class OccupancyGridResizer {
  public:
    /**
     * Given an occupancy grid, adds space around the grid, i.e.
     * an additional row below the grid and above the grid, and
     * an additional column to the left and right of the grid.
     *
     * The grid will also move its origin down by one cell and left by
     * one cell.
     *
     * @param grid the occupancy grid passed by reference
     */
    void static addSpaceAroundGrid(nav_msgs::OccupancyGrid& grid);

  private:
    /**
     * Given an occupancy grid, adds an additional column to the left
     * of the grid. The grid's origin is also moved to the left by one cell.
     *
     * @param grid
     */
    void static addSpaceLeft(nav_msgs::OccupancyGrid& grid);

    /**
     * Given an occupancy grid, adds an additional column to the right
     * of the grid. The grid's origin is unchanged.
     *
     * @param grid
     */
    void static addSpaceRight(nav_msgs::OccupancyGrid& grid);

    /**
     * Given an occupancy grid, adds an additional row above
     * the grid. The grid's origin is unchanged.
     *
     * @param grid
     */
    void static addSpaceUp(nav_msgs::OccupancyGrid& grid);

    /**
     * Given an occupancy grid, adds an additional row below
     * the grid. The grid's origin is also moved down by one cell.
     *
     * @param grid
     */
    void static addSpaceDown(nav_msgs::OccupancyGrid& grid);

    /**
     * Given an occupancy grid, updates origin by moving the origin
     * one cell down and one cell left.
     *
     * @param grid
     */
    void static updateOrigin(nav_msgs::OccupancyGrid& grid);
};

#endif // PATHFINDING_IGVC_OCCUPANCYGRIDRESIZER_H
