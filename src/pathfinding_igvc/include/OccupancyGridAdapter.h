/*
 * Created By: Min Gyo Kim
 * Created On: May 13th 2018
 * Description: A class that calculates the location of a point in map frame
 * in the occupancy grid, and vice versa.
 *              This class depends on FrameTransformer.
 */

#ifndef PATHFINDING_IGVC_OCCUPANCYGRIDADAPTER_H
#define PATHFINDING_IGVC_OCCUPANCYGRIDADAPTER_H

#include <AStar.h>
#include <FrameTransformer.h>
#include <nav_msgs/OccupancyGrid.h>

class OccupancyGridAdapter {
    nav_msgs::MapMetaData _grid_info;
    FrameTransformer* _frame_transformer;

  public:
    /**
     * Takes in occupancy grid meta data and creates a class
     * that calculates the location of a point represented in map frame in
     * occupancy grid
     *
     * @param info map meta data of occupancy grid
     * @return OccupancyGridAdapter
     */
    OccupancyGridAdapter(nav_msgs::MapMetaData info);

    /**
     * Takes in a point represented in map frame and returns a struct
     * that contains the corresponding row and column of the point in the
     * occupancy grid.
     *
     * @param point a point represented in map frame
     * @return AStar::GridPoint containing row and column of the point in grid
     */
    AStar::GridPoint convertFromMapToGridPoint(geometry_msgs::Point point);

    /**
     * Takes in a struct containing the row and column of a point in the
     * occupancy grid and returns its representation in map frame
     *
     * @param AStar::GridPoint containing row and column of a point in grid
     * @return point represented in map frame
     */
    geometry_msgs::Point convertFromGridToMapPoint(AStar::GridPoint point);
};

#endif // PATHFINDING_IGVC_OCCUPANCYGRIDADAPTER_H
