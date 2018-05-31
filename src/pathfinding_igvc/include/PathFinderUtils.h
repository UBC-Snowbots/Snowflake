/*
 * Created By: Min Gyo Kim
 * Created On: May 13th 2018
 * Description: A class that contains all generic helper functions for all
 * implementations of path finding
 */

#ifndef PATHFINDING_IGVC_PATHFINDERUTILS_H
#define PATHFINDING_IGVC_PATHFINDERUTILS_H

#include "AStar.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>

class PathFinderUtils {
  public:
    /**
     * Takes a geometry_msgs::Point and converts it to type tf::Vector3
     *
     * @param point point of type geometry_msgs::Point
     * @return vector of type tf::Vector3
     */
    static tf::Vector3 pointToVector(geometry_msgs::Point point) {
        return tf::Vector3(point.x, point.y, point.z);
    }

    /**
     * Takes a tf::Vector3 and converts it to type geometry_msgs::Point
     *
     * @param vector3 vector of type tf::Vector3
     * @return point of type geometry_msgs::Point
     */
    static geometry_msgs::Point vectorToPoint(tf::Vector3 vector3) {
        geometry_msgs::Point p;
        p.x = vector3.x();
        p.y = vector3.y();
        p.z = vector3.z();
        return p;
    }

    /**
     * Constructs a 2D geometry_msgs::PoseStamped given a 2D point (only care
     * about x and y)
     * and z-angle
     *
     * @param point 2D point (only care about x and y)
     * @param angle angle in z-axis
     * @return pose stamped
     */
    static geometry_msgs::PoseStamped
    constructPoseStamped(geometry_msgs::Point point, double angle) {
        geometry_msgs::Pose pose;

        pose.position    = point;
        tf::Quaternion q = getQuaternionFromAngle(angle);
        tf::quaternionTFToMsg(q, pose.orientation);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose = pose;

        return pose_stamped;
    }

    /**
     * Returns the angle of the vector connecting two 2D points
     *
     * @param from the origin point of vector
     * @param to the end point (arrow head) of vector
     * @return angle or direction of the vector
     */
    static double getAngleBetweenPoints(geometry_msgs::Point from,
                                        geometry_msgs::Point to) {
        tf::Vector3 vec = pointToVector(to) - pointToVector(from);
        return atan2(vec.y(), vec.x());
    }

    /**
     * Returns the quaternion that represents an angle in z-axis
     *
     * @param angle angle in z-axis
     * @return quaternion quaternion representation of the angle
     */
    static tf::Quaternion getQuaternionFromAngle(double angle) {
        tf::Quaternion q;
        tf::Matrix3x3 rotationMatrix = tf::Matrix3x3();
        rotationMatrix.setEulerYPR(
        angle, 0.0, 0.0); // only set Z rotation since it's 2D
        rotationMatrix.getRotation(q);

        return q;
    }

    /**
     * Returns whether a given grid point is inside the occupancy grid.
     *
     * @param grid_info the metadata of the occupancy grid
     * @param point the point in question
     * @return
     */
    static bool isPointInsideGrid(nav_msgs::MapMetaData grid_info,
                                  AStar::GridPoint point) {
        if (point.col < 0 || point.row < 0) return false;
        if (point.col >= grid_info.width) return false;
        if (point.row >= grid_info.height) return false;
        return true;
    }

    /**
     * Fits the grid point inside the occupancy grid by updating it to
     * be the closest cell to the goal that is still in the grid.
     *
     * @param grid_info
     * @param point
     */
    static void fitPointInsideGrid(nav_msgs::MapMetaData grid_info,
                                   AStar::GridPoint& point) {
        point.col = point.col < 0 ? 0 : point.col;
        point.col =
        point.col >= grid_info.width ? grid_info.width - 1 : point.col;

        point.row = point.row < 0 ? 0 : point.row;
        point.row =
        point.row >= grid_info.height ? grid_info.height - 1 : point.row;
    }

    static AStar::GridPoint getClosestFreeGridPointFromGridPoint(nav_msgs::OccupancyGrid grid, AStar::GridPoint point) {
        int radius = 0;

        int max_radius = 10000; //TODO: add to launch file?

        while (radius < max_radius) {
            // left
            if ( (point.col - radius) > 0) {
                AStar::GridPoint left(point.row, point.col - radius);
                if (isGridPointFree(grid, left)) {
                    return left;
                }
            }
            // right
            if ( (point.col + radius) < grid.info.width) {
                AStar::GridPoint right(point.row, point.col + radius);
                if (isGridPointFree(grid, right)) {
                    return right;
                }
            }
            // top
            if ( (point.row + radius) < grid.info.height) {
                AStar::GridPoint top(point.row + radius, point.col);
                if (isGridPointFree(grid, top)) {
                    return top;
                }
            }
            // bottom
            if ( (point.row - radius) > 0 ) {
                AStar::GridPoint bottom(point.row - radius, point.col);
                if (isGridPointFree(grid, bottom)) {
                    return bottom;
                }
            }

            // increase radius
            radius++;
        }

        // couldn't find closest free grid point, so just return original point
        return point;
    }

    static bool isGridPointFree(nav_msgs::OccupancyGrid grid, AStar::GridPoint point) {
        return grid.data[point.row * grid.info.width + point.col] == AStar::GRID_FREE;
    }
};

#endif // PATHFINDING_IGVC_PATHFINDERUTILS_H
