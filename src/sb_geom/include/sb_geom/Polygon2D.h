/*
 * Created By: Gareth Ellis
 * Created On:  Nov. 11th, 2018
 * Description: A class representing a 2D Polygon
 */

#pragma once

// Snowbots Includes
#include "Point2D.h"
#include <sb_geom_msgs/Polygon2D.h>

namespace sb_geom {

class Polygon2D {
  public:
    // Delete the default constructor
    Polygon2D() = delete;

    /**
     * Constructs a Polygon2D from a given list of points
     *
     * @param boundary_points A vector of points representing the boundary,
     * where the first and last points are assumed to be connected
     */
    explicit Polygon2D(const std::vector<Point2D>& boundary_points)
      : boundary_points(std::move(boundary_points)){};

    /**
     * Constructs a Polygon2D from a given list of point messages
     *
     * @param boundary_points A vector of point messages representing the
     * boundary, where the first and last points are assumed to be connected
     */
    explicit Polygon2D(
    const std::vector<sb_geom_msgs::Point2D>& boundary_points)
      : Polygon2D(
        std::vector<Point2D>(boundary_points.begin(), boundary_points.end())){};

    /**
     * Constructs a Polygon2D from a given Polygon2D ROS message
     */
    explicit Polygon2D(const sb_geom_msgs::Polygon2D& polygon2D)
      : Polygon2D(polygon2D.points){};

    /**
     * Gets the points making up the boundary of this Polygon
     */
    std::vector<Point2D> getBoundaryPoints() { return this->boundary_points; }

  private:
    // The points that make up the boundary of this area
    // The first and last points in the list are assumed to be connected
    std::vector<Point2D> boundary_points;
};

inline bool operator==(Polygon2D poly1, Polygon2D poly2) {
    return poly1.getBoundaryPoints() == poly2.getBoundaryPoints();
}
}
