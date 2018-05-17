#ifndef SB_GEOM_POINT2D_H
#define SB_GEOM_POINT2D_H

#include "sb_geom_msgs/Point2D.h"

namespace sb_geom {

class Point2D {
  public:
    /**
     * Construct a Point2D at 0,0
     */
    Point2D() : _x(0), _y(0){};

    /**
     * Construct a Point2D with given x and y values
     * @param x
     * @param y
     */
    Point2D(double x, double y) : _x(x), _y(y){};

    /**
     * Construct a Point2D from a Point2D msg via Copy Constructor
     *
     * @param point_msg
     */
    Point2D(const sb_geom_msgs::Point2D& point_msg)
      : _x(point_msg.x), _y(point_msg.y){};

    /**
     * Construct a Point2D from a Point2D msg via assignment
     * Copy Constructor
     * @param point_msg
     */
    Point2D& operator=(const sb_geom_msgs::Point2D& point_msg) {
        _x = point_msg.x;
        _y = point_msg.y;
    };

    /**
     * Get the x coordinate of this point
     * @return a reference to the x coordinate of this point
     */
    double& x() { return _x; }

    /**
     * Get the y coordinate of this point
     * @return a reference to the y coordinate of this point
     */
    double& y() { return _y; }

  private:
    // The x and y coordinates of this point
    double _x, _y;
};

inline bool operator==(Point2D p1, Point2D p2) {
    return (p1.x() == p2.x() && p1.y() == p2.y());
}
}

#endif // SB_GEOM_POINT2D_H
