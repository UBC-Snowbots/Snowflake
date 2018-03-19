/*
 * Created By: Gareth Ellis
 * Created On: January 27, 2018
 * Description: TODO
 */

#include "sb_geom/SplineLine.h"

using namespace sb_geom;

SplineLine::SplineLine(std::vector<Point2D> points) {
    // Convert the points into Eigen's representation
    Eigen::MatrixXd knot_points(2, points.size());
    for (int i = 0; i < points.size(); i++){
        knot_points(0, i) = points[i].x();
        knot_points(1, i) = points[i].y();
    }
    // Interpolate the internal spline through the points
    _spline = Eigen::SplineFitting<Eigen::Spline2d>::Interpolate(
            knot_points,points.size()-1);
}

double SplineLine::approxLength(int num_sample_points) {
    double length = 0;

    Point2D prev_point(_spline(0)(0), _spline(0)(1));
    Point2D curr_point;
    double increment = 1/num_sample_points;
    for (int i = increment; i < 1; i += increment){
        curr_point.x() = _spline(i)(0);
        curr_point.x() = _spline(i)(1);

        // Calculate distance between current and previous point
        double dx = curr_point.x() - prev_point.x();
        double dy = curr_point.y() - prev_point.y();
        length += std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

        // Update the prev point
        prev_point.x() = curr_point.x();
        prev_point.y() = curr_point.y();
    }

    // Explicitly add the last point in the line in case we had
    // a large increment
    curr_point.x() = _spline(1)(0);
    curr_point.x() = _spline(1)(1);
    double dx = curr_point.x() - prev_point.x();
    double dy = curr_point.y() - prev_point.y();
    length += std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

    return length;
}

Point2D SplineLine::operator()(double u){
    if (u < 0 || u > 1){
        // Throw an exception if u is outside of [0,1]
        std::string err_msg = "u must be between 0 and 1, given: " + std::to_string(u);
        throw std::out_of_range(err_msg);
    }
    return Point2D(_spline(u)(0), _spline(u)(1));
}
