/*
 * Created By: Gareth Ellis
 * Created On: January 27, 2018
 * Description: A class representing a Spline in 2D space.
 */

// Snowbots Includes
#include "sb_geom/Spline.h"
#include "sb_geom/utils.h"

// GNU Scientific Library Includes
#include <gsl/gsl_poly.h>

using namespace sb_geom;

Spline::Spline(std::vector<Point2D> points) : interpolation_points(points) {
    interpolate();
}

Spline::Spline(sb_geom::PolynomialSegment poly_segment) {
    // Get the interpolation points from the polynomial segment
    interpolation_points = getInterpolationPointsFromPolySegment(poly_segment);

    // Interpolate through the points
    interpolate();
}

double Spline::approxLength(int num_sample_points) {
    double length = 0;

    Point2D prev_point(this->getPointAtZeroToOneIndex(0));
    Point2D curr_point;
    double increment = 1.0 / num_sample_points;
    for (int i = 0; i <= num_sample_points; i++) {
        curr_point = this->getPointAtZeroToOneIndex(i * increment);
        // Calculate distance between current and previous point
        double dx = curr_point.x() - prev_point.x();
        double dy = curr_point.y() - prev_point.y();
        length += std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

        // Update the prev point
        prev_point = curr_point;
    }

    return length;
}

std::vector<Point2D> Spline::getInterpolationPointsInRange(double start_u,
                                                           double end_u) {
    // If the start is after the end, or the range is out of bounds, just return
    // an empty vector
    if (start_u > end_u || start_u > 1 || end_u < 0) {
        return std::vector<Point2D>();
    }

    // Cap the u values
    start_u = std::max(0.0, start_u);
    end_u   = std::min(1.0, end_u);

    // Scale the given u value from [0,1] -> [0,n-1] where n is the number of
    // points the spline interpolates through and round down and up respectively
    double start_index = std::ceil(start_u * (interpolation_points.size() - 1));
    double end_index =
    std::floor(end_u * (interpolation_points.size() - 1)) + 1;

    // Check for case where we're just getting the exactly one point
    if (start_index == end_index &&
        start_index < interpolation_points.size() - 1) {
        return {interpolation_points[start_index]};
    }
    // Check for case where we're getting exactly the last point
    // This is here because of the way the rounding works out, the index found
    // will be out of range
    if (start_u == 1 && end_u == 1) {
        return {interpolation_points[interpolation_points.size() - 1]};
    }

    // Get the interpolation points in the given index range
    return std::vector<Point2D>(interpolation_points.begin() + start_index,
                                interpolation_points.begin() + end_index);
}

Point2D Spline::getPointAtZeroToOneIndex(double u) {
    if (u < 0 || u > 1) {
        // Throw an exception if u is outside of [0,1]
        std::string err_msg =
        "u must be between 0 and 1, given: " + std::to_string(u);
        throw std::out_of_range(err_msg);
    }
    // Scale the given u value from [0,1] -> [0,n] where n is the number of
    // points
    // the spline interpolates through
    u = u * (interpolation_points.size() - 1);
    return Point2D(alglib::spline1dcalc(x_interpolant, u),
                   alglib::spline1dcalc(y_interpolant, u));
}

Point2D Spline::getPointAtZeroToNIndex(double u) {
    if (u < 0 || u > interpolation_points.size() - 1) {
        // Throw an exception if u is outside of [0,1]
        std::string err_msg =
        "u must be between 0 and the # of interpolation points " +
        std::to_string(interpolation_points.size()) +
        ", given: " + std::to_string(u);
        throw std::out_of_range(err_msg);
    }
    return Point2D(alglib::spline1dcalc(x_interpolant, u),
                   alglib::spline1dcalc(y_interpolant, u));
}

Point2D Spline::operator()(double u) {
    return getPointAtZeroToOneIndex(u);
}

void Spline::interpolate() {
    /**
     * Because a spline might curve back on itself (imagine a spiral) we can't
     * define it
     * in terms of x or y, as there could be multiple x values for a given y
     * value, or vice-versa.
     * Instead, we define it in terms of a third variable, u. The two splines we
     * define here are basically
     * x(u) and y(u). So if we want a point somewhere along the spline, we
     * choose the appropriate
     * u value, and then call x(u) and y(u) to get the (x,y) point at that u
     * value.
     */

    // Parametrize the points in terms of u in [0,n] where n=points.size()
    alglib::real_1d_array x;
    alglib::real_1d_array y;
    alglib::real_1d_array u;
    x.setlength(interpolation_points.size());
    y.setlength(interpolation_points.size());
    u.setlength(interpolation_points.size());
    for (int i = 0; i < interpolation_points.size(); i++) {
        Point2D& point = interpolation_points[i];
        x[i]           = point.x();
        y[i]           = point.y();
        u[i]           = i;
    }

    alglib::spline1dbuildakima(u, x, x_interpolant);
    alglib::spline1dbuildakima(u, y, y_interpolant);
}
