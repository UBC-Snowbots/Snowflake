/*
 * Created By: Gareth Ellis
 * Created On:  March 19th, 2018
 * Description: Geometry Utility Functions
 */

// Snowbots Includes
#include "sb_geom/utils.h"

// STD Includes
#include <functional>

// Boost Includes
#include <boost/math/tools/roots.hpp>

// GNU Scientific Library Includes
#include <gsl/gsl_poly.h>

// dlib Includes
#include <dlib/global_optimization/find_max_global.h>
#include <dlib/optimization.h>

using namespace sb_geom;

double sb_geom::minDistanceBetweenSplines(Spline s1,
                                          Spline s2,
                                          unsigned int max_iter) {
    // Define a function for the distance between the two splines
    // `u` is the distance along the first spline (in [0,1])
    // `t` is the distance along the second spline (in [0,1])
    auto distanceBetweenSplines = [&](double u, double t) {
        Point2D p1 = s1(u);
        Point2D p2 = s2(t);

        double dx = p1.x() - p2.x();
        double dy = p1.y() - p2.y();

        return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    };

    auto min_result =
    dlib::find_min_global(distanceBetweenSplines,
                          {0, 0}, // lower bounds on the search
                          {1, 1}, // upper bounds on the search
                          dlib::max_function_calls(max_iter));

    double minimizing_u = min_result.x(0);
    double minimizing_t = min_result.x(1);

    // Return the distance between the two found points
    return distanceBetweenSplines(minimizing_u, minimizing_t);
}

std::vector<double> sb_geom::findRealRoots(sb_geom::Polynomial poly) {
    double* coefficients = &poly.coefficients()[0];

    // Since `gsl_poly_complex_solve` requires the leading term of the
    // polynomial to be
    // non-zero, choose the degree to be less then the actual degree of the
    // polnomial so this is the case
    unsigned int degree = poly.getDegree();
    for (; degree > 0 && coefficients[degree - 1] == 0; degree--) {}

    // If the degree is 0, then there are no roots
    if (degree == 0) { return std::vector<double>(); }

    // Allocate a the `gsl_poly_complex_workspace` used to solve the polynomial
    // in later
    gsl_poly_complex_workspace* workspace =
    gsl_poly_complex_workspace_alloc(degree);

    // This is what our roots will be returned in
    double z[2 * degree];

    // Solve for the roots of the polynomial
    int success = gsl_poly_complex_solve(coefficients, degree, workspace, z);

    // Free the workspace we allocated above
    gsl_poly_complex_workspace_free(workspace);

    // Put all the real roots in a vector and return it
    std::vector<double> roots;
    for (int i = 0; i < poly.getDegree(); i++) {
        // Get the real and imaginary parts of the root
        double real_part      = z[2 * i];
        double imaginary_part = z[2 * i + 1];
        // Because of numerical instability, round to the 12th decimal place
        // (because zero values might show up as non-zero)
        real_part = std::round(real_part * 1e12) / 1e12;
        // If it's real, non-zero and we don't have it already, add it to our
        // roots
        bool already_found =
        std::find(roots.begin(), roots.end(), real_part) != roots.end();
        if (imaginary_part == 0 && real_part != 0 && !already_found) {
            roots.emplace_back(real_part);
        }
    }

    // Check for case where no roots have non-zero real parts, but there is a
    // real root at 0
    if (poly(0) == 0) { roots.emplace_back(0); }

    return roots;
}

double sb_geom::findClosestPointOnSplineToPoint(Spline spline,
                                                Point2D point,
                                                unsigned int max_iter) {
    // Define a function for the distance from the spline to the given point in
    // terms
    // of the position on the spline (`u`)
    auto distanceFromPointToSpline = [&](double u) {
        Point2D point_on_spline = spline(u);
        double dx               = point_on_spline.x() - point.x();
        double dy               = point_on_spline.y() - point.y();

        return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    };

    auto min_result = dlib::find_min_global(distanceFromPointToSpline,
                                            {0}, // lower bound on the search
                                            {1}, // upper bound on the search
                                            dlib::max_function_calls(max_iter));

    double minimizing_u = min_result.x(0);

    return minimizing_u;
}

std::vector<Point2D>
sb_geom::getInterpolationPointsFromPolySegment(PolynomialSegment poly_segment) {
    std::vector<Point2D> interpolation_points;

    // Add the start point of the polynomial segment
    interpolation_points.emplace_back(
    Point2D(poly_segment.x_min(), poly_segment(poly_segment.x_min())));

    // Add an interpolation point for every critical point in the polynomial
    // segment
    // We do this by find the roots of the first derivative
    std::vector<double> roots = findRealRoots(poly_segment.deriv(1));
    std::sort(roots.begin(), roots.end());
    for (double& root : roots) {
        if (root > poly_segment.x_min() && root < poly_segment.x_max())
            ;
        interpolation_points.emplace_back(Point2D(root, poly_segment(root)));
    }

    // Add the end point of the Polynomial Segment
    interpolation_points.emplace_back(
    Point2D(poly_segment.x_max(), poly_segment(poly_segment.x_max())));

    return interpolation_points;
}

double sb_geom::distance(Point2D p1, Point2D p2) {
    double dx = p1.x() - p2.x();
    double dy = p1.y() - p2.y();
    return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
}
