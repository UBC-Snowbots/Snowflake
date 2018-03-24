/*
 * Created By: Gareth Ellis
 * Created On:  March 19th, 2018
 * Description: Geometry Utility Functions
 */


// TODO: Should this file be called "utils.h" instead of "util.h"?

#ifndef SB_GEOM_UTIL_H
#define SB_GEOM_UTIL_H

// Snowbots Includes
#include "sb_geom/Point2D.h"
#include "sb_geom/PolynomialSegment.h"
#include "sb_geom/Spline.h"

namespace sb_geom {
    // TODO: Test me
    // TODO: Do we need this function any more
    /**
     * Computes the shortest distance between a given point and polynomial line segment
     *
     * Uses Halley Iteration
     * @param point TODO
     * @param line TODO
     * @param max_iter the maximum number of iterations to perform, more iterations
     * will lead to more accurate results, but increases computation time
     * @return the minimum distance between the point and polynomial line
     */
    double minDistanceFromPointToPolynomialSegment(
            Point2D point, PolynomialSegment line, uintmax_t max_iter = 20);

    /**
     * Computes the minimum distance between two given splines
     *
     * Will terminate if the # of iterations reaches `max_iter` or the error drops to at or below `min_err`
     * @param s1 the first spline
     * @param s2 the second spline
     * @param max_iter the maximum number of iterations to perform, more iterations
     * will lead to more accurate results, but increases computation time
     * @param min_err the maximum allowable error in the result
     * @return the minimum distance between `s1` and `s2`
     */
    double minDistanceBetweenSplines(Spline s1, Spline s2, uintmax_t max_iter);

    // TODO: Test me
    // TODO: Should this go in the Polynomial class?
    /**
     * Find the real roots of a given polynomial
     *
     * To do this we use the GNU Scientific Libraries General Polynomial Equations:
     * https://www.gnu.org/software/gsl/manual/html_node/General-Polynomial-Equations.html
     * @param poly the polynomial line to find the roots of
     * @return the x-values of the roots of the polynomial in *arbitrary* order
     */
    std::vector<double> findRealRoots(Polynomial poly);

    // TODO: Test me
    /**
     * Finds the closest point on the given spline to the given point
     *
     * It does this by first sampling points along the spline, finding the closest two to
     * the given point, and then re-sampling the section of line between the two closest points
     * until the difference in distance from each of the two points to the given point is < `max_err`
     *
     * @param spline TODO?
     * @param point TODO?
     * @param num_sample_points the number of sample points to sample from the spline
     * @param max_err the maximum allowable error in the distance
     * @return the distance along the spline (in [0,1]) closest to the given point
     */
    double findClosestPointOnSplineToPoint(
            Spline spline, Point2D point, unsigned int num_sample_points, double max_err
    );

    // TODO: Test me
    /**
     * Find a set of interpolation points in a Polynomial Segment
     *
     * We do this by choosing the points to be the start point, end point,
     * and critical points of the polynomial line
     *
     * @param poly_segment the Polynomial Segment to find the points in
     * @return the interpolation points
     */
    std::vector<Point2D> getInterpolationPointsFromPolySegment(PolynomialSegment poly_segment);

    /**
     * Find the distance between two given points
     * @param p1
     * @param p2
     * @return the distance from `p1` to `p2`
     */
    double distance(Point2D p1, Point2D p2);
}

#endif //SB_GEOM_UTIL_H
