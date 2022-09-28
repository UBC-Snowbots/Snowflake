/*
 * Created By: Gareth Ellis
 * Created On: January 27, 2018
 * Description: A class representing a Spline in 2D space.
 */

#ifndef SB_GEOM_SPLINELINE_H
#define SB_GEOM_SPLINELINE_H

// Snowbots Includes
#include "sb_geom/Point2D.h"
#include "sb_geom/PolynomialSegment.h"
#include "sb_geom_msgs/SplineLine.h"

// Alglib Includes
#include <libalglib/interpolation.h>

namespace sb_geom {

/**
 * The Spline class represents a Spline interpolated through some points
 *
 * Since the points may not be in strictly increasing order of x-value,
 * the way that a point on the spline is accessed is by a value in [0,1],
 * where 0 is the start of the start of the spline and 1 is the end of the
 * spline. For example:
 * <code>
 *    std::vector<sb_geom::Point2D> points = {
 *            {0,0},
 *            {3,3},
 *            {10,1},
 *            {4, 10}
 *    };
 *    sb_geom::Spline spline(points);
 *
 *    // The start point of the spline
 *    sb_geom::Point2D start = spline(0);
 *
 *    // The end point of the spline
 *    sb_geom::Point2D end = spline(1);
 *
 *    // Some point in the middle of the spline
 *    // (there are no guarantees made about where this point is
 *    sb_geom::Point2D somewhere_on_spline = spline(0.345);
 * </code>
 */

class Spline {
  public:
    /**
     * Construct a Spline through a list of points
     *
     * @param points the points to interpolate the spline through
     */
    Spline(std::vector<Point2D> points);

    /**
     * Construct a spline from a polynomial line segment
     *
     * We do this by sampling a spline through the endpoints of the polynomial
     * segment,
     * and through it's critical points
     *
     * @param poly_segment
     */
    Spline(sb_geom::PolynomialSegment poly_segment);

    /**
     * Calculate a rough approximation of the length of the spline
     *
     * Approximates the spline by drawing lines between sample points and
     * summing the length
     *
     * @param num_sample_points the number of sample points to use along
     * the spline
     * @return an approximation of the length of the spline
     */
    double approxLength(int num_sample_points = 100);

    /**
     * Finds all the interpolation points in a given range on the spline
     *
     * @param start_u a value in [0,1] indicating the start of the range to
     * get interpolation points in
     * @param end_u a value in [0,1] indicating the end of the range to
     * get interpolation points in
     * @return interpolation points in the given range (inclusive)
     * (ie. in [start_u, end_u] )
     */
    std::vector<Point2D> getInterpolationPointsInRange(double start_u,
                                                       double end_u);

    /**
     * Returns a point at some length in [0,1] along the spline
     *
     * @param u a value in [0,1], where:
     *          0 is the first point in the spline
     *          1 is the last point in the spline
     * @return the point at the given location on the spline
     * @throws {std::out_of_range} if `u` is not in [0,1]
     *
     * <code>
     *    std::vector<sb_geom::Point2D> points = {
     *            {0,0},
     *            {3,3},
     *            {10,1},
     *            {4, 10}
     *    };
     *    sb_geom::Spline spline(points);
     *
     *    // The start point of the spline
     *    sb_geom::Point2D start = spline.getPointAtZeroToOneIndex(0);
     *
     *    // The end point of the spline
     *    sb_geom::Point2D end = spline.getPointAtZeroToOneIndex(1);
     *
     *    // Some point in the middle of the spline
     *    // (there are no guarantees made about where this point is
     *    sb_geom::Point2D somewhere_on_spline =
     * spline.getPointAtZeroToOneIndex(0.345);
     * </code>
     */
    Point2D getPointAtZeroToOneIndex(double u);

    /**
     * Returns a point at some length along the spline
     *
     * @param u a value in [0,n-1], where:
     *          n is the number of interpolation points in the spline
     *          0 is the first point in the spline
     *          n is the last point in the spline
     *          every integer i in [0,n-1] is the interpolation point i
     * @return the point at the given location on the spline
     * @throws {std::out_of_range} if `u` is not in [0,n-1]
     *
     * <code>
     *    std::vector<sb_geom::Point2D> points = {
     *            {0,0},
     *            {3,3},
     *            {10,1},
     *            {4, 10}
     *    };
     *    sb_geom::Spline spline(points);
     *
     *    // The start point of the spline: {0,0}
     *    sb_geom::Point2D start = spline.getPointAtZeroToNIndex(0);
     *
     *    // The second point on the spline: {10,1}
     *    sb_geom::Point2D somewhere_on_spline =
     * spline.getPointAtZeroToNIndex(2);
     *
     *    // The end point of the spline: {4,10}
     *    sb_geom::Point2D end = spline.getPointAtZeroToNIndex(3);
     *
     * </code>
     */
    Point2D getPointAtZeroToNIndex(double u);

    /**
     * Override the () operator to return a point at some length along the
     * spline
     *
     * @param u a value in [0,1], where 0 is the first point on the spline
     * and 1 is the last point. Will throw an exception if u is not in [0,1]
     * @return the point at the given u value
     * @throws {std::out_of_range} if `u` is not in [0,1]
     */
    Point2D operator()(double u);

  private:
    /**
     * Interpolates this spline through the current `interpolation_points`
     */
    void interpolate();

    // make the `==` operator a "friend" function so it can access private
    // members of this class
    friend bool operator==(Spline s1, Spline s2);

    // The points this spline interpolates through
    std::vector<Point2D> interpolation_points;

    // The two splines that relate (u,x) and (u,y) respectively
    // Because a spline might curve back on itself (imagine a spiral) we can't
    // define it
    // in terms of x or y, as there could be multiple x values for a given y
    // value, or vice-versa.
    // Instead, we define it in terms of a third variable, u. These two splines
    // are basically
    // x(u) and y(u). So if we want a point somewhere along the spline, we
    // choose the appropriate
    // u value, and then call x(u) and y(u) to get the (x,y) point at that u
    // value.
    alglib::spline1dinterpolant x_interpolant;
    alglib::spline1dinterpolant y_interpolant;
};

inline bool operator==(Spline s1, Spline s2) {
    return s1.interpolation_points == s2.interpolation_points;
}
}

#endif // SB_GEOM_SPLINELINE_H
