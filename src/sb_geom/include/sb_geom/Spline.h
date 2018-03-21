/*
 * Created By: Gareth Ellis
 * Created On: January 27, 2018
 * Description: TODO
 */

#ifndef SB_GEOM_SPLINELINE_H
#define SB_GEOM_SPLINELINE_H

// Snowbots Includes
#include <sb_geom_msgs/SplineLine.h>
#include <sb_geom/Point2D.h>

// Alglib Includes
#include <interpolation.h>
#include "PolynomialSegment.h"

namespace sb_geom {

    // TODO: This class is rapidly becoming not very generic, might want to rename and move to mapping_igvc
    // TODO: If we're using akima, might want to make a `Spline` interface and then a `AkimaSpline` subclass?
    // TODO: This is more of a "curve" then a spline now? How to make this distinction?
    // A line, represented as a spline
    class Spline {
    public:

        // TODO: Test me
        /**
         * Construct a Spline through a list of points
         *
         * @param points the points to interpolate the spline through
         */
        Spline(std::vector<Point2D> &points);

        /**
         * Construct a spline from a polynomial line segment
         *
         * We do this by sampling a spline through the endpoints of the polynomial segment,
         * and through it's points of inflection.
         *
         * @param poly_segment
         */
        Spline(sb_geom::PolynomialSegment poly_segment);

        // TODO: Test me
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

        // TODO: Test
        // TODO: Test exception cases
        /**
         * Override the () operator to return a point at some length along the spline
         *
         * 0 is the first point in the spline
         * 1 is the last point in the spline
         *
         * @param u a value in [0,1], where 0 is the first point on the spline
         * and 1 is the last point. Will throw an exception if u is not in [0,1]
         * @return the point at the given u value
         */
        Point2D operator()(double u);

    private:

        /**
         * Interpolates this spline through the current `interpolation_points`
         */
        void interpolate();

        // The points this spline interpolates through
        std::vector<Point2D> interpolation_points;

        // TODO: More clear comment here
        // The two splines that relate (u,x) and (u,y) respectively
        alglib::spline1dinterpolant x_interpolant;
        alglib::spline1dinterpolant y_interpolant;
    };

}


#endif //SB_GEOM_SPLINELINE_H
