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

// Eigen Includes
#include <eigen3/unsupported/Eigen/Splines>

namespace sb_geom {

    // A line, represented as a spline
    class SplineLine {
    public:

        /**
         * Construct a SplineLine from a SplineLine msg
         * @param spline_line_msg the msg to construct the SplineLine from
         */
        // TODO
        //SplineLine(sb_geom_msgs::SplineLine spline_line_msg);

        /**
         * Construct a Spline through a list of points
         * @param points the points to interpolate the spline through
         */
        SplineLine(std::vector<Point2D> points);

        // TODO: Test me
        /**
         * Calculate a rough approximation of the length of the spline
         *
         * Approximates the spline by drawing lines between sample points and
         * summing the length
         * @param num_sample_points the number of sample points to use along
         * the spline
         * @return an approximation of the length of the spline
         */
        double approxLength(int num_sample_points = 100);

        // TODO: Test
        // TODO: Test exception cases
        /**
         * Override the () operator to return a point at some length along
         * the spline
         * 0 is the first point in the spline
         * 1 is the last point in the spline
         * @param u a value in [0,1], where 0 is the first point on the spline
         * and 1 is the last point. Will throw an exception if u is not in [0,1]
         * @return the point at the given u value
         */
        Point2D operator()(double u);

        // TODO: Descriptive comment
        // TODO: Make this private
        Eigen::Spline2d _spline;

    private:

    };

}


#endif //SB_GEOM_SPLINELINE_H
