/*
 * Created By: Gareth Ellis
 * Created On: January 27, 2018
 * Description: TODO
 */

#ifndef SB_GEOM_POLYLINE_H
#define SB_GEOM_POLYLINE_H

#include "sb_geom_msgs/PolyLine.h"

namespace sb_geom {
    // A line, represented as a n-ary polynomial
    class PolyLine {
    public:

        /**
         * Create an empty PolyLine
         */
        PolyLine();

        /**
         * Create a PolyLine from a PolyLine msg
         * @param polyLineMsg the PolyLine msg to make a PolyLine from
         */
        PolyLine(sb_geom_msgs::PolyLine polyLineMsg);

        /**
         * Create a PolyLine with given coefficients
         * @param coefficients the coefficients of this polynomial
         */
        PolyLine(std::vector<double>& coefficients);

        /**
         * Get the current coefficients for this polynomial
         * @return the coefficients for this polynomial
         */
        std::vector<double>& coefficients();

        /**
         * Get the degree of this polynomial
         * @return the degree of this polynomial
         */
        int getDegree();

        /**
         * Override the () operator so that we can use it to get the y value for a given x
         * @param x the x value to get the y value of the line ay
         * @return the y value of the line at the given x value
         */
        double operator()(double x);

    private:
        std::vector<double> _coefficients;
    };
}


#endif //SB_GEOM_POLYLINE_H
