/*
 * Created By: Gareth Ellis
 * Created On: January 27, 2018
 * Description: TODO
 */

#ifndef SB_GEOM_POLYLINE_H
#define SB_GEOM_POLYLINE_H

#include "sb_geom/Line.h"
#include "sb_geom_msgs/PolyLine.h"

namespace sb_geom {
    // A line, represented as a n-ary polynomial
    class PolyLine : Line {
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
         * Set the coefficients for the polynomial
         * @param coefficients the values to set the coefficients to
         */
        void setCoefficients(std::vector<double> coefficients);

        /**
         * Get the current coefficients for this polynomial
         * @return the coefficients for this polynomial
         */
        std::vector<double> getCoefficients();

        /**
         * Get the degree of this polynomial
         * @return the degree of this polynomial
         */
        int getDegree();

    private:
        std::vector<double> coefficients;
    };
}


#endif //SB_GEOM_POLYLINE_H
