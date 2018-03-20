/*
 * Created By: Gareth Ellis
 * Created On: January 27, 2018
 * Description: TODO
 */

#ifndef SB_GEOM_POLYNOMIAL_H
#define SB_GEOM_POLYNOMIAL_H

#include "sb_geom_msgs/Polynomial.h"

namespace sb_geom {
    // A line, represented as a n-ary polynomial
    class Polynomial {
    public:

        /**
         * Create an empty Polynomial
         */
        Polynomial();

        /**
         * Create a Polynomial from a Polynomial msg
         * @param polynomial_msg the Polynomial msg to make a Polynomial from
         */
        Polynomial(sb_geom_msgs::Polynomial polynomial_msg);

        /**
         * Create a Polynomial with given coefficients
         * @param coefficients the coefficients of this polynomial
         */
        Polynomial(std::vector<double>& coefficients);

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
         * @param x the x value to get the y value of the line at
         * @return the y value of the line at the given x value
         */
        double operator()(double x);

        // TODO: Test me
        /**
         * Calculate the derivative of the polyline at the given point
         * @param x the x value to get the derivative of the line at
         * @return the derivative of the line at the given x value
         */
        double deriv1st(double x);

        // TODO: Test me
        /**
         * Calculate the second derivative of the polyline at the given point
         * @param x the x value to get the derivative of the line at
         * @return the derivative of the line at the given x value
         */
        double deriv2nd(double x);

    private:
        std::vector<double> _coefficients;
    };
}


#endif //SB_GEOM_POLYNOMIAL_H
