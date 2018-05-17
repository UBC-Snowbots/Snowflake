/*
 * Created By: Gareth Ellis
 * Created On: January 27, 2018
 * Description: This file contains the declaration for the `Polynomial` class
 */

#ifndef SB_GEOM_POLYNOMIAL_H
#define SB_GEOM_POLYNOMIAL_H

#include "sb_geom_msgs/Polynomial.h"

namespace sb_geom {

/**
 * This class represents a n-ary polynomial line
 *
 * Example Usage:
 * <code>
 *      // y = 3 + 4x + 5x^2
 *      std::vector<double> coeff = {3, 4, 5};
 *      sb_geom::Polynomial y(coeff);

 *      std::cout << y(5);
 * </code>
 */
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
    Polynomial(std::vector<double> coefficients);

    /**
     * Get the current coefficients for this polynomial
     * @return the coefficients for this polynomial
     */
    std::vector<double>& coefficients();

    /**
     * Get the degree of this polynomial
     * @return the degree of this polynomial
     */
    unsigned int getDegree();

    /**
     * Override the () operator so that we can use it to get the y value for a
     * given x
     * @param x the x value to get the y value of the line at
     * @return the y value of the line at the given x value
     */
    double operator()(double x);

    /**
     * Calculate the derivative to a given degree at a given point
     * @param x the point to calculate the derivative at
     * @param degree the degree of the derivative (1st, 2nd, etc.)
     * @return the value of the degree'th derivative at `x`
     */
    double deriv(double x, unsigned int degree);

    /**
     * Return the derivative polynomial of this one to the given degree
     *
     * @param degree the degree of the derivative (1st, 2nd, 3rd, etc.)
     * @return the derivative polynomial of this one
     */
    Polynomial deriv(unsigned int degree);

  private:
    std::vector<double> _coefficients;
};
}

#endif // SB_GEOM_POLYNOMIAL_H
