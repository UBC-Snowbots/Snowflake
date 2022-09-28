/*
 * Created By: Gareth Ellis
 * Created On: January 27, 2018
 * Description: This file contains the declaration for the `Polynomial` class
 */

// Snowbots Includes
#include "sb_geom/Polynomial.h"

// Boost Includes
#include <boost/math/special_functions/factorials.hpp>

using namespace sb_geom;

Polynomial::Polynomial() : _coefficients({}) {}

Polynomial::Polynomial(sb_geom_msgs::Polynomial polynomial_msg)
  : Polynomial(polynomial_msg.coefficients) {}

Polynomial::Polynomial(std::vector<double> coefficients)
  : _coefficients(coefficients) {}

std::vector<double>& Polynomial::coefficients() {
    return _coefficients;
}

unsigned int Polynomial::getDegree() {
    return _coefficients.size();
}

double Polynomial::operator()(double x) {
    // Calculate what the y value should be
    double y = 0;
    for (int i = 0; i < _coefficients.size(); i++) {
        y += _coefficients[i] * std::pow(x, i);
    }
    return y;
}

Polynomial Polynomial::deriv(unsigned int degree) {
    std::vector<double> new_coeffs;
    for (unsigned int i = degree; i < _coefficients.size(); i++) {
        new_coeffs.emplace_back((boost::math::factorial<double>(i) /
                                 boost::math::factorial<double>(i - degree)) *
                                _coefficients[i]);
    }
    return Polynomial(new_coeffs);
}

double Polynomial::deriv(double x, unsigned int degree) {
    double y = 0;
    for (unsigned int i = degree; i < _coefficients.size(); i++) {
        y += (boost::math::factorial<double>(i) /
              boost::math::factorial<double>(i - degree)) *
             _coefficients[i] * std::pow<double>(x, i - degree);
    }
    return y;
}
