#include <vector>

/*
 * Created By: Gareth Ellis
 * Created On: January 27, 2018
 * Description: TODO
 */

#include "sb_geom/Polynomial.h"

using namespace sb_geom;

Polynomial::Polynomial():
        _coefficients({})
{}

Polynomial::Polynomial(sb_geom_msgs::Polynomial polynomial_msg):
    Polynomial(polynomial_msg.coefficients)
{
}

Polynomial::Polynomial(std::vector<double>& coefficients):
_coefficients(coefficients)
{}

std::vector<double>& Polynomial::coefficients() {
    return _coefficients;
}

int Polynomial::getDegree() {
    return _coefficients.size();
}

double Polynomial::operator()(double x){
    // Calculate what the y value should be
    double y = 0;
    for (int i = 0; i < _coefficients.size(); i++){
        y += _coefficients[i]*std::pow(x, i);
    }
    return y;
}

double Polynomial::deriv1st(double x) {
    // Calculate what the 1st derivative value should be
    double y = 0;
    for (int i = 1; i < _coefficients.size(); i++){
        y += i*_coefficients[i]*std::pow(x, i-1);
    }
    return y;
}

double Polynomial::deriv2nd(double x) {
    // Calculate what the 2nd derivative value should be
    double y = 0;
    for (int i = 2; i < _coefficients.size(); i++){
        y += i*(i-1)*_coefficients[i]*std::pow(x, i-2);
    }
    return y;
}

