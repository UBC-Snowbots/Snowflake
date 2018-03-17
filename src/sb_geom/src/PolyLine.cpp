#include <vector>

/*
 * Created By: Gareth Ellis
 * Created On: January 27, 2018
 * Description: TODO
 */

#include "sb_geom/PolyLine.h"

using namespace sb_geom;

PolyLine::PolyLine():
        _coefficients({})
{}

PolyLine::PolyLine(sb_geom_msgs::PolyLine polyLineMsg):
    PolyLine(polyLineMsg.coefficients)
{
}

PolyLine::PolyLine(std::vector<double>& coefficients):
_coefficients(coefficients)
{}

std::vector<double>& PolyLine::coefficients() {
    return _coefficients;
}

int PolyLine::getDegree() {
    return _coefficients.size();
}

double PolyLine::operator()(double x){
    // Calculate what the y value should be
    double y = 0;
    for (int i = 0; i < _coefficients.size(); i++){
        y += _coefficients[i]*std::pow(x, i);
    }
    return y;
}

