// TODO: Start of file comment

// Snowbots Includes
#include "sb_geom/util.h"

// Boost Includes
#include <boost/math/tools/roots.hpp>

// STD Includes
#include <functional>

using namespace sb_geom;

double sb_geom::minDistanceFromPointToPolynomialSegment(
        Point2D point, PolynomialSegment line, uintmax_t max_iter) {
    std::vector<double>& coeffecients = line.coefficients();

    // Define a function that returns f(x), f'(x), f''(x)
    std::function<std::tuple<double, double, double>(double x)> f =
    [](double x) {
        double fx = line(x);
        double dx = line.deriv1st(x);
        double d2x = line.deriv2nd(x);
        return std::make_tuple(fx, dx, d2x);
    };

    // TODO: better initialguess
    // Just guess one of the end points of the line
    double guess = line.x_min();
    // Maximum possible binary digits accuracy for doubles
    // digits used to control how accurate to try to make the result.
    const int digits = std::numeric_limits<double>::digits;
    // Accuracy triples with each step, so stop when just
    // over one third of the digits are correct.
    int get_digits = static_cast<int>(digits * 0.4);

    double best_x = boost::math::tools::halley_iterate(
            f, guess, line.x_min(), line.x_max(), get_digits, max_iter);

    double dx = point.x() - best_x;
    double dy = point.y() - line(best_x);
    return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
}


