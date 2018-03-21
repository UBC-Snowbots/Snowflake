// TODO: Start of file comment

// Snowbots Includes
#include "sb_geom/util.h"

// STD Includes
#include <functional>

// Boost Includes
#include <boost/math/tools/roots.hpp>

// GNU Scientific Library Includes
#include <gsl/gsl_poly.h>


using namespace sb_geom;

double sb_geom::minDistanceFromPointToPolynomialSegment(
        Point2D point, PolynomialSegment line, uintmax_t max_iter) {
    std::vector<double>& coeffecients = line.coefficients();

    // Define a function of the distance from the point to the line that we can optimize
    // to find the shortest distance. For `halley_iterate` to work, we need to return the function
    // value, along with it's first and second derivatives
    std::function<std::tuple<double, double, double>(double x)> f =
    [](double x) {
        double x0 = point.x();
        double y0 = point.y();

        double f_x = line(x);
        double f_dx = line.deriv1st(x);
        double f_d2x = line.deriv2nd(x);

        // g(x) is the function representing the distance from the point to the polynomial line
        // for g'(x) see https://www.wolframalpha.com/input/?i=d%2Fdx+sqrt((f(x)-y_0)%5E2%2B(x-x_0)%5E2)

        // g(x)
        double g_x = std::sqrt(std::pow(f_x - point.y(), 2) + std::pow(x - point.x(), 2));
        // g'(x): see https://www.wolframalpha.com/input/?i=d%2Fdx+sqrt((f(x)-y_0)%5E2%2B(x-x_0)%5E2)
        double g_dx = (2*(f_x-y0)*f_dx + 2*(x - x0))/
                (2 * std::sqrt(std::pow(f_x-y0,2) + std::pow(x-x0,2)));
        // g''(x): see https://www.wolframalpha.com/input/?i=2nd+derivative+sqrt((f(x)-y_0)%5E2%2B(x-x_0)%5E2)+with+respect+to+x
        double g_d2x =
                (2*(f_x-y0)*f_d2x + 2*std::pow(f_dx,2) + 2)/
                (2 * std::sqrt(std::pow(f_x-y0,2) + std::pow(x-x0,2)))
                -
                std::pow(2*(f_x-y0)*f_dx + 2*(x-x0), 2)/
                (4*std::pow(std::pow(f_x-y0,2) + std::pow(x-x0,2),3/2));
        return std::make_tuple(g_x, g_dx, g_d2x);
    };

    // Just initial_guess the mid point of the line
    double initial_guess = (line.x_min() + line.x_max())/2;
    // Maximum possible binary digits accuracy for doubles
    // digits used to control how accurate to try to make the result.
    const int digits = std::numeric_limits<double>::digits;
    // Accuracy triples with each step, so stop when just
    // over one third of the digits are correct.
    int get_digits = static_cast<int>(digits * 0.4);

    // Find the x value of the closest point on f(x)
    double best_x = boost::math::tools::halley_iterate(
            f, initial_guess, line.x_min(), line.x_max(), get_digits, max_iter);

    // Calculate the distance between the given point and the closest point on f(x)
    double dx = point.x() - best_x;
    double dy = point.y() - line(best_x);
    return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
}

std::vector<double> findRoots(Polynomial poly){
    // Allocate a the `gsl_poly_complex_workspace` used to solve the polynomial in later
    gsl_poly_complex_workspace* workspace = gsl_poly_complex_workspace_alloc(poly.getDegree());

    // This is what our roots will be returned in
    gsl_complex_packed_ptr z;

    // TODO: Check return value, what if it is `GSL_EFAILED`? Throw an exception?
    // TODO: See docs: https://www.gnu.org/software/gsl/manual/html_node/General-Polynomial-Equations.html
    // Solve for the roots of the polynomial
    double* coefficients = &poly.coefficients()[0];
    int success = gsl_poly_complex_solve(coefficients, poly.getDegree(), workspace, z);

    // Free the workspace we allocated above
    gsl_poly_complex_workspace_free(workspace);

    // Put all the roots in a vector and return it
    std::vector<double> roots;
    roots.reserve(poly.getDegree());
    for (int i = 0; i < poly.getDegree(); i++){
        roots.emplace_back(z[2*i]);
    }

    return roots;
}


