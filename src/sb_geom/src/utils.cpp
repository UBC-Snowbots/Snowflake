/*
 * Created By: Gareth Ellis
 * Created On:  March 19th, 2018
 * Description: Geometry Utility Functions
 */

// Snowbots Includes
#include "sb_geom/utils.h"

// STD Includes
#include <functional>

// Boost Includes
#include <boost/math/tools/roots.hpp>

// GNU Scientific Library Includes
#include <gsl/gsl_poly.h>
#include <gsl/gsl_multimin.h>


using namespace sb_geom;

double sb_geom::minDistanceFromPointToPolynomialSegment(
        Point2D point, PolynomialSegment line, uintmax_t max_iter) {
    std::vector<double>& coeffecients = line.coefficients();

    // Define a function of the distance from the point to the line that we can optimize
    // to find the shortest distance. For `halley_iterate` to work, we need to return the function
    // value, along with it's first and second derivatives
    std::function<std::tuple<double, double, double>(double x)> f =
    [&](double x) {
        double x0 = point.x();
        double y0 = point.y();

        double f_x = line(x);
        double f_dx = line.deriv(x, 1);
        double f_d2x = line.deriv(x, 2);

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

double minDistanceBetweenSplines(Spline s1, Spline s2, uintmax_t max_iter, double max_err) {
    // Define a function of two variables representing the distance between two points,
    // s1(u) and s2(t), that we can minimize to find the closest distance betweent the two splines

    // f(u,t)
    double f(const gsl_vector *v, void *params){
        // Get the values of `u` and `t`
        double u, t;
        double *p = (double *)params;
        u = gsl_vector_get(v, 0);
        t = gsl_vector_get(v, 1);

        // Get the points on the two splines at the given points
        Point2D p1 = s1(u);
        Point2D p2 = s2(t);

        // Return the distance between the points
        double dx = p1.x() - p2.x();
        double dy = p1.y() - p2.y();
        return std::abs(std::pow(dx, 2) + std::pow(dy, 2));
    }

    // TODO: We could probably make this much faster if we calculate the derivatives of the distance function
    // TODO: but this is quite difficult, as each function returns a vector, need to use vector calculus

    const gsl_multimin_fminimizer_type *T =
            gsl_multimin_fminimizer_nmsimplex2;
    gsl_multimin_fminimizer *minimizer = NULL;
    gsl_vector *step_sizes, *curr_state;
    gsl_multimin_function minex_func;

    size_t iter = 0;
    int status;
    double size

    // Starting point
    curr_state = gsl_vector_alloc(2);
    gsl_vector_set(curr_state, 0, 0.5);
    gsl_vector_set(curr_state, 1, 0.5);

    // Set initial step sizes
    step_sizes = gsl_vector_alloc(2);
    gsl_vector_set_all(step_sizes, 0.1);

    // Initialize method and iterate
    minex_func.n = 2;
    minex_func.f = f;
    minex_func.params = {};

    minimizer = gsl_multimin_fminimizer_alloc(T, 2);
    gsl_multimin_fminimizer_set(minimizer, &minex_func, curr_state, step_sizes);

    do {
        iter++;
        status = gsl_multimin_fminimizer_iterate(minimizer);

        if (status)
            break;

        size = gsl_multimin_fminimizer_size(minimizer);
        status = gsl_multimin_test_size(size, max_err);
    } while (status == GSL_CONTINUE && iter < max_iter);

    // Get the two points we found
    double u = gsl_vector_get(minimizer->x, 0);
    double t = gsl_vector_get(minimizer->x, 1);
    Point2D p1 = s1(u);
    Point2D p2 =  s2(t);

    // Free memory
    gsl_vector_free(curr_state);
    gsl_vector_free(step_sizes);
    gsl_multimin_fminimizer_free(minimizer);

    // Calculate and return the distance between the two found points
    double dx = p1.x() - p2.x();
    double dy = p1.y() - p2.y();

    return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
}

std::vector<double> sb_geom::findRealRoots(sb_geom::Polynomial poly){
    double* coefficients = &poly.coefficients()[0];

    // Since `gsl_poly_complex_solve` requires the leading term of the polynomial to be
    // non-zero, choose the degree to be less then the actual degree of the polnomial so this is the case
    unsigned int degree = poly.getDegree();
    for (; degree > 0 && coefficients[degree-1] == 0; degree--){}

    // Allocate a the `gsl_poly_complex_workspace` used to solve the polynomial in later
    gsl_poly_complex_workspace* workspace = gsl_poly_complex_workspace_alloc(degree);

    // This is what our roots will be returned in
    double z[2*degree];

    // TODO: Check return value, what if it is `GSL_EFAILED`? Throw an exception?
    // TODO: See docs: https://www.gnu.org/software/gsl/manual/html_node/General-Polynomial-Equations.html
    // Solve for the roots of the polynomial
    int success = gsl_poly_complex_solve(coefficients, degree, workspace, z);

    // Free the workspace we allocated above
    gsl_poly_complex_workspace_free(workspace);

    // Put all the real roots in a vector and return it
    std::vector<double> roots;
    for (int i = 0; i < poly.getDegree(); i++){
        // Get the real part of the root
        double real_part = z[2*i];
        // Because of numerical instability, round to the 12th decimal place
        // (because zero values might show up as non-zero)
        real_part = std::round(real_part * 1e12) / 1e12;
        // If it's non-zero, add it to our roots
        if (real_part != 0){
            roots.emplace_back(real_part);
        }
    }

    // Check for case where no roots have non-zero real parts, but there is a real root at 0
    if (poly(0) == 0){
        roots.emplace_back(0);
    }

    return roots;
}

// TODO: We could probably do this much faster with GSL and just minimizing the distance function
double sb_geom::findClosestPointOnSplineToPoint(
        Spline spline, Point2D point, unsigned int num_sample_points, double max_err){

    // The start and end of the section of spline we're sampling
    // Initially just choose the entire spline
    double u1 = 0;
    double u2 = 1;
    double distance_to_u1 = distance(spline(u1), point);
    double distance_to_u2 = distance(spline(u2), point);

    do {
        // Calculate how much to step each time, depending on the length
        // of spline we're sampling
        double len_of_sub_spline = std::max(u1, u2) - std::min(u1, u2);
        double u_step = len_of_sub_spline/num_sample_points;

        for (int i = 0; i < num_sample_points; i++){
            double curr_u = i * u_step;
            // Find the distance to the polynomial from the sample point on the spline
            Point2D curr_p = spline(i * 1/num_sample_points);
            double dist_to_p = distance(curr_p, point);

            // Check if this point is better then at least one of the points we have
            if (dist_to_p < std::max(distance_to_u1, distance_to_u2)){
                // If it is better, overwrite the furthest currently known point
                if (distance_to_u1 > distance_to_u2){
                    u1 = curr_u;
                    distance_to_u1 = dist_to_p;
                } else { // (distance_to_p1 <= distance_to_p2)
                    u2 = curr_u;
                    distance_to_u2 = dist_to_p;
                }
            }
        }
    } while (std::abs(distance_to_u1 - distance_to_u2) > max_err);

    // Return the average of the two closest points
    return (distance_to_u1 + distance_to_u2)/2;
}

std::vector<Point2D>
sb_geom::getInterpolationPointsFromPolySegment(PolynomialSegment poly_segment) {
    std::vector<Point2D> interpolation_points;

    // Add the start point of the polynomial segment
    interpolation_points.emplace_back(Point2D(
            poly_segment.x_min(), poly_segment(poly_segment.x_min())));

    // Add an interpolation point for every critical point in the polynomial segment
    // We do this by find the roots of the first derivative
    std::vector<double> roots = findRealRoots(poly_segment.deriv(1));
    // TODO: Do we need this sort?
    std::sort(roots.begin(), roots.end());
    for (double& root : roots){
        if (root > poly_segment.x_min() && root < poly_segment.x_max());
        interpolation_points.emplace_back(Point2D(
                root, poly_segment(root)
        ));
    }

    // Add the end point of the Polynomial Segment
    interpolation_points.emplace_back(Point2D(
            poly_segment.x_max(), poly_segment(poly_segment.x_max())
    ));

    return interpolation_points;
}

double sb_geom::distance(Point2D p1, Point2D p2) {
    double dx = p1.x() - p2.x();
    double dy = p1.y() - p2.y();
    return std::sqrt(std::pow(dx,2) + std::pow(dy,2));
}



