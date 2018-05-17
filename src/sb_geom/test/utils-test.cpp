/*
 * Created By: Gareth Ellis
 * Created On:  February 24th, 2018
 * Description: Tests for the geometry utility functions
 */

// GTest Includes
#include <gtest/gtest.h>

// Snowbots Includes
#include "sb_geom/Point2D.h"
#include "sb_geom/Spline.h"
#include "sb_geom/utils.h"

// STD Includes
#include <vector>

// dlib Includes
#include <dlib/global_optimization/find_max_global.h>

using namespace sb_geom;

class UtilsTest : public testing::Test {
  protected:
    UtilsTest(){};

    virtual void SetUp() {}
};

TEST_F(UtilsTest, findRealRoots_3th_degree_integer_coefficients) {
    // y = 4x^3 + 9x^2
    std::vector<double> coeff = {0, 0, 9, 4};
    Polynomial polynomial(coeff);

    std::vector<double> roots = findRealRoots(polynomial);
    EXPECT_EQ(roots.size(), 2);
    EXPECT_NEAR(-9.0 / 4.0, roots[0], 1e-11);
    EXPECT_NEAR(0, roots[1], 1e-11);
}

TEST_F(UtilsTest, findRealRoots_2nd_degree_decimal_leading_coefficient) {
    // y = 1.5x^2 + 4x
    std::vector<double> coeff = {0, 4, 1.5};
    Polynomial polynomial(coeff);

    std::vector<double> roots = findRealRoots(polynomial);
    EXPECT_EQ(roots.size(), 2);
    EXPECT_NEAR(-8.0 / 3.0, roots[0], 1e-11);
    EXPECT_NEAR(0, roots[1], 1e-11);
}

TEST_F(UtilsTest, findRealRoots_3rd_degree_neg_and_positive_roots) {
    // y = -4x^3 -9x^2 + 2x + 1/5
    std::vector<double> coeff = {1.0 / 5.0, 2, -9, -4};
    Polynomial polynomial(coeff);

    std::vector<double> roots = findRealRoots(polynomial);
    EXPECT_EQ(roots.size(), 3);
    EXPECT_NEAR(-0.075323428127562776000477285, roots[0], 1e-11);
    EXPECT_NEAR(0.27137752045040101656387732, roots[1], 1e-11);
    EXPECT_NEAR(-2.4460540923228382405634000, roots[2], 1e-11);
}

// Find the roots of a Polynomial where the coefficient of the highest power is
// 0
TEST_F(UtilsTest, findRealRoots_polynomial_with_zero_leading_coefficient) {
    // y = 0x^3 + x^2
    std::vector<double> coeff = {0, 0, 1, 0};
    Polynomial polynomial(coeff);

    std::vector<double> roots = findRealRoots(polynomial);
    EXPECT_EQ(roots.size(), 1);
    EXPECT_NEAR(0, roots[0], 1e-11);
}

TEST_F(UtilsTest, findRealRoots_polynomial_with_all_zero_coefficients) {
    // y = 0
    std::vector<double> coeff = {0, 0, 0, 0};
    Polynomial polynomial(coeff);

    std::vector<double> roots = findRealRoots(polynomial);
    EXPECT_EQ(0, roots.size());
}

TEST_F(UtilsTest, findRealRoots_complex_polynomial) {
    // y = -32x^31 + 4x^3 + 1
    std::vector<double> coeff(32);
    coeff[0]  = 1;
    coeff[3]  = 4;
    coeff[31] = -32;
    Polynomial polynomial(coeff);

    std::vector<double> expected_roots = {
    -0.91544111165991093096935295,
    -0.62996456269339499629737930,
    0.93725891366674881690617031,
    };

    std::vector<double> roots = findRealRoots(polynomial);
    EXPECT_EQ(expected_roots.size(), roots.size());

    // Compare the roots to the expected ones
    // we're using a for loop here because doubles are not _exactly_ equal
    std::sort(roots.begin(), roots.end());
    for (int i = 0; i < expected_roots.size(); i++) {
        EXPECT_NEAR(expected_roots[i], roots[i], 1e-10);
    }
}

// Test finding the distance between two splines that are just straight lines
TEST_F(UtilsTest, minDistanceBetweenSplines_two_straight_lines) {
    Spline spline1({{0, 1}, {10, 1}});
    Spline spline2({{0, 4}, {10, 4}});

    EXPECT_DOUBLE_EQ(3, minDistanceBetweenSplines(spline1, spline2));
}

// Test finding the minimum distance between two splines with their arcs
// pointing towards each other
TEST_F(UtilsTest, minDistanceBetweenSplines_one_arc) {
    // (roughly) y = x^2 + 5
    Spline spline1({{-1, 6}, {0, 5}, {1, 6}});
    // (roughly) y = -x^2
    Spline spline2({{-1, -1}, {0, 0}, {1, -1}});

    EXPECT_DOUBLE_EQ(5, minDistanceBetweenSplines(spline1, spline2));
}

// Test finding the minimum distance between two splines with multiple arcs
TEST_F(UtilsTest, minDistanceBetweenSplines_multiple_arcs) {
    Spline spline1({{-1, 6}, {0, 5}, {1, 7}, {4, 6}, {10, 10}});
    Spline spline2({{-1, -1}, {0, 0}, {1, -1}, {4, -10}, {7, 0}});

    EXPECT_NEAR(5, minDistanceBetweenSplines(spline1, spline2), 0.3);
}

// Test finding the minimum distance between two splines where the closest point
// is at the endpoint of one of the splines and somewhere in the middle of the
// other
TEST_F(
UtilsTest,
minDistanceBetweenSplines_min_point_at_end_of_one_and_midpoint_of_other) {
    // Closest point is at (1,1) on this spline
    Spline spline1({{0, 0}, {1, 1}, {7, 0}});
    // Closest point is at (1,3) on this spline
    Spline spline2({{-10, 5}, {0, 8}, {1, 3}});

    EXPECT_NEAR(2, minDistanceBetweenSplines(spline1, spline2), 0.1);
}

// Test finding the minimum distance between two splines where the closest point
// is at the endpoints of both splines
TEST_F(UtilsTest, minDistanceBetweenSplines_min_point_at_end_of_both_splines) {
    // Closest point is at (1,1) on this spline
    Spline spline1({{1, 1}, {7, -10}});
    // Closest point is at (1,3) on this spline
    Spline spline2({{-10, 5}, {0, 8}, {1, 3}});

    EXPECT_DOUBLE_EQ(2, minDistanceBetweenSplines(spline1, spline2));
}

// Test finding the closest point on a straight spline to a given point
TEST_F(UtilsTest, findClosestPointOnSplineToPoint_straight_spline) {
    Spline spline({{0, 0}, {10, 0}});
    Point2D point(5, 7);

    EXPECT_EQ(0.5, findClosestPointOnSplineToPoint(spline, point));
}

// Test finding the closest point on a spline in the shape of an arc
// to a point directly over the center of the arc
TEST_F(UtilsTest, findClosestPointOnSplineToPoint_arc_spline_2) {
    Spline spline({{-1, -1}, {0, 0}, {1, -1}});
    Point2D point(0, 5);

    EXPECT_EQ(0.5, findClosestPointOnSplineToPoint(spline, point));
}

// Test finding the closest point on a spline in the shape of an arc
// to a given point with the point directly over the center of one of
// the arcs
TEST_F(UtilsTest, findClosestPointOnSplineToPoint_arc_spline) {
    Spline spline({{-1, -1}, {0, 0}, {1, -1}, {4, -10}, {7, 0}});
    Point2D point(4, -12);

    Point2D closest_point_on_spline =
    spline(findClosestPointOnSplineToPoint(spline, point));
    Point2D expected_closest_point_on_spline(4, -10);

    EXPECT_NEAR(
    expected_closest_point_on_spline.x(), closest_point_on_spline.x(), 0.1);
    EXPECT_NEAR(
    expected_closest_point_on_spline.y(), closest_point_on_spline.y(), 0.1);
}

// Test finding interpolation points for a fairly simply polynomial
TEST_F(UtilsTest, getInterpolationPointsFromPolySegment_simple_polynomial) {
    // y = x^2 + 4
    PolynomialSegment poly_segment({4, 0, 1}, -1, 1);

    std::vector<Point2D> expected_points = {{-1, 5}, {0, 4}, {1, 5}};
    EXPECT_EQ(expected_points,
              getInterpolationPointsFromPolySegment(poly_segment));
}

// Test finding interpolation points for a more complex polynomial
TEST_F(UtilsTest, getInterpolationPointsFromPolySegment_complex_polynomial) {
    // y = -x^{32} + x^{4} + x
    std::vector<double> coeff(33);
    std::fill(coeff.begin(), coeff.end(), 0);
    coeff[1]  = 1;
    coeff[4]  = 1;
    coeff[32] = -1;
    PolynomialSegment poly_segment(coeff, -1, 1);

    std::vector<Point2D> interpolation_points =
    getInterpolationPointsFromPolySegment(poly_segment);
    std::vector<Point2D> expected_points = {
    {-1, -1},
    {-0.91544111165991093096935295, -0.2723225189933820843419262},
    {-0.62996456269339499629737930, -0.4724707722152933636982830},
    {0.93725891366674881690617031, 1.5831912409539385747402411},
    {1, 1}};

    ASSERT_EQ(expected_points.size(), interpolation_points.size());
    for (int i = 0; i < expected_points.size(); i++) {
        Point2D expected = expected_points[i];
        Point2D actual   = interpolation_points[i];
        EXPECT_NEAR(expected.x(), actual.x(), 1e-10);
        EXPECT_NEAR(expected.y(), actual.y(), 1e-10);
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}