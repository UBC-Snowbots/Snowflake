/*
 * Created By: Gareth Ellis
 * Created On:  March 19th, 2018
 * Description: Test for the `Spline` class
 */

// GTest Includes
#include <gtest/gtest.h>

// Snowbots Includes
#include "sb_geom/Point2D.h"
#include "sb_geom/Spline.h"

// STD Includes
#include <vector>

using namespace sb_geom;

class SplineTest : public testing::Test {
  protected:
    SplineTest(){};

    virtual void SetUp() {}

    /**
     * Prints out a given number of values along a spline for debugging purposes
     *
     * @param s the spline to print
     * @param num_points the number of points to print (will be spread out
     * equally over the spline)
     */
    static void printSpline(Spline s, int num_points = 100) {
        std::cout << "u,x,y" << std::endl;
        for (int i = 0; i < num_points; i++) {
            double u = (double) i * 1.0 / num_points;
            std::cout << u << "," << s(u).x() << "," << s(u).y() << std::endl;
        }
    }
};

// Tests the version of the constructor that takes a vector of points
// with a small sample of points
TEST_F(SplineTest, constructor_from_points_small_case) {
    std::vector<Point2D> points = {{1, 1}, {3, 3}, {2, 4}, {5, 1}};
    Spline spline(points);

    // Check the start and end points of the spline
    // (ie. check that the spline passes through the start and end points)
    EXPECT_EQ(Point2D(1, 1), spline(0));
    EXPECT_EQ(Point2D(5, 1), spline(1));

    // Check the values at the interpolation points
    // (ie. check that the spline passes through the interpolation points)
    EXPECT_EQ(Point2D(3, 3), spline.getPointAtZeroToNIndex(1));
    EXPECT_EQ(Point2D(2, 4), spline.getPointAtZeroToNIndex(2));
    EXPECT_EQ(Point2D(5, 1), spline.getPointAtZeroToNIndex(3));

    // Check a couple points in between interpolation points
    Point2D p;

    // Point between the (1,1) and (3,3) interpolation points
    p = spline.getPointAtZeroToNIndex(0.5);
    EXPECT_GE(p.x(), 1);
    EXPECT_LE(p.x(), 3);
    EXPECT_GE(p.y(), 1);
    EXPECT_LE(p.y(), 3);

    // Point between the (2,4) and (5,1) interpolation points
    p = spline.getPointAtZeroToNIndex(2.5);
    EXPECT_GE(p.x(), 2);
    EXPECT_LE(p.x(), 5);
    EXPECT_GE(p.y(), 1);
    EXPECT_LE(p.y(), 4);
}

// Test the constructor from a PolynomialSegment
TEST_F(SplineTest, constructor_from_PolynomialSegment) {
    // y = 0.5x^3 + 2x^2 + 1
    std::vector<double> coeff = {1, 0, 2, 0.5};
    PolynomialSegment poly_segment(coeff, -5, 5);

    Spline spline(poly_segment);

    // Check that the spline starts and ends at the start and end of the segment
    EXPECT_EQ(poly_segment.getStartPoint(), spline(0));
    EXPECT_EQ(poly_segment.getEndPoint(), spline(1));

    // Check that the spline passes through the critical points of the
    // polynomial
    // We know where these critical points are by understanding a bit of the
    // implementation. Since we use the start, end, and critical points as the
    // interpolation points, they will be a u=0,1,2,etc.

    Point2D expected_crit_point_1(-8.0 / 3.0, 155.0 / 27.0);
    Point2D actual_crit_point_1 = spline.getPointAtZeroToNIndex(1);
    EXPECT_NEAR(expected_crit_point_1.x(), actual_crit_point_1.x(), 1e-10);
    EXPECT_NEAR(expected_crit_point_1.y(), actual_crit_point_1.y(), 1e-10);

    Point2D expected_crit_point_2(0, 1);
    Point2D actual_crit_point_2 = spline.getPointAtZeroToNIndex(2);
    EXPECT_NEAR(expected_crit_point_2.x(), actual_crit_point_2.x(), 1e-10);
    EXPECT_NEAR(expected_crit_point_2.y(), actual_crit_point_2.y(), 1e-10);
}

TEST_F(SplineTest, approxLength_straight_line) {
    std::vector<Point2D> points = {{-5, -4}, {3, 4}};
    Spline spline(points);

    EXPECT_NEAR(std::sqrt(128), spline.approxLength(), 1e-10);
}

TEST_F(SplineTest, approxLength_arc) {
    // Interpolate a spline through a quarter circle
    std::vector<Point2D> points = {
    {0, 0}, {4 * cos(45.0), 4 * sin(45.0)}, {4, 4},
    };
    Spline spline(points);

    EXPECT_NEAR(2 * M_PI, spline.approxLength(), 0.2);
}

TEST_F(SplineTest, approxLength_complex_polynomial) {
    // Try and approximate a complicated polynomial with a spline
    std::vector<double> coeff = {0, 0, 10, 3, 0.23};
    PolynomialSegment poly_segment(coeff, -8, 2);
    Spline spline(poly_segment);

    EXPECT_NEAR(153, spline.approxLength(), 3);
}

// Test with equal start_u/end_u points. Since we should be getting points
// in a range that is inclusive (ie. in [start_u, end_u] ), we should be
// able to get points
TEST_F(SplineTest, getInterpolationPointsInRange_start_and_end_equal) {
    Point2D start_point(0, 0);
    Point2D end_point(9.33, 2313.3);
    std::vector<sb_geom::Point2D> points = {
    start_point, {2, 2}, {0, 3}, {10, 10}, end_point,
    };
    Spline spline(points);

    // Check getting the endpoints of the spline

    EXPECT_EQ(std::vector<Point2D>({start_point}),
              spline.getInterpolationPointsInRange(0, 0));
    EXPECT_EQ(std::vector<Point2D>({end_point}),
              spline.getInterpolationPointsInRange(1, 1));
}

// Test getting interpolation points in a range that overlaps the first point on
// the spline
TEST_F(SplineTest, getInterpolationPointsInRange_including_first_point) {
    Spline spline({{0, 0}, {5, 0}, {10, 0}});

    std::vector<Point2D> expected = {{0, 0}, {5, 0}};
    EXPECT_EQ(expected, spline.getInterpolationPointsInRange(0, 0.7));
}

// Test getting interpolation points in a range that overlaps the first point on
// the spline
TEST_F(SplineTest, getInterpolationPointsInRange_including_last_point) {
    Spline spline({{0, 0}, {5, 0}, {10, 0}});

    std::vector<Point2D> expected = {{5, 0}, {10, 0}};
    EXPECT_EQ(expected, spline.getInterpolationPointsInRange(0.3, 1));
}

// Test getting interpolation points in ranges that include the endpoints of
// the spline
TEST_F(SplineTest, getInterpolationPointsInRange_including_endpoints) {
    // Create a spline in a straight line through 5 points
    Point2D p1(0, 0);
    Point2D p2(5, 5);
    Point2D p3(10, 10);
    Point2D p4(15, 15);
    Point2D p5(20, 20);
    std::vector<sb_geom::Point2D> points = {p1, p2, p3, p4, p5};
    Spline spline(points);

    std::vector<Point2D> expected;

    // Range including the start point of the Spline
    expected = {p1, p2};
    EXPECT_EQ(expected, spline.getInterpolationPointsInRange(0, 0.27));

    // Range including the end point of the Spline
    expected = {p4, p5};
    EXPECT_EQ(expected, spline.getInterpolationPointsInRange(0.72, 1));
}

// Test getting interpolation points in the middle section of the spline
TEST_F(SplineTest, getInterpolationPointsInRange_mid_section_of_spline) {
    // Create a spline in a straight line through 5 points
    Point2D p1(0, 0);
    Point2D p2(5, 5);
    Point2D p3(10, 10);
    Point2D p4(15, 15);
    Point2D p5(20, 20);
    std::vector<sb_geom::Point2D> points = {p1, p2, p3, p4, p5};
    Spline spline(points);

    std::vector<Point2D> expected;

    // Test getting a few ranges
    expected = {p2, p3};
    EXPECT_EQ(expected, spline.getInterpolationPointsInRange(0.2, 0.52));

    expected = {p2, p3, p4};
    EXPECT_EQ(expected, spline.getInterpolationPointsInRange(0.2, 0.79));
}

// try to get a point outside the valid range of the spline
TEST_F(SplineTest, getPointAtZeroToOneIndex_out_of_range) {
    Spline s({{0, 0}, {3, 3}});

    EXPECT_THROW(s(1.1), std::out_of_range);
    EXPECT_THROW(s(-0.1), std::out_of_range);
}

TEST_F(SplineTest, getInterpolationPointsInRange_out_of_bounds) {
    Spline s({{0, 0}, {3, 3}});

    std::vector<Point2D> expected_points;

    // Test ranges entirely outside the spline
    expected_points = {};
    EXPECT_EQ(expected_points, s.getInterpolationPointsInRange(1.1, 2));
    EXPECT_EQ(expected_points, s.getInterpolationPointsInRange(-0.2, -0.1));

    // Test a range overlapping the first point
    expected_points = {{0, 0}};
    EXPECT_EQ(expected_points, s.getInterpolationPointsInRange(-0.1, 0.1));
    expected_points = {{3, 3}};
    EXPECT_EQ(expected_points, s.getInterpolationPointsInRange(0.9, 2));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}