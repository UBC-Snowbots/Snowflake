/*
 * Created By: Gareth Ellis
 * Created On:  February 24th, 2018
 * Description: Test for the `PolyLine` class
 */

// GTest Includes
#include <gtest/gtest.h>

// Snowbots Includes
#include "sb_geom/Spline.h"
#include "sb_geom/Point2D.h"

// STD Includes
#include <vector>

using namespace sb_geom;

class SplineTest : public testing::Test {
protected:
    SplineTest(){};

    virtual void SetUp() {
    }
};

// Tests the version of the constructor that takes a vector of points
// with a small sample of points
TEST_F(SplineTest, constructor_from_points_small_case){
    std::vector<Point2D> points = {{1,1}, {3,3}, {2,4}, {5, 1}};
    Spline spline(points);

    // Check the start and end points of the spline
    // (ie. check that the spline passes through the start and end points)
    EXPECT_EQ(Point2D(1,1), spline(0));
    EXPECT_EQ(Point2D(5,1), spline(1));

    // Check the values at the interpolation points
    // (ie. check that the spline passes through the interpolation points)
    EXPECT_EQ(Point2D(3,3), spline.getPointAtZeroToNIndex(1));
    EXPECT_EQ(Point2D(2,4), spline.getPointAtZeroToNIndex(2));
    EXPECT_EQ(Point2D(5,1), spline.getPointAtZeroToNIndex(3));

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
TEST_F(SplineTest, constructor_from_PolynomialSegment){

    // y = 0.5x^3 + 2x^2 + 1
    std::vector<double> coeff = {1, 0, 2, 0.5};
    PolynomialSegment poly_segment(coeff, -5, 5);

    Spline spline(poly_segment);

    // Check that the spline starts and ends at the start and end of the segment
    EXPECT_EQ(poly_segment.getStartPoint(), spline(0));
    EXPECT_EQ(poly_segment.getEndPoint(), spline(1));

    // Check that the spline passes through the critical points of the polynomial
    // We know where these critical points are by understanding a bit of the
    // implementation. Since we use the start, end, and critical points as the
    // interpolation points, they will be a u=0,1,2,etc.
    Point2D crit_point_1(-8.0/3.0, 155.0/27.0);
    EXPECT_EQ(crit_point_1, spline.getPointAtZeroToNIndex(1));

    // TODO: YOU ARE HERE, need to check second critical point:
    // https://www.wolframalpha.com/input/?i=critical+points+of+1%2F2x%5E3%2B2x%5E2%2B1
    // https://www.wolframalpha.com/input/?i=value+of+1%2F2x%5E3%2B2x%5E2%2B1+at+-8%2F3
}

// TODO: Not a real test - delete me
TEST_F(SplineTest, messing_about){
    std::vector<sb_geom::Point2D> points = {
            {0,0},
            {2,2},
            {0,3},
            {10, 10},
            {6,2}
    };
    sb_geom::Spline spline_line(points);

    std::cout << "~~ Points ~~"  << std::endl;
    for (double i = 0; i <= 1; i += 0.01){
        std::cout << i << ", " << spline_line(i).x() << ", " << spline_line(i).y() << std::endl;
    }
    int i = 1;
    std::cout << i << ", " << spline_line(i).x() << ", " << spline_line(i).y() << std::endl;

}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}