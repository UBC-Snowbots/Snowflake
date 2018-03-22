/*
 * Created By: Gareth Ellis
 * Created On:  February 24th, 2018
 * Description: Tests for the geometry utility functions
 */

// GTest Includes
#include <gtest/gtest.h>

// Snowbots Includes
#include "sb_geom/Spline.h"
#include "sb_geom/Point2D.h"
#include "sb_geom/utils.h"

// STD Includes
#include <vector>

using namespace sb_geom;

class UtilsTest : public testing::Test {
protected:
    UtilsTest(){};

    virtual void SetUp() {
    }
};

TEST_F(UtilsTest, findRoots_3th_degree_integer_coefficients){
    // y = 4x^3 + 9x^2
    std::vector<double> coeff = {0, 0, 9, 4};
    Polynomial polynomial(coeff);

    std::vector<double> roots = findRealRoots(polynomial);
    EXPECT_EQ(roots.size(), 2);
    EXPECT_NEAR(-9.0/4.0, roots[0], 1e-11);
    EXPECT_NEAR(0, roots[1], 1e-11);
}

TEST_F(UtilsTest, findRoots_2nd_degree_decimal_leading_coefficient){
    // y = 1.5x^2 + 4x
    std::vector<double> coeff = {0, 4, 1.5};
    Polynomial polynomial(coeff);

    std::vector<double> roots = findRealRoots(polynomial);
    EXPECT_EQ(roots.size(), 2);
    EXPECT_NEAR(-8.0/3.0, roots[0], 1e-11);
    EXPECT_NEAR(0, roots[1], 1e-11);
}

TEST_F(UtilsTest, findRoots_3rd_degree_neg_and_positive_roots){
    // y = -4x^3 -9x^2 + 2x + 1/5
    std::vector<double> coeff = {1.0/5.0, 2, -9, -4};
    Polynomial polynomial(coeff);

    std::vector<double> roots = findRealRoots(polynomial);
    EXPECT_EQ(roots.size(), 3);
    EXPECT_NEAR(-0.075323428127562776000477285, roots[0], 1e-11);
    EXPECT_NEAR(0.27137752045040101656387732, roots[1], 1e-11);
    EXPECT_NEAR(-2.4460540923228382405634000, roots[2], 1e-11);
}

// Find the roots of a Polynomial where the coefficient of the highest power is 0
TEST_F(UtilsTest, findRoots_polynomial_with_zero_leading_coefficient){
    // y 0x^3 + x^2
    std::vector<double> coeff = {0, 0, 1, 0};
    Polynomial polynomial(coeff);

    std::vector<double> roots = findRealRoots(polynomial);
    EXPECT_EQ(roots.size(), 1);
    EXPECT_NEAR(0, roots[0], 1e-11);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}