/*
 * Created By: Gareth Ellis
 * Created On:  February 24th, 2018
 * Description: Tests for the `Polynomial` class
 */

#include "sb_geom/Polynomial.h"
#include <gtest/gtest.h>

class PolynomialTest : public testing::Test {
  protected:
    PolynomialTest(){};

    virtual void SetUp() {}
};

TEST_F(PolynomialTest, default_constructor) {
    sb_geom::Polynomial poly_line;
    std::vector<double> expected = {};
    EXPECT_EQ(expected, poly_line.coefficients());
}

TEST_F(PolynomialTest, constructor_with_coefficients) {
    std::vector<double> coefficients = {1, 2, 3};
    sb_geom::Polynomial poly_line(coefficients);
    EXPECT_EQ(coefficients, poly_line.coefficients());
}

TEST_F(PolynomialTest, constructor_from_Polynomial_msg) {
    sb_geom_msgs::Polynomial poly_line_msg;
    poly_line_msg.coefficients = {1, 2, 3, 4.421};
    sb_geom::Polynomial poly_line(poly_line_msg);
    EXPECT_EQ(poly_line_msg.coefficients, poly_line.coefficients());
    EXPECT_EQ(4, poly_line.getDegree());
}

TEST_F(PolynomialTest, getDegree) {
    sb_geom::Polynomial poly_line;
    poly_line.coefficients() = {1, 2, 3};
    EXPECT_EQ(3, poly_line.getDegree());
}

TEST_F(PolynomialTest, get_y_from_x_simple) {
    sb_geom::Polynomial poly_line;
    poly_line.coefficients() = {1, 1};

    EXPECT_EQ(0, poly_line(-1));
    EXPECT_EQ(1, poly_line(0));
    EXPECT_EQ(2, poly_line(1));
}

TEST_F(PolynomialTest, get_y_from_x_complex) {
    sb_geom::Polynomial poly_line;
    poly_line.coefficients() = {1, 1, 3, 1};

    EXPECT_EQ(1, poly_line(0));
    EXPECT_EQ(1 + 3 + 3 * (3 * 3) + (3 * 3 * 3), poly_line(3));
    EXPECT_EQ(1 + 10 + 3 * (10 * 10) + (10 * 10 * 10), poly_line(10));
    EXPECT_EQ(1 - 1.2 + 3 * (-1.2 * -1.2) + (-1.2 * -1.2 * -1.2),
              poly_line(-1.2));
}

// Test out using the () operator to access a y value at
// a given x value
TEST_F(PolynomialTest, round_bracket_operator_test) {
    // y = x^7 + 3x^4
    std::vector<double> coeff = {0, 0, 0, 0, 3, 0, 0, 1};
    sb_geom::Polynomial polynomial(coeff);

    // x = 3.7, y = 10055.436013300003
    EXPECT_DOUBLE_EQ(10055.436013300003, polynomial(3.7));

    // x = -7.5 y = 0.81573486328125
    EXPECT_DOUBLE_EQ(0.81573486328125, polynomial(-0.75));
}

// Test the `deriv` function that returns the derivative Polynomial
TEST_F(PolynomialTest, deriv_get_polynomial) {
    // y = x^7 + 3x^4 + 9.3x^2 + 3.2
    std::vector<double> coeff = {3.2, 0, 9.3, 0, 3, 0, 0, 1};
    sb_geom::Polynomial polynomial(coeff);

    // 0th derivative should just be y
    EXPECT_EQ(coeff, polynomial.deriv(0).coefficients());

    // 1st deriv. of y = 7x^6 + 12x^3 + 18.6x
    std::vector<double> dx_coeff = {0, 18.6, 0, 12, 0, 0, 7};
    EXPECT_EQ(dx_coeff, polynomial.deriv(1).coefficients());

    // 2nd deriv. of y = 42x^5 + 36x^2 + 18.6
    std::vector<double> d2x_coeff = {18.6, 0, 36, 0, 0, 42};
    EXPECT_EQ(d2x_coeff, polynomial.deriv(2).coefficients());

    // 10th deriv. of y = 0
    EXPECT_EQ(std::vector<double>(), polynomial.deriv(10).coefficients());
}

// Test the `deriv` function that returns the value of the derivative
TEST_F(PolynomialTest, deriv_get_value) {
    // y = x^7 + 3x^4 + 9.3x^2 + 3.2
    std::vector<double> coeff = {3.2, 0, 9.3, 0, 3, 0, 0, 1};
    sb_geom::Polynomial polynomial(coeff);

    // 0th derivative
    EXPECT_DOUBLE_EQ(4722.097597699998, polynomial.deriv(3.3, 0));

    // 1st deriv. of y = 7x^6 + 12x^3 + 18.6x
    EXPECT_DOUBLE_EQ(2998.359423000001, polynomial.deriv(2.7, 1));

    // 2nd deriv. of y = 42x^5 + 36x^2 + 18.6
    std::vector<double> d2x_coeff = {18.6, 0, 36, 0, 0, 42};
    EXPECT_DOUBLE_EQ(29.077499309399997, polynomial.deriv(-0.73, 2));

    // 10th deriv. of y = 0
    EXPECT_DOUBLE_EQ(0, polynomial.deriv(19, 10));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}