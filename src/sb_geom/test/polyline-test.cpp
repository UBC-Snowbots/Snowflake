/*
 * Created By: Gareth Ellis
 * Created On:  February 24th, 2018
 * Description: Test for the `PolyLine` class
 */

#include <gtest/gtest.h>
#include "sb_geom/PolyLine.h"

class PolyLineTest : public testing::Test {
protected:
    PolyLineTest(){};

    virtual void SetUp() {
    }
};

TEST_F(PolyLineTest, default_constructor){
    sb_geom::PolyLine poly_line;
    std::vector<double> expected = {};
    EXPECT_EQ(expected, poly_line.getCoefficients());
}

TEST_F(PolyLineTest, constructor_with_coefficients){
    std::vector<double> coefficients = {1,2,3};
    sb_geom::PolyLine poly_line(coefficients);
    EXPECT_EQ(coefficients, poly_line.getCoefficients());
}

TEST_F(PolyLineTest, constructor_from_PolyLine_msg){
    sb_geom_msgs::PolyLine poly_line_msg;
    poly_line_msg.coefficients = {1,2,3,4.421};
    sb_geom::PolyLine poly_line(poly_line_msg);
    EXPECT_EQ(poly_line_msg.coefficients, poly_line.getCoefficients());
    EXPECT_EQ(4, poly_line.getDegree());
}

TEST_F(PolyLineTest, getDegree){
    sb_geom::PolyLine poly_line;
    std::vector<double> coefficients = {1,2,3};
    poly_line.setCoefficients(coefficients);
    EXPECT_EQ(3, poly_line.getDegree());
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}