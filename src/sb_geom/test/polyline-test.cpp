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
    EXPECT_EQ(expected, poly_line.coefficients());
}

TEST_F(PolyLineTest, constructor_with_coefficients){
    std::vector<double> coefficients = {1,2,3};
    sb_geom::PolyLine poly_line(coefficients);
    EXPECT_EQ(coefficients, poly_line.coefficients());
}

TEST_F(PolyLineTest, constructor_from_PolyLine_msg){
    sb_geom_msgs::PolyLine poly_line_msg;
    poly_line_msg.coefficients = {1,2,3,4.421};
    sb_geom::PolyLine poly_line(poly_line_msg);
    EXPECT_EQ(poly_line_msg.coefficients, poly_line.coefficients());
    EXPECT_EQ(4, poly_line.getDegree());
}

TEST_F(PolyLineTest, getDegree){
    sb_geom::PolyLine poly_line;
    poly_line.coefficients() = {1,2,3};
    EXPECT_EQ(3, poly_line.getDegree());
}

TEST_F(PolyLineTest, get_y_from_x_simple){
    sb_geom::PolyLine poly_line;
    poly_line.coefficients() = {1,1};

    EXPECT_EQ(0, poly_line(-1));
    EXPECT_EQ(1, poly_line(0));
    EXPECT_EQ(2, poly_line(1));
}

TEST_F(PolyLineTest, get_y_from_x_complex){
    sb_geom::PolyLine poly_line;
    poly_line.coefficients() = {1,1,3,1};

    EXPECT_EQ(1, poly_line(0));
    EXPECT_EQ(1 + 3 + 3*(3*3) + (3*3*3), poly_line(3));
    EXPECT_EQ(1 + 10 + 3*(10*10) + (10*10*10), poly_line(10));
    EXPECT_EQ(1 - 1.2 + 3*(-1.2*-1.2) + (-1.2*-1.2*-1.2), poly_line(-1.2));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}