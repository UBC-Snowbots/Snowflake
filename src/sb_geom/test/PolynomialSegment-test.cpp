/*
 * Created By: Gareth Ellis
 * Created On:  February 24th, 2018
 * Description: Tests for the `PolynomialSegment` class
 */

#include "sb_geom/PolynomialSegment.h"
#include <gtest/gtest.h>

class PolynomialSegmentTest : public testing::Test {
  protected:
    PolynomialSegmentTest(){};

    virtual void SetUp() {}
};

TEST_F(PolynomialSegmentTest, constructor) {
    // y = 3 + x
    sb_geom::PolynomialSegment poly_segment({3, 1}, 2, 10);

    EXPECT_EQ(poly_segment.x_min(), 2);
    EXPECT_EQ(poly_segment.x_max(), 10);
}

TEST_F(PolynomialSegmentTest, getStartPoint) {
    // y = 3 + x
    sb_geom::PolynomialSegment poly_segment({3, 1}, 2, 10);

    EXPECT_EQ(5, poly_segment.getStartPoint().y());
    EXPECT_EQ(2, poly_segment.getStartPoint().x());
}

TEST_F(PolynomialSegmentTest, getEndPoint) {
    // y = 3 + x
    sb_geom::PolynomialSegment poly_segment({3, 1}, 2, 10);

    EXPECT_EQ(13, poly_segment.getEndPoint().y());
    EXPECT_EQ(10, poly_segment.getEndPoint().x());
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}