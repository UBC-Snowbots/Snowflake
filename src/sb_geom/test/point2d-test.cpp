/*
 * Created By: Gareth Ellis
 * Created On:  February 24th, 2018
 * Description: A test to make sure that we're building the Point2D msg
 * correctly
 */

#include "sb_geom/Point2D.h"
#include <gtest/gtest.h>

using namespace sb_geom;

class Point2DTest : public testing::Test {
  protected:
    Point2DTest(){};

    virtual void SetUp() {}
};

// This test doesn't really testing anything in particular, it's just here to
// make sure the Point2D message builds correctly
TEST_F(Point2DTest, test_message_building) {
    sb_geom::Point2D point;
    point.x = 1;
    point.y = 2;
    EXPECT_EQ(1, point.x);
    EXPECT_EQ(2, point.y);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}