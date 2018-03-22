/*
 * Created By: Gareth Ellis
 * Created On:  February 24th, 2018
 * Description: Test for the `PolyLine` class
 */

#include <gtest/gtest.h>
#include "sb_geom/Spline.h"
#include "sb_geom/Point2D.h"

class SplineTest : public testing::Test {
protected:
    SplineTest(){};

    virtual void SetUp() {
    }
};

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