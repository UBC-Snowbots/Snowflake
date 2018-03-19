/*
 * Created By: Gareth Ellis
 * Created On:  February 24th, 2018
 * Description: Test for the `PolyLine` class
 */

#include <gtest/gtest.h>
#include "sb_geom/SplineLine.h"
#include "sb_geom/Point2D.h"

class SplineLineTest : public testing::Test {
protected:
    SplineLineTest(){};

    virtual void SetUp() {
    }
};

// TODO: Not a real test - delete me
TEST_F(SplineLineTest, messing_about){
    std::vector<sb_geom::Point2D> points = {
            {0,0},
            {2,2},
            {0,3},
    };
    sb_geom::SplineLine spline_line(points);

    std::cout << "~~ Points ~~"  << std::endl;
    for (double i = 0; i <= 1; i += 0.05){
        std::cout << spline_line._spline(i)(0) << ", "
                << spline_line._spline(i)(1) << std::endl;
    }
    std::cout << "~~ Derivatives ~~" << std::endl;
    for (double i = 0; i <= 1; i += 0.05){
        std::cout << spline_line._spline.derivatives(i, 3) << std::endl;
        std::cout << "--" << std::endl;
    }

}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}