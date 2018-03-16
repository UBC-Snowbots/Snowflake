/*
 * Created By: Gareth Ellis
 * Created On:  February 19th, 2018
 * Description: TODO
 */

#include <ObstacleManager.h>
#include <gtest/gtest.h>

class SplineLineTest : public testing::Test {
protected:
    SplineLineTest():{};

    virtual void SetUp() {
    }
};

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}