/*
 * Created By: William Gu
 * Created On: Jan 19 2019
 * Description: G-unit tests for Reactive System
 */

#include <ReactiveSystemTwist.h>
#include <gtest/gtest.h>


TEST(ReactiveSystem, mytest) {
    EXPECT_EQ(0,0);
}



int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
