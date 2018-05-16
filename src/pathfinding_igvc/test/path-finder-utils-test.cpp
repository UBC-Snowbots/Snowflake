//
// Created by min on 14/05/18.
//
#include <PathFinderUtils.h>
#include <gtest/gtest.h>

TEST(PathFinderUtils, TestGetAngleBetweenPoints) {
    geometry_msgs::Point from;
    from.x = -99.0;
    from.y = -99.0;
    from.z = 0.0;

    geometry_msgs::Point to;
    to.x = from.x - sqrt(3);
    to.y = from.y - 1.0;
    to.z = 0.0;

    double angle     = PathFinderUtils::getAngleBetweenPoints(from, to);
    tf::Quaternion q = PathFinderUtils::getQuaternionFromAngle(angle);
    EXPECT_FLOAT_EQ(M_PI + M_PI / 6, q.getAngle());
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
