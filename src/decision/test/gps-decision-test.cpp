/*
 * Created By: Yu Chen
 * Created On: Oct 15th, 2016
 * Description: Tests for GpsDecision
 */

#include <GpsDecision.h>
#include <gtest/gtest.h>
#include <geometry_msgs/Point.h>

TEST(GpsDecison,distance) {
    geometry_msgs::Point mypoint;
    mypoint.x = 6;
    mypoint.y = 3;
    mypoint.z = 0;
    double expected=sqrt(pow(mypoint.x,2)+pow(mypoint.y,2)+pow(mypoint.z,2));
    geometry_msgs::Point::ConstPtr mypointer(new geometry_msgs::Point(mypoint));
    EXPECT_DOUBLE_EQ(expected, GpsDecision::distance(mypointer));
}

TEST(GpsDecison,desiredAngle) {
    #define PI acos(-1.0)
    geometry_msgs::Point mypoint;
    mypoint.x = 0;
    mypoint.y = 0;
    mypoint.z = 0;
    float my_heading=0;

    double expected=20;
    geometry_msgs::Point::ConstPtr mypointer(new geometry_msgs::Point(mypoint));
    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(mypointer,my_heading));

}





int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
