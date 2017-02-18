/*
 * Created By: Yu Chen
 * Created On: Oct 15th, 2016
 * Description: Tests for GpsDecision
 */

#include <GpsDecision.h>
#include <gtest/gtest.h>
#include <geometry_msgs/Point.h>

//Basic test 1 for distance function with float values
TEST(GpsDecision_Baisc1,distance) {
    geometry_msgs::Point mypoint;
    mypoint.x = 7.2;
    mypoint.y = 3.5;
    mypoint.z = 0;
    double expected=sqrt(pow(mypoint.x,2)+pow(mypoint.y,2)+pow(mypoint.z,2));
    geometry_msgs::Point::ConstPtr mypointer(new geometry_msgs::Point(mypoint));
    EXPECT_DOUBLE_EQ(expected, GpsDecision::distance(mypointer));
}

//Basic test 2 for distance function
TEST(GpsDecision_Basic2,distance) {
    geometry_msgs::Point mypoint;
    mypoint.x = -2.23;
    mypoint.y = -4.12;
    mypoint.z = 0;
    double expected=sqrt(pow(mypoint.x,2)+pow(mypoint.y,2)+pow(mypoint.z,2));
    geometry_msgs::Point::ConstPtr mypointer(new geometry_msgs::Point(mypoint));
    EXPECT_DOUBLE_EQ(expected, GpsDecision::distance(mypointer));
}


// desiredAngle test1 to test the basic functionality
// the current heading towards north(at 0 degree)
// nextpoint is at north east of the current point(i.e at 45 degree)
// the robot should rotate +45 degrees clockwise
TEST(GpsDecision_test1,desiredAngle) {

    geometry_msgs::Point nextpoint;
    geometry_msgs::Point currentPoint;
    nextpoint.x = 1;
    nextpoint.y = 1;
    nextpoint.z = 0;
    currentPoint.x = 0;
    currentPoint.y = 0;
    currentPoint.z = 0;

    float my_heading=0;

    double expected=(45*M_PI/180);

    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(nextpoint,my_heading,currentPoint));

}

// desiredAngle test2 to test different headings in clockwise
// the current heading towards north-east(at 30 degree)
// nextpoint is at north east of the current point(i.e at 45 degree)
// the robot should rotate +15 degrees
TEST(GpsDecision_test2,desiredAngle) {
    geometry_msgs::Point nextpoint;
    geometry_msgs::Point currentPoint;
    nextpoint.x = 1;
    nextpoint.y = 1;
    nextpoint.z = 0;
    currentPoint.x = 0;
    currentPoint.y = 0;
    currentPoint.z = 0;

    float my_heading=30;

    double expected=(15*M_PI/180);

    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(nextpoint,my_heading,currentPoint));

}

// desiredAngle test3 to test different headings but rotate in counter-clockwise
// the current heading towards north-east(at 70 degree)
// nextpoint is at north east of the current point(i.e at 45 degree)
// the robot should rotate -25 degrees
TEST(GpsDecision_test3,desiredAngle) {
    geometry_msgs::Point nextpoint;
    geometry_msgs::Point currentPoint;
    nextpoint.x = 1;
    nextpoint.y = 1;
    nextpoint.z = 0;
    currentPoint.x = 0;
    currentPoint.y = 0;
    currentPoint.z = 0;

    float my_heading=70;

    double expected=(-25*M_PI/180);

    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(nextpoint,my_heading,currentPoint));

}

// desiredAngle test4 to test different headings at different quarant
// the current heading is at 137 degree
// nextpoint is at north east of the current point(i.e at 45 degree)
// the robot should rotate -(137-45)=-92 degrees
TEST(GpsDecision_test4,desiredAngle) {
    geometry_msgs::Point nextpoint;
    geometry_msgs::Point currentPoint;
    nextpoint.x = 1;
    nextpoint.y = 1;
    nextpoint.z = 0;
    currentPoint.x = 0;
    currentPoint.y = 0;
    currentPoint.z = 0;

    float my_heading=137;

    double expected=(-92*M_PI/180);

    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(nextpoint,my_heading,currentPoint));

}

// desiredAngle test5 to test different headings at different quarant
// the current heading is at 267 degree
// nextpoint is at north east of the current point(i.e at 45 degree)
// the robot should rotate 360-(267-45)=138 degrees
TEST(GpsDecision_test5,desiredAngle) {
    geometry_msgs::Point nextpoint;
    geometry_msgs::Point currentPoint;
    nextpoint.x = 1;
    nextpoint.y = 1;
    nextpoint.z = 0;
    currentPoint.x = 0;
    currentPoint.y = 0;
    currentPoint.z = 0;

    float my_heading=267;

    double expected=(138*M_PI/180);

    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(nextpoint,my_heading,currentPoint));

}

// desiredAngle test6 to test different next point
// the current heading is at 0 degree
// nextpoint is at north west of the current point(i.e at -45 degree)
// the robot should rotate -45 degrees
TEST(GpsDecision_test6,desiredAngle) {
    geometry_msgs::Point nextpoint;
    geometry_msgs::Point currentPoint;
    nextpoint.x = -1;
    nextpoint.y = 1;
    nextpoint.z = 0;
    currentPoint.x = 0;
    currentPoint.y = 0;
    currentPoint.z = 0;

    float my_heading=0;

    double expected=(-45*M_PI/180);

    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(nextpoint,my_heading,currentPoint));

}

// desiredAngle test7 to test complex situation
// the current heading is at -17 degree
// nextpoint is at -225 degree of the current point
// the robot should rotate -225-(-17)+360 degrees
TEST(GpsDecision_test7,desiredAngle) {
    geometry_msgs::Point nextpoint;
    geometry_msgs::Point currentPoint;
    nextpoint.x = 1;
    nextpoint.y = -1;
    nextpoint.z = 0;
    currentPoint.x = 0;
    currentPoint.y = 0;
    currentPoint.z = 0;

    float my_heading=-17;

    double expected=(152*M_PI/180);

    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(nextpoint,my_heading,currentPoint));

}

// desiredAngle test8 to test complex situation
// the current heading is at 43 degree
// nextpoint is at -60 degree of the current point
// the robot should rotate -(43+60)=-103 degrees
TEST(GpsDecision_test8,desiredAngle) {
    geometry_msgs::Point nextpoint;
    geometry_msgs::Point currentPoint;
    nextpoint.x = -sqrt(3);
    nextpoint.y = 1;
    nextpoint.z = 0;
    currentPoint.x = 0;
    currentPoint.y = 0;
    currentPoint.z = 0;

    float my_heading=43;

    double expected=(-103*M_PI/180);

    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(nextpoint,my_heading,currentPoint));

}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
