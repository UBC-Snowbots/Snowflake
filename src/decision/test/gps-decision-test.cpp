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
    geometry_msgs::Point myPoint;
    myPoint.x = 7.2;
    myPoint.y = 3.5;
    myPoint.z = 0;
    double expected=sqrt(pow(myPoint.x,2)+pow(myPoint.y,2)+pow(myPoint.z,2));
    geometry_msgs::Point::ConstPtr myPointer(new geometry_msgs::Point(myPoint));
    EXPECT_DOUBLE_EQ(expected, GpsDecision::distance(myPointer));
}

//Basic test 2 for distance function
TEST(GpsDecision_Basic2,distance) {
    geometry_msgs::Point myPoint;
    myPoint.x = -2.23;
    myPoint.y = -4.12;
    myPoint.z = 0;
    double expected=sqrt(pow(myPoint.x,2)+pow(myPoint.y,2)+pow(myPoint.z,2));
    geometry_msgs::Point::ConstPtr myPointer(new geometry_msgs::Point(myPoint));
    EXPECT_DOUBLE_EQ(expected, GpsDecision::distance(myPointer));
}


// desiredAngle test1 to test the basic functionality when x>0 and y>0
// all the degrees are relative to +x, in the range of [-180,180],
// positive means the right part , negative means the left part
// the current heading towards north(at 0 degree)
// nextPoint is at forward-left of the current point(i.e at -60 degrees)
// the robot should rotate 60 degrees clockwise(i.e 60 degrees)
TEST(GpsDecision_test1,desiredAngle) {

    geometry_msgs::Point nextPoint;
    geometry_msgs::Point currentPoint;
    nextPoint.x = 1;
    nextPoint.y = sqrt(3);
    nextPoint.z = 0;
    currentPoint.x = 0;
    currentPoint.y = 0;
    currentPoint.z = 0;

    float my_heading=0;

    double expected=(60*M_PI/180);

    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(nextPoint,my_heading,currentPoint));

}

// desiredAngle test2 to test the basic functionality when x>0 and y<0
// all the degrees are relative to +x, in the range of [-180,180],
// positive means the right part , negative means the left part
// the current heading towards north (at 0 degree)
// nextPoint is at north east of the current point(i.e at 60 degrees)
// the robot should rotate 60 degrees counter-clock wise(i.e -60 degrees)
TEST(GpsDecision_test2,desiredAngle) {
    geometry_msgs::Point nextPoint;
    geometry_msgs::Point currentPoint;
    nextPoint.x = 1;
    nextPoint.y = -sqrt(3);
    nextPoint.z = 0;
    currentPoint.x = 0;
    currentPoint.y = 0;
    currentPoint.z = 0;

    float my_heading=0;

    double expected=(-60);

    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(nextPoint,my_heading,currentPoint)*180/M_PI);

}

// desiredAngle test3 to test the basic functionality when x<0 and y<0
// all the degrees are relative to +x, in the range of [-180,180],
// positive means the right part , negative means the left part
// the current heading towards north (at 0 degree)
// nextPoint is at south east of the current point(i.e at 60 degrees)
// the robot should rotate 60 degrees counter-clock wise(i.e -60 degrees)
TEST(GpsDecision_test3,desiredAngle) {
    geometry_msgs::Point nextPoint;
    geometry_msgs::Point currentPoint;
    nextPoint.x = -sqrt(3);
    nextPoint.y = -1;
    nextPoint.z = 0;
    currentPoint.x = 0;
    currentPoint.y = 0;
    currentPoint.z = 0;

    float my_heading=0;

    double expected=(-150);

    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(nextPoint,my_heading,currentPoint)*180/M_PI);

}

// desiredAngle test4 to test the basic functionality when x<0 and y>0
// all the degrees are relative to +x, in the range of [-180,180],
// positive means the right part , negative means the left part
// the current heading towards north (at 0 degree)
// nextPoint is at south west of the current point(i.e at -120 degrees)
// the robot should rotate 60 degrees counter-clock wise(i.e 120 degrees)
TEST(GpsDecision_test4,desiredAngle) {
    geometry_msgs::Point nextPoint;
    geometry_msgs::Point currentPoint;
    nextPoint.x = -1;
    nextPoint.y = sqrt(3);
    nextPoint.z = 0;
    currentPoint.x = 0;
    currentPoint.y = 0;
    currentPoint.z = 0;

    float my_heading=0;

    double expected=(120);

    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(nextPoint,my_heading,currentPoint)*180/M_PI);

}




/*
// desiredAngle test4 to test different headings at different quarant
// the current heading is at 137 degree
// nextPoint is at north east of the current point(i.e at 45 degree)
// the robot should rotate -(137-45)=-92 degrees
TEST(GpsDecision_test4,desiredAngle) {
    geometry_msgs::Point nextPoint;
    geometry_msgs::Point currentPoint;
    nextPoint.x = 1;
    nextPoint.y = 1;
    nextPoint.z = 0;
    currentPoint.x = 0;
    currentPoint.y = 0;
    currentPoint.z = 0;

    float my_heading=137;

    double expected=(-92*M_PI/180);

    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(nextPoint,my_heading,currentPoint));

}

// desiredAngle test5 to test different headings at different quarant
// the current heading is at 267 degree
// nextPoint is at north east of the current point(i.e at 45 degree)
// the robot should rotate 360-(267-45)=138 degrees
TEST(GpsDecision_test5,desiredAngle) {
    geometry_msgs::Point nextPoint;
    geometry_msgs::Point currentPoint;
    nextPoint.x = 1;
    nextPoint.y = 1;
    nextPoint.z = 0;
    currentPoint.x = 0;
    currentPoint.y = 0;
    currentPoint.z = 0;

    float my_heading=267;

    double expected=(138*M_PI/180);

    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(nextPoint,my_heading,currentPoint));

}

// desiredAngle test6 to test different next point
// the current heading is at 0 degree
// nextPoint is at north west of the current point(i.e at -45 degree)
// the robot should rotate -45 degrees
TEST(GpsDecision_test6,desiredAngle) {
    geometry_msgs::Point nextPoint;
    geometry_msgs::Point currentPoint;
    nextPoint.x = -1;
    nextPoint.y = 1;
    nextPoint.z = 0;
    currentPoint.x = 0;
    currentPoint.y = 0;
    currentPoint.z = 0;

    float my_heading=0;

    double expected=(-45*M_PI/180);

    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(nextPoint,my_heading,currentPoint));

}

// desiredAngle test7 to test complex situation
// the current heading is at -17 degree
// nextPoint is at -225 degree of the current point
// the robot should rotate -225-(-17)+360 degrees
TEST(GpsDecision_test7,desiredAngle) {
    geometry_msgs::Point nextPoint;
    geometry_msgs::Point currentPoint;
    nextPoint.x = 1;
    nextPoint.y = -1;
    nextPoint.z = 0;
    currentPoint.x = 0;
    currentPoint.y = 0;
    currentPoint.z = 0;

    float my_heading=-17;

    double expected=(152*M_PI/180);

    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(nextPoint,my_heading,currentPoint));

}

// desiredAngle test8 to test complex situation
// the current heading is at 43 degree
// nextPoint is at -60 degree of the current point
// the robot should rotate -(43+60)=-103 degrees
TEST(GpsDecision_test8,desiredAngle) {
    geometry_msgs::Point nextPoint;
    geometry_msgs::Point currentPoint;
    nextPoint.x = -sqrt(3);
    nextPoint.y = 1;
    nextPoint.z = 0;
    currentPoint.x = 0;
    currentPoint.y = 0;
    currentPoint.z = 0;

    float my_heading=43;

    double expected=(-103*M_PI/180);

    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(nextPoint,my_heading,currentPoint));

}*/

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
