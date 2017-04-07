/*
 * Created By: Yu Chen
 * Created On: Oct 15th, 2016
 * Description: Tests for GpsDecision
 */

#include <GpsDecision.h>
#include <gtest/gtest.h>
#include <geometry_msgs/Point.h>

//initalize the current point to the origin point to make tests easy.
geometry_msgs::Point currentPoint;

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
// the robot should rotate -60 degrees clockwise(i.e -60 degrees)
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

    double expected=(-60*M_PI/180);

    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(nextPoint,my_heading,currentPoint));

}

// desiredAngle test2 to test the basic functionality when x>0 and y<0
// all the degrees are relative to +x, in the range of [-180,180],
// positive means the right part , negative means the left part
// the current heading towards north (at 0 degree)
// nextPoint is at north east of the current point(i.e at 60 degrees)
// the robot should rotate 60 degrees counter-clock wise(i.e 60 degrees)
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

    double expected=(60);

    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(nextPoint,my_heading,currentPoint)*180/M_PI);

}

// desiredAngle test3 to test the basic functionality when x<0 and y<0
// all the degrees are relative to +x, in the range of [-180,180],
// positive means the right part , negative means the left part
// the current heading towards north (at 0 degree)
// nextPoint is at south east of the current point(i.e at 120 degrees)
// the robot should rotate 120 degrees counter-clock wise(i.e 120 degrees)
TEST(GpsDecision_test3,desiredAngle) {
    geometry_msgs::Point nextPoint;
    geometry_msgs::Point currentPoint;
    nextPoint.x = -1;
    nextPoint.y = -sqrt(3);
    nextPoint.z = 0;
    currentPoint.x = 0;
    currentPoint.y = 0;
    currentPoint.z = 0;

    float my_heading=0;

    double expected=(120);

    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(nextPoint,my_heading,currentPoint)*180/M_PI);

}

// desiredAngle test4 to test the basic functionality when x<0 and y>0
// all the degrees are relative to +x, in the range of [-180,180],
// positive means the right part , negative means the left part
// the current heading towards north (at 0 degree)
// nextPoint is at south west of the current point(i.e at -120 degrees)
// the robot should rotate 120 degrees counter-clock wise(i.e -120 degrees)
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

    double expected=(-120);

    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(nextPoint,my_heading,currentPoint)*180/M_PI);

}

// desiredAngle test5 to test the basic functionality with heading!=0 when x>0 and y>0
// At this heading situation, the robot no need to turn in another direction
// all the degrees are relative to +x, in the range of [-180,180],
// positive means the right part , negative means the left part
// the current heading towards north (at -15 degree)
// nextPoint is at south west of the current point(i.e at -60 degrees)
// the robot should rotate 120 degrees counter-clock wise(i.e -45 degrees)
TEST(GpsDecision_test5,desiredAngle) {
    geometry_msgs::Point nextPoint;
    geometry_msgs::Point currentPoint;
    nextPoint.x = 1;
    nextPoint.y = sqrt(3);
    nextPoint.z = 0;
    currentPoint.x = 0;
    currentPoint.y = 0;
    currentPoint.z = 0;

    float my_heading=-15;

    double expected=(-45);

    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(nextPoint,my_heading,currentPoint)*180/M_PI);
}

// desiredAngle test6 to test the basic functionality with heading!=0 when x>0 and y>0
// At this heading situation, the robot no need to turn in another direction
// all the degrees are relative to +x, in the range of [-180,180],
// positive means the right part , negative means the left part
// the current heading towards north (at 75 degree)
// nextPoint is at south west of the current point(i.e at -60 degrees)
// the robot should rotate 120 degrees counter-clock wise(i.e -135 degrees)
TEST(GpsDecision_test6,desiredAngle) {
    geometry_msgs::Point nextPoint;
    geometry_msgs::Point currentPoint;
    nextPoint.x = 1;
    nextPoint.y = sqrt(3);
    nextPoint.z = 0;
    currentPoint.x = 0;
    currentPoint.y = 0;
    currentPoint.z = 0;

    float my_heading=75;

    double expected=(-135);

    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(nextPoint,my_heading,currentPoint)*180/M_PI);
}
// desiredAngle test7 to test the basic functionality with heading!=0 when x>0 and y>0
// At this heading situation, the robot needs to turn in another direction
// all the degrees are relative to +x, in the range of [-180,180],
// positive means the right part , negative means the left part
// the current heading towards north (at -70 degree)
// nextPoint is at south west of the current point(i.e at -60 degrees)
// the robot should rotate 120 degrees counter-clock wise(i.e 10 degrees)
TEST(GpsDecision_test7,desiredAngle) {
    geometry_msgs::Point nextPoint;
    geometry_msgs::Point currentPoint;
    nextPoint.x = 1;
    nextPoint.y = sqrt(3);
    nextPoint.z = 0;
    currentPoint.x = 0;
    currentPoint.y = 0;
    currentPoint.z = 0;

    float my_heading=-70;

    double expected=(10);

    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(nextPoint,my_heading,currentPoint)*180/M_PI);
}

// desiredAngle test8 to test the basic functionality with heading!=0 when x>0 and y>0
// At this heading situation, the robot needs to turn in another direction
// all the degrees are relative to +x, in the range of [-180,180]
// the current heading towards north (at 175 degree)
// nextPoint is at south west of the current point(i.e at -60 degrees)
// the robot should rotate 120 degrees counter-clock wise(i.e 10 degrees)
TEST(GpsDecision_test8,desiredAngle) {
    geometry_msgs::Point nextPoint;
    geometry_msgs::Point currentPoint;
    nextPoint.x = 1;
    nextPoint.y = sqrt(3);
    nextPoint.z = 0;
    currentPoint.x = 0;
    currentPoint.y = 0;
    currentPoint.z = 0;

    float my_heading=175;

    double expected=(125);

    EXPECT_DOUBLE_EQ(expected, GpsDecision::desiredAngle(nextPoint,my_heading,currentPoint)*180/M_PI);
}



int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
