/*
 * Created By: Gareth Ellis
 * Created On: July 22, 2016
 * Description: Tests for the GpsMover class
 */

#include <GpsMover.h>
#include <gtest/gtest.h>

/**
 * Creates a geometry_msgs::Point with given x and y coordinates
 *
 * @param x the x coordinate of the point
 * @param y the y coordinate of the point
 * @return a point with given x and y coordinates
 */
geometry_msgs::Point createPoint(double x, double y){
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0;
    return p;
}


class GpsMoverTest : public testing::Test {
protected:
    virtual void SetUp(){
    }

    GpsMover mover1;
};

// Test with robot facing straight ahead, with the waypoint to
// the right and forwards of the robot
TEST_F(GpsMoverTest, createTwistMessage_heading0) {
    auto current_location = createPoint(0,0);
    auto waypoint = createPoint(2,2);

    auto twist = mover1.createTwistMessage(
            current_location,
            0,
            waypoint
            );

    // We should be trying to turn left
    EXPECT_GE(twist.angular.z, 0);
}

// Test of the robot facing "backwards" relative to initial heading,
// with the waypoint (relative to the robot) behind and to the left
TEST_F(GpsMoverTest, createTwistMessage_headingPiObstacleBehindAndLeft) {
    auto current_location = createPoint(0,0);
    auto waypoint = createPoint(2,-2);

    auto twist = mover1.createTwistMessage(
            current_location,
            M_PI,
            waypoint
    );

    // We should be trying to turn left
    EXPECT_GE(twist.angular.z, 0);
}

// Test of the robot facing "backwards" relative to initial heading,
// with the waypoint behind and to the right (relative to the robot)
TEST_F(GpsMoverTest, createTwistMessage_headingPiObstacleBehindAndRight) {
    auto current_location = createPoint(0,0);
    auto waypoint = createPoint(2,2);

    auto twist = mover1.createTwistMessage(
            current_location,
            M_PI,
            waypoint
    );

    // We should be trying to turn right
    EXPECT_LE(twist.angular.z, 0);
}

// Test with the robot facing straight forward (relative to initial heading)
// with the waypoint far behind and to the left
TEST_F(GpsMoverTest, createTwistMessage_headingLeftWaypointBehindAndLeft) {
    auto current_location = createPoint(18.9, -6.96);
    auto waypoint = createPoint(9.28, 2.47);

    auto twist = mover1.createTwistMessage(
            current_location,
            -0.5876,
            waypoint
    );

    // We should be trying to turn left
    EXPECT_GE(twist.angular.z, 0);
}

// Test with the robot facing straight forward (relative to initial heading)
// with the waypoint far behind and to the right
TEST_F(GpsMoverTest, createTwistMessage_headingLeftWaypointBehindAndRight) {
    auto current_location = createPoint(18.9, 0);
    auto waypoint = createPoint(9.28, -8.47);

    auto twist = mover1.createTwistMessage(
            current_location,
            -0.5876,
            waypoint
    );

    // We should be trying to turn right
    EXPECT_LE(twist.angular.z, 0);
}

// Test of angleBetweenPoints with the destination point
// ahead and to the left of the start point
TEST_F(GpsMoverTest, angleBetweenPoints_FrontLeft){
    auto startPoint = createPoint(0,0);
    auto endPoint = createPoint(1,0.5);
    double angle = mover1.angleBetweenPoints(startPoint, endPoint);
    EXPECT_DOUBLE_EQ(atan(0.5), angle);
}

// Test of angleBetweenPoints with the destination point
// behind and to the left of the start point
TEST_F(GpsMoverTest, angleBetweenPoints_BackLeft){
    auto startPoint = createPoint(0,0);
    auto endPoint = createPoint(-1,0.5);
    double angle = mover1.angleBetweenPoints(startPoint, endPoint);
    EXPECT_DOUBLE_EQ(M_PI - atan(0.5), angle);
}

// Test of angleBetweenPoints with the destination point
// ahead and to the right of the start point
TEST_F(GpsMoverTest, angleBetweenPoints_FrontRight){
    auto startPoint = createPoint(0,0);
    auto endPoint = createPoint(1,-0.5);
    double angle = mover1.angleBetweenPoints(startPoint, endPoint);
    EXPECT_DOUBLE_EQ(-atan(0.5), angle);
}

// Test of angleBetweenPoints with the destination point
// behind and to the right of the start point
TEST_F(GpsMoverTest, angleBetweenPoints_BackRight){
    auto startPoint = createPoint(0,0);
    auto endPoint = createPoint(-1,-0.5);
    double angle = mover1.angleBetweenPoints(startPoint, endPoint);
    EXPECT_DOUBLE_EQ(-M_PI + atan(0.5), angle);
}

// Simple test where the angle are already separated by an acute angle if
// just subtracted
TEST_F(GpsMoverTest, minAngularChangeTest_acuteAngle){
    double angle = mover1.minAngularChange(-M_PI, -M_PI/4);
    EXPECT_DOUBLE_EQ(3.0/4.0 * M_PI, angle);
}

// Test where the angle returned should be negative
TEST_F(GpsMoverTest, minAngularChangeTest_negativeReturnAngle){
    double angle = mover1. minAngularChange(M_PI, M_PI/4);
    EXPECT_DOUBLE_EQ(-3.0/4.0 * M_PI, angle);
}

// Test where angles are separated by an obtuse angle (if just subtracted)
TEST_F(GpsMoverTest, minAngularChangeTest_obtuseAngle){
    double should_be_acute = mover1.minAngularChange(M_PI, -M_PI/4);
    EXPECT_DOUBLE_EQ(3.0/4.0 * M_PI, should_be_acute);
}

//Test where angle are separated by a negative obtuse angle (if just subtracted)
TEST_F(GpsMoverTest, minAngularChangeTest_negativeObtuseAngle){
    double should_be_acute = mover1.minAngularChange(-M_PI, M_PI/4);
    EXPECT_DOUBLE_EQ(-3.0/4.0 * M_PI, should_be_acute);
}

// Test out our magic function......
TEST_F(GpsMoverTest, magicFunctionTest_basic){
    EXPECT_DOUBLE_EQ((1/(double)10 + sqrt((double)5))/2, mover1.magicFunction(10, 5, 1, 1));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
