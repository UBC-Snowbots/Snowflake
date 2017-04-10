/*
 * Created By: Gareth Ellis
 * Created On: July 22, 2016
 * Description: Tests for GpsDecision
 */

#include <GpsDecision.h>
#include <gtest/gtest.h>



class MoverTest : public testing::Test {
protected:
    virtual void SetUp(){
        mover1.setFactors(1,1);
    }

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

    Mover mover1;
};

// Test with robot facing straight ahead, with the obstacle to
// the right and forwards of the robot
TEST_F(MoverTest, createTwistMessage_heading0) {
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
// with the obstacle (relative to the robot) behind and to the left
TEST_F(MoverTest, createTwistMessage_headingPiObstacleBehindAndLeft) {
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
// with the obstacle behind and to the right (relative to the robot)
TEST_F(MoverTest, createTwistMessage_headingPiObstacleBehindAndRight) {
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

// Simple test where the angle are already separated by an acute angle if
// just subtracted
TEST_F(MoverTest, minAngularChangeTest_acuteAngle){
    double angle = mover1.minAngularChange(-M_PI, -M_PI/4);
    EXPECT_DOUBLE_EQ(3.0/4.0 * M_PI, angle);
}

// Test where the angle returned should be negative
TEST_F(MoverTest, minAngularChangeTest_negativeReturnAngle){
    double angle = mover1. minAngularChange(M_PI, M_PI/4);
    EXPECT_DOUBLE_EQ(-3.0/4.0 * M_PI, angle);
}

// Test where angles are separated by an obtuse angle (if just subtracted)
TEST_F(MoverTest, minAngularChangeTest_obtuseAngle){
    double should_be_acute = mover1.minAngularChange(M_PI, -M_PI/4);
    EXPECT_DOUBLE_EQ(3.0/4.0 * M_PI, should_be_acute);
}

//Test where angle are separated by a negative obtuse angle (if just subtracted)
TEST_F(MoverTest, minAngularChangeTest_negativeObtuseAngle){
    double should_be_acute = mover1.minAngularChange(-M_PI, M_PI/4);
    EXPECT_DOUBLE_EQ(-3.0/4.0 * M_PI, should_be_acute);
}

TEST_F(MoverTest, magicFunctionTest_basic){
    EXPECT_DOUBLE_EQ((1/(double)10 + sqrt((double)5))/2, mover1.magicFunction(10, 5, 1, 1));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
