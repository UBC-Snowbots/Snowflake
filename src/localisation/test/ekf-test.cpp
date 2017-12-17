/*
 * Created By: Marcus Swift
 * Created On: November 20th, 2017
 * Description: Tests for EKF
 */

#include <EKF.h>
#include <gtest/gtest.h>
#include <cmath>


TEST(EKF, AngleReboundingZeroCases) {
  EXPECT_DOUBLE_EQ(0, EKF::defineAngleInBounds(-4.*M_PI));
  EXPECT_DOUBLE_EQ(0, EKF::defineAngleInBounds(-2.*M_PI));
  EXPECT_DOUBLE_EQ(0, EKF::defineAngleInBounds(2.*M_PI));
  EXPECT_DOUBLE_EQ(0, EKF::defineAngleInBounds(4.*M_PI));
  EXPECT_DOUBLE_EQ(0, EKF::defineAngleInBounds(0));
}
  
TEST(EKF, AngleReboundingM_PICases) {
  EXPECT_DOUBLE_EQ(M_PI, EKF::defineAngleInBounds(-M_PI));
  EXPECT_DOUBLE_EQ(M_PI, EKF::defineAngleInBounds(M_PI));
  EXPECT_DOUBLE_EQ(M_PI, EKF::defineAngleInBounds(-3.*M_PI));
  EXPECT_DOUBLE_EQ(M_PI, EKF::defineAngleInBounds(3.*M_PI));
}

TEST(EKF, AngleReboundingNegHalfM_PICases) {
  EXPECT_DOUBLE_EQ(-(M_PI/2.), EKF::defineAngleInBounds((7./2.)*M_PI));
  EXPECT_DOUBLE_EQ(-(M_PI/2.), EKF::defineAngleInBounds((-5./2.)*M_PI));
  EXPECT_DOUBLE_EQ(-(M_PI/2.), EKF::defineAngleInBounds((3./2.)*M_PI));
  EXPECT_DOUBLE_EQ(-(M_PI/2.), EKF::defineAngleInBounds((-1./2.)*M_PI));
}
  
TEST(EKF, AngleReboundingHalfM_PICases) {
  EXPECT_DOUBLE_EQ(M_PI/2., EKF::defineAngleInBounds((1./2.)*M_PI));
  EXPECT_DOUBLE_EQ(M_PI/2., EKF::defineAngleInBounds((-3./2.)*M_PI));
  EXPECT_DOUBLE_EQ(M_PI/2., EKF::defineAngleInBounds((5./2.)*M_PI));
  EXPECT_DOUBLE_EQ(M_PI/2., EKF::defineAngleInBounds((-7./2.)*M_PI));
}

TEST(EKF, AngleReboundingMiddleCases) {
  EXPECT_DOUBLE_EQ(-3.*M_PI/4., EKF::defineAngleInBounds((-3./4.)*M_PI));
  EXPECT_DOUBLE_EQ(M_PI/3., EKF::defineAngleInBounds((1./3.)*M_PI));
  EXPECT_DOUBLE_EQ(-M_PI/3., EKF::defineAngleInBounds((-1./3.)*M_PI));
  EXPECT_DOUBLE_EQ(2.*M_PI/3., EKF::defineAngleInBounds((2./3.)*M_PI));
  EXPECT_DOUBLE_EQ(-2.*M_PI/3., EKF::defineAngleInBounds((-2./3.)*M_PI));
  EXPECT_DOUBLE_EQ(-2.*M_PI/3., EKF::defineAngleInBounds((4./3.)*M_PI));
  EXPECT_DOUBLE_EQ(2.*M_PI/3., EKF::defineAngleInBounds((-4./3.)*M_PI));
  EXPECT_DOUBLE_EQ(1, EKF::defineAngleInBounds(1));
  EXPECT_DOUBLE_EQ(-1, EKF::defineAngleInBounds(-1));
}


TEST(EKF, AngleToQuaternionZeroDegrees) {
  geometry_msgs::Quaternion quat_angle;
  quat_angle.w = cos((0)/2.);
  quat_angle.x = 0;
  quat_angle.y = 0;
  quat_angle.z = sin((0)/2.);
  EXPECT_DOUBLE_EQ(quat_angle.w, (EKF::angleToQuaternion(0)).w);
  EXPECT_DOUBLE_EQ(quat_angle.x, (EKF::angleToQuaternion(0)).x);
  EXPECT_DOUBLE_EQ(quat_angle.y, (EKF::angleToQuaternion(0)).y);
  EXPECT_DOUBLE_EQ(quat_angle.z, (EKF::angleToQuaternion(0)).z);
  EXPECT_DOUBLE_EQ(quat_angle.w, (EKF::angleToQuaternion(2.*M_PI)).w);
  EXPECT_DOUBLE_EQ(quat_angle.x, (EKF::angleToQuaternion(2.*M_PI)).x);
  EXPECT_DOUBLE_EQ(quat_angle.y, (EKF::angleToQuaternion(2.*M_PI)).y);
  EXPECT_DOUBLE_EQ(quat_angle.z, (EKF::angleToQuaternion(2.*M_PI)).z);
}

TEST(EKF, AngleToQuaternion45Degrees) {
  geometry_msgs::Quaternion quat_angle;
  quat_angle.w = cos((M_PI/4.)/2.);
  quat_angle.x = 0;
  quat_angle.y = 0;
  quat_angle.z = sin((M_PI/4.)/2.);
  EXPECT_DOUBLE_EQ(quat_angle.w, (EKF::angleToQuaternion(M_PI/4.)).w);
  EXPECT_DOUBLE_EQ(quat_angle.x, (EKF::angleToQuaternion(M_PI/4.)).x);
  EXPECT_DOUBLE_EQ(quat_angle.y, (EKF::angleToQuaternion(M_PI/4.)).y);
  EXPECT_DOUBLE_EQ(quat_angle.z, (EKF::angleToQuaternion(M_PI/4.)).z);
  EXPECT_DOUBLE_EQ(quat_angle.w, (EKF::angleToQuaternion(-7.*M_PI/4.)).w);
  EXPECT_DOUBLE_EQ(quat_angle.x, (EKF::angleToQuaternion(-7.*M_PI/4.)).x);
  EXPECT_DOUBLE_EQ(quat_angle.y, (EKF::angleToQuaternion(-7.*M_PI/4.)).y);
  EXPECT_DOUBLE_EQ(quat_angle.z, (EKF::angleToQuaternion(-7.*M_PI/4.)).z);
}

TEST(EKF, AngleToQuaternion90Degrees) {
  geometry_msgs::Quaternion quat_angle;
  quat_angle.w = cos((M_PI/2.)/2.);
  quat_angle.x = 0;
  quat_angle.y = 0;
  quat_angle.z = sin((M_PI/2.)/2.);
  EXPECT_DOUBLE_EQ(quat_angle.w, (EKF::angleToQuaternion(M_PI/2.)).w);
  EXPECT_DOUBLE_EQ(quat_angle.x, (EKF::angleToQuaternion(M_PI/2.)).x);
  EXPECT_DOUBLE_EQ(quat_angle.y, (EKF::angleToQuaternion(M_PI/2.)).y);
  EXPECT_DOUBLE_EQ(quat_angle.z, (EKF::angleToQuaternion(M_PI/2.)).z);
  EXPECT_DOUBLE_EQ(quat_angle.w, (EKF::angleToQuaternion(-3.*M_PI/2.)).w);
  EXPECT_DOUBLE_EQ(quat_angle.x, (EKF::angleToQuaternion(-3.*M_PI/2.)).x);
  EXPECT_DOUBLE_EQ(quat_angle.y, (EKF::angleToQuaternion(-3.*M_PI/2.)).y);
  EXPECT_DOUBLE_EQ(quat_angle.z, (EKF::angleToQuaternion(-3.*M_PI/2.)).z);
}

TEST(EKF, AngleToQuaternion135Degrees) {
  geometry_msgs::Quaternion quat_angle;
  quat_angle.w = cos((3.*M_PI/4.)/2.);
  quat_angle.x = 0;
  quat_angle.y = 0;
  quat_angle.z = sin((3.*M_PI/4.)/2.);
  EXPECT_DOUBLE_EQ(quat_angle.w, (EKF::angleToQuaternion(3.*M_PI/4.)).w);
  EXPECT_DOUBLE_EQ(quat_angle.x, (EKF::angleToQuaternion(3.*M_PI/4.)).x);
  EXPECT_DOUBLE_EQ(quat_angle.y, (EKF::angleToQuaternion(3.*M_PI/4.)).y);
  EXPECT_DOUBLE_EQ(quat_angle.z, (EKF::angleToQuaternion(3.*M_PI/4.)).z);
  EXPECT_DOUBLE_EQ(quat_angle.w, (EKF::angleToQuaternion(-5.*M_PI/4.)).w);
  EXPECT_DOUBLE_EQ(quat_angle.x, (EKF::angleToQuaternion(-5.*M_PI/4.)).x);
  EXPECT_DOUBLE_EQ(quat_angle.y, (EKF::angleToQuaternion(-5.*M_PI/4.)).y);
  EXPECT_DOUBLE_EQ(quat_angle.z, (EKF::angleToQuaternion(-5.*M_PI/4.)).z);
}

TEST(EKF, AngleToQuaternion180Degrees) {
  geometry_msgs::Quaternion quat_angle;
  quat_angle.w = cos((M_PI)/2.);
  quat_angle.x = 0;
  quat_angle.y = 0;
  quat_angle.z = sin((M_PI)/2.);
  EXPECT_DOUBLE_EQ(quat_angle.w, (EKF::angleToQuaternion(M_PI)).w);
  EXPECT_DOUBLE_EQ(quat_angle.x, (EKF::angleToQuaternion(M_PI)).x);
  EXPECT_DOUBLE_EQ(quat_angle.y, (EKF::angleToQuaternion(M_PI)).y);
  EXPECT_DOUBLE_EQ(quat_angle.z, (EKF::angleToQuaternion(M_PI)).z);
  EXPECT_DOUBLE_EQ(quat_angle.w, (EKF::angleToQuaternion(-M_PI)).w);
  EXPECT_DOUBLE_EQ(quat_angle.x, (EKF::angleToQuaternion(-M_PI)).x);
  EXPECT_DOUBLE_EQ(quat_angle.y, (EKF::angleToQuaternion(-M_PI)).y);
  EXPECT_DOUBLE_EQ(quat_angle.z, (EKF::angleToQuaternion(-M_PI)).z);
}

TEST(EKF, AngleToQuaternionNeg135Degrees) {
  geometry_msgs::Quaternion quat_angle;
  quat_angle.w = cos((-3.*M_PI/4.)/2.);
  quat_angle.x = 0;
  quat_angle.y = 0;
  quat_angle.z = sin((-3.*M_PI/4.)/2.);
  EXPECT_DOUBLE_EQ(quat_angle.w, (EKF::angleToQuaternion(-3.*M_PI/4.)).w);
  EXPECT_DOUBLE_EQ(quat_angle.x, (EKF::angleToQuaternion(-3.*M_PI/4.)).x);
  EXPECT_DOUBLE_EQ(quat_angle.y, (EKF::angleToQuaternion(-3.*M_PI/4.)).y);
  EXPECT_DOUBLE_EQ(quat_angle.z, (EKF::angleToQuaternion(-3.*M_PI/4.)).z);
  EXPECT_DOUBLE_EQ(quat_angle.w, (EKF::angleToQuaternion(5.*M_PI/4.)).w);
  EXPECT_DOUBLE_EQ(quat_angle.x, (EKF::angleToQuaternion(5.*M_PI/4.)).x);
  EXPECT_DOUBLE_EQ(quat_angle.y, (EKF::angleToQuaternion(5.*M_PI/4.)).y);
  EXPECT_DOUBLE_EQ(quat_angle.z, (EKF::angleToQuaternion(5.*M_PI/4.)).z);
}

TEST(EKF, AngleToQuaternionNeg90Degrees) {
  geometry_msgs::Quaternion quat_angle;
  quat_angle.w = cos((-M_PI/2.)/2.);
  quat_angle.x = 0;
  quat_angle.y = 0;
  quat_angle.z = sin((-M_PI/2.)/2.);
  EXPECT_DOUBLE_EQ(quat_angle.w, (EKF::angleToQuaternion(-M_PI/2.)).w);
  EXPECT_DOUBLE_EQ(quat_angle.x, (EKF::angleToQuaternion(-M_PI/2.)).x);
  EXPECT_DOUBLE_EQ(quat_angle.y, (EKF::angleToQuaternion(-M_PI/2.)).y);
  EXPECT_DOUBLE_EQ(quat_angle.z, (EKF::angleToQuaternion(-M_PI/2.)).z);
  EXPECT_DOUBLE_EQ(quat_angle.w, (EKF::angleToQuaternion(3.*M_PI/2.)).w);
  EXPECT_DOUBLE_EQ(quat_angle.x, (EKF::angleToQuaternion(3.*M_PI/2.)).x);
  EXPECT_DOUBLE_EQ(quat_angle.y, (EKF::angleToQuaternion(3.*M_PI/2.)).y);
  EXPECT_DOUBLE_EQ(quat_angle.z, (EKF::angleToQuaternion(3.*M_PI/2.)).z);
}

TEST(EKF, AngleToQuaternionNeg45Degrees) {
  geometry_msgs::Quaternion quat_angle;
  quat_angle.w = cos((-M_PI/4.)/2.);
  quat_angle.x = 0;
  quat_angle.y = 0;
  quat_angle.z = sin((-M_PI/4.)/2.);
  EXPECT_DOUBLE_EQ(quat_angle.w, (EKF::angleToQuaternion(-M_PI/4.)).w);
  EXPECT_DOUBLE_EQ(quat_angle.x, (EKF::angleToQuaternion(-M_PI/4.)).x);
  EXPECT_DOUBLE_EQ(quat_angle.y, (EKF::angleToQuaternion(-M_PI/4.)).y);
  EXPECT_DOUBLE_EQ(quat_angle.z, (EKF::angleToQuaternion(-M_PI/4.)).z);
  EXPECT_DOUBLE_EQ(quat_angle.w, (EKF::angleToQuaternion(7.*M_PI/4.)).w);
  EXPECT_DOUBLE_EQ(quat_angle.x, (EKF::angleToQuaternion(7.*M_PI/4.)).x);
  EXPECT_DOUBLE_EQ(quat_angle.y, (EKF::angleToQuaternion(7.*M_PI/4.)).y);
  EXPECT_DOUBLE_EQ(quat_angle.z, (EKF::angleToQuaternion(7.*M_PI/4.)).z);
}


TEST(EKF, QuaternionToAngle0Degrees) {
  geometry_msgs::Quaternion quat_angle;
  quat_angle.w = cos((0)/2.);
  quat_angle.x = 0;
  quat_angle.y = 0;
  quat_angle.z = sin((0)/2.);
  EXPECT_DOUBLE_EQ(0, EKF::quaternionToAngle(quat_angle));
}

TEST(EKF, QuaternionToAngle45Degrees) {
  geometry_msgs::Quaternion quat_angle;
  quat_angle.w = cos((M_PI/4.)/2.);
  quat_angle.x = 0;
  quat_angle.y = 0;
  quat_angle.z = sin((M_PI/4.)/2.);
  EXPECT_DOUBLE_EQ(M_PI/4., EKF::quaternionToAngle(quat_angle));
}

TEST(EKF, QuaternionToAngle90Degrees) {
  geometry_msgs::Quaternion quat_angle;
  quat_angle.w = cos((M_PI/2.)/2.);
  quat_angle.x = 0;
  quat_angle.y = 0;
  quat_angle.z = sin((M_PI/2.)/2.);
  EXPECT_DOUBLE_EQ(M_PI/2., EKF::quaternionToAngle(quat_angle));
}

TEST(EKF, QuaternionToAngle135Degrees) {
  geometry_msgs::Quaternion quat_angle;
  quat_angle.w = cos((3.*M_PI/4.)/2.);
  quat_angle.x = 0;
  quat_angle.y = 0;
  quat_angle.z = sin((3.*M_PI/4.)/2.);
  EXPECT_DOUBLE_EQ(3.*M_PI/4., EKF::quaternionToAngle(quat_angle));
}

TEST(EKF, QuaternionToAngle180Degrees) {
  geometry_msgs::Quaternion quat_angle;
  quat_angle.w = cos((M_PI)/2.);
  quat_angle.x = 0;
  quat_angle.y = 0;
  quat_angle.z = sin((M_PI)/2.);
  EXPECT_DOUBLE_EQ(M_PI, EKF::quaternionToAngle(quat_angle));
}

TEST(EKF, QuaternionToAngleNeg180Degrees) {
  geometry_msgs::Quaternion quat_angle;
  quat_angle.w = cos((-M_PI)/2.);
  quat_angle.x = 0;
  quat_angle.y = 0;
  quat_angle.z = sin((-M_PI)/2.);
  EXPECT_DOUBLE_EQ(M_PI, EKF::quaternionToAngle(quat_angle));
}

TEST(EKF, QuaternionToAngleNeg135Degrees) {
  geometry_msgs::Quaternion quat_angle;
  quat_angle.w = cos((-3.*M_PI/4.)/2.);
  quat_angle.x = 0;
  quat_angle.y = 0;
  quat_angle.z = sin((-3.*M_PI/4.)/2.);
  EXPECT_DOUBLE_EQ(-3.*M_PI/4., EKF::quaternionToAngle(quat_angle));
}

TEST(EKF, QuaternionToAngleNeg90Degrees) {
  geometry_msgs::Quaternion quat_angle;
  quat_angle.w = cos((-M_PI/2.)/2.);
  quat_angle.x = 0;
  quat_angle.y = 0;
  quat_angle.z = sin((-M_PI/2.)/2.);
  EXPECT_DOUBLE_EQ(-M_PI/2., EKF::quaternionToAngle(quat_angle));
}

TEST(EKF, QuaternionToAngleNeg45Degrees) {
  geometry_msgs::Quaternion quat_angle;
  quat_angle.w = cos((-M_PI/4.)/2.);
  quat_angle.x = 0;
  quat_angle.y = 0;
  quat_angle.z = sin((-M_PI/4.)/2.);
  EXPECT_DOUBLE_EQ(-M_PI/4., EKF::quaternionToAngle(quat_angle));
}

TEST(EKF, QuaternionToAngle60Degrees) {
  geometry_msgs::Quaternion quat_angle;
  quat_angle.w = cos((M_PI/3.)/2.);
  quat_angle.x = 0;
  quat_angle.y = 0;
  quat_angle.z = sin((M_PI/3.)/2.);
  EXPECT_DOUBLE_EQ(M_PI/3., EKF::quaternionToAngle(quat_angle));
}

TEST(EKF, QuaternionToAngleNeg60Degrees) {
  geometry_msgs::Quaternion quat_angle;
  quat_angle.w = cos((-M_PI/3.)/2.);
  quat_angle.x = 0;
  quat_angle.y = 0;
  quat_angle.z = sin((-M_PI/3.)/2.);
  EXPECT_DOUBLE_EQ(-M_PI/3., EKF::quaternionToAngle(quat_angle));
}

TEST(EKF, QuaternionToAngle120Degrees) {
  geometry_msgs::Quaternion quat_angle;
  quat_angle.w = cos((2.*M_PI/3.)/2.);
  quat_angle.x = 0;
  quat_angle.y = 0;
  quat_angle.z = sin((2.*M_PI/3.)/2.);
  EXPECT_DOUBLE_EQ(2.*M_PI/3., EKF::quaternionToAngle(quat_angle));
}

TEST(EKF, QuaternionToAngleNeg120Degrees) {
  geometry_msgs::Quaternion quat_angle;
  quat_angle.w = cos((-2.*M_PI/3.)/2.);
  quat_angle.x = 0;
  quat_angle.y = 0;
  quat_angle.z = sin((-2.*M_PI/3.)/2.);
  EXPECT_DOUBLE_EQ(-2.*M_PI/3., EKF::quaternionToAngle(quat_angle));
}

TEST(EKF, QuaternionToAngle240Degrees) {
  geometry_msgs::Quaternion quat_angle;
  quat_angle.w = cos((4.*M_PI/3.)/2.);
  quat_angle.x = 0;
  quat_angle.y = 0;
  quat_angle.z = sin((4.*M_PI/3.)/2.);
  EXPECT_DOUBLE_EQ(-2.*M_PI/3., EKF::quaternionToAngle(quat_angle));
}

TEST(EKF, QuaternionToAngleNeg240Degrees) {
  geometry_msgs::Quaternion quat_angle;
  quat_angle.w = cos((-4.*M_PI/3.)/2.);
  quat_angle.x = 0;
  quat_angle.y = 0;
  quat_angle.z = sin((-4.*M_PI/3.)/2.);
  EXPECT_DOUBLE_EQ(2.*M_PI/3., EKF::quaternionToAngle(quat_angle));
}

TEST(EKF, QuaternionToAngle225Degrees) {
  geometry_msgs::Quaternion quat_angle;
  quat_angle.w = cos((5.*M_PI/4.)/2.);
  quat_angle.x = 0;
  quat_angle.y = 0;
  quat_angle.z = sin((5.*M_PI/4.)/2.);
  EXPECT_DOUBLE_EQ(-3.*M_PI/4., EKF::quaternionToAngle(quat_angle));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}