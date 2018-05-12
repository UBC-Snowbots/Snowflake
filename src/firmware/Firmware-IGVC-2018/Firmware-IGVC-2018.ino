/* Created May 5th by Chris Heathe
 *  
 * Basic arduino motor driver that subsribes to the /cmd_vel topic 
 * and prints corresponding servo values to the left and write motors 
 * of Elsa.
 * 
 * PID loop has now been integrated, Kp, Ki, Kd values will need to be calibrated in physical testing
 * The library that was used can be found at:
 * 
 * https://playground.arduino.cc/Code/PIDLibrary
 * 
 * !!NOTES ON USING THE ros.h ARDUINO LIBRARY!!
 * 
 * Install the 'ros-kinetic-rosserial-server' package
 * 
 * Once installed it is necessary to run the following command in terminal
 * to establish a serial connection and ROS:
 * 
 * rosrun rosserial_python serial_node.py /dev/ttyACM0
 * 
 */


#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include<Servo.h>
#include<PID_v1.h>


Servo leftMotor;
Servo rightMotor;
const int leftMotorPin = 10;
const int rightMotorPin = 11;


// Will be used to initialized the motors so that fluctuations in printing will not create jerky movements
int minPWM = 1000;
int maxPWM = 2000;


// Maximum and minimum values we can write to the motor
int maxMotorWrite = 180;
int minMotorWrite = 0;

// The numbers we want to print to the motor ranging from 0 to 180 with 90 as the centerpoint (not moving)
double setLinear = 90;
double setAngular = 90;

// The readings we receive from the encoders ranging from 0 to 180 with 90 as the centerpoint (not moving)
double inputLinear = 90;
double inputAngular = 90;

// The value computed by the PID function from the set and input velocities ranging from 0 to 180 with 90 as the centerpoint (not moving)
double outputLinear = 90;
double outputAngular = 90;

// The numbers we will print to the motor based on converting PID outputs to left and right motor values
int setLeftMotor = 90;
int setRightMotor = 90;


// Setpoint for PID loop
double Setpoint = 100;

//Tuning parameters for the PID controllers:
int linearKp = 2;
int linearKi = 5;
int linearKd = 1;
int angularKp = 2;
int angularKi = 5;
int angularKd = 1;

// Specify links for PID controller
PID linearPID(&inputLinear, &outputLinear, &Setpoint,linearKp,linearKi,linearKd, DIRECT);
PID angularPID(&inputAngular, &outputAngular, &Setpoint,angularKp,angularKi,angularKd, DIRECT);


ros::NodeHandle nh;

// Translates information from the pathfinding node to the corresponding value to print to the left and right motors
// Assuming that counterclockwise turning is a positive angular z velocity
void TwistCb( const geometry_msgs::Twist& twist_msg){

map(twist_msg.linear.x, 0, 255, 0, 180);
map(twist_msg.angular.z, 0, 255, 0, 180);

setLinear = twist_msg.linear.x;
setAngular = twist_msg.angular.z;

if (setLinear < minMotorWrite) { setLinear = minMotorWrite;}
if (setLinear > maxMotorWrite) { setLinear = maxMotorWrite;}
if (setAngular < minMotorWrite) { setAngular = minMotorWrite;}
if (setAngular > maxMotorWrite) { setAngular = maxMotorWrite;}

}

void OdomCb( const nav_msgs::Odometry& odom_msg) {

map(odom_msg.twist.twist.linear.x, 0, 255, 0, 180);
map(odom_msg.twist.twist.angular.z, 0, 255, 0, 180);

inputLinear = odom_msg.twist.twist.linear.x;
inputAngular = odom_msg.twist.twist.angular.z;

if (inputLinear < minMotorWrite) { inputLinear = minMotorWrite;}
if (inputLinear > maxMotorWrite) { inputLinear = maxMotorWrite;}
if (inputAngular < minMotorWrite) { inputAngular = minMotorWrite;}
if (inputAngular > maxMotorWrite) { inputAngular = maxMotorWrite;}

}


ros::Subscriber<geometry_msgs::Twist> twist("/cmd_vel", &TwistCb );

ros::Subscriber<nav_msgs::Odometry> odom("/encoder/odom", &OdomCb);


void setup() {
pinMode(leftMotorPin, OUTPUT);
pinMode(rightMotorPin, OUTPUT);
leftMotor.attach(leftMotorPin, minPWM, maxPWM);
rightMotor.attach(rightMotorPin, minPWM, maxPWM);


linearPID.SetMode(AUTOMATIC);
angularPID.SetMode(AUTOMATIC);



nh.initNode();
nh.subscribe(twist);
nh.subscribe(odom);
}

void loop() {
nh.spinOnce();

linearPID.Compute();
angularPID.Compute();

if (outputLinear < minMotorWrite) { outputLinear = minMotorWrite;}
if (outputLinear > maxMotorWrite) { outputLinear = maxMotorWrite;}
if (outputAngular < minMotorWrite) { outputAngular = minMotorWrite;}
if (outputAngular > maxMotorWrite) { outputAngular = maxMotorWrite;}

setLeftMotor = ((outputLinear - outputAngular - 90)/2) + 90;
setRightMotor = ((outputLinear + outputAngular - 90)/2) + 90;

leftMotor.write(setLeftMotor);
rightMotor.write(setRightMotor);

}


