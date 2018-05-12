/* Created May 5th by Chris Heathe
 *  
 * Basic arduino motor driver that subsribes to the /cmd_vel topic 
 * and prints corresponding servo values to the left and write motors 
 * of Elsa.
 * 
 * Currently there is no PID loop however I am investigating the following PID
 * library that was written for arduino:
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
int setLeftMotor = 90;
int setRightMotor = 90;

ros::NodeHandle nh;

// Translates information from the pathfinding node to the corresponding value to print to the left and right motors
// Assuming that counterclockwise turning is a positive angular z velocity
void TwistCb( const geometry_msgs::Twist& twist_msg){

map(twist_msg.linear.x, 0, 255, 0, 180);
map(twist_msg.angular.z, 0, 255, 0, 180);

setLeftMotor = ((twist_msg.linear.x - twist_msg.angular.z - 90)/2) + 90;
setRightMotor = ((twist_msg.linear.x + twist_msg.angular.z - 90)/2) + 90;

if (setLeftMotor < minMotorWrite) { setLeftMotor = minMotorWrite;}
if (setLeftMotor > maxMotorWrite) { setLeftMotor = maxMotorWrite;}
if (setRightMotor < minMotorWrite) { setRightMotor = minMotorWrite;}
if (setRightMotor > maxMotorWrite) { setRightMotor = maxMotorWrite;}


}


ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &TwistCb );



void setup() {
pinMode(leftMotorPin, OUTPUT);
pinMode(rightMotorPin, OUTPUT);
leftMotor.attach(leftMotorPin, minPWM, maxPWM);
rightMotor.attach(rightMotorPin, minPWM, maxPWM);

nh.initNode();
nh.subscribe(sub);
}

void loop() {
nh.spinOnce();

leftMotor.write(setLeftMotor);
rightMotor.write(setRightMotor);

}
