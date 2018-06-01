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
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include<Servo.h>
#include<PID_v1.h>


Servo leftMotor;
Servo rightMotor;
const int leftMotorPin = 10;
const int rightMotorPin = 11;
const int lightRelayPin = A0;

// timeout shut off variables
int timeoutLength = 1000;
unsigned long currentMillisMotorTimeout = 0;
unsigned long previousMillisMotorTimeout = 0;

// light blinking delay
unsigned long currentMillisLightBlinker = 0;
unsigned long previousMillisLightBlinker = 0;
boolean autonomousMode = false;

//debug variables
String debugString1 = "";
char debugMessage1[100];
String debugString2 = "";
char debugMessage2[100];

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

//Tuning parameters for the PID controllers:
float linearKp = 2;
float linearKi = 5;
float linearKd = 1;
float angularKp = 2;
float angularKi = 5;
float angularKd = 1;




// Specify links for PID controller
PID linearPID(&inputLinear, &outputLinear, &setLinear,linearKp,linearKi,linearKd, DIRECT);
PID angularPID(&inputAngular, &outputAngular, &setAngular,angularKp,angularKi,angularKd, DIRECT);


ros::NodeHandle nh;


/*
 * Translates information from the pathfinding node to the corresponding value to print to the left and right motors
 * 
 * This function maps the twist message from a range of (0, 255) to (0, 180) to be compatible with the Servo.h library
 * It then takes the linear x and angular z velocities and constrains them to the range in case there are any values
 * that are too large to be printed to the servo motors
 * 
 * @param twist_msg the twist message from the pathfinding node
 */
void twistCallback( const geometry_msgs::Twist& twist_msg){

    previousMillisMotorTimeout = millis();

    setLinear = map(twist_msg.linear.x*100, -100, 100, 0, 180);
    setAngular = map(twist_msg.angular.z*100, -314, 314, 0, 180) - 90;

    setLinear = constrain(setLinear, minMotorWrite, maxMotorWrite);
    setAngular = constrain(setAngular, minMotorWrite, maxMotorWrite);

    autonomousMode = true;
    
}


/*
 * Translates an odometry message from the encoders to monitor realized velocities that can be compared to set velocities by the PID loop
 * 
 * This function maps the odometry message from a range of (0, 255) to (0, 180) to be compatible with the Servo.h library
 * It then takes the linear x and angular z velocities from the odometry message and constrains them to the range in case 
 * there are any values that are too large to be printed to the servo motors
 * 
 * @param odom_msg the odometry message from the encoders
 */
void odomCallback( const nav_msgs::Odometry& odom_msg) {

    map(odom_msg.twist.twist.linear.x, 0, 255, 0, 180);
    map(odom_msg.twist.twist.angular.z, 0, 255, 0, 180);

    inputLinear = odom_msg.twist.twist.linear.x;
    inputAngular = odom_msg.twist.twist.angular.z;

    inputLinear = constrain(inputLinear, minMotorWrite, maxMotorWrite);
    inputAngular = constrain(inputAngular, minMotorWrite, maxMotorWrite);

}

std_msgs::String str_msg;
ros::Publisher firmwareTestLog("firmwareTestLog", &str_msg);

ros::Subscriber<geometry_msgs::Twist> twist("/cmd_vel", &twistCallback );

ros::Subscriber<nav_msgs::Odometry> odom("/encoder/odom", &odomCallback);


void setup() {
    pinMode(leftMotorPin, OUTPUT);
    pinMode(rightMotorPin, OUTPUT);
    leftMotor.attach(leftMotorPin, minPWM, maxPWM);
    rightMotor.attach(rightMotorPin, minPWM, maxPWM);

    linearPID.SetMode(AUTOMATIC);
    angularPID.SetMode(AUTOMATIC);
    
    pinMode(lightRelayPin, OUTPUT);
    digitalWrite(lightRelayPin, LOW);

    nh.initNode();
    nh.subscribe(twist);
    nh.subscribe(odom);
    nh.advertise(firmwareTestLog);

    while(!nh.connected()) {nh.spinOnce();}
    nh.getParam("~linear_Kp", &linearKp, 1);
    nh.getParam("~linear_Ki", &linearKi, 1);
    nh.getParam("~linear_Kd", &linearKd, 1);
    nh.getParam("~angular_Kp", &angularKp, 1);
    nh.getParam("~angular_Kp", &angularKi, 1);
    nh.getParam("~angular_Kp", &angularKd, 1);
    
    
}

void loop() {
    
    nh.spinOnce();

    currentMillisMotorTimeout = millis() - previousMillisMotorTimeout;
    currentMillisLightBlinker = millis() - previousMillisLightBlinker;
    
    if(currentMillisMotorTimeout < 1000)  {
      linearPID.Compute();
      angularPID.Compute();

      outputLinear = constrain(outputLinear, minMotorWrite, maxMotorWrite);
      outputAngular = constrain(outputAngular, minMotorWrite, maxMotorWrite);

      setLeftMotor = ((outputLinear - outputAngular - 90)/2) + 90;
      setRightMotor = ((outputLinear + outputAngular - 90)/2) + 90;

      leftMotor.write(setLeftMotor);
      rightMotor.write(setRightMotor);
    }

    else {
      autonomousMode = false;
      digitalWrite(lightRelayPin, LOW);
      setLeftMotor = 90;
      setRightMotor = 90;
      leftMotor.write(setLeftMotor);
      rightMotor.write(setRightMotor);
    }

    if(currentMillisLightBlinker > 500 && autonomousMode == true) {
      
      digitalWrite(lightRelayPin, (1 - digitalRead(lightRelayPin)));
      previousMillisLightBlinker = millis();
      
    }

    debug(2);

    
}


/*
 * Publishes debugging messages to the firmwareTestLog topic
 * 
 * Publishes a different debugging messages to the firmwareTestLog topic 
 * depending on what debugNumber is passed to the function
 * 
 * @param debugNumber tells the debug function which message to print:
 *        1: outputLinear, outputAngular, setLeftMotor, setRightMotor
 *        2: linear and angular PID loop K values
 */
void debug(int debugNumber) {

  switch(debugNumber) {
    case 1:
    debugString1 = "output linear: " + String(outputLinear) + " output angular: " + String(outputAngular) + " set left motor: " + String(setLeftMotor) + " set right motor: " + String(setRightMotor);
    debugString1.toCharArray(debugMessage1, 100);
    str_msg.data = debugMessage1;
    firmwareTestLog.publish( &str_msg );
    break;

    case 2:
    debugString2 = "linear kp: " + String(linearKp) + " ki: " + String(linearKi) + " kd: " + String(linearKd) + " angular kp: " + String(angularKp) + " ki: " + String(angularKi) + " kd: " + String(angularKd);
    debugString2.toCharArray(debugMessage2, 100);
    str_msg.data = debugMessage2;
    firmwareTestLog.publish( &str_msg );
    break;
    
  }
  
}
