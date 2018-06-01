/* Created May 5th by Chris Heathe
 *  
 * Basic arduino motor driver that subsribes to the /cmd_vel topic 
 * and prints corresponding servo values to the left and write motors 
 * of Elsa.
 * 
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
#include<Servo.h>


Servo leftMotor;
Servo rightMotor;
const int leftMotorPin = 10;
const int rightMotorPin = 11;

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
double setAngular = 0;

// The numbers we will print to the motor based on converting PID outputs to left and right motor values
int setLeftMotor = 90;
int setRightMotor = 90;

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



std_msgs::String str_msg;
ros::Publisher firmwareTestLog("firmwareTestLog", &str_msg);

ros::Subscriber<geometry_msgs::Twist> twist("/cmd_vel", &twistCallback );



void setup() {
    pinMode(leftMotorPin, OUTPUT);
    pinMode(rightMotorPin, OUTPUT);
    leftMotor.attach(leftMotorPin, minPWM, maxPWM);
    rightMotor.attach(rightMotorPin, minPWM, maxPWM);
    
    pinMode(lightRelayPin, OUTPUT);
    digitalWrite(lightRelayPin, LOW);

    nh.initNode();
    nh.subscribe(twist);
    nh.advertise(firmwareTestLog);
    
}

void loop() {
    
    nh.spinOnce();

    currentMillisMotorTimeout = millis() - previousMillisMotorTimeout;
    currentMillisLightBlinker = millis() - previousMillisLightBlinker;

    if(currentMillisMotorTimeout < 1000)  {

      setLeftMotor = ((setLinear - setAngular - 90)/2) + 90;
      setRightMotor = ((setLinear + setAngular - 90)/2) + 90;

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


    debug(1);

    
}


/*
 * Publishes debugging messages to the firmwareTestLog topic
 * 
 * Publishes a different debugging messages to the firmwareTestLog topic 
 * depending on what debugNumber is passed to the function
 * 
 * @param debugNumber tells the debug function which message to print:
 *        1: outputLinear, outputAngular, setLeftMotor, setRightMotor
 */
void debug(int debugNumber) {

  switch(debugNumber) {
    case 1:
    debugString1 = "set linear: " + String(setLinear) + " set angular: " + String(setAngular) + " set left motor: " + String(setLeftMotor) + " set right motor: " + String(setRightMotor);
    debugString1.toCharArray(debugMessage1, 100);
    str_msg.data = debugMessage1;
    firmwareTestLog.publish( &str_msg );
    break;
    
  }
  
}

