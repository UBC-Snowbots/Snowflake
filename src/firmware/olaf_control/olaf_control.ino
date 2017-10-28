/* Firmware for IARRC 2017
 * Author: Vincent Yuan, 
 * Modified: Nick Wu
 * Modified: James Asefa
 * Modified: Jinhao Lu
 * Modified: Gareth Ellis
 * Modified: Valerian Ratu
 * Date Last Modified: October 22, 2017
 */

/*
 * UBC Snowbots - IARRC 2017
 * Firmware for Control of an RC car
 * 
 * This firmware will take in a message of the form:
 * `B<linear x><linear y><linear z><angular x><angular y><angular z>`
 * where each `<>` is a single byte. The degree to rotate the servo controlling
 * the front wheels is determined from `angular z`, and the speed of the car by `linear x`
 * (the rest of the values are discarded)
 */

#include <SoftwareSerial.h>
#include <stdlib.h>
#include <Servo.h>

// !!!!WARNING!!!! 
// DEBUGGING might affect performance and alter normal behaviour
// !!!!WARNING!!!! 
// Uncomment this to enable debug message logging - messages sent over serial will be echoed back
//#define DEBUG_MESSAGES
// Uncomment this to enable PWM debug logging - PWM signals will be echoed
//#define DEBUG_PWM

#define BAUD_RATE 9600

// size of character buffer being passed over serial connection
#define BUFFER_SIZE 7

// Robot speed will be received in a char buffer with each value between 0 and 255
// as a result, this assigns 128 as stop. Then:
// 0 < speed < 128 is reverse
// 128 < speed <= 255 is forward
// 255 is then full speed forward, 0 is full speed backward
#define UNMAPPED_STOP_SPEED 128
#define MAPPED_STRAIGHT_ANGLE 90

// Robot will keep executing the last command unless the period exceeds the SAFETY_TIMEOUT
// SAFETY_TIMEOUT is in ms
#define SAFETY_TIMEOUT 500

void serial_read();
void convert();
void drive(int, int);

// buffer inputs
int linear_x = UNMAPPED_STOP_SPEED;
int linear_y = UNMAPPED_STOP_SPEED;
int linear_z = UNMAPPED_STOP_SPEED;
int angular_x = MAPPED_STRAIGHT_ANGLE;
int angular_y = MAPPED_STRAIGHT_ANGLE;
int angular_z = MAPPED_STRAIGHT_ANGLE;

// motor pins
const int MOTOR_PIN = A0;
const int DIRECTION_PIN = A2;

// defines start of buffer
const char BUFFER_HEAD = 'B';

// max and min linear speeds and stopping condition
const int LINEAR_MAX = 256;
const int LINEAR_MIN = 0;
const int LINEAR_STOP = 128;

// max and min angular speeds and stopping condition
const int ANGULAR_MAX = 180;
const int ANGULAR_MIN = 0;
const int ANGULAR_STOP = 90;

// The minimum and maximum PWM signals to map received values to
// This is the default for both the Servo and the Talon ESC
// 1000us - 2000us
const int MIN_PWM = 1000;
const int MAX_PWM = 2000;

// The safety cutoffs to prevent us drawing too much current
// from the motor
const int MIN_MOTOR_PWM_CUTOFF = 1300;
const int MAX_MOTOR_PWM_CUTOFF = 1700;

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

Servo motor;
Servo direction_motor;

void setup() {
    Serial.begin(BAUD_RATE);

    // Initialiase the Servo and Motor connections
    motor.attach(MOTOR_PIN, MIN_PWM, MAX_PWM);
    direction_motor.attach(DIRECTION_PIN, MIN_PWM, MAX_PWM);
}

void loop() {
    serial_read(); 
    convert();
    drive(linear_x, angular_z);
}

void serial_read(){
    // BUFFER_HEAD identifies the start of the buffer
    if ( Serial.available() >= BUFFER_SIZE && Serial.read() == BUFFER_HEAD ) {
        linear_x = Serial.read();
        linear_y = Serial.read();
        linear_z = Serial.read();
        angular_x = Serial.read();
        angular_y = Serial.read();
        angular_z = Serial.read();
        previousMillis = currentMillis;

#ifdef DEBUG_MESSAGES
        Serial.println("Got message");
        Serial.print("Linear X:"); Serial.println(linear_x);
        Serial.print("Linear Y:"); Serial.println(linear_y);
        Serial.print("Linear Z:"); Serial.println(linear_z);
        Serial.print("Angular X:"); Serial.println(angular_x);
        Serial.print("Angular Y:"); Serial.println(angular_y);
        Serial.print("Angular Z:"); Serial.println(angular_z);
#endif
    } else {
        currentMillis = millis();
        if(currentMillis - previousMillis > SAFETY_TIMEOUT){
            linear_x = UNMAPPED_STOP_SPEED;
        }
    }
}

void convert() {
    // safety check if values are out of range
    if (linear_x > LINEAR_MAX)
        linear_x = LINEAR_MAX; 
    else if (linear_x < LINEAR_MIN)
        linear_x = LINEAR_MIN;

    if (angular_z > ANGULAR_MAX)
        angular_z = ANGULAR_MAX; 
    else if (angular_z < ANGULAR_MIN)
        angular_z = ANGULAR_MIN;
}

void drive(int linear_speed, int angular_speed){
    // Default velocity and angle to the midpoint in the PWM range
    int velocity = (MAX_PWM - MIN_PWM)/2;
    int angle = (MAX_PWM - MIN_PWM)/2;

    velocity = map(linear_speed, LINEAR_MIN, LINEAR_MAX, MIN_PWM, MAX_PWM); 
    angle = map(angular_speed, ANGULAR_MIN, ANGULAR_MAX, MIN_PWM, MAX_PWM);

    // Check that we're not outside our safety cutoffs
    if (velocity > MAX_MOTOR_PWM_CUTOFF)
        velocity = MAX_MOTOR_PWM_CUTOFF;
    else if (velocity < MIN_MOTOR_PWM_CUTOFF)
        velocity = MIN_MOTOR_PWM_CUTOFF;

    // Write the commands to the motor and the servo
    motor.writeMicroseconds(velocity);
    direction_motor.writeMicroseconds(angle);

#ifdef DEBUG_PWM
    Serial.print(velocity);Serial.print(" / ");
    Serial.print(angle);Serial.print(" / ");
    Serial.print(linear_speed);Serial.print(" / ");
    Serial.print(angular_speed);Serial.print(" / ");
    Serial.println();
#endif
}

