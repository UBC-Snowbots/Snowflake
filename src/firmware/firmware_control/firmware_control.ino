/* Firmware for Elsa
   Author: Vincent Yuan, 
   Modified: Nick Wu
   Modified: James Asefa
   Date Last Modified: Oct 30, 2016
*/

#include <SoftwareSerial.h>
#include <stdlib.h>
#include <Servo.h>


#define TRIM 8 // error margin for joysticks

#define BAUD_RATE 115200

// size of character buffer being passed over serial connection
#define BUFFER_SIZE 6

// Robot speed will be received in a char buffer with each value between 0 and 255
// as a result, this assigns 128 as stop. Then:
// 0 < speed < 128 is reverse
// 128 < speed <= 255 is forward
// 255 is then full speed forward, 0 is full speed backward
#define UNMAPPED_STOP_SPEED 128

// DRIVING MODIFICATIONS
// modify FR, FL, BR, BL to change speeds: (+) correpsonds to faster speed, (-) for going reverse
// F:FRONT, B:BACK, R:RIGHT, L:LEFT e.g. FR = FRONT RIGHT WHEEL.
const int OFFSET = 1;
const int JOYSTICK_MARGIN = 150;

// buffer inputs
int linear_x = 0;
int linear_y = 0;
int linear_z = 0;
int angular_x = 0;
int angular_y = 0;
int angular_z = 0;

// motor pins
const int LEFT_MOTOR_PIN = 9;
const int RIGHT_MOTOR_PIN = 10;
// defines start of buffer
const char BUFFER_HEAD = 'B';


// max and min linear speeds and stopping condition
const int LINEAR_MAX = 100;
const int LINEAR_MIN = 80;
const int LINEAR_STOP = 90;

// max and min angular speeds and stopping condition
const int ANGULAR_MAX = 100;
const int ANGULAR_MIN = 80;
const int ANGULAR_STOP = 90;

Servo LeftM;
Servo RightM;


// these variables will store the joystick ranges - used to figure out direction, turn etc
int range1 = 0, range2 = 0, range3 = 0, range4 = 0, B2 = 0, B4 = 0, Mode = 0;

int linearXHigh = 0, linearXLow = 0, angularZHigh = 0, angularZLow = 0;

void setup() {
  Serial.begin(BAUD_RATE);
  
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  
  LeftM.attach(LEFT_MOTOR_PIN);
  RightM.attach(RIGHT_MOTOR_PIN);
  
  // calculate OFFSET
  set_offset();
}

void loop() {

  // read RC input
  rc_read();
  
  
  if (Mode == -1) { //Auto Mode
    serial_read(); 
    
    convert();
    drive(linear_x, angular_z);
  }
  else if (Mode == 0) { //RC Mode
    linear_x = range1; 
    angular_z = range2;
    
    convert();
    drive(linear_x, angular_z);
    Serial.flushRX();
  }
  else { // STOP MODE
    linear_x = LINEAR_STOP; 
    angular_z = ANGULAR_STOP;
    
    convert();
    drive(linear_x, angular_z);
    Serial.flushRX();
  }
}

/*
* Calculates OFFSET for the joystick controllers. 
*/
void set_offset() {
  int linear_x_mid = 1550; //RX standard - radio signal midpoint
  int angular_z_mid = 1550; //RY standard - radio signal midpoint
  
  // JOYSTICK_MARGIN is an error margin for joystick control
  // e.g. if the joystick is moved just a little bit, it is assumed that no movement 
  // is desired.
  linearXHigh = linear_x_mid + JOYSTICK_MARGIN; 
  linearXLow = linear_x_mid - JOYSTICK_MARGIN;
  
  angularZHigh = angular_z_mid + JOYSTICK_MARGIN; 
  angularZLow = angular_z_mid - JOYSTICK_MARGIN;
}

/*
* Read in from RC controller. Mode and speed are determined here.
*/
void rc_read() {
  range1 = pulseIn(2, HIGH); // 1140 - 1965 RX LEFT-RIGHT -> turn on spot
  range2 = pulseIn(3, HIGH); // 1965 - 1140 RY UP-DOWN -> Steer left/right y axis (turn while moving)
  range3 = pulseIn(4, HIGH); // 1970 - 1115 linear_x UP-DOWN -> forward/backward
  range4 = pulseIn(5, HIGH); // 1970 - 1115 mode
  
  if (range1 < linearXHigh && range1 > linearXLow) 
    range1 = LINEAR_STOP;
  else 
    range1 = map (range1, 1140, 1965, 0, 255);

  if (range2 < angularZHigh && range2 > angularZLow) 
    range2 = ANGULAR_STOP;
  else 
    range2 = map (range2, 1140, 1965, 0, 255);
  
  if (1100 < range3 && range3 < 1400) 
    Mode = 1; // STOP mode
  else if (1400 < range3 && range3 < 1700) 
    Mode = 0; // auto mode
  else if (1700 < range3 && range3 < 2000) 
    Mode = -1; // RC mode
  else 
    Mode = -2; // STOP mode
    
  // if joystick movement is not outside of the error range, assume no movement is desired
  if (abs(range1 - 90) < TRIM)
    range1 = LINEAR_STOP;
  if (abs(range2 - 90) < TRIM) 
    range2 = ANGULAR_STOP;
}

void serial_read(){
    //reading in 6 chars from Serial
  if (Serial.available() > BUFFER_SIZE) {

      // BUFFER_HEAD identifies the start of the buffer
      if (Serial.read() == BUFFER_HEAD) {
          linear_x = Serial.read();
          linear_y = Serial.read();
          linear_z = Serial.read();
          angular_x = Serial.read();
          angular_y = Serial.read();
          angular_z = Serial.read();
      } else {
          linear_x = angular_z = UNMAPPED_STOP_SPEED;
      }

  } else {
      linear_x = angular_z = UNMAPPED_STOP_SPEED;
    }
  
  //flushRX defined here: https://forum.sparkfun.com/viewtopic.php?f=32&t=32715 
  Serial.flushRX();
}

void convert() {
  if (linear_x > 255)
    linear_x = 255; 
  else if (linear_x < 0)
    linear_x = 0;
  
  if (angular_z > 255)
    angular_z = 255; 
  else if (angular_z < 0)
    angular_z = 0;
  
  // map to our pre-defined max and mins
  linear_x = map(linear_x, 0, 255, LINEAR_MIN, LINEAR_MAX);
  angular_z = map(angular_z, 0, 255, ANGULAR_MIN, ANGULAR_MAX);
}


// moves the robot. Turning is taken into account
void drive(int angular_speed, int linear_speed){
  
  // the convention being used is that, after mapping serial/RC inputs
  // to the range between linear_min - linear_max or angular_min - angular_max, we will have:
  // 
  // an angular_speed value between 70 - 90 will be "turn left"
  // an angular_speed value between 90 - 110 will be "turn right"
  // 
  // a linear_speed value between 70 - 90 will be "go backward"
  // a linear_speed value between 90 - 110 will be "go forward"
  // 
  // if angular speed == ANGULAR_STOP, this resolves to only move in the linear direction.
  // if linear_speed == LINEAR_STOP, this resolves to turning on the spot, using the above definitions for
  // angular turning
  int left_throttle = linear_speed + (angular_speed - ANGULAR_STOP);
  int right_throttle = linear_speed - (angular_speed - ANGULAR_STOP);
  
  servo_write(LeftM, left_throttle);
  servo_write(RightM, right_throttle);
}

// this writes to motor using the duty cycle of our Talon motors
void servo_write(Servo motor, int throttle) {
  // we are using the Talon SRX - looks like duty cycle is between 1 - 2ms
  // seen here https://www.ctr-electronics.com/Talon%20SRX%20User's%20Guide.pdf
  // throttle can be as high as 110 and as low as 70 after calculation
  // PWM input pulse high time can be between 1 and 2 ms. So 1000-2000 microseconds
  
  // note if using the Servo library to do PWM produces issues
  // alternate implementations can be found here: http://www.circuitstoday.com/pwm-generation-and-control-using-arduino 
  throttle = map(throttle, 70, 110, 1000, 2000); 
  motor.writeMicroseconds(throttle);
}
