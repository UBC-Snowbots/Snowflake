/* Firmware for Elsa
   Test version
   Author: Vincent Yuan, Modified: Nick Wu
     Modified: James Asefa
   Date: Oct 30, 2016
*/

#include <SoftwareSerial.h>
#include <stdlib.h>
#include <Servo.h>


#define TRIM 8 // error margin for joysticks

#define BAUD_RATE 115200
#define BUFFER_SIZE 6
// assign 128 to be not moving
// will be used to hold robot in place while
// reading from the buffer. Robot will be stopped initially.
#define UNMAPPED_STOP_SPEED 128

// DRIVING MODIFICATIONS
// modify FR, FL, BR, BL to change speeds: (+) correpsonds to faster speed, (-) for going reverse
// F:FRONT, B:BACK, R:RIGHT, L:LEFT e.g. FR = FRONT RIGHT WHEEL.
const int OFFSET = 1;
const int joystick_margin = 150;

// buffer inputs
int linear_x = 0;
int linear_y = 0;
int linear_z = 0;
int angular_x = 0;
int angular_y = 0;
int angular_z = 0;

// motor pins
const int left_motor_pin = 9;
const int right_motor_pin = 10;
// defines start of buffer
const char buffer_head = 'B';


// max and min linear speeds and stopping condition
const int linear_max = 100;
const int linear_min = 80;
const int linear_stop = 90;

// max and min angular speeds and stopping condition
const int angular_max = 100;
const int angular_min = 80;
const int angular_stop = 90;

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
  
  LeftM.attach(left_motor_pin);
  RightM.attach(right_motor_pin);
  
  // calculate offset
  set_off();
}

void loop() {

  //if (Serial.read() == 'I') {Serial.print("DRIVE");}
  //else Serial.flushRX();
  //responding to queries from usb
  sig_read();
  
  
  //serial_read();
  if (Mode == -1) { //Auto Mode
    serial_read(); 
    
    convert();
    drive(linear_x, angular_z);
    //Serial.print("angularZLow: ");Serial.print(angularZLow);Serial.print("linear_x: ");Serial.print(linear_x);Serial.print("angular_z: ");Serial.println(angular_z);
  }
  else if (Mode == 0) { //RC Mode
    //Serial.println(angular_z);
    linear_x = range1; 
    angular_z = range2;
    
    convert();
    drive(linear_x, angular_z);
    //Serial.print("angularZLow: ");Serial.print(angularZLow);Serial.print("linear_x: ");Serial.print(linear_x);Serial.print("angular_z: ");Serial.println(angular_z);
    Serial.flushRX();
  }
  else { //STOP MODE
    linear_x = linear_stop; 
    angular_z = angular_stop;
    
    convert();
    drive(linear_x, angular_z);
    // Serial.print("angularZLow: ");Serial.print(angularZLow);Serial.print("linear_x: ");Serial.print(linear_x);Serial.print("angular_z: ");Serial.println(angular_z);
    Serial.flushRX();
  }
  //Serial.print("linear_x: ");Serial.print(linear_x);Serial.print(" linear_x: ");Serial.print(linear_x);Serial.print(" angular_z: ");Serial.println(angular_z);

}

/*
* 
*/
void set_off() {
  int linear_x_mid = 1550; //RX standard - radio signal midpoint
  int angular_z_mid = 1550; //RY standard - radio signal midpoint
  
  // joystick_margin is an error margin for joystick control
  // e.g. if the joystick is moved just a little bit, it is assumed that no movement 
  // is desired.
  linearXHigh = linear_x_mid + joystick_margin; 
  linearXLow = linear_x_mid - joystick_margin;
  
  angularZHigh = angular_z_mid + joystick_margin; 
  angularZLow = angular_z_mid - joystick_margin;
}

/*
* Read in from RC controller. Mode and speed are determined here.
*/
void sig_read() {
  range1 = pulseIn(2, HIGH); // 1140 - 1965 RX LEFT-RIGHT -> turn on spot
  range2 = pulseIn(3, HIGH); // 1965 - 1140 RY UP-DOWN -> Steer left/right y axis (turn while moving)
  range3 = pulseIn(4, HIGH); // 1970 - 1115 linear_x UP-DOWN -> forward/backward
  range4 = pulseIn(5, HIGH); //1970 - 1115 mode
  
  //Serial.print("range1: "); Serial.print(range1);Serial.print(" range2: "); Serial.print(range2);Serial.print(" range3: "); Serial.println(range3);Serial.println("Mode: ");Serial.println(Mode);
  if (range1 < linearXHigh && range1 > linearXLow) 
    range1 = linear_stop;
  else 
    range1 = map (range1, 1140, 1965, 0, 255);

  if (range2 < angularZHigh && range2 > angularZLow) 
    range2 = angular_stop;
  else 
    range2 = map (range2, 1140, 1965, 0, 255);
  
  //Serial.print("range3: ");Serial.println(range3);
  if (1100 < range3 && range3 < 1400) 
    Mode = 1; // STOP mode?
  else if (1400 < range3 && range3 < 1700) 
    Mode = 0; // auto mode
  else if (1700 < range3 && range3 < 2000) 
    Mode = -1; // RC mode
  else 
    Mode = -2; // STOP mode
  if (abs(range1 - 90) < TRIM) // error control
    range1 = linear_stop;
  if (abs(range2 - 90) < TRIM) 
    range2 = angular_stop;
}

void serial_read(){
    //reading in 6 chars from Serial
  if (Serial.available() > BUFFER_SIZE) {

      // buffer_head identifies the start of the buffer
      if (Serial.read() == buffer_head) {
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
    
  //Serial.end();
  //Serial.begin(9600);
  
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
  linear_x = map(linear_x, 0, 255, linear_min, linear_max);
  angular_z = map(angular_z, 0, 255, angular_min, angular_max);
}


/*
 * moves the robot - turning is taken into account.
 */
void drive(int angular_speed, int linear_speed){
  
  int left_throttle = linear_speed + (angular_speed - angular_stop);
  int right_throttle = linear_speed - (angular_speed - angular_stop);
  
  servo_write(LeftM, left_throttle);
  servo_write(RightM, right_throttle);
}

// this writes to motor using duty cycle rather than servo arm angle
void servo_write(Servo motor, int throttle) {
  // we are using the Talon SRX - looks like duty cycle is between 1 - 12ms
  // seen here https://www.ctr-electronics.com/Talon%20SRX%20User's%20Guide.pdf
  // throttle can be as high as 110 and as low as 70 after calculation
  // PWM input pulse high time can be between 1 and 2 ms. So 1000-2000 microseconds
  
  throttle = map(throttle, 70, 110, 1000, 2000); 
  motor.writeMicroseconds(throttle);
}
