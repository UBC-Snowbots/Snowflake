#include <SoftwareSerial.h>
#include <stdlib.h>
#include <Servo.h>

#define BAUD_RATE 115200

// we will be reading 6 chars
// [linear_x,linear_y, linear_z, angular_x, angular_y, angular_z]

#define BUFFER_SIZE 6

// assign 128 to be not moving
// will be used to hold robot in place while
// reading from the buffer. Robot will be stopped initially.
#define UNMAPPED_STOP_SPEED 128

// distance between wheels of the robot
// once accurate distance is measured can move this
// to const double
#define WIDTH 10

// motors
Servo LeftM;
Servo RightM;

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


void setup()  { 
  Serial.begin(BAUD_RATE);
  Serial.println("ready");
  LeftM.attach(left_motor_pin);
  RightM.attach(right_motor_pin);
} 

void loop()  { 
  delay(100);
  serial_read();  
  convert();
  drive(angular_z, linear_x);
}  

void serial_read(){
  //reading in 6 chars from Serial
  if (Serial.available() > BUFFER_SIZE){

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

/* Serial.read() reads values in [0,255]
   * we map them to lower and upper speeds defined above
   * for both linear and angular velocity
  */
void convert(){
  linear_x = map(linear_x, 0, 255, linear_min, linear_max);
  angular_z = map(angular_z, 0, 255, angular_min, angular_max);
}


/*
 * moves the robot - turning is taken into account.
 */
void drive(int angular_speed, int linear_speed){
  
  // mapping should map to a percentage of max and min duty cycle
  // for our motors
  // we are using the Talon SRX - looks like duty cycle is between 2.9 - 100ms
  // seen here https://www.ctr-electronics.com/Talon%20SRX%20User's%20Guide.pdf
  int left_throttle = linear_speed + (angular_speed - angular_stop);
  int right_throttle = linear_speed - (angular_speed - angular_stop);
  
  servo_write(LeftM, left_throttle);
  servo_write(RightM, right_throttle);
}

// this writes to motor using duty cycle rather than servo arm angle
void servo_write(Servo motor, int throttle) {
  // throttle can be as high as 110 and as low as 70 after calculation
  // PWM input pulse high time can be between 1 and 2 ms. So 1000-2000 microseconds
  throttle = map(throttle, 70, 110, 1000, 2000); 
  motor.writeMicroseconds(throttle);
}

