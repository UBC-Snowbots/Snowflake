#include <SoftwareSerial.h>
#include <stdlib.h>
#include <math.h>
#include <Servo.h>

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

// max and min linear speeds and stopping condition
const int linear_max = 100;
const int linear_min = 80;
const int linear_stop = 90;

// max and min angular speeds and stopping condition
const int angular_max = 100;
const int angular_min = 80;
const int angular_stop = 90;

// defines start of buffer
const char buffer_head = 'B';

SoftwareSerial mySerial(11, 12);

void setup(){
  Serial.begin(9600);
  while (!Serial) {
   ; 
  }
  
  Serial.println("ready");
  LeftM.attach(left_motor_pin);
  RightM.attach(right_motor_pin);
}

void loop()  { 
  delay(100);
  serial_read(); 
 
  Serial.println("linear_speed before conversion = ");Serial.println(linear_x); Serial.println("angular speed before conversion = ");Serial.println(angular_z);
  convert();
  Serial.println("linear_speed after conversion = ");Serial.println(linear_x); Serial.println("angular speed after conversion = ");Serial.println(angular_z);
  drive(angular_z, linear_x);
  int left_speed = LeftM.read();
  int right_speed = RightM.read();
  
  Serial.println("left speed = ");Serial.println(left_speed);
  Serial.println("right speed = ");Serial.println(right_speed);
}

void serial_read() {
 //reading in 6 chars from Serial
  if (Serial.available() >= BUFFER_SIZE){

      /* if (Serial.read() == 'B'){
      linear_x = (Serial.read()-'0')*100 + (Serial.read()-'0')*10 + (Serial.read()-'0');
      ly =(Serial.read()-'0')*100 + (Serial.read()-'0')*10 + (Serial.read()-'0');
      angular_z = (Serial.read()-'0')*100 + (Serial.read()-'0')*10 + (Serial.read()-'0');*/

      // buffer_head identifies the start of the buffer
      if (Serial.read() == buffer_head) {
          linear_x = Serial.read();
          linear_y = Serial.read();
          linear_z = Serial.read();
          angular_x = Serial.read();
          angular_y = Serial.read();
          angular_z = Serial.read();
          
          serial_write();
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

void serial_write() {
 int addx = linear_x + 1;
 int addy = linear_y + 1;
 int addz = linear_z + 1;
 int angx = angular_x + 1;
 int angy = angular_y + 1;
 int angz = angular_z + 1;
 
 int bufferhead_sent = Serial.write(buffer_head);
 int linx_sent = Serial.write(addx);
 int liny_sent = Serial.write(addy);
 int linz_sent = Serial.write(addz);
 int angx_sent = Serial.write(angx);
 int angy_sent = Serial.write(angy);
 int angz_sent = Serial.write(angz);
}

void drive(int angular_speed, int linear_speed){

  LeftM.write(linear_speed + (angular_speed - angular_stop));
  RightM.write(linear_speed - (angular_speed - angular_stop));
   /* if (angular_speed < angular_stop) {
      
    } else {
      LeftM.write(linear_speed - (angular_speed - angular_stop));
      RightM.write(linear_speed + (angular_speed - angular_stop));
    } */
}

void convert() {
  /* Serial.read() reads values in [0,255]
   * we map them to lower and upper speeds defined above
   * for both linear and angular velocity
  */
  linear_x = map(linear_x, 0, 255, linear_min, linear_max);
  angular_z = map(angular_z, 0, 255, angular_min, angular_max);
}
