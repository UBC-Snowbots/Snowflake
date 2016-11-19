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
  mySerial.begin(9600);
  //LeftM.attach(left_motor_pin);
  //RightM.attach(right_motor_pin);
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

      // buffer_head identifies the start of the buffer
      if (Serial.read() == buffer_head) {
          linear_x = mySerial.write(Serial.read());
          linear_y = mySerial.write(Serial.read());
          linear_z = mySerial.write(Serial.read());
          angular_x = mySerial.write(Serial.read());
          angular_y = mySerial.write(Serial.read());
          angular_z = mySerial.write(Serial.read());
          
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
  if (mySerial.available()) {
     Serial.write(mySerial.read());
  }
 
}

void drive(int angular_speed, int linear_speed){
  LeftM.write(linear_speed + (angular_speed - angular_stop));
  RightM.write(linear_speed - (angular_speed - angular_stop));
}

void convert() {
  /* Serial.read() reads values in [0,255]
   * we map them to lower and upper speeds defined above
   * for both linear and angular velocity
  */
  linear_x = map(linear_x, 0, 255, linear_min, linear_max);
  angular_z = map(angular_z, 0, 255, angular_min, angular_max);
}
