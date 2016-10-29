#include <SoftwareSerial.h>
#include <stdlib.h>
#include <math.h>
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
double angular_z = 0.0;

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
const double angular_max = M_PI / 2;
const double angular_min = - M_PI / 2;
const double angular_stop = 0.0;


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
          angular_z = (double) Serial.read();
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

void convert(){
  /* Serial.read() reads values in [0,255]
   * we map them to lower and upper speeds defined above
   * for both linear and angular velocity
  */
  linear_x = map (linear_x, 0, 255, linear_min, linear_max);
  angular_z = mapToDouble (angular_z, 0, 255, angular_min, angular_max);
}


/*
 * moves the robot - turning is taken into account.
 */
void drive(double angular_speed, int linear_speed){

    if (angular_speed < angular_stop) {
        LeftM.write(linear_speed - round((sin(angular_speed) * WIDTH / 2)));
        RightM.write(linear_speed + round((sin(angular_speed) * WIDTH / 2)));
    } else {
        LeftM.write(linear_speed + round((sin(angular_speed) * WIDTH / 2)));
        RightM.write(linear_speed - round(sin(angular_speed) * WIDTH / 2));
    }
     /*if(ly == LINEAR_STOP){
   if (angular_z == ANGULAR_STOP){
       LeftM.write(LINEAR_STOP);
       RightM.write(LINEAR_STOP);
   } else{
       LeftM.write(angular_z);
       RightM.write(angular_z);
   }
  }
  else{
      LeftM.write(ly);
      RightM.write(ly);
   }
 }
 else{
   if (linear_x > LINEAR_STOP){
       LeftM.write(linear_x);//if 80 //if 100
       RightM.write(linear_x+20);//needs to be 100 //needs to be 80
   } else{
       LeftM.write(linear_x);
       RightM.write(linear_x-20);}
   }*/
}




double mapToDouble(int val, int in_min, int in_max, double out_min, double out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
