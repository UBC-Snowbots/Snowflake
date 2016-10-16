#include <SoftwareSerial.h>
#include <stdlib.h>
#include <math.h>
#include <Servo.h>

#define BAUD_RATE 115200

// we will be reading 2 chars
// 1 for linear speed, 1 for angular speed
#define BUFFER_SIZE 2

// assign 128 to be not moving
// will be used to hold robot in place while
// reading from the buffer. Robot will be stopped initially.
#define UNMAPPED_STOP_SPEED 128


// buffer values are mapped to [80, 100] to define linear speed
// define 80 < linear speed < 90 -> move backward
// define 90 < linear speed < 100 -> move forward
/*
 * Need to figure out desired max speed. We have 10 speed levels
 * in the forward/backward directions. Given max speed,
 * we will move at x/10 of max speed for x [1, 10]
 */
#define UPPER_LINEAR_SPEED 100
#define LOWER_LINEAR_SPEED 80


/*
 * as above, will move at x/20 of max angular speed
 * for x [1, 20]
 */
// define 75 < angular_speed < 95 -> turn left
#define UPPER_ANGULAR_SPEED 100
// define 90 < angular_speed < 115 -> turn right
#define LOWER_ANGULAR_SPEED 80

//stop values
#define LINEAR_STOP 90
#define ANGULAR_STOP 90

// distance between wheels of the robot
// once accurate distance is measured can move this
// to const double
#define WIDTH 10


Servo LeftM;//5
Servo RightM;
int lx = 0;
double az = 0.0;

const int left_motor_pin = 9;
const int right_motor_pin = 10;
const char buffer_head = 'B';

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
  drive();
}  

void serial_read(){
    //reading in 2 chars from Serial
  if (Serial.available()>BUFFER_SIZE){

      /*
    if (Serial.read() == 'B'){
      lx = (Serial.read()-'0')*100 + (Serial.read()-'0')*10 + (Serial.read()-'0');
      ly =(Serial.read()-'0')*100 + (Serial.read()-'0')*10 + (Serial.read()-'0');
      az = (Serial.read()-'0')*100 + (Serial.read()-'0')*10 + (Serial.read()-'0');*/

      // buffer_head identifies the start of the buffer
      if (Serial.read() == buffer_head) {
          lx = Serial.read();
          az = Serial.read();
      } else {
          lx = az = UNMAPPED_STOP_SPEED;
      }

  } else {
      lx = az = UNMAPPED_STOP_SPEED;
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
  lx = map (lx, 0, 255, LOWER_LINEAR_SPEED, UPPER_LINEAR_SPEED);
  az = mapToDouble (az, 0, 255, - M_PI / 2, M_PI / 2);
}


void drive(){

    move(az, lx);
     /*if(ly == LINEAR_STOP){
   if (az == ANGULAR_STOP){
       LeftM.write(LINEAR_STOP);
       RightM.write(LINEAR_STOP);
   } else{
       LeftM.write(az);
       RightM.write(az);
   }
  }
  else{
      LeftM.write(ly);
      RightM.write(ly);
   }
 }
 else{
   if (lx > LINEAR_STOP){
       LeftM.write(lx);//if 80 //if 100
       RightM.write(lx+20);//needs to be 100 //needs to be 80
   } else{
       LeftM.write(lx);
       RightM.write(lx-20);}
   }*/
}

/*
 * moves the robot - turning is taken into account.
 */

void move(double angular_speed, int linear_speed) {
    if (angular_speed < ANGULAR_STOP) {
        LeftM.write(linear_speed - round((sin(angular_speed) * WIDTH / 2)));
        RightM.write(linear_speed + round((sin(angular_speed) * WIDTH / 2)));
    } else {
        LeftM.write(linear_speed + round((sin(angular_speed) * WIDTH / 2)));
        RightM.write(linear_speed - round(sin(angular_speed) * WIDTH / 2));
    }
}

double mapToDouble(double val, long in_min, long in_max, double out_min, double out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
