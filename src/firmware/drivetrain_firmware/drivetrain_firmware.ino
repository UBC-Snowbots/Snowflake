#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <stdlib.h>
#include <Servo.h>

#define BAUD_RATE 115200
#define BUFFER_SIZE 3
#define STOP 128
#define UPPER_LINEAR_SPEED 100
#define LOWER_LINEAR_SPEED 80
#define UPPER_ANGULAR_SPEED 115
#define LOWER_ANGULAR_SPEED 75


Servo LeftM;//5
Servo RightM;
int lx,ly,az = 0;

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
    //reading in 3 chars from Serial
  if (Serial.available()>BUFFER_SIZE){

      /*
    if (Serial.read() == 'B'){
      lx = (Serial.read()-'0')*100 + (Serial.read()-'0')*10 + (Serial.read()-'0');
      ly =(Serial.read()-'0')*100 + (Serial.read()-'0')*10 + (Serial.read()-'0');
      az = (Serial.read()-'0')*100 + (Serial.read()-'0')*10 + (Serial.read()-'0');*/

      // B identifies the start of the buffer
      if (Serial.read() == buffer_head) {
          lx = Serial.read();
          ly = Serial.read();
          az = Serial.read();
      } else {
          lx = ly = az = STOP;
      }

  } else {
      lx = ly = az = STOP;
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
  ly = map (ly, 0, 255, LOWER_LINEAR_SPEED, UPPER_LINEAR_SPEED);
  az = map (az, 0, 255, LOWER_ANGULAR_SPEED, UPPER_ANGULAR_SPEED);
}


void drive(){
/*
* This needs to be re-written
*/
 if(lx == 90){
  if(ly == 90){
   if (az == 90){
     LeftM.write(90);
     RightM.write(90);
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
   if (lx > 90){
   LeftM.write(lx);//if 80 //if 100
   RightM.write(lx+20);//needs to be 100 //needs to be 80
   }
   else{LeftM.write(lx);
   RightM.write(lx-20);}
 }
}
