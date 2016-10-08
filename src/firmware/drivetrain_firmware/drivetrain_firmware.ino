#include <SoftwareSerial.h>
#include <stdlib.h>
#include <Servo.h>

#define BAUD_RATE 115200

Servo LeftM;//5
Servo RightM;
int lx,ly,az = 0;

const int left_motor_pin = 9;
const int right_motor_pin = 10;

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
  if (Serial.available()>9){
    
    if (Serial.read() == 'B'){
      lx = (Serial.read()-'0')*100 + (Serial.read()-'0')*10 + (Serial.read()-'0');
      ly =(Serial.read()-'0')*100 + (Serial.read()-'0')*10 + (Serial.read()-'0');
      az = (Serial.read()-'0')*100 + (Serial.read()-'0')*10 + (Serial.read()-'0');
    } else {
      lx = ly = az = 128;
    }
    
  } else {  
      lx = ly = az = 128;
    }
    
  //Serial.end();
  //Serial.begin(9600);
  
  //flushRX defined here: https://forum.sparkfun.com/viewtopic.php?f=32&t=32715 
  Serial.flushRX();
}

void convert(){
  /* [0, 255] for 8 bit values
  *  [80, 100] for linear motor speed
  *  [75, 115] for angular speed
  */
  lx = map (lx, 0, 255, 80, 100);
  ly = map (ly, 0, 255, 80, 100);
  az = map (az, 0, 255, 75, 115);
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
