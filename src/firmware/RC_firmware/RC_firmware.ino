/* Firmware for Elsa
   Test version
   Author: Vincent Yuan, Modified: Nick Wu
   Date: May 21, 2016
*/

#include <SoftwareSerial.h>
#include <stdlib.h>
#include <Servo.h>
#include <math.h>
#define TRIM 8


// DRIVING MODIFICATIONS
// modify FR, FL, BR, BL to change speeds: (+) correpsonds to faster speed, (-) for going reverse
// F:FRONT, B:BACK, R:RIGHT, L:LEFT e.g. FR = FRONT RIGHT WHEEL.
const int OFFSET = 1, FR = 9, FL = 9, BR = 9, BL = 9;

Servo LeftMF, LeftMB, RightMB, RightMF;

int lx = 0, ly = 0, az = 0;
int R1 = 0, R2 = 0, R3 = 0, R4 = 0, B2 = 0, B4 = 0, Mode = 0;
int lx_h = 0, lx_l = 0, az_h = 0, az_l = 0;

void setup() {
  Serial.begin(115200);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  
  LeftMB.attach(10);
  RightMF.attach(9);
  
  LeftMF.attach(12);
  RightMB.attach(11);
  
  set_off();
}

void loop() {

  //if (Serial.read() == 'I'){Serial.print("DRIVE");}
  //else Serial.flushRX();
  //responding to queries from usb
  sig_read();
  Serial.println(Mode);
  //serial_read();
  if (Mode == -1) { //Auto Mode
    serial_read(); lx = 128; ly = 128; az = B4;
    convert();
    drive();
  }
  else if (Mode == 0) { //RC Mode
    //Serial.println(az);
    lx = 128; ly = R2; az = R1;
    convert();
    drive();
    Serial.flushRX();
  }
  else { //STOP MODE
    lx = 128; ly = 128; az = 128;
    convert();
    drive();
    Serial.flushRX();
  }
  //Serial.print("lx: ");Serial.print(lx);Serial.print(" ly: ");Serial.print(ly);Serial.print(" az: ");Serial.println(az);

}

void set_off() {
  int lx_mid = 1550; //RX standard
  int az_mid = 1550; //RY standard
  lx_h = lx_mid + 150; lx_l = lx_mid - 150;
  az_h = az_mid + 150; az_l = az_mid - 150;
}

void sig_read() {
  R1 = pulseIn(2, HIGH); //1140 - 1965 RX LEFT-RIGHT
  R2 = pulseIn(3, HIGH); //1965 - 1140 RY UP-DOWN
  R3 = pulseIn(4, HIGH); //1970 - 1115 LY UP-DOWN
  
  //R4 = pulseIn(5,HIGH); //1970 - 1115 LX LEFT-RIGHT
  //Serial.print("R1: "); Serial.print(R1);Serial.print(" R2: "); Serial.print(R2);Serial.print(" R3: "); Serial.println(R3);
  if (R1 < lx_h && R1 > lx_l) R1 = 128;
  else R1 = map (R1, 1140, 1965, 0, 255);

  if (R2 < az_h && R2 > az_l) R2 = 128;
  else R2 = map (R2, 1140, 1965, 0, 255);
  
  //Serial.print("R3: ");Serial.println(R3);
  if (1100 < R3 && R3 < 1400) Mode = 1;
  else if (1400 < R3 && R3 < 1700) Mode = 0;
  else if (1700 < R3 && R3 < 2000) Mode = -1;
  else Mode = -2;
  //Serial.print("R1: "); Serial.println(R1);
  //Serial.print("R2: "); Serial.println(R2);
  if (abs(R1 - 90) < TRIM) R1 = 128;
  if (abs(R2 - 90) < TRIM) R2 = 128;
}

void serial_read() {
  if (Serial.available() > 9) {
    if (Serial.read() == 'B') {
      int B9 = (Serial.read() - '0') * 100 + (Serial.read() - '0') * 10 + (Serial.read() - '0');
      B2 = (Serial.read() - '0') * 100 + (Serial.read() - '0') * 10 + (Serial.read() - '0');
      B4 = (Serial.read() - '0') * 100 + (Serial.read() - '0') * 10 + (Serial.read() - '0');
      //Serial.print("B4: "); Serial.print(B4);
    }
    else {
      B2 = B4 = 128;
    }
    //B2 = 128;
  }
  else {
    B2 = B4 = 128;
  }
  //Serial.end();
  //Serial.begin(9600);
  Serial.flushRX();
}

void convert() {
  if (ly > 255)ly = 255; else if (ly < 0)ly = 0;
  if (az > 255)az = 255; else if (az < 0)az = 0;
  lx = map (lx, 0, 255, 80, 100);//80 100
  ly = map (ly, 0, 255, 85, 95);//85 95
  az = map (az, 0, 255, 80, 100);//80 100
}

void left(int front, int back) {
  LeftMF.write(90 + OFFSET - front);
  LeftMB.write(90 + OFFSET - back);
}
void right(int front, int back) {
  RightMF.write(90 + OFFSET + front);
  RightMB.write(90 + OFFSET + back);
}
void drive() {
  if (az == 90) {
    if (ly == 90) {
      right(0, 0); left(0, 0);
    } else if (ly > 90) {
      right(FR, BR); left(FL, BL);
      //Serial.println("FORWARD");
    } else {
      right(-FR, -BR); left(-BL, -BL);
      //Serial.println("BACK");
    }
  } else if (az < 90) {
    right(FR, BR); left(-FL, -BL);
    //Serial.println("RIGHT");
  } else {
    right(-FR, -BR); left(FL, BL);
    //Serial.println("LEFT");
  }
}
