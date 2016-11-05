#include <SoftwareSerial.h>
#include <stdlib.h>
#include <math.h>
#include <Servo.h>

long randNumber;
Servo LeftM;
Servo RightM;

int linear_x;
int angular_z;

// max and min linear speeds and stopping condition
const int linear_max = 100;
const int linear_min = 80;
const int linear_stop = 90;

// max and min angular speeds and stopping condition
const int angular_max = 100;
const int angular_min = 80;
const int angular_stop = 90;

void setup(){
  Serial.begin(9600);

  // if analog input pin 0 is unconnected, random analog
  // noise will cause the call to randomSeed() to generate
  // different seed numbers each time the sketch runs.
  // randomSeed() will then shuffle the random function.
  randomSeed(analogRead(5));
}

void loop() {
  // print a random number from 0 to 299
  
  int linear_rand = random(0, 255);
  //Serial.println(randNumber);

  // print a random number from 10 to 19
  int angular_rand = random(0, 255);
  //Serial.println(randNumber);
  linear_x = map(linear_rand, 0, 255, linear_min, linear_max);
  Serial.println("linear_speed = ");Serial.println(linear_x);
  
  angular_z = map(angular_rand, 0, 255, angular_min, angular_max);  
  Serial.println("angular speed = ");Serial.println(angular_z);
  
  drive(angular_z, linear_x);
  
  int left_speed = LeftM.read();
  int right_speed = RightM.read();
  
  Serial.println("left speed = ");Serial.println(left_speed);
  Serial.println("right speed = ");Serial.println(right_speed);
  
  delay(50);
  
  
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

void convert(){
  /* Serial.read() reads values in [0,255]
   * we map them to lower and upper speeds defined above
   * for both linear and angular velocity
  */
  linear_x = map (linear_x, 0, 255, linear_min, linear_max);
  angular_z = map (angular_z, 0, 255, angular_min, angular_max);
}



