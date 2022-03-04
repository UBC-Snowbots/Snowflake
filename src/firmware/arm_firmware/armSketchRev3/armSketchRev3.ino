/*
Created By: Tate Kolton, Rowan Zawadzki
Created On: December 21, 2021
Description: Main code for testing 6-axis stepper motor driven arm functionality. Interfaces with windows form through serial communication. Intended for use with Teensy 4.1
Warning: May run into errors if attempting to use with Arduino Mega, Uno, or Nano. Insufficient DRAM / Flash, and clock speed. 
*/

#include <AccelStepper.h>

// general parameters
#define NUM_AXES 6
#define NUM_PARAMS 7
#define ON 0
#define OFF 1
#define SW_ON 0
#define SW_OFF 1

// motor pins
int stepPins[6] = {3, 5, 6, 8, 10, 12};
int dirPins[6] = {4, 7, 9, 11, 13, 14};

// limit switch pins
int limPins[6] = {15, 16, 17, 18, 19, 20};

// pulses per revolution for motors
long ppr[6] = {400, 400, 400, 400, 400, 400};

// Gear Reduction Ratios
long red[6] = {40, 161, 150, 70, 45, 45};

// Encoder pulses per revolution
long pprEnc = 512;

// Motor speeds and accelerations
int maxSpeed[6] = {3000, 3000, 3000, 3000, 3000, 3000};
int maxAccel[6] = {4000, 4000, 4000, 4000, 4000, 4000};
int homeSpeed[6] = {1000, 1000, 1000, 1000, 1000, 1000};
int homeAccel[6] = {2000, 2000, 2000, 2000, 2000, 2000};

// Range of motion (degrees) for each axis
long maxAngles[6] = {180, 120, 120, 120, 120, 120};

// stepper motor objects for AccelStepper library
AccelStepper steppers[6];


// variable declarations
int i, count, axis, temp, flag, sign,int_ang;
long angle;
char axisC, garbage, mode, serialBit;
String char_angle;
long angles[] = {0, 0, 0, 0, 0, 0};
long max_steps[] = {red[0]*maxAngles[0]/360.0*ppr[0], red[1]*maxAngles[1]/360.0*ppr[1], red[2]*maxAngles[2]/360.0*ppr[2], red[3]*maxAngles[3]/360.0*ppr[3], red[4]*maxAngles[4]/360.0*ppr[4], red[5]*maxAngles[5]/360.0*ppr[5]};
int axisDir[6] = {1, 1, 1, 1, 1, 1};

// variables for homing / arm calibration
long homePosConst = -100000;
long homeCompConst = 500;
long homePos[] = {axisDir[0]*homePosConst, axisDir[1]*homePosConst, axisDir[2]*homePosConst, axisDir[3]*homePosConst, axisDir[4]*homePosConst, axisDir[5]*homePosConst};
long homeComp[] = {axisDir[0]*homeCompConst, axisDir[1]*homeCompConst, axisDir[2]*homeCompConst, axisDir[3]*homeCompConst, axisDir[4]*homeCompConst, axisDir[5]*homeCompConst};
long homeCompAngles[] = {axisDir[0]*25, axisDir[1]*25, axisDir[2]*25, axisDir[3]*25, axisDir[4]*25, axisDir[5]*25};
long homeCompSteps[] = {homeCompAngles[0]*red[0]*ppr[0]/360.0, homeCompAngles[1]*red[1]*ppr[1]/360.0, homeCompAngles[2]*red[2]*ppr[2]/360.0, homeCompAngles[3]*red[3]*ppr[3]/360.0, homeCompAngles[4]*red[4]*ppr[4]/360.0, homeCompAngles[5]*red[5]*ppr[5]/360.0}; 

// setup function to initialize pins and provide initial homing to the arm
void setup() {

  Serial.begin(9600);
  flag = 0;

  // initializing steppers
  for(int i=0; i<NUM_AXES; i++) {
    steppers[i] = AccelStepper(1, stepPins[i], dirPins[i]);
  }

  // initializing limit switches
  for(i = 0; i<NUM_AXES; i++) {
    pinMode(limPins[i], INPUT_PULLUP);
  }

  // initializing step pins
  for(i = 0; i<NUM_AXES; i++) {
    pinMode(stepPins[i], INPUT_PULLUP);
  }

  // initializing direction pins
  for(i = 0; i<NUM_AXES; i++) {
    pinMode(dirPins[i], INPUT_PULLUP);
  }

  // initializing max speeds and accelerations for motors
  initializeMotion();
  
  // homes all axes of arm upon powerup
    home_arm();

}

// main loop where motion functions are called and serial communication occurs
void loop() {
  if (Serial.available() > 0) { // parsing of incoming serial data
    char first = Serial.read();

    if (first == 'H') {
      home_arm();
    }

    else {
      flag = 1;
      mode = Serial.read();
      delay(10);
      garbage = Serial.read();
      delay(10);
      axisC = Serial.read();
      axis = axisC - '0';
      delay(10);
      char_angle = Serial.readStringUntil('\n');
      int_ang = char_angle.toInt();
      angle = (long)int_ang;

      while (Serial.available() > 0) {
        char bufferGarbage = Serial.read();
      }
    }
  }

  if (flag == 1) {
    step_angle(axis, angle, red[axis-1], ppr[axis-1], maxAngles[axis-1]);
    flag = 0;
  }

  delay(100);
}

// functions for arm motion
void step_angle(int axis, long ang, long red, long ppr, long max_ang) { 
  
  long step_in;

  if (mode == 'I') { 
    step_in = red*ppr*(ang / 360.0);
    
    if(ang == 0) {
    }

    else if(ang + angles[axis-1] < max_ang && ang + angles[axis-1]>0) {
      angles[axis - 1] += ang;
    }

    else {
      if(ang>0) {
        step_in = (red*ppr/360.0)*(max_ang - angles[axis-1]);
        angles[axis-1] = max_ang; 
      }

      else {
        step_in = -(red*ppr/360.0)*angles[axis-1];
        angles[axis-1] = 0; 
      }
    }

    step_in = step_in*axisDir[axis-1];
    incMove(axis, step_in);   
  } 

  else if (mode == 'A') {
      
    if(ang<=0) {
      step_in = 0;
      angles[axis-1] = 0;
    }

    else if(ang>=max_ang) {
      step_in = red*ppr/360.0*max_ang;
      angles[axis-1] = max_ang;
    }
    
    else {
      step_in = red*ppr*ang/360.0;
      angles[axis - 1] = ang; 
    }

    step_in = step_in*axisDir[axis-1];
    absMove(axis, step_in);
  }
   
  limTrig(axis);       
  sendAngles();
}

// Function for homing of full arm
void home_arm() {

  bool stopFlags[6] = {false, false, false, false, false, false};
  bool finishFlags[6] = {false, false, false, false, false, false};
  bool completeFlag = false;
  int count = 0;
  initializeHomingMotion();

  while(!completeFlag) {

    for(i = 0; i<NUM_AXES; i++) {

      if(!finishFlags[i]) {

        if(!digitalRead(limPins[i]) && !stopFlags[i]) {
          steppers[i].run();
        }

        else if(!stopFlags[i]) {
          steppers[i].setCurrentPosition(-homeComp[i]);
          steppers[i].stop();
          steppers[i].setMaxSpeed(maxSpeed[i]/2);
          steppers[i].setAcceleration(maxAccel[i]/2);
          steppers[i].moveTo(homeCompSteps[i]);
          stopFlags[i] = true;
        }

        else if(steppers[i].distanceToGo() !=0) {
          steppers[i].run();
        }

        else {
          finishFlags[i] = true;
          count++;
        }
      }
    }
    if(count == NUM_AXES) {
      completeFlag = true;
    }
  }

  for(i=0; i<NUM_AXES; i++) {
    angles[i] = abs(homeCompAngles[i]);
  }

  sendAngles();
}

// sets initial speed and acceleration, and sets target position
void initializeHomingMotion() {

  for(i = 0; i<NUM_AXES; i++) {
    steppers[i].moveTo(homePos[i]);
    steppers[i].setMaxSpeed(homeSpeed[i]);
    steppers[i].setAcceleration(homeAccel[i]);
  } 
}

// sets speeds for each axis 
void initializeMotion() {

  for(i = 0; i<NUM_AXES; i++) {
    steppers[i].setMaxSpeed(maxSpeed[i]);
    steppers[i].setAcceleration(maxAccel[i]);
  }
}

// sets current position of all axes to 0
void zeroAxes() { 

  for(i = 0; i<NUM_AXES; i++) {
    steppers[i].setCurrentPosition(0);
  }
}

// Incremental motion of individual axis
void incMove(int axis, long steps) {

   steppers[axis-1].move(steps);
   while(steppers[axis-1].distanceToGo() !=0 && !digitalRead(limPins[axis-1])) {
      steppers[axis-1].run();
   }
}

// absolute motion of individual axis
void absMove(int axis, long steps) {

   steppers[axis-1].moveTo(steps);
   while(steppers[axis-1].distanceToGo() !=0 && !digitalRead(limPins[axis-1])) {
       steppers[axis-1].run();
   }
}

// routine for if limit switches are triggered unexpectedly during a move
void limTrig(int axis) {

  if(digitalRead(limPins[axis-1])) {
    steppers[axis-1].setCurrentPosition(0);
    steppers[axis-1].stop();
    steppers[axis-1].move(homeComp[axis-1]);
    steppers[axis-1].setMaxSpeed(homeSpeed[axis-1]);
    steppers[axis-1].setAcceleration(homeAccel[axis-1]);
        
    while (steppers[axis-1].distanceToGo() != 0) {
      steppers[axis-1].run();
    }

    steppers[axis-1].setCurrentPosition(0);
    steppers[axis-1].setMaxSpeed(maxSpeed[axis-1]);
    steppers[axis-1].setAcceleration(maxAccel[axis-1]);
    angles[axis-1] = 0;
  }
}

// Sending angles through serial port
void sendAngles() {

  Serial.println('r');
  delay(10);

  for (int i = 0; i < NUM_AXES; i++) {
    Serial.println(angles[i]);
    delay(10);
  }
}

// Sending zero angles through serial port
void sendZeroAngles() {
  
  Serial.println('r');
  delay(10);
  
  for (int i = 0; i < NUM_AXES; i++) {
    angles[i] = 0;
    Serial.println(angles[i]);
    delay(10);
  }
}