/*
Created By: Tate Kolton, Rowan Zawadzki
Created On: December 21, 2021
Description: Main code for testing 6-axis stepper motor driven arm functionality. Interfaces with windows form through serial communication. Intended for use with Teensy 4.1
Warning: May run into errors if attempting to use with Arduino Mega, Uno, or Nano. Insufficient DRAM / Flash, and clock speed. 
*/

#include <AccelStepper.h>

// motor pins
#define STEP_INDEX 0
#define DIR_INDEX 1
#define a1_step 3
#define a1_dir 4
#define a2_step 5
#define a2_dir 5
#define a3_step 6
#define a3_dir 7
#define a4_step 8
#define a4_dir 9
#define a5_step 10
#define a5_dir 11
#define a6_step 12
#define a6_dir 13

// limit switch pins
#define SW_INDEX 2
#define a1_sw 2
#define a2_sw 48
#define a3_sw 47
#define a4_sw 46
#define a5_sw 45
#define a6_sw 44

#define PPR_INDEX 4
long a1_PPR = 400;
long a2_PPR = 400;
long a3_PPR = 400;
long a4_PPR = 400;
long a5_PPR = 400;
long a6_PPR = 400;

// axis speeds and accelerations
#define SPEED_INDEX 3
#define a1_SPEED 4500
#define a2_SPEED 2000
#define a3_SPEED 2000
#define a4_SPEED 2000
#define a5_SPEED 2000
#define a6_SPEED 2000

#define ACC_INDEX 4
#define a1_ACC 5000
#define a2_ACC 2000
#define a3_ACC 2000
#define a4_ACC 2000
#define a5_ACC 2000
#define a6_ACC 2000

// homing speeds and accelerations
#define a1_home_SPEED 1000
#define a1_home_ACC 2000
#define a2_home_SPEED 1000
#define a2_home_ACC 2000
#define a3_home_SPEED 1000
#define a3_home_ACC 2000
#define a4_home_SPEED 1000
#define a4_home_ACC 2000
#define a5_home_SPEED 1000
#define a5_home_ACC 2000
#define a6_home_SPEED 1000
#define a6_home_ACC 2000

// gear reduction ratios
#define REDUCTION_INDEX 5
#define a1_RED  161
#define a2_RED  161
#define a3_RED  150
#define a4_RED  64
#define a5_RED  57
#define a6_RED  57

// maximum angles of travel relative to limit switch
#define ANG_INDEX 6
#define a1_MAX_ANG 180
#define a2_MAX_ANG 120
#define a3_MAX_ANG 120
#define a4_MAX_ANG 120
#define a5_MAX_ANG 120
#define a6_MAX_ANG 120

// general parameters
#define ENC_PPR 512
#define NUM_AXES 6
#define NUM_PARAMS 7
#define ON 0
#define OFF 1
#define SW_ON 0
#define SW_OFF 1

// stepper motor objects for AccelStepper library
AccelStepper accelSteppers[] = {AccelStepper(1, a1_step, a1_dir), AccelStepper(1, a2_step, a2_dir), AccelStepper(1, a3_step, a3_dir), AccelStepper(1, a5_step, a5_dir), AccelStepper(1, a6_step, a6_dir)}

// variable declarations
int i, count, axis, temp, flag, sign,int_ang;
long angle;
char axisC, garbage, mode, serialBit;
String char_angle;
long angles[] = {0, 0, 0, 0, 0, 0};
long max_steps[] = {a1_RED*a1_MAX_ANG/360.0*a1_PPR, a2_RED*a2_MAX_ANG/360.0*a2_PPR, a3_RED*a3_MAX_ANG/360.0*a3_PPR, a3_RED*a3_MAX_ANG/360.0*a3_PPR, a3_RED*a3_MAX_ANG/360.0*a3_PPR, a3_RED*a3_MAX_ANG/360.0*a3_PPR};
long step_in = 0;
int axisDir[] = {1, 1, 1, 1, 1, 1};

// variables for homing / arm calibration
long homePosConst = -100000;
long homeCompConst = 500;
long homePos[] = {axisDir[0]*homePosConst, axisDir[1]*homePosConst, axisDir[2]*homePosConst, axisDir[3]*homePosConst, axisDir[4]*homePosConst, axisDir[5]*homePosConst};
long homeComp[] = {axisDir[0]*homeCompConst, axisDir[1]*homeCompConst, axisDir[2]*homeCompConst, axisDir[3]*homeCompConst, axisDir[4]*homeCompConst, axisDir[5]*homeCompConst};
long homeCompAngles[] = {axisDir[0]*25, axisDir[1]*25, axisDir[2]*25, axisDir[3]*25, axisDir[4]*25, axisDir[5]*25};
long homeCompSteps[] = {homeCompAngles[0]*a1_RED*a1_PPR/360.0, homeCompAngles[1]*a2_RED*a2_PPR/360.0, homeCompAngles[2]*a3_RED*a3_PPR/360.0, homeCompAngles[3]*a4_RED*a4_PPR/360.0, homeCompAngles[4]*a5_RED*a5_PPR/360.0, homeCompAngles[5]*a6_RED*a6_PPR/360.0};

// 2D array lookup table that stores all the stepper motor parameters: step pin, direction pin, limit switch pin, speed, pulses per revolution, gear reduction, and maximum angles for each axis

long stepper_data_lookup[NUM_PARAMS][NUM_AXES] = {{a1_step, a2_step, a3_step, a4_step, a5_step, a6_step}, {a1_dir, a2_dir, a3_dir, a4_dir, a5_dir, a6_dir}, {a1_sw, a2_sw, a3_sw, a4_sw, a5_sw, a6_sw}, {a1_SPEED, a2_SPEED, a3_SPEED, a4_SPEED, a5_SPEED, a6_SPEED}, {a1_PPR, a2_PPR, a3_PPR, a4_PPR, a5_PPR, a6_PPR}, {a1_RED, a2_RED, a3_RED, a4_RED, a5_RED, a6_RED}, {a1_MAX_ANG, a2_MAX_ANG, a3_MAX_ANG, a4_MAX_ANG, a5_MAX_ANG, a6_MAX_ANG}};

// setup function to initialize pins and provide initial homing to the arm

void setup() {

    Serial.begin(9600);
    flag = 0;

    // initializing limit switches
    pinMode(a1_sw, INPUT_PULLUP);
    pinMode(a2_sw, INPUT_PULLUP);
    pinMode(a3_sw, INPUT_PULLUP);
    pinMode(a4_sw, INPUT_PULLUP);
    pinMode(a5_sw, INPUT_PULLUP);
    pinMode(a6_sw, INPUT_PULLUP);

    // initializing step pins
    pinMode(a1_step, OUTPUT);
    pinMode(a2_step, OUTPUT);
    pinMode(a3_step, OUTPUT);
    pinMode(a4_step, OUTPUT);
    pinMode(a5_step, OUTPUT);
    pinMode(a6_step, OUTPUT);

    // initializing direction pins
    pinMode(a1_dir, OUTPUT);
    pinMode(a2_dir, OUTPUT);
    pinMode(a3_dir, OUTPUT);
    pinMode(a4_dir, OUTPUT);
    pinMode(a5_dir, OUTPUT);
    pinMode(a6_dir, OUTPUT);

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
        step_angle(axis, mode, angle, stepper_data_lookup[STEP_INDEX][axis-1], stepper_data_lookup[DIR_INDEX][axis-1], stepper_data_lookup[SW_INDEX][axis-1], stepper_data_lookup[SPEED_INDEX][axis-1], stepper_data_lookup[PPR_INDEX][axis-1], stepper_data_lookup[REDUCTION_INDEX][axis-1], stepper_data_lookup[ANG_INDEX][axis-1]);
        flag = 0;
    }

    delay(100);
}

// functions for arm motion

void step_angle(int axis, char mode, long ang, int stp, int dir, int sw, int spd, long ppr, long red, long max_ang) { // STEPS MOTORS BY CALCULATED ANGLE
  
    if (mode == 'I') { // calculates required steps and direction in incremental mode
        step_in = red*ppr*(ang / 360.0);

        if(ang == 0) {
        }
    
        else if(ang + angles[axis-1] < max_ang && ang + angles[axis-1]>0) { // if the desired move doesn't put the axis outside of its operating range
            angles[axis - 1] += ang;
        }

        else {
            if(ang>0) { // If the input command is to increase position, and last condition failed, we know that the upper arm range is exceeded, so move arm to max position (angle = max)
                step_in = (red*ppr/360.0)*(max_ang - angles[axis-1]);
                angles[axis-1] = max_ang; 
            }

            else { // otherwise, we know the command wants to try to move the rover past the limit switch, so home the axis (angle = 0)
                step_in = -(red*ppr/360.0)*angles[axis-1];
                angles[axis-1] = 0; 
            }
        }

        step_in = step_in*axisDir[axis-1];

        if(axis == 1) 
          a1IncMove();

        else if(axis == 2) 
          a2IncMove();

        else if(axis == 3) 
          a3IncMove();

        else if(axis == 4) 
          a4IncMove();

        else if(axis == 5) 
          a5IncMove();

        else if(axis == 6) 
          a6IncMove();   
    } 

   else if (mode == 'A') { // calculates required steps and direction in absolute mode
      
        if(ang<=0) { // negative input angle, home arm
            step_in = 0;
            angles[axis-1] = 0;
        }

        else if(ang>=max_ang) { // angle greater than maximum, move axis to maximum
            step_in = red*ppr/360.0*max_ang;
            angles[axis-1] = max_ang;
        }
    
        else { // angle within bounds, calculate the step input from angle and move axis
            step_in = red*ppr*ang/360.0;
            angles[axis - 1] = ang; 
        }

        step_in = step_in*axisDir[axis-1];

        if(axis == 1) 
          a1AbsMove();

        else if(axis == 2) 
          a2AbsMove();

        else if(axis == 3) 
          a3AbsMove();

        else if(axis == 4) 
          a4AbsMove();

        else if(axis == 5) 
          a5AbsMove();

        else if(axis == 6) 
          a6AbsMove();
   }
   
   a1LimTrigger();
   a2LimTrigger();
   a3LimTrigger();
   a4LimTrigger();
   a5LimTrigger();
   a6LimTrigger();        
   sendAngles();
}

// function for homing full arm 
void home_arm() {

    initializeHomingMotion();
    bool completeFlag = false;
    bool a1StopFlag = false;
    bool a2StopFlag = false;
    bool a3StopFlag = false;
    bool a4StopFlag = false;
    bool a5StopFlag = false;
    bool a6StopFlag = false;
    bool a1FinishFlag = false;
    bool a2FinishFlag = false;
    bool a3FinishFlag = false;
    bool a4FinishFlag = false;
    bool a5FinishFlag = false;
    bool a6FinishFlag = false;

    while (!completeFlag) {
      
          if(!a1FinishFlag) {
              if (!digitalRead(a1_sw)) {
                  a1.run();
              }

              else if(!a1StopFlag) {
                a1.setCurrentPosition(-homeComp[0]);
                a1.stop();
                a1.setMaxSpeed(a1_SPEED);
                a1.setAcceleration(a1_ACC);
                a1.moveTo(homeCompSteps[0]);
                a1StopFlag = true;
              }
          
              else if(a1.distanceToGo() != 0) {
                  a1.run();
              }
              
              else {
                 a1FinishFlag = true;
              }
          }

          if(!a2FinishFlag) {
              if (!digitalRead(a2_sw)) {
                  a2.run();
              }

              else if(!a2StopFlag) {
                a2.setCurrentPosition(-homeComp[1]);
                a2.stop();
                a2.setMaxSpeed(a2_SPEED);
                a2.setAcceleration(a2_ACC);
                a2.moveTo(homeCompSteps[1]);
                a2StopFlag = true;
              }
          
              else if(a2.distanceToGo() != 0) {
                  a2.run();
              }
              
              else {
                 a2FinishFlag = true;
              }
          }

          if(!a3FinishFlag) {
              if (!digitalRead(a3_sw)) {
                  a3.run();
              }

              else if(!a3StopFlag) {
                a3.setCurrentPosition(-homeComp[2]);
                a3.stop();
                a3.setMaxSpeed(a3_SPEED);
                a3.setAcceleration(a3_ACC);
                a3.moveTo(homeCompSteps[2]);
                a3StopFlag = true;
              }
          
              else if(a3.distanceToGo() != 0) {
                  a3.run();
              }
              
              else {
                 a3FinishFlag = true;
              }
          }

          if(!a4FinishFlag) {
              if (!digitalRead(a4_sw)) {
                  a4.run();
              }

              else if(!a4StopFlag) {
                a4.setCurrentPosition(-homeComp[3]);
                a4.stop();
                a4.setMaxSpeed(a4_SPEED);
                a4.setAcceleration(a4_ACC);
                a4.moveTo(homeCompSteps[3]);
                a4StopFlag = true;
              }
          
              else if(a4.distanceToGo() != 0) {
                  a4.run();
              }
              
              else {
                 a4FinishFlag = true;
              }
          }

          if(!a5FinishFlag) {
              if (!digitalRead(a5_sw)) {
                  a5.run();
              }

              else if(!a5StopFlag) {
                a5.setCurrentPosition(-homeComp[4]);
                a5.stop();
                a5.setMaxSpeed(a5_SPEED);
                a5.setAcceleration(a5_ACC);
                a5.moveTo(homeCompSteps[4]);
                a5StopFlag = true;
              }
          
              else if(a5.distanceToGo() != 0) {
                  a5.run();
              }
              
              else {
                 a5FinishFlag = true;
              }
          }

          if(!a6FinishFlag) {
              if (!digitalRead(a6_sw)) {
                  a6.run();
              }

              else if(!a6StopFlag) {
                a6.setCurrentPosition(-homeComp[5]);
                a6.stop();
                a6.setMaxSpeed(a6_SPEED);
                a6.setAcceleration(a6_ACC);
                a6.moveTo(homeCompSteps[5]);
                a6StopFlag = true;
              }
          
              else if(a6.distanceToGo() != 0) {
                  a6.run();
              }
              
              else {
                a6FinishFlag = true;
              }
          }

          if(a1FinishFlag && a2FinishFlag && a3FinishFlag && a4FinishFlag && a5FinishFlag && a6FinishFlag) { 
            completeFlag = true;
          }
    }

    for(int i=0; i<NUM_AXES; i++) {
      angles[i] = abs(homeCompAngles[i]);
    }
    
    sendAngles();  
}

// sets initial speed and acceleration, and sets target position
void initializeHomingMotion() {

    a1.moveTo(homePos[0]);
    a2.moveTo(homePos[1]);
    a3.moveTo(homePos[2]);
    a4.moveTo(homePos[3]);
    a5.moveTo(homePos[4]);
    a6.moveTo(homePos[5]);
    a1.setMaxSpeed(a1_home_SPEED);
    a2.setMaxSpeed(a2_home_SPEED);
    a3.setMaxSpeed(a3_home_SPEED);
    a4.setMaxSpeed(a4_home_SPEED);
    a5.setMaxSpeed(a5_home_SPEED);
    a6.setMaxSpeed(a6_home_SPEED);
    a1.setAcceleration(a1_home_ACC);
    a2.setAcceleration(a2_home_ACC);
    a3.setAcceleration(a3_home_ACC);
    a4.setAcceleration(a4_home_ACC);
    a5.setAcceleration(a5_home_ACC);
    a6.setAcceleration(a6_home_ACC); 
}

// sets speeds for each axis 
void initializeMotion() {

  a1.setMaxSpeed(a1_SPEED);
  a2.setMaxSpeed(a2_SPEED);
  a3.setMaxSpeed(a3_SPEED);
  a4.setMaxSpeed(a4_SPEED);
  a5.setMaxSpeed(a5_SPEED);
  a6.setMaxSpeed(a6_SPEED);
  a1.setAcceleration(a1_ACC);
  a2.setAcceleration(a2_ACC);
  a3.setAcceleration(a3_ACC);
  a4.setAcceleration(a4_ACC);
  a5.setAcceleration(a5_ACC);
  a6.setAcceleration(a6_ACC);
}

// sets current position of all axes to 0
void zeroAxes() { 

  a1.setCurrentPosition(0);
  a2.setCurrentPosition(0);
  a3.setCurrentPosition(0);
  a4.setCurrentPosition(0);
  a5.setCurrentPosition(0);
  a6.setCurrentPosition(0);
}

// Incremental motion of individual axis
void a1IncMove() {
   a1.move(step_in);
   while(a1.distanceToGo() !=0 && !digitalRead(a1_sw)) {
       a1.run();
   }
}

void a2IncMove() {
   a2.move(step_in);
   while(a2.distanceToGo() !=0 && !digitalRead(a2_sw)) {
       a2.run();
   }
}

void a3IncMove() {
   a3.move(step_in);
   while(a3.distanceToGo() !=0 && !digitalRead(a3_sw)) {
       a3.run();
   }
}

void a4IncMove() {
   a4.move(step_in);
   while(a4.distanceToGo() !=0 && !digitalRead(a4_sw)) {
       a4.run();
   }
}

void a5IncMove() {
   a5.move(step_in);
   while(a5.distanceToGo() !=0 && !digitalRead(a5_sw)) {
       a5.run();
   }
}

void a6IncMove() {
   a6.move(step_in);
   while(a6.distanceToGo() !=0 && !digitalRead(a6_sw)) {
       a6.run();
   }
}

// absolute motion of individual axis
void a1AbsMove() {
   a1.moveTo(step_in);
   while(a1.distanceToGo() !=0 && !digitalRead(a1_sw)) {
       a1.run();
   }
}

void a2AbsMove() {
   a2.moveTo(step_in);
   while(a2.distanceToGo() !=0 && !digitalRead(a2_sw)) {
       a2.run();
   }
}

void a3AbsMove() {
   a3.moveTo(step_in);
   while(a3.distanceToGo() !=0 && !digitalRead(a3_sw)) {
       a3.run();
   }
}

void a4AbsMove() {
   a4.moveTo(step_in);
   while(a4.distanceToGo() !=0 && !digitalRead(a4_sw)) {
       a4.run();
   }
}

void a5AbsMove() {
   a5.moveTo(step_in);
   while(a5.distanceToGo() !=0 && !digitalRead(a5_sw)) {
       a5.run();
   }
}

void a6AbsMove() {
   a6.moveTo(step_in);
   while(a6.distanceToGo() !=0 && !digitalRead(a6_sw)) {
       a6.run();
   }
}

// routine for if limit switches are triggered unexpectedly during a move
void a1LimTrigger() {

  if(digitalRead(a1_sw)) {
     int axis = 1;
     a1.setCurrentPosition(0);
     a1.stop();
     a1.move(homeComp[axis-1]);
     a1.setMaxSpeed(a1_home_SPEED);
     a1.setAcceleration(a1_home_ACC);
        
     while (a1.distanceToGo() != 0) {
          a1.run();
     }

     a1.setCurrentPosition(0);
     a1.setMaxSpeed(a1_SPEED);
     a1.setAcceleration(a1_ACC);
     angles[axis-1] = 0;
  }
}

void a2LimTrigger() {

  if(digitalRead(a2_sw)) {
     int axis = 2;
     a2.setCurrentPosition(0);
     a2.stop();
     a2.move(homeComp[axis-1]);
     a2.setMaxSpeed(a2_home_SPEED);
     a2.setAcceleration(a2_home_ACC);
        
     while (a2.distanceToGo() != 0) {
          a2.run();
     }

     a2.setCurrentPosition(0);
     a2.setMaxSpeed(a2_SPEED);
     a2.setAcceleration(a2_ACC);
     angles[axis-1] = 0;
  }
}

void a3LimTrigger() {

  if(digitalRead(a3_sw)) {
     int axis = 3;
     a3.setCurrentPosition(0);
     a3.stop();
     a3.move(homeComp[axis-1]);
     a3.setMaxSpeed(a3_home_SPEED);
     a3.setAcceleration(a3_home_ACC);
        
     while (a3.distanceToGo() != 0) {
          a3.run();
     }

     a3.setCurrentPosition(0);
     a3.setMaxSpeed(a3_SPEED);
     a3.setAcceleration(a3_ACC);
     angles[axis-1] = 0;
  }
}

void a4LimTrigger() {

  if(digitalRead(a4_sw)) {
     int axis = 4;
     a4.setCurrentPosition(0);
     a4.stop();
     a4.move(homeComp[axis-1]);
     a4.setMaxSpeed(a4_home_SPEED);
     a4.setAcceleration(a4_home_ACC);
        
     while (a4.distanceToGo() != 0) {
          a4.run();
     }

     a4.setCurrentPosition(0);
     a4.setMaxSpeed(a4_SPEED);
     a4.setAcceleration(a4_ACC);
     angles[axis-1] = 0;
  }
}

void a5LimTrigger() {

  if(digitalRead(a5_sw)) {
     int axis = 5;
     a5.setCurrentPosition(0);
     a5.stop();
     a5.move(homeComp[axis-1]);
     a5.setMaxSpeed(a5_home_SPEED);
     a5.setAcceleration(a5_home_ACC);
        
     while (a5.distanceToGo() != 0) {
          a5.run();
     }

     a5.setCurrentPosition(0);
     a5.setMaxSpeed(a5_SPEED);
     a5.setAcceleration(a5_ACC);
     angles[axis-1] = 0;
  }
}

void a6LimTrigger() {

  if(digitalRead(a6_sw)) {
     int axis = 6;
     a6.setCurrentPosition(0);
     a6.stop();
     a6.move(homeComp[axis-1]);
     a6.setMaxSpeed(a6_home_SPEED);
     a6.setAcceleration(a6_home_ACC);
        
     while (a6.distanceToGo() != 0) {
          a6.run();
     }

     a6.setCurrentPosition(0);
     a6.setMaxSpeed(a6_SPEED);
     a6.setAcceleration(a6_ACC);
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