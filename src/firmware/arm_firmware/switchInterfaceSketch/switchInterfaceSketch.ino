/*
Created By: Tate Kolton, Rowan Zawadzki
Created On: December 21, 2021
Description: Main code for driving 6 axis robotic arm using Nintendo Switch pro (or any XInput type) Controller
*/
 
#include <AccelStepper.h>

// general parameters
#define NUM_AXES 1
#define NUM_PARAMS 7
#define ON 0
#define OFF 1
#define SW_ON 0
#define SW_OFF 1
#define FWD 1
#define REV 0

// motor pins
int stepPins[6] = {2, 6, 99, 8, 10, 12};
int dirPins[6] = {4, 7, 9, 11, 13, 14};

// limit switch pins
int limPins[6] = {3, 16, 17, 18, 19, 20};

// pulses per revolution for motors
long ppr[6] = {3200, 400, 400, 400, 400, 400};

// Gear Reduction Ratios
long red[6] = {30, 161, 150, 70, 45, 45};

// Encoder pulses per revolution
long pprEnc = 512;

// Motor speeds and accelerations
int maxSpeed[6] = {5000, 1000, 3000, 3000, 3000, 3000};
int maxAccel[6] = {5000, 1000, 4000, 4000, 4000, 4000};
int homeSpeed[6] = {1000, 1000, 1000, 1000, 1000, 1000};
int homeAccel[6] = {2000, 2000, 2000, 2000, 2000, 2000};

// Range of motion (degrees) for each axis
long maxAngles[6] = {180, 120, 120, 120, 120, 120};

const unsigned long readInterval = 20;
unsigned long currentTime;
unsigned long previousTime = 0;

// stepper motor objects for AccelStepper library
AccelStepper steppers[6];

// variable declarations
long max_steps[6] = {red[0]*maxAngles[0]/360.0*ppr[0], red[1]*maxAngles[1]/360.0*ppr[1], red[2]*maxAngles[2]/360.0*ppr[2], red[3]*maxAngles[3]/360.0*ppr[3], red[4]*maxAngles[4]/360.0*ppr[4], red[5]*maxAngles[5]/360.0*ppr[5]};
int axisDir[6] = {1, -1, 1, 1, 1, 1};
int currentAxis = 1;
int runFlags = {0, 0, 0, 0, 0, 0};
int i;
char fwdValues[] = {'a', 'c', 'e', 'g', 'i', 'k'};
char revValues[] = {'b', 'd', 'f', 'h', 'j', 'l'};
char axisSelect[] = {'m', 'm', 'n', 'n', 'o', 'o'};
char releaseChars[] = {'q', 'r', 'q', 'r', 'q', 'r'};
char homeVal = 'z';
char garbage;
char garbageChar = 'p';

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

  // initializing steppers
  for(int i=0; i<NUM_AXES; i++) {
    steppers[i] = AccelStepper(1, stepPins[i], dirPins[i]);
    steppers[i].setMinPulseWidth(100);
  }

  // initializing limit switches
  for(i = 0; i<NUM_AXES; i++) {
    pinMode(limPins[i], INPUT_PULLUP);
  }

  // initializing step pins
  for(i = 0; i<NUM_AXES; i++) {
    pinMode(stepPins[i], OUTPUT);
  }

  // initializing direction pins
  for(i = 0; i<NUM_AXES; i++) {
    pinMode(dirPins[i], OUTPUT);
  }

  // initializing max speeds and accelerations for motors
  initializeMotion();

  Serial.println("ALL SYSTEMS GO");
}

// main loop where motion functions are called and serial communication occurs
void loop() {

  currentTime = millis();

  if((currentTime - previousTime) > readInterval) {
    controllerParse();
    previousTime = currentTime;
  }

  runCurrentSteppers(currentAxis);
}

void controllerParse() {
  char value = Serial.read();

  switch(value) {

    case(fwdValues[currentAxis-1]):
    runAxes(FWD, currentAxis);
    break;

    case(revValues[currentAxis-1]):
    runAxes(REV, currentAxis);
    break;

    case(fwdValues[currentAxis]):
    runAxes(FWD, currentAxis + 1);
    break;

    case(revValues[currentAxis]):
    runAxes(REV, currentAxis + 1);
    break;

    case(relaseEvent(value)):
    break;

    case(changeAxis(value, currentAxis)):
    break;

    case(homeVal): 
    home_arm();
    break;
  }
}

void runAxes(int dir, int axis) { // assigns run flags to indicate forward / reverse motion and sets target position

  if(runFlags[axis-1] == 1 && dir == FWD) {
  }

  else if(runFlags[axis-1] == -1 && dir == REV) {
  }
    
  else if(dir == FWD) {
    steppers[axis-1].moveTo(axisDir[axis-1]*max_steps[axis-1]);
    runFlags[axis-1] = 1;
  }

  else {
    steppers[axis-1].moveTo(axisDir[axis-1]*max_steps[axis-1]);
    runFlags[axis-1] = -1;
  }
}

char changeAxis(char value, int axis) { // when user hits specified button, axis targets change

  for(i=0; i<NUM_AXES; i+=2) {
    if(axisSelect[i] == value) {
      if(axis == i+1) {
        return value;
      }

      else {
        currentAxis = i+1;
        zeroRunFlags();
        return value;
      }
    }
  }
}

char releaseEvent(char value) { // when user releases a joystick serial sends a character

  for(i=currentAxis-1; i<=currentAxis; i++) {
    if(releaseChars[i] == value) {
      steppers[i].stop();
      runFlags[i] = 0;
      return value;
    }
  }
  return garbageChar;
}

void zeroRunFlags() { // when user changes axis to control on switch, slow current moving axes to a stop and reset run flags (all motors stagnant)

  for(i = 0; i<NUM_AXES; i++) {
    steppers[i].stop();
    runFlags[i] = 0;
  }

  for(i = 0; i < NUM_AXES; i++) {
    if(steppers[i].distanceToGo() != 0) {
      steppers[i].run();
    }
  }

  while(Serial.available() > 0) {
    garbage = Serial.read();
  }
}

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

  initializeMotion();
  zeroRunFlags();
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

void runCurrentSteppers(int current) {
    
  steppers[current-1].run();
  steppers[current].run();
}
    
