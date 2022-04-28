/*
Created By: Tate Kolton, Rowan Zawadzki
Created On: December 21, 2021
Description: Main code for driving 6 axis robotic arm using Nintendo Switch pro (or any XInput type) Controller
*/
 
#include <AccelStepper.h>

// general parameters
#define NUM_AXES 6
#define NUM_AXES_EX_WRIST 4
#define NUM_AXES_EFF 8
#define NUM_PARAMS 7
#define ON 0
#define OFF 1
#define SW_ON 0
#define SW_OFF 1
#define FWD 1
#define REV 0

// motor pins
// **NOTE** axis 3 and 4 are switched and axis 5 and 6 are switched. i.e. stepPins[3] is not axis 4, it is axis 3
int stepPins[8] =   {11, 9, 5, 7, 1, 3, 1, 3};
int dirPins[8] =    {10, 8, 4, 6, 0, 2, 0, 2};

// limit switch pins
int limPins[6] = {23, 22, 20, 21, 18, 19};

// pulses per revolution for motors
long ppr[6] = {400, 400, 400, 400, 400, 400};

// Gear Reduction Ratios
float red[6] = {30.0, 161.0, 44.8, 100, 57.34, 57.34};

// Encoder pulses per revolution
long pprEnc = 512;

// Motor speeds and accelerations
int maxSpeed[8] = {1200, 1800, 3000, 2500, 2200, 2200, 2200, 2200};
int maxAccel[8] = {900, 3000, 4000, 3000, 5000, 5000, 5000, 5000};
int homeSpeed[8] = {500, 1200, 600, 400, 2000, 2000, 2000, 2000}; // {500, 1500, 700, 1200, 1200, 1200, 1200, 1200};
int homeAccel[8] = {500, 2000, 1500, 1000, 1500, 1500, 1500, 1500}; //{500, 2000, 1000, 1500, 1500, 1500, 1500, 1500};

// Range of motion (degrees) for each axis
int maxAngles[6] = {180, 70, 180, 120, 140, 100};

const unsigned long readInterval = 10;
unsigned long currentTime;
unsigned long previousTime = 0;

// stepper motor objects for AccelStepper library
AccelStepper steppers[8];

// variable declarations
long max_steps[] = {red[0]*maxAngles[0]/360.0*ppr[0], red[1]*maxAngles[1]/360.0*ppr[1], red[2]*maxAngles[2]/360.0*ppr[2], red[3]*maxAngles[3]/360.0*ppr[3], red[4]*maxAngles[4]/360.0*ppr[4], red[5]*maxAngles[5]/360.0*ppr[5]};
int axisDir[8] = {1, -1, 1, -1, 1, 1, -1, 1}; 
int currentAxis = 1;
int runFlags[] = {0, 0, 0, 0, 0, 0};
int i;
static const char fwdValues[] = {'a', 'b'};
static const char revValues[] = {'c', 'd'};
static const char fwdWrist[] = {'f', 'g'};
static const char revWrist[] = {'h', 'i'};
static const char axisSelect[] = {'m', 'm', 'n', 'n'};
static const char releaseChars[] = {'q', 'r', 'q', 'r'};
static const char wristRelease[] = {'j', 'k'};
static const char homeVal = 'z';
char garbage;
bool initFlag = false;

// variables for homing / arm calibration
long homePosConst = -99000;
long homePos[] = {axisDir[0]*homePosConst, axisDir[1]*homePosConst, axisDir[2]*homePosConst, axisDir[3]*homePosConst, axisDir[4]*homePosConst, axisDir[5]*homePosConst, axisDir[6]*homePosConst, axisDir[7]*homePosConst};
long homeCompAngles[] = {axisDir[0]*90, axisDir[1]*5, axisDir[2]*93, axisDir[3]*12, axisDir[4]*102, axisDir[5]*102, axisDir[6]*80, axisDir[7]*80};
long homeCompConst[] = {2000, 2000, 1000, 500, 500, 500, 500, 500};
long homeComp[] = {axisDir[0]*homeCompConst[0], axisDir[1]*homeCompConst[1], axisDir[2]*homeCompConst[2], axisDir[3]*homeCompConst[3], axisDir[4]*homeCompConst[4], axisDir[5]*homeCompConst[5], axisDir[6]*homeCompConst[6], axisDir[7]*homeCompConst[7]};
long homeCompSteps[] = {homeCompAngles[0]*red[0]*ppr[0]/360.0, homeCompAngles[1]*red[1]*ppr[1]/360.0, homeCompAngles[2]*red[2]*ppr[2]/360.0, homeCompAngles[3]*red[3]*ppr[3]/360.0, homeCompAngles[4]*red[4]*ppr[4]/360.0, homeCompAngles[5]*red[5]*ppr[5]/360.0, homeCompAngles[6]*red[4]*ppr[4]/360.0, homeCompAngles[7]*red[5]*ppr[5]/360.0};
char value;

// values for changing speed
const int maxSpeedIndex = 2;
int speedVals[maxSpeedIndex+1][NUM_AXES_EFF] = {{600, 900, 1500, 1250, 1050, 1050, 1050, 1050}, {900, 1200, 2000, 1665, 1460, 1460, 1460, 1460}, {1200, 1800, 3000, 2500, 2200, 2200, 2200, 2200}};
int speedIndex = maxSpeedIndex;
char faster = 'l';
char slower = 's';
// setup function to initialize pins and provide initial homing to the arm
void setup() {

  Serial.begin(9600);

  // initializing steppers
  for(int i=0; i<NUM_AXES_EFF; i++) {
    steppers[i] = AccelStepper(1, stepPins[i], dirPins[i]);
    steppers[i].setMinPulseWidth(200);
  }

  // initializing limit switches
  for(i = 0; i<NUM_AXES_EFF; i++) {
    pinMode(limPins[i], INPUT_PULLUP);
  }

  // initializing step pins
  for(i = 0; i<NUM_AXES_EFF; i++) {
    pinMode(stepPins[i], OUTPUT);
  }

  // initializing direction pins
  for(i = 0; i<NUM_AXES_EFF; i++) {
    pinMode(dirPins[i], OUTPUT);
  }

  // initializing max speeds and accelerations for motors

  Serial.println("HOME ARM TO UNLOCK MOTION");
  while(!initFlag) {
    if(Serial.available()>0){
      char init = Serial.read();
      if(init == 'z') {
        initFlag = true;
      }
    }
    delay(readInterval);
  }
}

// main loop where motion functions are called and serial communication occurs
void loop() {

  currentTime = millis();

  if((currentTime - previousTime) > readInterval) {
    if(Serial.available() > 0){
      
      value = Serial.read();
     controllerParse(value);
    }
    
    previousTime = currentTime;
  }

  runSteppers();
}

void controllerParse(char data) {
  
  if(data == 'a') {
    runAxes(FWD, currentAxis);
  }
  else if(data == 'b') {
    runAxes(REV, currentAxis);
  }
  else if(data == 'c') {
    runAxes(FWD, currentAxis+1);
  }
  else if(data == 'd') {
    runAxes(REV, currentAxis+1);
  }
  else if(data == 'f') {
    runWrist(FWD, 5);
  }
  else if(data == 'h') {
    runWrist(REV, 5);
  }
  else if(data == 'g') {
    runWrist(FWD, 6);
  }
  else if(data == 'i') {
    runWrist(REV, 6);
  }
 else if(data == 'z') {
    homeWrist();
    home_arm();
  }
  else {
    releaseEvent(data);
    changeAxis(data, currentAxis);
    changeSpeed(data);
  }
}

void runAxes(int dir, int axis) { // assigns run flags to indicate forward / reverse motion and sets target position

  if(axis == 3) {
    dir = !dir;
  }
  
  if(runFlags[axis-1] == 1 && dir == FWD) {
  }

  else if(runFlags[axis-1] == -1 && dir == REV) {
  }
    
  else if(dir == FWD) {
    steppers[axis-1].moveTo(max_steps[axis-1]*axisDir[axis-1]);
    runFlags[axis-1] = 1;
  }

  else {
    steppers[axis-1].moveTo(0);
    runFlags[axis-1] = -1;
  }
}

void runWrist(int dir, int axis) {

  if(axis == 5) { // axis 5 motion -> both wrist motors spin in opposite directions
    if(runFlags[5] == 1 && dir == FWD) {
    }

    else if(runFlags[5] == -1 && dir == REV) {
    }
    
    else if(dir == FWD) {
      steppers[6].moveTo(axisDir[6]*max_steps[5]);
      steppers[7].moveTo(axisDir[7]*max_steps[5]);
      runFlags[5] = 1;
    }

    else {
      steppers[6].moveTo(0);
      steppers[7].moveTo(0);
      runFlags[5] = -1;
    } 
  }

  else if(axis == 6) { // axis 6 motion -> both wrist motors spin in same direction
    dir = !dir;
    if(runFlags[4] == 1 && dir == FWD) {
    }

    else if(runFlags[4] == -1 && dir == REV) {
    }
    
    else if(dir == FWD) {
      steppers[4].moveTo(axisDir[4]*max_steps[4]);
      steppers[5].moveTo(axisDir[5]*max_steps[4]);
      runFlags[4] = 1;
    }

    else {
      steppers[4].moveTo(0);
      steppers[5].moveTo(0);
      runFlags[4] = -1;
    }
  } 
}

void changeAxis(char value, int axis) { // when user hits specified button, axis targets change

  for(i=0; i<NUM_AXES_EX_WRIST; i+=2) {
    if(axisSelect[i] == value) {
      if(axis == i+1) {
      }

      else {
        currentAxis = i+1;
        zeroRunFlags();
      }
    }
  }
}

void releaseEvent(char value) { // when user releases a joystick serial sends a character

  for(i=currentAxis-1; i<=currentAxis; i++) {
    if(releaseChars[i] == value) {
        steppers[i].stop();
      }
      runFlags[i] = 0;
    }

  if(value == wristRelease[0]) { // axis 5 released
    steppers[6].stop();
    steppers[7].stop();
    runFlags[5] = 0;
  }

  else if(value == wristRelease[1]) { // axis 6 released
    steppers[4].stop();
    steppers[5].stop();
    runFlags[4] = 0;
  }
}

void zeroRunFlags() { // when user changes axis to control on switch, slow current moving axes to a stop and reset run flags (all motors stagnant)

  for(i = 0; i<NUM_AXES_EX_WRIST; i++) {
    steppers[i].stop();
  }

  for(i=0; i<NUM_AXES_EX_WRIST; i++) {
    runFlags[i] = 0;
  }
}

void home_arm() {
  
  bool stopFlags[4] = {false, false, false, false};
  bool finishFlags[4] = {false, false, false, false};
  bool completeFlag = false;
  int count = 0;
  initializeHomingMotion();
  
  while(!completeFlag) {

    for(i = 0; i < NUM_AXES_EX_WRIST; i++) {

      if(!finishFlags[i]) {

        if(!digitalRead(limPins[i]) && !stopFlags[i]) {
          steppers[i].run();
        }

        else if(!stopFlags[i]) {
          steppers[i].setCurrentPosition(-homeComp[i]);
          steppers[i].stop();
          steppers[i].setMaxSpeed(maxSpeed[i]/2);
          steppers[i].setAcceleration(homeAccel[i]);
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
    if(count == NUM_AXES_EX_WRIST) {
      completeFlag = true;
    }
  }

  initializeMotion();
}

void homeWrist() {

initializeWristHomingMotion();
initializeHomingMotion();

  bool stopFlags[2] = {false, false};
  bool calibFlags[2] = {false, false};
  bool completeFlag = false;

  while(!completeFlag) {

    if(!digitalRead(limPins[5]) && !stopFlags[0]) {
      steppers[6].run();
      steppers[7].run();
    }

    else if(!stopFlags[0]) {
      steppers[6].setCurrentPosition(-homeComp[6]);
      steppers[7].setCurrentPosition(-homeComp[7]);
      steppers[6].stop();
      steppers[7].stop();
      steppers[6].setMaxSpeed(maxSpeed[6]);
      steppers[7].setMaxSpeed(maxSpeed[7]);
      steppers[6].setAcceleration(maxAccel[6]);
      steppers[7].setAcceleration(maxAccel[7]);
      steppers[6].moveTo(homeCompSteps[6]);
      steppers[7].moveTo(homeCompSteps[7]);
      stopFlags[0] = true;      
    }

    else if(steppers[6].distanceToGo()!= 0 && !calibFlags[0]) {
      steppers[6].run();
      steppers[7].run();
    }

    else if(!calibFlags[0]) {
      calibFlags[0] = true;
    }

    else if(!digitalRead(limPins[4]) && !stopFlags[1]) {
      steppers[4].run();
      steppers[5].run();
    }

    else if(!stopFlags[1]) {
      steppers[4].setCurrentPosition(-homeComp[4]);
      steppers[5].setCurrentPosition(-homeComp[5]);
      steppers[4].stop();
      steppers[5].stop();
      steppers[4].setMaxSpeed(maxSpeed[4]);
      steppers[5].setMaxSpeed(maxSpeed[5]);
      steppers[4].setAcceleration(maxAccel[4]);
      steppers[5].setAcceleration(maxAccel[5]);
      steppers[4].moveTo(homeCompSteps[4]);
      steppers[5].moveTo(homeCompSteps[5]);
      stopFlags[1] = true;      
    }

    else if(steppers[4].distanceToGo()!= 0 && !calibFlags[1]) {
      steppers[4].run();
      steppers[5].run();
    }

    else if(!calibFlags[1]) {
      calibFlags[1] = true;
      completeFlag = true;
    }
  }
  zeroRunFlags();
}

// sets initial speed and acceleration, and sets target position
void initializeHomingMotion() {

  for(i = 0; i<NUM_AXES_EX_WRIST; i++) {

    steppers[i].setMaxSpeed(homeSpeed[i]);
    steppers[i].setAcceleration(homeAccel[i]);
    steppers[i].setCurrentPosition(0);
    steppers[i].move(homePos[i]);
  }
}

void initializeWristHomingMotion() {

  for(i = 4; i<NUM_AXES_EFF; i++) {

    steppers[i].setMaxSpeed(homeSpeed[i]);
    steppers[i].setAcceleration(homeAccel[i]);
    steppers[i].setCurrentPosition(0);
    steppers[i].move(homePos[i]);
  }
}

// sets speeds for each axis
void initializeMotion() {

  for(i = 0; i<NUM_AXES_EFF; i++) {
    steppers[i].setMaxSpeed(speedVals[maxSpeedIndex][i]);
    steppers[i].setAcceleration(maxAccel[i]);
  }
}

// sets current position of all axes to 0
void zeroAxes() {

  for(i = 0; i<NUM_AXES_EFF; i++) {
    steppers[i].setCurrentPosition(0);
  }
}

// Runs steppers on all axes
void runSteppers() {
    
  for(i = 0; i<NUM_AXES_EFF; i++) {
    steppers[i].run();
  }
}

void changeSpeed(char speedVal) {
  
  if(speedVal == faster){
    if(speedIndex < maxSpeedIndex) {
      speedIndex++;
      for(i=0;i<NUM_AXES_EFF;i++) {
        steppers[i].setMaxSpeed(speedVals[speedIndex][i]);
      }
    }
  }

  else if(speedVal == slower) {
    if(speedIndex > 0) {
      speedIndex--;
      for(i=0;i<NUM_AXES_EFF;i++) {
        steppers[i].setMaxSpeed(speedVals[speedIndex][i]);
      }
    }
  }
}
