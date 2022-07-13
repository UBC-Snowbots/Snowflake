/*
Created By: Tate Kolton, Rowan Zawadzki
Created On: December 21, 2021
Updated: April 27, 2022
Description: Main firmware for driving a 6 axis arm via ROS on a teensy 4.1 MCU
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

int curEncSteps[NUM_AXES], cmdEncSteps[NUM_AXES];

static const char release = 'R';
static const char move = 'M';
static const char change = 'A';
static const char speed = 'S';
static const char right = 'R';
static const char left = 'L';
static const char up = 'U';
static const char down = 'D';
static const char wrist = 'W';

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
char faster = 'U';
char slower = 'D';


void setup() { // setup function to initialize pins and provide initial homing to the arm

  Serial.begin(9600);

  // initializes step pins, direction pins, limit switch pins, and stepper motor objects for accelStepper library
  for(i = 0; i<NUM_AXES_EFF; i++) {
    pinMode(dirPins[i], OUTPUT);
    pinMode(stepPins[i], OUTPUT);
    pinMode(limPins[i], INPUT_PULLUP);
    steppers[i] = AccelStepper(1, stepPins[i], dirPins[i]);
    steppers[i].setMinPulseWidth(200);
  }
  // waits for user to press "home" button before rest of functions are available
  waitForHome();
}

void loop()
{
    currentTime = millis();

    if((currentTime - previousTime) > readInterval) {
        commandArm()
        previousTime = currentTime;
    }

  runSteppers();
}

void commandArm()
{
    String inData = "";
    char recieved = "";
    if(Serial.available())
    {
        do {
            recieved = Serial.read();
            inData += recieved;
        } while(recieved != '\n');
    }

    if(recieved == '\n')
    {
        parseMessage(inData);
    }
}

void parseMessage(String inMsg)
{
    String function = inMsg.substring(0, 2);
    
    if(function == "MT")
    {
        cartesianCommands(inMsg);
    }

    else if(function == "JM")
    {
        jointCommands(inMsg);
    }

    else if(function == "EE")
    {
        endEffectorCommands(inMsg);
    }

    else if(function == "DM")
    {
        drillCommands(inMsg);
    }

    else if(function == "HM")
    {
        homeArm();
    }
}

//****//CARTESIAN MODE FUNCTIONS//****//

void cartesianCommands()
{
    // read current joint positions
    readEncPos(curEncSteps);

    // update host with joint positions
    String msg = String("JP") + String("A") + String(curEncSteps[0]) + String("B") + String(curEncSteps[1]) + String("C") + String(curEncSteps[2])
                + String("D") + String(curEncSteps[3]) + String("E") + String(curEncSteps[4]) + String("F") + String(curEncSteps[5]) + String("\n");
    Serial.print(msg);

    // get new position commands
    int msgIdxJ1 = inData.indexOf('A');
    int msgIdxJ2 = inData.indexOf('B');
    int msgIdxJ3 = inData.indexOf('C');
    int msgIdxJ4 = inData.indexOf('D');
    int msgIdxJ5 = inData.indexOf('E');
    int msgIdxJ6 = inData.indexOf('F');
    cmdEncSteps[0] = inData.substring(msgIdxJ1 + 1, msgIdxJ2).toInt();
    cmdEncSteps[1] = inData.substring(msgIdxJ2 + 1, msgIdxJ3).toInt();
    cmdEncSteps[2] = inData.substring(msgIdxJ3 + 1, msgIdxJ4).toInt();
    cmdEncSteps[3] = inData.substring(msgIdxJ4 + 1, msgIdxJ5).toInt();
    cmdEncSteps[4] = inData.substring(msgIdxJ5 + 1, msgIdxJ6).toInt();
    cmdEncSteps[5] = inData.substring(msgIdxJ6 + 1).toInt();

    // update target joint positions
    readEncPos(curEncSteps);
    for (int i = 0; i < NUM_AXES; i++)
    { 
        int diffEncSteps = cmdEncSteps[i] - curEncSteps[i];
        if (abs(diffEncSteps) > 2)
        {
        int diffMotSteps = diffEncSteps / ENC_MULT[i];
        if (diffMotSteps < MOTOR_STEPS_PER_REV[i])
        {
            // for the last rev of motor, introduce artificial decceleration
            // to help prevent overshoot
            diffMotSteps = diffMotSteps / 2;
        }
        stepperJoints[i].move(diffMotSteps);
        }
    }
}

// parses which commands to execute when in joint space mode
void jointCommands(String inMsg)
{
    char function = inMsg[2];
    char detail1 = inMsg[3];

    switch(function) 
    {
        case release: releaseEvent(detail1, inMsg[4]); break;
        case speed: changeSpeed(detail1); break;
        case axis: changeAxis(detail1); break;
        case move: jointMovement(detail1, inMsg[4]); break;
    }
}

// Sets movement target positions when in joint space mode
void jointMovement(char joystick, char dir)
{
    if(joystick == wrist)
    {
        switch(dir)
        {
            case up: runWrist(FWD, 5); break;
            case down: runWrist(REV, 5); break;
            case left: runWrist(FWD, 6); break;
            case right: runWrist(REV, 6); break;
        }
    }

    else if(joystick == left)
    {
        switch(dir)
        {
            case left: runAxes(FWD, currentAxis); break;
            case right: runAxes(REV, currentAxis); break;
        }
    }

    else
    {
        switch(dir)
        {
            case up: runAxes(FWD, currentAxis+1); break;
            case down: runAxes(REV, currentAxis+1); break;
        }
    }
}

//****//ENCODER RELATED FUNCTIONS//****//

void readEncPos(int* encPos)
{
  encPos[0] = J1encPos.read() * ENC_DIR[0];
  encPos[1] = J2encPos.read() * ENC_DIR[1];
  encPos[2] = J3encPos.read() * ENC_DIR[2];
  encPos[3] = J4encPos.read() * ENC_DIR[3];
  encPos[4] = J5encPos.read() * ENC_DIR[4];
  encPos[5] = J6encPos.read() * ENC_DIR[5];
}

//****//JOINT SPACE MODE FUNCTIONS//****//

// sets target position for axes in joint space mode
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

void runWrist(int dir, int axis) { // assigns target position for selected axis based on user input. 

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

void changeAxis(int dir) { // when user hits specified button, axis targets change

    if((dir == up) && (currentAxis == 1))
    {
        currentAxis = 3;
        zeroRunFlags();
    }

    else if((dir == down) && (currentAxis == 3))
    {
        currentAxis = 1;
        zeroRunFlags();
    }
}

void releaseEvent(char joystick, char dir) { // when user releases a joystick serial sends a character

    if(joystick == wrist)
    {
        if ((dir == up) || (dir == down))
        {
            steppers[6].stop();
            steppers[7].stop();
            runFlags[5] = 0;
        }

        else
        {
            steppers[4].stop();
            steppers[5].stop();
            runFlags[4] = 0; 
        }
    }

    else if(joystick == left)
    {
        steppers[currentAxis-1].stop();
        runFlags[currentAxis-1].stop();
    }

    else 
    {
        steppers[currentAxis].stop();
        runFlags[currentAxis].stop();
    }
}

void changeSpeed(char speedVal) { // changes speed of all axes based on user input
  
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

void zeroRunFlags() { // when user changes axis to control on switch, slow current moving axes to a stop and reset run flags (all motors stagnant)

  for(i = 0; i<NUM_AXES_EX_WRIST; i++) {
    steppers[i].stop();
  }

  for(i=0; i<NUM_AXES_EX_WRIST; i++) {
    runFlags[i] = 0;
  }
}








//****// ARM CALIBRATION FUNCTIONS//****//

void homeArm() { // main function for full arm homing
  initializeWristHomingMotion();
  homeWrist();
  initializeHomingMotion();
  homeBase();
  initializeMotion();
}

void homeBase() { // homes axes 1-4
  
  bool stopFlags[4] = {false, false, false, false};
  bool finishFlags[4] = {false, false, false, false};
  bool completeFlag = false;
  int count = 0;
  
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

void homeWrist() { // homes axes 5-6

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
}

void initializeHomingMotion() { // sets homing speed and acceleration for axes 1-4 and sets target homing position

  for(i = 0; i<NUM_AXES_EX_WRIST; i++) {

    steppers[i].setMaxSpeed(homeSpeed[i]);
    steppers[i].setAcceleration(homeAccel[i]);
    steppers[i].setCurrentPosition(0);
    steppers[i].move(homePos[i]);
  }
}

void initializeWristHomingMotion() { // sets homing speed and acceleration for axes 5-6 and sets target homing position

  for(i = 4; i<NUM_AXES_EFF; i++) {

    steppers[i].setMaxSpeed(homeSpeed[i]);
    steppers[i].setAcceleration(homeAccel[i]);
    steppers[i].setCurrentPosition(0);
    steppers[i].move(homePos[i]);
  }
}

void initializeMotion() { // sets main program speeds for each axis

  for(i = 0; i<NUM_AXES_EFF; i++) {
    steppers[i].setMaxSpeed(speedVals[maxSpeedIndex][i]);
    steppers[i].setAcceleration(maxAccel[i]);
  }
}

void zeroAxes() { // sets current position of all axes to 0

  for(i = 0; i<NUM_AXES_EFF; i++) {
    steppers[i].setCurrentPosition(0);
  }
}

void runSteppers() { // runs all stepper motors (if no target position has been assigned, stepper will not move)
    
  for(i = 0; i<NUM_AXES_EFF; i++) {
    steppers[i].run();
  }
}

void waitForHome() { // stops arm motion until user homes arm after firmware is flashed

    String inData = "";
    char recieved = "";
    bool initFlag = false;
    while(!initFlag) {
        
        if(Serial.available())
        {
            do {
                recieved = Serial.read();
                inData += recieved;
            } while(recieved != '\n');
        }

        if(recieved == '\n')
        {
            if(inData = "HM")
            {
                homeArm();
                initFlag = true;
            }
        }
        delay(readInterval);
    }
}