/*
Created By: Tate Kolton, Rowan Zawadzki
Created On: December 21, 2021
Updated: April 27, 2022
Description: Main firmware for driving a 6 axis arm via ROS on a teensy 4.1 MCU
*/
 
#include <AccelStepper.h>
#include <HX711.h>

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

static const char release = 'R';
static const char move = 'M';
static const char change = 'A';
static const char speed = 'S';
static const char right = 'R';
static const char left = 'L';
static const char up = 'U';
static const char down = 'D';
static const char wrist = 'W';
static const char garbage = 'G';
static const char faster = 'U';
static const char slower = 'D';
static const char open = left;
static const char close = right;
static const char prepare = 'P';
static const char collect = 'C';
static const char deposit = 'D';
static const char manual = 'M';
static const char drillRelease = 'X';

// Encoder Variables

int curEncSteps[NUM_AXES], cmdEncSteps[NUM_AXES];
int pprEnc = 512;
int ENC_DIR[6] = {1, 1, -1, -1, -1, -1};
const float ENC_MULT[] = {5.12, 5.12, 5.12, 5.12, 5.12, 5.12};

// Motor variables
int stepPins[8] =   {6, 8, 10, 2, 12, 25, 12, 25};
int dirPins[8] =    {5, 7, 9, 1, 11, 24, 11, 24};
int stepPinsIK[6] = {6, 8, 2, 10, 25, 12};
int dirPinsIK[6] = {5, 7, 1, 9, 24, 11};
int encPinA[6] = {17, 38, 40, 36, 15, 13};
int encPinB[6] = {16, 37, 39, 35, 14, 41};

// end effector variables
const int maxForce = 30; // needs to be checked
const float calibrationFactor = -111.25
float force;
HX711 scale;
int calPos = 0;
int closePos = 0;
int openPos = 2000; // needs to be calibrated
int EEstepPin = 20;
int EEdirPin = 21;
int speedEE = 1000;
int accEE = 5000;
const int MOTOR_DIR_EE = 1;

// limit switch pins
int limPins[6] = {18, 19, 21, 20, 23, 22};

// pulses per revolution for motors
long ppr[6] = {400, 400, 400, 400, 400, 400};

// Gear Reduction Ratios
float red[6] = {50.0, 161.0, 44.8, 93.07, 57.34, 57.34};

// Motor speeds and accelerations
int maxSpeed[8] = {1200, 1800, 3000, 2500, 2200, 2200, 2200, 2200};
int maxAccel[8] = {900, 3000, 4000, 3000, 5000, 5000, 5000, 5000};
int homeSpeed[8] = {500, 1200, 600, 400, 2000, 2000, 2000, 2000}; // {500, 1500, 700, 1200, 1200, 1200, 1200, 1200};
int homeAccel[8] = {500, 2000, 1500, 1000, 1500, 1500, 1500, 1500}; //{500, 2000, 1000, 1500, 1500, 1500, 1500, 1500};

// Range of motion (degrees) for each axis
int maxAngles[6] = {180, 140, 180, 120, 140, 100};

// Time variables
const unsigned long readInterval = 10;
const unsigned long currentTime;
const unsigned long previousTime = 0;
const unsigned long previousTimeEE = 0;
const unsigned long timeIntervalEE = 100;

// stepper motor objects for AccelStepper library
AccelStepper steppers[8];
AccelStepper endEff(1, EEstepPin, EEdirPin);
AccelStepper steppersIK[8];
Encoder encoders[6];

// variable declarations
long max_steps[] = {red[0]*maxAngles[0]/360.0*ppr[0], red[1]*maxAngles[1]/360.0*ppr[1], red[2]*maxAngles[2]/360.0*ppr[2], red[3]*maxAngles[3]/360.0*ppr[3], red[4]*maxAngles[4]/360.0*ppr[4], red[5]*maxAngles[5]/360.0*ppr[5]};
int axisDir[8] = {1, -1, 1, -1, 1, 1, -1, 1}; 
int currentAxis = 1;
int runFlags[] = {0, 0, 0, 0, 0, 0};
int i;
bool initFlag = false;
bool jointFlag = true;
bool IKFlag = false;

// variables for homing / arm calibration
long homePosConst = -99000;
long homePos[] = {axisDir[0]*homePosConst, axisDir[1]*homePosConst, axisDir[2]*homePosConst, axisDir[3]*homePosConst, axisDir[4]*homePosConst, axisDir[5]*homePosConst, axisDir[6]*homePosConst, axisDir[7]*homePosConst};
long homeCompAngles[] = {axisDir[0]*93, axisDir[1]*5, axisDir[2]*93, axisDir[3]*12, axisDir[4]*102, axisDir[5]*102, axisDir[6]*80, axisDir[7]*80};
long homeCompConst[] = {500, 2000, 1000, 500, 500, 500, 500, 500};
long homeComp[] = {axisDir[0]*homeCompConst[0], axisDir[1]*homeCompConst[1], axisDir[2]*homeCompConst[2], axisDir[3]*homeCompConst[3], axisDir[4]*homeCompConst[4], axisDir[5]*homeCompConst[5], axisDir[6]*homeCompConst[6], axisDir[7]*homeCompConst[7]};
long homeCompSteps[] = {homeCompAngles[0]*red[0]*ppr[0]/360.0, homeCompAngles[1]*red[1]*ppr[1]/360.0, homeCompAngles[2]*red[2]*ppr[2]/360.0, homeCompAngles[3]*red[3]*ppr[3]/360.0, homeCompAngles[4]*red[4]*ppr[4]/360.0, homeCompAngles[5]*red[5]*ppr[5]/360.0, homeCompAngles[6]*red[4]*ppr[4]/360.0, homeCompAngles[7]*red[5]*ppr[5]/360.0};
char value;

// values for changing speed
const int maxSpeedIndex = 2;
int speedVals[maxSpeedIndex+1][NUM_AXES_EFF] = {{600, 900, 1500, 1250, 1050, 1050, 1050, 1050}, {900, 1200, 2000, 1665, 1460, 1460, 1460, 1460}, {1200, 1800, 3000, 2500, 2200, 2200, 2200, 2200}};
int speedIndex = maxSpeedIndex;

void setup() { // setup function to initialize pins and provide initial homing to the arm

  Serial.begin(9600);

  // initializes end effector motor
  pinMode(EEstepPin, OUTPUT);
  pinMode(EEdirPin, OUTPUT);

  // initializes step pins, direction pins, limit switch pins, and stepper motor objects for accelStepper library
  for(i = 0; i<NUM_AXES; i++) {
      pinMode(stepPinsIK[i], OUTPUT);
      pinMode(dirPinsIK[i], OUTPUT);
      steppersIK[i] = AccelStepper(1, stepPinsIK[i], dirPinsIK[i]);
      steppersIK[i].setMinPulseWidth(200);
      encoders[i] = Encoder(encPinA[i], encPinB[i]);
    }

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
  recieveCommand();

  if(!IKFlag)
    runSteppers();

  else
    runSteppersIK();
}

void recieveCommand()
{
  String inData = "";
  char recieved = garbage;
  if(Serial.available()>0)
  {
    do {
      recieved = Serial.read();
      inData += String(recieved);
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
    IKFlag = true;
  }

  else if(function == "JM")
  {
    jointCommands(inMsg);
    IKFlag = false;
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

  else if(function == "JP")
  {
    if(!IKFlag)
    {
    readEncPos(curEncSteps);
    sendCurrentPosition();
    }
  }
}

//****//CARTESIAN MODE FUNCTIONS//****//

void cartesianCommands(String inMsg)
{

  jointFlag = false;
  // read current joint positions
  readEncPos(curEncSteps);

  // update host with joint positions
  sendCurrentPosition();

  // get new position commands
  int msgIdxJ1 = inMsg.indexOf('A');
  int msgIdxJ2 = inMsg.indexOf('B');
  int msgIdxJ3 = inMsg.indexOf('C');
  int msgIdxJ4 = inMsg.indexOf('D');
  int msgIdxJ5 = inMsg.indexOf('E');
  int msgIdxJ6 = inMsg.indexOf('F');
  cmdEncSteps[0] = inMsg.substring(msgIdxJ1 + 1, msgIdxJ2).toInt();
  cmdEncSteps[1] = inMsg.substring(msgIdxJ2 + 1, msgIdxJ3).toInt();
  cmdEncSteps[2] = inMsg.substring(msgIdxJ3 + 1, msgIdxJ4).toInt();
  cmdEncSteps[3] = inMsg.substring(msgIdxJ4 + 1, msgIdxJ5).toInt();
  cmdEncSteps[4] = inMsg.substring(msgIdxJ5 + 1, msgIdxJ6).toInt();
  cmdEncSteps[5] = inMsg.substring(msgIdxJ6 + 1).toInt();

  // update target joint positions
  readEncPos(curEncSteps);
  cmdArmBase();
  cmdArmWrist();
}

// parses which commands to execute when in joint space mode
void jointCommands(String inMsg)
{
  jointFlag = true;
  char function = inMsg[2];
  char detail1 = inMsg[3];

  if(function == release)
    releaseEvent(detail1, inMsg[4]); 
  else if(function == speed)
    changeSpeed(detail1); 
  else if(function == axis)
    changeAxis(detail1);
  else if(function == move)
    jointMovement(detail1, inMsg[4]);
  }

void endEffectorCommands(String inMsg)
{
  char data = inMsg[2];
  int force = getForce();

  //opening code
  if ((data == open) && (force < 100)) { //check if open button pressed and if force is less than max
    endEff.moveTo(openPos*MOTOR_DIR_EE); //continue to move to open position
  }

  //closing code
  else if ((data == close) && (force < 100)) { //check if open button pressed and if force is less than max
    endEff.moveTo(closePos*MOTOR_DIR_EE); //continue to move to closed position
  }

  else if ((data == release) || (force >= 100)) { //else check if release button pressed
    endEff.stop(); // stop when above condition reached
  }

  // need to only send force if a certain time interval defined by timeVal has passed
  sendForce(force);
}

int getForce()
{
  force=scale.get_units()/1000*9.81;
  int forcePercent = force*100/maxForce;
  return forcePercent;
}

void sendForce(int forcePercent)
{
  long currentTime = millis(); //checks total runtime
  long timeDiff = currentTime - previousTimeEE; //finds interval between runtime and previous checked time
  if (timeDiff >= timeIntervalEE)
  {
    String force_value = String(forcePercent);
    String force_message = String("EE: Gripper Force: ") + String(force_value) + String(" %\n");
    Serial.print(force_message);
    previousTimeEE = currentTime;
  }
}

void drillMotion(String inMsg)
{
  char function = inMsg[2];

  switch(function)
  {
    case manual: manualDrill(inMsg[3]); break;
    case drillRelease: stopDrill(); break;
    case prepare: prepDrill(); break;
    case collect: collectSample(); break;
    case deposit: depositSample(); break;
  }
}

void manualDrill(char dir)
{
  if(dir == left)
  {
    endEff.move(1000);
  }

  else
  {
    endEff.move(-1000);
  }
}

void drillRelease()
{
  endEff.stop();
}

void prepDrill()
{

}

void collectSample()
{

}

void depositSample()
{

}

void sendCurrentPosition() 
{
  String outMsg = String("JP") + String("A") + String(curEncSteps[0]) + String("B") + String(curEncSteps[1]) + String("C") + String(curEncSteps[2])
                + String("D") + String(curEncSteps[3]) + String("E") + String(curEncSteps[4]) + String("F") + String(curEncSteps[5]) + String("\n");
    Serial.print(outMsg);
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
  for(i=0; i<NUM_AXES; i++)
  {
    encPos[i] = encoders[i].read()*ENC_DIR[i];
  }
}

void zeroEncoders()
{
  for(i=0; i<NUM_AXES; i++)
  {
    encoders[i].write(0);
  }
}

void cmdArmBase()
{
  for (int i = 0; i < NUM_AXES_EX_WRIST; i++)
  { 
    int diffEncSteps = cmdEncSteps[i] - curEncSteps[i];
    if (abs(diffEncSteps) > 2)
    {
    int diffMotSteps = diffEncSteps / ENC_MULT[i];
    if (diffMotSteps < ppr[i])
    {
      // for the last rev of motor, introduce artificial decceleration
      // to help prevent overshoot
      diffMotSteps = diffMotSteps / 2;  
    }

    steppersIK[i].move(diffMotSteps);
    }
  }
}

void cmdArmWrist()
{
  int diffEncStepsA5 = cmdEncSteps[4] - curEncSteps[4];
  int diffEncStepsA6 = cmdEncSteps[5] - curEncSteps[5];

  if(abs(diffEncStepsA5) > 2)
  {
    int diffMotStepsA5 = diffEncStepsA5 / ENC_MULT[4];
    if(diffMotStepsA5 < ppr[i])
    {
      diffMotStepsA5 = diffEncStepsA5 / 2;
    }
  }

  if(abs(diffEncStepsA6) > 2)
  {
    int diffMotStepsA6 = diffEncStepsA6 / ENC_MULT[5];
    if(diffMotStepsA6 < ppr[i])
    {
      diffMotStepsA6 = diffEncStepsA6 / 2;
    }
  }

  int actualMotStepsA5 = diffMotStepsA5/2 + diffMotStepsA6/2;
  int actualMotStepsA6 = diffMotStepsA6/2 - diffMotStepsA5/2;

  steppersIK[4].move(actualMotStepsA5);
  steppersIK[5].move(actualMotStepsA6);
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
  //homeEE();
  initializeMotion();
  zeroEncoders();
  Serial.println("Arm Homed\n");

  for(int i=0; i<NUM_AXES; i++)
  {
    steppersIK[i].setCurrentPosition(0);
  }

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

void homeEE()
{
  int force = getForce();
  // target position for end effector in closed direction
  endEff.move(-99000*MOTOR_DIR);

  while(force < 100) 
  {
    force=getForce(); //converting mass to force
    // close end effector
    endEff.run();
  }
  //Set calibrated position as closed position
  endEff.setCurrentPosition(0);
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

  for(i = 0; i<NUM_AXES; i++)
  {
    steppersIK[i].setMaxSpeed(speedVals[maxSpeedIndex][i]);
    steppersIK[i].setAcceleration(maxAccel[i]);
  }

  endEff.setMaxSpeed(speedEE);
  endEff.setAcceleration(accEE);
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

  EE.run();
}

void runSteppersIK() { // runs all stepper motors (if no target position has been assigned, stepper will not move)
    
  for(i = 0; i<NUM_AXES; i++) {
    steppersIK[i].run();
  }

  endEff.run();
}

void waitForHome() { // stops arm motion until user homes arm after firmware is flashed

  String inData = "";
  char recieved;
  bool initFlag = false;
  bool serialFlag = false;

  while(!initFlag) {

    if(Serial.available() > 0)
    {
      recieved = Serial.read();
      inData += String(recieved)
      if(recieved == '\n')
      {
        serialFlag = true;
      }
    }

    if(serialFlag)
    {
        if(inData = "HM")
        {
          homeArm();
          initFlag = true;
        }

        else
        {
          inData = "";
          serialFlag = false;
        }
    }
  }
}