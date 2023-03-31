/*
Created By: Tate Kolton, Rowan Zawadzki
Created On: December 21, 2021
Updated: April 27, 2022
Description: Main firmware for driving a 6 axis arm via ROS on a teensy 4.1 MCU
*/

#include <AccelStepper.h>
#include <HX711.h>
#include <Encoder.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64MultiArray.h>
#include <sb_msgs/ArmPosition.h>

//for if no arm is connected, simulation only needed
#define simOnly 1
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

//ros stuff
ros::NodeHandle nh;
std_msgs::Int16 beat;
sb_msgs::ArmPosition INangles;
sb_msgs::ArmPosition OBSangles;
//node slowdowns, to relieve serial processing
int beatTEMP = millis();
int beatINTERVAL = 1000; //ms
int posTEMP = millis();
int posINTERVAL = 100; //ms


ros::Publisher heart("/heartbeat", &beat, 100);
ros::Publisher observer("/observed_arm_pos", &OBSangles, 100);





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
static const char prepare = 'P';
static const char collect = 'C';
static const char deposit = 'D';
static const char manual = 'M';
static const char drillRelease = 'X';
static const char open = 'O';
static const char close = 'C';
static const char joint = 'J';
static const char EEval = 'E';
static const char homeValEE = 'H';
static const char moveBase = 'T';
static const char moveWrist = 'M';
static const char relWrist = 'R';
static const char relBase = 'W';

// Motor variables
int stepPins[8] = { 6, 8, 10, 2, 12, 25, 12, 25 };
int dirPins[8] = { 5, 7, 9, 1, 11, 24, 11, 24 };
int stepPinsIK[6] = { 6, 8, 2, 10, 25, 12 };
int dirPinsIK[6] = { 5, 7, 1, 9, 24, 11 };
int encPinA[6] = { 17, 38, 40, 36, 15, 13 };
int encPinB[6] = { 16, 37, 39, 35, 14, 41 };

// end effector variables
const float calibrationFactor = -111.25;
float force;
HX711 scale;
const int dataPin = 34;
const int clkPin = 33;
int calPos = 0;
int closePos = 0;
int openPos = 500;  // needs to be calibrated
int EEstepPin = 4;
int EEdirPin = 3;
int speedEE = 100;
int accEE = 500;
int speedDrill = 3000;
int accDrill = 1000;
const int MOTOR_DIR_EE = 1;
const int openButton = 5;
const int closeButton = 4;
const float calForce = 0.3;
const float maxForce = 10.0;
float EEforce;
int forcePct = 0;

// limit switch pins
int limPins[6] = { 18, 19, 21, 20, 23, 22 };

// pulses per revolution for motors
long ppr[6] = { 400, 400, 400, 400, 400, 400 };

// Gear Reduction Ratios
float red[6] = { 50.0, 161.0, 44.8, 93.07, 57.34, 57.34 };
float redIK[6] = { 50.0, 161.0, 93.07, 44.8, 57.34, 57.34 };


// Encoder Variables
int curEncSteps[NUM_AXES], cmdEncSteps[NUM_AXES];
int pprEnc = 512;
int ENC_DIR[6] = { -1, -1, -1, -1, 1, 1 };
const float ENC_MULT[] = { 5.12, 5.12, 5.12, 5.12, 5.12, 5.12 };
float ENC_STEPS_PER_DEG[NUM_AXES];

// Motor speeds and accelerations
int maxSpeed[8] = { 1200, 1800, 3000, 2500, 2200, 2200, 2200, 2200 };
int maxAccel[8] = { 1300, 3500, 4600, 3300, 5000, 5000, 5000, 5000 };
int homeSpeed[8] = { 300, 1000, 1000, 400, 2000, 2000, 2000, 2000 };   // {500, 1200, 600, 400, 2000, 2000, 2000, 2000};
int homeAccel[8] = { 500, 2000, 1500, 1000, 1500, 1500, 1500, 1500 };  //{500, 2000, 1000, 1500, 1500, 1500, 1500, 1500};


// Time variables
const unsigned long readInterval = 10;
unsigned long currentTime;
unsigned long currentTimeJP;
unsigned long currentTimeEE;
unsigned long previousTime = 0;
unsigned long previousTimeEE = 0;
const unsigned long timeIntervalEE = 500;
const unsigned long timeIntervalJP = 250;
unsigned long previousTimeJP = 0;

// stepper motor objects for AccelStepper library
AccelStepper steppers[8];
AccelStepper endEff(1, EEstepPin, EEdirPin);
AccelStepper steppersIK[8];


Encoder enc1(encPinA[0], encPinB[0]);
Encoder enc2(encPinA[1], encPinB[1]);
Encoder enc3(encPinA[2], encPinB[2]);
Encoder enc4(encPinA[3], encPinB[3]);
Encoder enc5(encPinA[4], encPinB[4]);
Encoder enc6(encPinA[5], encPinB[5]);

Encoder encoders[] = { enc1, enc2, enc3, enc4, enc5, enc6 };

// variable declarations

int axisDir[8] = { 1, -1, 1, -1, 1, 1, -1, 1 };
int axisDirIK[6] = { -1, -1, -1, 1, -1, -1 };
int currentAxis = 1;
int runFlags[] = { 0, 0, 0, 0, 0, 0 };
int i;
bool initFlag = false;
bool jointFlag = true;
bool IKFlag = false;
bool J1Flag = false;
bool resetEE = false;
bool vertFlag = false;
bool horizFlag = false;

// variables for homing / arm calibration
long homePosConst = -99000;
long homePos[] = { axisDir[0] * homePosConst, axisDir[1] * homePosConst, axisDir[2] * homePosConst, axisDir[3] * homePosConst, axisDir[4] * homePosConst, axisDir[5] * homePosConst, axisDir[6] * homePosConst, axisDir[7] * homePosConst };
long homeCompAngles[] = { axisDir[0] * 54, axisDir[1] * 10, axisDir[2] * 90, axisDir[3] * 1, axisDir[4] * 85, axisDir[5] * 85, axisDir[6] * 170, axisDir[7] * 170 };
long homeCompConst[] = { 500, 2000, 1000, 500, 500, 500, 500, 500 };
long homeComp[] = { axisDir[0] * homeCompConst[0], axisDir[1] * homeCompConst[1], axisDir[2] * homeCompConst[2], axisDir[3] * homeCompConst[3], axisDir[4] * homeCompConst[4], axisDir[5] * homeCompConst[5], axisDir[6] * homeCompConst[6], axisDir[7] * homeCompConst[7] };
long homeCompSteps[] = { homeCompAngles[0] * red[0] * ppr[0] / 360.0, homeCompAngles[1] * red[1] * ppr[1] / 360.0, homeCompAngles[2] * red[2] * ppr[2] / 360.0, homeCompAngles[3] * red[3] * ppr[3] / 360.0, homeCompAngles[4] * red[4] * ppr[4] / 360.0, homeCompAngles[5] * red[5] * ppr[5] / 360.0, homeCompAngles[6] * red[4] * ppr[4] / 360.0, homeCompAngles[7] * red[5] * ppr[5] / 360.0 };
// Range of motion (degrees) for each axis
int maxAngles[6] = { 190, 160, 180, 120, 160, 180 };
long max_steps[] = { axisDir[0] * red[0] * maxAngles[0] / 360.0 * ppr[0], axisDir[1] * red[1] * maxAngles[1] / 360.0 * ppr[1], axisDir[2] * red[2] * maxAngles[2] / 360.0 * ppr[2], axisDir[3] * red[3] * maxAngles[3] / 360.0 * ppr[3], red[4] * maxAngles[4] / 360.0 * ppr[4], red[5] * maxAngles[5] / 360.0 * ppr[5] };
long min_steps[NUM_AXES];
char value;

// values for changing speed
const int maxSpeedIndex = 2;
int speedVals[maxSpeedIndex + 1][NUM_AXES_EFF] = { { 600, 900, 1500, 1250, 1050, 1050, 1050, 1050 }, { 900, 1200, 2000, 1665, 1460, 1460, 1460, 1460 }, { 900, 1600, 2500, 2200, 2000, 2000, 2000, 2000 } };
int speedIndex = maxSpeedIndex;

// Cartesian mode speed settings
float IKspeeds[] = { 0.2, 0.2, 0.2, 0.2, 0.2, 0.2 };
float IKaccs[] = { 0.3, 0.3, 0.3, 0.3, 0.3, 0.3 };

void runSteppers();
void runSteppersIK();
void homeArm();
void readEncPos(int* encPos);
void cmdArmBase();
void cmdArmWrist();
void cartesianToJointSpace();
void runWrist(int dir, int axis);
void sendCurrentPosition();


void setup() {  // setup function to initialize pins and provide initial homing to the arm
 // nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(heart);
  nh.advertise(observer);
  nh.negotiateTopics();
  //Serial.begin(9600);



  // for (int i = 0; i < NUM_AXES; i++) {
  //   ENC_STEPS_PER_DEG[i] = ppr[i] * redIK[i] * (ENC_MULT[i] / 360.0);
  // }
  // for (int i = 0; i < NUM_AXES_EX_WRIST; i++) {
  //   min_steps[i] = -homeCompSteps[i];
  //   max_steps[i] = max_steps[i] - homeCompSteps[i];
  // }
  // min_steps[0] = -(homeCompSteps[0] * axisDir[0] * 180) / homeCompAngles[0];
  // min_steps[4] = -homeCompSteps[4] / axisDir[4];
  // min_steps[5] = -homeCompSteps[6] / axisDir[6];
  // max_steps[4] = max_steps[4] - homeCompSteps[4] / axisDir[4];
  // max_steps[5] = max_steps[5] - homeCompSteps[6] / axisDir[6];


  // initializes end effector motor
  // pinMode(EEstepPin, OUTPUT);
  // pinMode(EEdirPin, OUTPUT);
  // endEff.setMinPulseWidth(200);
  // endEff.setMaxSpeed(speedEE);
  // endEff.setAcceleration(accEE);
  // endEff.setCurrentPosition(1000);

  // initializes step pins, direction pins, limit switch pins, and stepper motor objects for accelStepper library
  for (i = 0; i < NUM_AXES; i++) {
    pinMode(stepPinsIK[i], OUTPUT);
    pinMode(dirPinsIK[i], OUTPUT);
    steppersIK[i] = AccelStepper(1, stepPinsIK[i], dirPinsIK[i]);
    steppersIK[i].setMinPulseWidth(200);
  }

  for (i = 0; i < NUM_AXES_EFF; i++) {
    pinMode(dirPins[i], OUTPUT);
    pinMode(stepPins[i], OUTPUT);
    pinMode(limPins[i], INPUT_PULLUP);
    steppers[i] = AccelStepper(1, stepPins[i], dirPins[i]);
    steppers[i].setMinPulseWidth(200);
  }
  // waits for user to press "home" button before rest of functions are available
  //Serial.println("diagnosticing");
 // waitForHome();
  delay(90);

//homeArm();
  nh.spinOnce();
}

void loop() {
  if (millis() - posTEMP > posINTERVAL) {
    //sendCurrentPosition();
    posTEMP = 0;
  } 
if(millis() - beatTEMP > beatINTERVAL){
    beat.data += 1;
    heart.publish(&beat);
    beatTEMP = millis();
  }

  // if(!IKFlag)
  //   runSteppers();

  // else
  //   runSteppersIK();


  nh.spinOnce();
}

// void recieveCommand()
// {
//   String inData = "";
//   char recieved = '\0';

//   if(Serial.available()>0)
//   {
//     do {
//       recieved = Serial.read();
//       inData += String(recieved);
//     } while(recieved != '\n');
//   }

//   if(recieved == '\n')
//   {

//     parseMessage(inData);
//   }
// }

// void parseMessage(String inMsg)
// {
//   String function = inMsg.substring(0, 2);

//   if(function == "MT")
//   {
//     cartesianCommands(inMsg);
//   }

//   else if(function == "JM")
//   {
//     jointCommands(inMsg);
//   }

//   else if(function == "EE")
//   {
//     endEffectorCommands(inMsg);
//   }

//   else if(function == "DM")
//   {
//     drillCommands(inMsg);
//   }

//   else if(function == "FB")
//   {
//     sendFeedback(inMsg);
//   }

//   else if(function == "HM")
//   {
//     homeArm();
//   }
// }

// void sendMessage(char outChar)
// {
//   String outMsg = String(outChar);
//   Serial.print(outMsg);
// }

// void sendFeedback(String inMsg)
// {
//   char function = inMsg[2];

//   if(function == joint)
//   {
//     readEncPos(curEncSteps);
//     sendCurrentPosition();
//   }
// }

//****//CARTESIAN MODE FUNCTIONS//****//

void cartesianCommands(String inMsg) {
  // read current joint positions
  readEncPos(curEncSteps);

  // update host with joint positions
  sendCurrentPosition();

  if (!IKFlag) {
    setCartesianSpeed();
    IKFlag = true;
    jointFlag = false;
  }

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

void setCartesianSpeed() {
  float JOINT_MAX_SPEED[NUM_AXES];
  float MOTOR_MAX_SPEED[NUM_AXES];
  float JOINT_MAX_ACCEL[NUM_AXES];
  float MOTOR_MAX_ACCEL[NUM_AXES];

  for (int i = 0; i < NUM_AXES; i++) {
    JOINT_MAX_SPEED[i] = IKspeeds[i] * (180.0 / 3.14159);
    JOINT_MAX_ACCEL[i] = IKaccs[i] * (180.0 / 3.14159);
    MOTOR_MAX_SPEED[i] = JOINT_MAX_SPEED[i] * ENC_STEPS_PER_DEG[i] / ENC_MULT[i];
    MOTOR_MAX_ACCEL[i] = JOINT_MAX_ACCEL[i] * ENC_STEPS_PER_DEG[i] / ENC_MULT[i];
    steppersIK[i].setAcceleration(MOTOR_MAX_ACCEL[i]);
    steppersIK[i].setMaxSpeed(MOTOR_MAX_SPEED[i]);
  }
}

// parses which commands to execute when in joint space mode
void jointCommands(String inMsg) {
  char function = 3;  //inMsg[2];
  char detail1 = 4;   //inMsg[3];

  if (!jointFlag) {
    cartesianToJointSpace();
    IKFlag = false;
    jointFlag = true;
  }

  if (function == moveBase)
    moveArmBase(detail1, inMsg[4]);
  else if (function == moveWrist) {
    char dir = inMsg[4];
    if (dir == up)
      runWrist(FWD, 6);
    else if (dir == down)
      runWrist(REV, 6);
    else if (dir == left)
      runWrist(FWD, 5);
    else if (dir == right)
      runWrist(REV, 5);
  } else if (function == relBase)
    relArmBase(detail1);
  else if (function == relWrist)
    releaseEvent(detail1, inMsg[4]);
}

void moveArmBase(char axis, char dir) {
  int axisNum = String(axis).toInt();

  if (axisNum == 3)
    axisNum = 4;

  else if (axisNum == 4)
    axisNum = 3;

  if ((dir == left) || (dir == up)) {
    moveBaseAxis((axisNum - 1), FWD);
  } else if ((dir == right) || (dir == down)) {
    moveBaseAxis((axisNum - 1), REV);
  }
}

void relArmBase(char axis) {
  int axisNum = String(axis).toInt();

  if (axisNum == 3)
    axisNum = 4;

  else if (axisNum == 4)
    axisNum = 3;

  steppers[axisNum - 1].stop();
}

void moveBaseAxis(int axis, int dir) {
  if ((axis == 0) || (axis == 1)) {
    dir = !dir;
  }

  if (dir == FWD) {
    steppers[axis].moveTo(max_steps[axis]);
    runFlags[axis] = 1;
  } else if (dir == REV) {
    steppers[axis].moveTo(min_steps[axis]);
    runFlags[axis] = -1;
  }
}

void endEffectorCommands(String inMsg) {
  char data = inMsg[2];

  //opening code
  if (data == open) {                       //check if open button pressed and if force is less than max
    endEff.moveTo(openPos * MOTOR_DIR_EE);  //continue to move to open position
  }

  //closing code
  else if (data == close) {                  //check if open button pressed and if force is less than max
    endEff.moveTo(closePos * MOTOR_DIR_EE);  //continue to move to closed position
  }

  else if (data == release) {  //else check if release button pressed
    endEff.stop();             // stop when above condition reached
  }

  else if (data == homeValEE) {
    if (resetEE) {
      endEff.setCurrentPosition(1000);
      resetEE = false;
    }

    else {
      endEff.setCurrentPosition(0);
      resetEE = true;
    }
  }
}

void getEEForce() {
  // if(scale.wait_ready_timeout(1))
  // {
  //   float force = scale.get_units()/1000*9.81;
  //   forcePct = force*100.0/maxForce;
  // }
}

void sendEEForce() {
  //String force_value = String(forcePct);
  //String force_message = String("EE: Gripper Force: ") + String(force_value) + String(" Z");
  // String force_message = String("EEForceZ");
  // Serial.print(force_message);
}

void drillCommands(String inMsg) {
  char function = inMsg[2];

  if (function == manual)
    manualDrill(inMsg[3]);
  else if (function == drillRelease)
    stopDrill();
  else if (function == prepare)
    spinDrill();
  else if (function == collect)
    stopDrill();
  else if (function == deposit)
    depositSample();
}

void manualDrill(char dir) {
  if (dir == left) {
    endEff.move(99000);
  }

  else {
    endEff.move(-99000);
  }
}

void stopDrill() {
  endEff.stop();
}

void spinDrill() {
  endEff.move(99000);
}

void collectSample() {
}

void depositSample() {
}

void sendCurrentPosition() {
  for (int i = 0; i < NUM_AXES; i++) {
    OBSangles.positions[i] = float(steppers[i].currentPosition()) / axisDir[i] / red[i] / ppr[i];
  }
  observer.publish(&OBSangles);
}

// converts cartesian angles to joint space stepper current positions
void cartesianToJointSpace() {
  long curJointPos[NUM_AXES_EFF];
  readEncPos(curEncSteps);
  curJointPos[0] = curEncSteps[0] / 5.12;
  curJointPos[1] = curEncSteps[1] / 5.12;
  curJointPos[3] = curEncSteps[2] / 5.12;
  curJointPos[2] = curEncSteps[3] / 5.12;
  curJointPos[6] = curEncSteps[4] / 5.12;
  curJointPos[7] = curEncSteps[4] / 5.12;
  curJointPos[4] = curEncSteps[5] / 5.12;
  curJointPos[5] = curEncSteps[5] / 5.12;

  for (int i = 0; i < NUM_AXES_EFF; i++) {
    steppers[i].setCurrentPosition(curJointPos[i] * axisDir[i]);
  }
}

// Sets movement target positions when in joint space mode
void jointMovement(char joystick, char dir) {
  if (joystick == wrist) {
    if (dir == up)
      runWrist(FWD, 6);
    else if (dir == down)
      runWrist(REV, 6);
    else if (dir == left)
      runWrist(FWD, 5);
    else if (dir == right)
      runWrist(REV, 5);
  }

  else if (joystick == left) {
    if (dir == left)
      runAxes(FWD, currentAxis);
    else if (dir == right)
      runAxes(REV, currentAxis);
  }

  else {
    if (dir == up)
      runAxes(FWD, currentAxis + 1);
    else if (dir == down)
      runAxes(REV, currentAxis + 1);
  }
}

//****//ENCODER RELATED FUNCTIONS//****//

void readEncPos(int* encPos) {
  encPos[0] = enc1.read() * ENC_DIR[0];
  encPos[1] = enc2.read() * ENC_DIR[1];
  encPos[2] = enc3.read() * ENC_DIR[2];
  encPos[3] = enc4.read() * ENC_DIR[3];
  encPos[4] = enc5.read() * ENC_DIR[4];
  encPos[5] = enc6.read() * ENC_DIR[5];
  long temp = encPos[4];
  encPos[5] = temp + encPos[5];
  encPos[4] = encPos[5] - temp;
}

void zeroEncoders() {
  enc1.write(0);
  enc2.write(0);
  enc3.write(0);
  enc4.write(0);
  enc5.write(0);
  enc6.write(0);

  readEncPos(curEncSteps);
}

void cmdArmBase() {
  for (int i = 0; i < NUM_AXES_EX_WRIST; i++) {
    int diffEncSteps = cmdEncSteps[i] - curEncSteps[i];
    if (abs(diffEncSteps) > 6) {
      int diffMotSteps = diffEncSteps / ENC_MULT[i];
      if (diffMotSteps < ppr[i]) {
        // for the last rev of motor, introduce artificial decceleration
        // to help prevent overshoot
        diffMotSteps = diffMotSteps / 2;
      }

      steppersIK[i].move(diffMotSteps * axisDirIK[i]);
    }
  }
}

void cmdArmWrist() {
  int diffMotStepsA5, diffMotStepsA6, diffEncStepsA5, diffEncStepsA6;

  diffEncStepsA5 = cmdEncSteps[4] - curEncSteps[4];
  diffEncStepsA6 = cmdEncSteps[5] - curEncSteps[5];

  if (abs(diffEncStepsA5) > 10) {
    diffMotStepsA5 = diffEncStepsA5 / ENC_MULT[4];
    if (diffMotStepsA5 < ppr[4]) {
      diffMotStepsA5 = diffEncStepsA5 / 2;
    }
  }

  if (abs(diffEncStepsA6) > 10) {
    diffMotStepsA6 = diffEncStepsA6 / ENC_MULT[5];
    if (diffMotStepsA6 < ppr[5]) {
      diffMotStepsA6 = diffEncStepsA6 / 2;
    }
  }

  int actualMotStepsA5 = diffMotStepsA6 / 2 - diffMotStepsA5 / 2;
  int actualMotStepsA6 = diffMotStepsA6 / 2 + diffMotStepsA5 / 2;

  steppersIK[4].move(actualMotStepsA5 * axisDirIK[4]);
  steppersIK[5].move(actualMotStepsA6 * axisDirIK[5]);
}



//****//JOINT SPACE MODE FUNCTIONS//****//

// sets target position for axes in joint space mode
void runAxes(int dir, int axis) {  // assigns run flags to indicate forward / reverse motion and sets target position

  if ((axis == 3) || (axis == 1) || (axis == 2)) {
    dir = !dir;
  }

  if (runFlags[axis - 1] == 1 && dir == FWD) {
  }

  else if (runFlags[axis - 1] == -1 && dir == REV) {
  }

  else if (dir == FWD) {
    steppers[axis - 1].moveTo(max_steps[axis - 1]);
    runFlags[axis - 1] = 1;
  }

  else {
    steppers[axis - 1].moveTo(min_steps[axis - 1]);
    runFlags[axis - 1] = -1;
  }
}

void runWrist(int dir, int axis) {  // assigns target position for selected axis based on user input.

  if (axis == 5) {
    if (runFlags[5] == 1 && dir == FWD) {
    }

    else if (runFlags[5] == -1 && dir == REV) {
    }

    else if (dir == FWD) {
      steppers[6].moveTo(axisDir[6] * max_steps[5]);
      steppers[7].moveTo(axisDir[7] * max_steps[5]);
      runFlags[5] = 1;
    }

    else {
      steppers[6].moveTo(axisDir[6] * min_steps[5]);
      steppers[7].moveTo(axisDir[7] * min_steps[5]);
      runFlags[5] = -1;
    }
  }

  else if (axis == 6) {
    dir = !dir;
    if (runFlags[4] == 1 && dir == FWD) {
    }

    else if (runFlags[4] == -1 && dir == REV) {
    }

    else if (dir == FWD) {
      steppers[4].moveTo(axisDir[4] * max_steps[4]);
      steppers[5].moveTo(axisDir[5] * max_steps[4]);
      runFlags[4] = 1;
    }

    else {
      steppers[4].moveTo(axisDir[4] * min_steps[4]);
      steppers[5].moveTo(axisDir[5] * min_steps[4]);
      runFlags[4] = -1;
    }
  }
}

void changeAxis(int dir) {  // when user hits specified button, axis targets change

  if ((dir == up) && (currentAxis == 1)) {
    currentAxis = 3;
    zeroRunFlags();
  }

  else if ((dir == down) && (currentAxis == 3)) {
    currentAxis = 1;
    zeroRunFlags();
  }
}

void releaseEvent(char joystick, char dir) {  // when user releases a joystick serial sends a character

  if (joystick == wrist) {
    if ((dir == left) || (dir == right)) {
      steppers[6].stop();
      steppers[7].stop();
      runFlags[5] = 0;
    }

    else {
      steppers[4].stop();
      steppers[5].stop();
      runFlags[4] = 0;
    }
  }

  else if (joystick == left) {
    steppers[currentAxis - 1].stop();
    runFlags[currentAxis - 1] = 0;
  }

  else if (joystick == right) {
    steppers[currentAxis].stop();
    runFlags[currentAxis] = 0;
  }
}

void changeSpeed(char speedVal) {  // changes speed of all axes based on user input

  if (speedVal == faster) {
    if (speedIndex < maxSpeedIndex) {
      speedIndex++;
      for (i = 0; i < NUM_AXES_EFF; i++) {
        steppers[i].setMaxSpeed(speedVals[speedIndex][i]);
      }
    }
  }

  else if (speedVal == slower) {
    if (speedIndex > 0) {
      speedIndex--;
      for (i = 0; i < NUM_AXES_EFF; i++) {
        steppers[i].setMaxSpeed(speedVals[speedIndex][i]);
      }
    }
  }
}

void zeroRunFlags() {  // when user changes axis to control on switch, slow current moving axes to a stop and reset run flags (all motors stagnant)

  for (i = 0; i < NUM_AXES_EX_WRIST; i++) {
    steppers[i].stop();
  }

  for (i = 0; i < NUM_AXES_EX_WRIST; i++) {
    runFlags[i] = 0;
  }
}

//****// ARM CALIBRATION FUNCTIONS//****//

void homeArm() {  // main function for full arm homing
  if (!simOnly) {
    initializeWristHomingMotion();
    homeWrist();
    initializeHomingMotion();
    homeBase();
    initializeMotion();
    zeroEncoders();
  }

  J1Flag = true;

  for (int i = 0; i < NUM_AXES; i++) {
    steppersIK[i].setCurrentPosition(0);
  }

  // clear serial port in case of garbage values
  while (Serial.available() != 0) {
    Serial.read();
  }

  // notify hardware driver that homing has completed
  Serial.print("HCZ");
}

void homeBase() {  // homes axes 1-4

  bool stopFlags[4] = { false, false, false, false };
  bool finishFlags[4] = { false, false, false, false };
  bool completeFlag = false;
  int count = 0;

  while (!completeFlag) {

    for (i = 0; i < NUM_AXES_EX_WRIST; i++) {

      if (!finishFlags[i]) {

        if (!digitalRead(limPins[i]) && !stopFlags[i]) {
          steppers[i].run();
        }

        else if (!stopFlags[i]) {
          steppers[i].setCurrentPosition(-homeComp[i]);
          steppers[i].stop();
          steppers[i].setMaxSpeed(maxSpeed[i] / 2);
          steppers[i].setAcceleration(homeAccel[i]);
          steppers[i].moveTo(homeCompSteps[i]);
          stopFlags[i] = true;
        }

        else if (steppers[i].distanceToGo() != 0) {
          steppers[i].run();
        }

        else {
          finishFlags[i] = true;
          count++;
        }
      }
    }
    if (count == NUM_AXES_EX_WRIST) {
      completeFlag = true;
    }
  }
  initializeMotion();
}

void homeWrist() {  // homes axes 5-6

  bool stopFlags[2] = { false, false };
  bool calibFlags[2] = { false, false };
  bool completeFlag = false;

  while (!completeFlag) {

    if (!digitalRead(limPins[5]) && !stopFlags[0]) {
      steppers[6].run();
      steppers[7].run();
    }

    else if (!stopFlags[0]) {
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

    else if (steppers[6].distanceToGo() != 0 && !calibFlags[0]) {
      steppers[6].run();
      steppers[7].run();
    }

    else if (!calibFlags[0]) {
      calibFlags[0] = true;
    }

    else if (!digitalRead(limPins[4]) && !stopFlags[1]) {
      steppers[4].run();
      steppers[5].run();
    }

    else if (!stopFlags[1]) {
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

    else if (steppers[4].distanceToGo() != 0 && !calibFlags[1]) {
      steppers[4].run();
      steppers[5].run();
    }

    else if (!calibFlags[1]) {
      calibFlags[1] = true;
      completeFlag = true;
    }
  }
}

//void homeEE()
//{
//  EEforce=scale.get_units()/1000*9.81;
//
//  // target position for end effector in closed direction
//  endEff.move(-99000*MOTOR_DIR_EE);
//
//  while(abs(EEforce) < calForce)
//  {
//    if (scale.wait_ready_timeout(1)) {
//      EEforce=scale.get_units()/1000*9.81; //converting mass to force
//    // close end effector
//    }
//    endEff.run();
//  }
//
//  endEff.setCurrentPosition(-30);
//  endEff.moveTo(openPos*MOTOR_DIR_EE);
//  while(endEff.distanceToGo() != 0)
//  {
//    endEff.run();
//  }
//}

void initializeHomingMotion() {  // sets homing speed and acceleration for axes 1-4 and sets target homing position

  for (i = 0; i < NUM_AXES_EX_WRIST; i++) {

    steppers[i].setMaxSpeed(homeSpeed[i]);
    steppers[i].setAcceleration(homeAccel[i]);
    steppers[i].setCurrentPosition(0);

    if (i != 0) {
      steppers[i].move(homePos[i]);
    } else if (J1Flag == true) {
      if ((steppers[0].currentPosition() / axisDir[0] < 0) && (abs(steppers[0].currentPosition() / axisDir[0]) > (homeCompSteps[0] + homeComp[0]) / axisDir[0])) {
        steppers[0].move(-homePos[i]);
      } else {
        steppers[0].move(homePos[i]);
      }
    } else {
      steppers[0].move(homePos[i]);
    }
  }
}

void initializeWristHomingMotion() {  // sets homing speed and acceleration for axes 5-6 and sets target homing position

  for (i = 4; i < NUM_AXES_EFF; i++) {

    steppers[i].setMaxSpeed(homeSpeed[i]);
    steppers[i].setAcceleration(homeAccel[i]);
    steppers[i].setCurrentPosition(0);
    steppers[i].move(homePos[i]);
  }
}

void initializeMotion() {  // sets main program speeds for each axis

  for (i = 0; i < NUM_AXES_EFF; i++) {
    steppers[i].setMaxSpeed(speedVals[maxSpeedIndex][i]);
    steppers[i].setAcceleration(maxAccel[i]);
    steppers[i].setCurrentPosition(0);
  }

  for (i = 0; i < NUM_AXES; i++) {
    steppersIK[i].setMaxSpeed(speedVals[maxSpeedIndex][i]);
    steppersIK[i].setAcceleration(maxAccel[i]);
  }
}

void zeroAxes() {  // sets current position of all axes to 0

  for (i = 0; i < NUM_AXES_EFF; i++) {
    steppers[i].setCurrentPosition(0);
  }
}

void runSteppers() {  // runs all stepper motors (if no target position has been assigned, stepper will not move)

  for (i = 0; i < NUM_AXES_EFF; i++) {
    steppers[i].run();
  }

  endEff.run();
}

void runSteppersIK() {  // runs all stepper motors (if no target position has been assigned, stepper will not move)

  for (i = 0; i < NUM_AXES; i++) {
    steppersIK[i].run();
  }

  endEff.run();
}

void waitForHome() {
  String inData;
  char recieved = '\0';

  while (!initFlag) {
    inData = "";
    if (Serial.available() > 0) {
      do {
        recieved = Serial.read();
        inData += String(recieved);
      } while (recieved != '\n');
    }

    if (recieved == '\n') {
      if (inData.substring(0, 2) == "HM")
        homeArm();
      initFlag = true;
    }
  }
}

// updated aug 22, 2022