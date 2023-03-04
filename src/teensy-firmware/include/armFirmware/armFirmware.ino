/*
Created By: Tate Kolton, Graeme Dockrill
Created On: December 21, 2021
Updated On: January 10, 2023
Description: Main firmware for driving a 6 axis arm via ROS on a teensy 4.1 MCU
*/

// header file with all constants defined and libraries included
#include "armFirmware.h"

// setup function to initialize pins and provide initial homing to the arm.
void setup() {

  Serial.begin(115200);

  for (int i = 0; i < NUM_AXES; i++) {
    ENC_STEPS_PER_DEG[i] = ppr[i] * red[i] * (ENC_MULT[i] / 360.0);

    min_steps[i] = -homeCompSteps[i];
    max_steps[i] = max_steps[i] - homeCompSteps[i];
  }

  min_steps[0] = -max_steps[0];

  // initializes end effector motor
  pinMode(EEstepPin, OUTPUT);
  pinMode(EEdirPin, OUTPUT);
  endEff.setMinPulseWidth(200);
  endEff.setMaxSpeed(speedEE);
  endEff.setAcceleration(accEE);
  endEff.setCurrentPosition(1000);

  // initializes step pins, direction pins, limit switch pins, and stepper motor objects for accelStepper library
  for (i = 0; i < NUM_AXES; i++) {
    pinMode(dirPins[i], OUTPUT);
    pinMode(stepPins[i], OUTPUT);
    pinMode(limPins[i], INPUT_PULLUP);
    steppers[i] = AccelStepper(1, stepPins[i], dirPins[i]);
    steppers[i].setMinPulseWidth(200);
    steppers[i].setCurrentPosition(0);
  }

  // waits for user to press "home" button before rest of functions are available
  // waitForHome();
}

// main program loop
void loop() {

  // receives command from serial and executes accoringly
  recieveCommand();

  // run steppers to target position
  runSteppers();
}

void recieveCommand() {
  String inData = "";
  char recieved = '\0';

  // if a message is present, copy it to inData
  if (Serial.available() > 0) {
    do {
      recieved = Serial.read();
      inData += String(recieved);
    } while (recieved != '\n');
  }

  // parse the received data
  if (recieved == '\n') {
    parseMessage(inData);
  }
}

void parseMessage(String inMsg) {
  String function = inMsg.substring(0, 2);

  // choose which function to run
  if (function == "MT") {
    cartesianCommands(inMsg);
  }

  else if (function == "JM") {
    jointCommands(inMsg);
  }

  else if (function == "EE") {
    endEffectorCommands(inMsg);
  }

  else if (function == "PM") {
    executePose(inMsg);
  }

  else if (function == "HM") {
    homeArm();
  }

  // Send arm angles and gripper force to hardware driver
  sendArmFeedback();
}

void executePose(String inMsg) {
  int completeCount = 0;

  int msgIdxJ1 = inMsg.indexOf('A');
  int msgIdxJ2 = inMsg.indexOf('B');
  int msgIdxJ3 = inMsg.indexOf('C');
  int msgIdxJ4 = inMsg.indexOf('D');
  int msgIdxJ5 = inMsg.indexOf('E');
  int msgIdxJ6 = inMsg.indexOf('F');
  cmdEncSteps[0] = inMsg.substring(msgIdxJ1 + 1, msgIdxJ2).toInt() * IK_DIR[0];
  cmdEncSteps[1] = inMsg.substring(msgIdxJ2 + 1, msgIdxJ3).toInt() * IK_DIR[1];
  cmdEncSteps[2] = inMsg.substring(msgIdxJ3 + 1, msgIdxJ4).toInt() * IK_DIR[2];
  cmdEncSteps[3] = inMsg.substring(msgIdxJ4 + 1, msgIdxJ5).toInt() * IK_DIR[3];
  cmdEncSteps[4] = inMsg.substring(msgIdxJ5 + 1, msgIdxJ6).toInt() * IK_DIR[4];
  cmdEncSteps[5] = inMsg.substring(msgIdxJ6 + 1).toInt() * IK_DIR[5];


  while (completeCount != NUM_AXES) {
    readEncPos(curEncSteps);
    completeCount = cmdArm();
    runSteppers();
  }
}

void sendArmFeedback() {

  char garbage;
  while (Serial.available() > 0) {
    garbage = Serial.read();
  }

  int gripperForce = 0;  //readGripperForce();
  readEncPos(curEncSteps);

  String outMsg = String("JP") + String("A") + String(curEncSteps[0]) + String("B") + String(curEncSteps[1]) + String("C") + String(curEncSteps[2])
                  + String("D") + String(curEncSteps[3]) + String("E") + String(curEncSteps[4]) + String("F") + String(curEncSteps[5]) + String("G") + String(gripperForce) + String("Z");
  Serial.print(outMsg);
}

void runSteppers() {

  for (i = 0; i < NUM_AXES; i++) {
    steppers[i].run();
  }

  endEff.run();
}

//****//JOINT MODE FUNCTIONS//****//

// Parses which commands to execute when in joint space mode
void jointCommands(String inMsg) {
  char function = inMsg[2];
  char axis = inMsg[3];
  char dir = inMsg[4];

  if (!jointFlag)  // Check if we are switching from cartesian mode
  {
    IKFlag = false;
    jointFlag = true;
    setJointSpeed();
  }

  if (function == move)
    moveArm(axis, dir);

  else if (function == release)
    relArm(axis);
}

void moveArm(char axis, char dir) {
  int axisNum = String(axis).toInt();  // String to int for math

  if ((dir == left) || (dir == up)) {
    moveAxis((axisNum - 1), FWD);
  } else if ((dir == right) || (dir == down)) {
    moveAxis((axisNum - 1), REV);
  }
}

void relArm(char axis) {
  int axisNum = String(axis).toInt();  // String to int for math
  steppers[axisNum - 1].stop();
}

void moveAxis(int axis, int dir) {
  if ((axis == 0) || (axis == 1)) {  // switching direction if axis 1 or 2 (arm moves in intuitive dir)
    dir = !dir;
  }

  if (dir == FWD) {  // sets stepper to move to max steps and sets run flag so doesn't keep executing
    steppers[axis].moveTo(max_steps[axis]);

  } else if (dir == REV) {  // sets stepper to move to min steps and sets run flag so doesn't keep executing
    steppers[axis].moveTo(min_steps[axis]);
  }
}

// sets Joint Mode speeds after switching out of cartesian mode
void setJointSpeed() {
  for (i = 0; i < NUM_AXES; i++) {
    steppers[i].setMaxSpeed(maxSpeed[i]);
    steppers[i].setAcceleration(maxAccel[i]);
  }
}

//***//CARTESIAN MODE FUNCTIONS//***//

void cartesianCommands(String inMsg) {
  if (jointFlag) {
    IKFlag = true;
    jointFlag = false;
    setCartesianSpeed();
  }

  // get new position commands

  int msgIdxJ1 = inMsg.indexOf('A');
  int msgIdxJ2 = inMsg.indexOf('B');
  int msgIdxJ3 = inMsg.indexOf('C');
  int msgIdxJ4 = inMsg.indexOf('D');
  int msgIdxJ5 = inMsg.indexOf('E');
  int msgIdxJ6 = inMsg.indexOf('F');
  cmdEncSteps[0] = inMsg.substring(msgIdxJ1 + 1, msgIdxJ2).toInt() * IK_DIR[0];
  cmdEncSteps[1] = inMsg.substring(msgIdxJ2 + 1, msgIdxJ3).toInt() * IK_DIR[1];
  cmdEncSteps[2] = inMsg.substring(msgIdxJ3 + 1, msgIdxJ4).toInt() * IK_DIR[2];
  cmdEncSteps[3] = inMsg.substring(msgIdxJ4 + 1, msgIdxJ5).toInt() * IK_DIR[3];
  cmdEncSteps[4] = inMsg.substring(msgIdxJ5 + 1, msgIdxJ6).toInt() * IK_DIR[4];
  cmdEncSteps[5] = inMsg.substring(msgIdxJ6 + 1).toInt() * IK_DIR[5];

  readEncPos(curEncSteps);
  cmdArm();
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
    steppers[i].setAcceleration(MOTOR_MAX_ACCEL[i]);
    steppers[i].setMaxSpeed(MOTOR_MAX_SPEED[i]);
  }
}

// Set target position for cartesian movements of arm
int cmdArm() {
  int count = 0;

  for (int i = 0; i < NUM_AXES; i++) {
    int diffEncSteps = cmdEncSteps[i] - curEncSteps[i];
    if (abs(diffEncSteps) > 10) {
      int diffMotSteps = diffEncSteps / ENC_MULT[i];
      if (diffMotSteps < ppr[i]) {
        diffMotSteps = diffMotSteps / 2;
      }
      steppers[i].move(diffMotSteps);
    } else {
      count++;
    }
  }
  return count;
}

//***// END EFFECTOR RELATED FUNCTIONS //***//

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

int readGripperForce() {
  int force = analogRead(forcePin);
  int forcePct = 100 * (1023 - force) / 1023.0;
  return forcePct;
}

//***//ENCODER RELATED FUNCTIONS//***//

void readEncPos(int* encPos) {
  encPos[0] = enc1.read() * IK_DIR[0] * -1;
  encPos[1] = enc2.read() * IK_DIR[1] * -1;
  encPos[2] = enc3.read() * IK_DIR[2] * -1;
  encPos[3] = enc4.read() * IK_DIR[3] * -1;
  encPos[4] = enc5.read() * IK_DIR[4] * -1;
  encPos[5] = enc6.read() * IK_DIR[5] * -1;
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

//****// ARM CALIBRATION FUNCTIONS//****//

void homeArm() {  // main function for full arm homing

  initializeHomingMotion();
  homeAxes();
  initializeMotion();
}

// Runs through and checks if each axis has reached its limit switch, then runs it to specified home position
void homeAxes() {
  bool stopFlags[6] = { false, false, false, false, false, false };
  bool finishFlags[6] = { false, false, false, false, false, false };
  bool completeFlag = false;
  int count = 0;

  while (!completeFlag) {

    for (i = 0; i < NUM_AXES; i++) {

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
    if (count == NUM_AXES) {
      completeFlag = true;
    }
  }
}

void initializeHomingMotion() {  // sets homing speed and acceleration and sets target homing position

  for (i = 0; i < NUM_AXES; i++) {

    steppers[i].setMaxSpeed(homeSpeed[i]);
    steppers[i].setAcceleration(homeAccel[i]);

    if (i == 0)  // Special case for axis 1
    {
      if ((axisDir[0] * steppers[0].currentPosition() < -homeCompSteps[0])) {
        steppers[0].move(-homePos[i]);
      } else {
        steppers[0].move(homePos[i]);
      }
    }

    else {
      steppers[i].move(homePos[i]);
    }
  }
}

void initializeMotion() {  // sets main program speeds for each axis and zeros position

  setJointSpeed();
  zeroEncoders();

  for (int i = 0; i < NUM_AXES; i++) {
    steppers[i].setCurrentPosition(0);
  }
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