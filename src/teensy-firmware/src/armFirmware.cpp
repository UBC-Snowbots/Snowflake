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

  Serial.begin(9600);

  for(int i=0; i<NUM_AXES; i++) {
    ENC_STEPS_PER_DEG[i] = ppr[i]*red[i]*(ENC_MULT[i]/360.0);

    min_steps[i] = -homeCompSteps[i];
    max_steps[i] = max_steps[i]-homeCompSteps[i];
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
  for(i = 0; i<NUM_AXES; i++) {
      pinMode(dirPins[i], OUTPUT);
      pinMode(stepPins[i], OUTPUT);
      pinMode(limPins[i], INPUT_PULLUP);
      steppers[i] = AccelStepper(1, stepPins[i], dirPins[i]);
      steppers[i].setMinPulseWidth(200);
    }
    
// waits for user to press "home" button before rest of functions are available
waitForHome();
}

// main program loop
void loop()
{
  // receives command from serial and executes accoringly
  recieveCommand();

  // run steppers to target position
    runSteppers();
}

void recieveCommand()
{
  String inData = "";
  char recieved = '\0';

  // if a message is present, copy it to inData
  if(Serial.available()>0)
  {
    do {
      recieved = Serial.read();
      inData += String(recieved);
    } while(recieved != '\n');
  }

  // parse the received data
  if(recieved == '\n')
  {

    parseMessage(inData);
  }
}

void parseMessage(String inMsg)
{
  String function = inMsg.substring(0, 2);
  
  // choose which function to run
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

  else if(function == "FB")
  {
    sendFeedback(inMsg);
  }

  else if(function == "HM")
  {
    homeArm();
  }
}

void sendMessage(char outChar)
{
  String outMsg = String(outChar);
  Serial.print(outMsg);
}

void sendCurrentPosition() 
{
  String outMsg = String("JP") + String("A") + String(curEncSteps[0]) + String("B") + String(curEncSteps[1]) + String("C") + String(curEncSteps[2])
                + String("D") + String(curEncSteps[3]) + String("E") + String(curEncSteps[4]) + String("F") + String(curEncSteps[5]) + String("Z");
    Serial.print(outMsg);
}

void sendFeedback(String inMsg)
{
  char function = inMsg[2];

  // if in joint mode, send feedback over serial
  if(function == joint)
  {
    readEncPos(curEncSteps);
    sendCurrentPosition();
  }
}

void runSteppers() {
    
  for(i = 0; i<NUM_AXES; i++) {
    steppers[i].run();
  }

  endEff.run();
}

//****//JOINT MODE FUNCTIONS//****//

// Parses which commands to execute when in joint space mode
void jointCommands(String inMsg)
{
  char function = inMsg[2];
  char detail1 = inMsg[3];

  if(!jointFlag) // Check if we are switching from cartesian mode
  {
    IKFlag = false;
    jointFlag = true;
    cartesianToJoint();
    setJointSpeed();
  }

if(function == move)
  moveArm(detail1, inMsg[4]);

else if(function == release)
  relArm(detail1);
}

void moveArm(char axis, char dir) 
{
   int axisNum = String(axis).toInt(); // String to int for math
    
   if((dir == left) || (dir == up))
   {
      moveAxis((axisNum-1), FWD);
   }
   else if((dir == right) || (dir == down))
   {
      moveAxis((axisNum-1), REV); 
   }
}

void relArm(char axis)
{
  int axisNum = String(axis).toInt(); // String to int for math
  steppers[axisNum-1].stop();
  runFlags[axis] = 0;
}

void moveAxis(int axis, int dir)
{
 if((axis == 0) || (axis == 1)) { // switching direction if axis 1 or 2 (arm moves in intuitive dir)
  dir = !dir;
 }
 
 if(dir == FWD && runFlags[axis] != 1) { // sets stepper to move to max steps and sets run flag so doesn't keep executing
  steppers[axis].moveTo(max_steps[axis]);
  runFlags[axis] = 1;
  }
else if (dir == REV && runFlags[axis] != -1) { // sets stepper to move to min steps and sets run flag so doesn't keep executing
  steppers[axis].moveTo(min_steps[axis]);
  runFlags[axis] = -1;
  }
}

// sets Joint Mode speeds after switching out of cartesian mode
void setJointSpeed()
{
  for(i = 0; i<NUM_AXES; i++) {
  steppers[i].setMaxSpeed(speedVals[maxSpeedIndex][i]);
  steppers[i].setAcceleration(maxAccel[i]);
  }
}

void cartesianToJoint() 
{
  readEncPos(curEncSteps);
  for(i=0; i<NUM_AXES; i++)
  {
    steppers[i].setCurrentPosition(curEncSteps[i]/ENC_MULT[i]);
  }
}

//***//CARTESIAN MODE FUNCTIONS//***//

void cartesianCommands(String inMsg)
{

  if(jointFlag) 
  {
    IKFlag = true;
    jointFlag = false;
    setCartesianSpeed();
  }
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
  cmdArm();
}

void setCartesianSpeed()
{
  float JOINT_MAX_SPEED[NUM_AXES];
  float MOTOR_MAX_SPEED[NUM_AXES];
  float JOINT_MAX_ACCEL[NUM_AXES];
  float MOTOR_MAX_ACCEL[NUM_AXES];

  for(int i=0; i<NUM_AXES; i++)
  {
    JOINT_MAX_SPEED[i] = IKspeeds[i]*(180.0/3.14159);
    JOINT_MAX_ACCEL[i] = IKaccs[i]*(180.0/3.14159);
    MOTOR_MAX_SPEED[i] = JOINT_MAX_SPEED[i] * ENC_STEPS_PER_DEG[i] / ENC_MULT[i];
    MOTOR_MAX_ACCEL[i] = JOINT_MAX_ACCEL[i] * ENC_STEPS_PER_DEG[i] / ENC_MULT[i];
    steppers[i].setAcceleration(MOTOR_MAX_ACCEL[i]);
    steppers[i].setMaxSpeed(MOTOR_MAX_SPEED[i]);
  }
}


 //***// END EFFECTOR RELATED FUNCTIONS //***//

void endEffectorCommands(String inMsg)
{
  char data = inMsg[2];

  //opening code
  if (data == open) { //check if open button pressed and if force is less than max
    endEff.moveTo(openPos*MOTOR_DIR_EE); //continue to move to open position
  }

  //closing code
  else if (data == close) { //check if open button pressed and if force is less than max
    endEff.moveTo(closePos*MOTOR_DIR_EE); //continue to move to closed position
  }

  else if (data == release) { //else check if release button pressed
    endEff.stop(); // stop when above condition reached
  }

  else if (data == homeValEE)
  {
    if(resetEE)
    {
      endEff.setCurrentPosition(1000);
      resetEE = false;
    }

    else
    {
      endEff.setCurrentPosition(0);
      resetEE = true;
    }
  }
}

void getEEForce()
{
  if(scale.wait_ready_timeout(1))
  {
    float force = scale.get_units()/1000*9.81;
    forcePct = force*100.0/maxForce;
  }
}

void sendEEForce()
{
  String force_value = String(forcePct);
  String force_message = String("EE: Gripper Force: ") + String(force_value) + String(" Z");
  Serial.print(force_message);
}

//***// DRILL RELATED FUNCTIONS //***//

void drillCommands(String inMsg)
{
  char function = inMsg[2];

  if(function == manual)
    manualDrill(inMsg[3]);
  else if(function == drillRelease)
    stopDrill();
  else if(function == prepare)
    spinDrill();
  else if(function == collect)
    stopDrill();
  else if(function == deposit)
    depositSample();
}

void manualDrill(char dir)
{
  if(dir == left)
  {
    endEff.move(99000);
  }

  else
  {
    endEff.move(-99000);
  }
}

void stopDrill()
{
  endEff.stop();
}

void spinDrill()
{
  endEff.move(99000);
}

void collectSample()
{
 // not currently implemented
}

void depositSample()
{
 // not currently implemented
}

// Set target position for cartesian movements of arm
void cmdArm()
{
  for (int i = 0; i < NUM_AXES; i++)
  { 
    int diffEncSteps = cmdEncSteps[i] - curEncSteps[i];
    if (abs(diffEncSteps) > 5)
    {
    int diffMotSteps = diffEncSteps / ENC_MULT[i];
    if (diffMotSteps < ppr[i])
    {
      diffMotSteps = diffMotSteps / 2;  
    }

    steppers[i].move(diffMotSteps*axisDirIK[i]);
    }
  }
}

//***//ENCODER RELATED FUNCTIONS//***//

void readEncPos(int* encPos)
{
  encPos[0] = enc1.read()*ENC_DIR[0];
  encPos[1] = enc2.read()*ENC_DIR[1];
  encPos[2] = enc3.read()*ENC_DIR[2];
  encPos[3] = enc4.read()*ENC_DIR[3];
  encPos[4] = enc5.read()*ENC_DIR[4];
  encPos[5] = enc6.read()*ENC_DIR[5];
}

void zeroEncoders()
{
  enc1.write(0);
  enc2.write(0);
  enc3.write(0);
  enc4.write(0);
  enc5.write(0);
  enc6.write(0);

  readEncPos(curEncSteps);
}

//****// ARM CALIBRATION FUNCTIONS//****//

void homeArm() { // main function for full arm homing
  initializeHomingMotion();
  homeAxes();
  initializeMotion();
  zeroEncoders();

  // clear serial port in case of garbage values
  while(Serial.available() !=0)
  {
    Serial.read();
  }

  // notify hardware driver that homing has completed
  Serial.print("HCZ");
}

// Runs through and checks if each axis has reached its limit switch, then runs it to specified home position
void homeAxes() {
  bool stopFlags[6] = {false, false, false, false, false, false};
  bool finishFlags[6] = {false, false, false, false, false, false};
  bool completeFlag = false;
  int count = 0;
  
  while(!completeFlag) {

    for(i = 0; i < NUM_AXES; i++) {

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
    if(count == NUM_AXES) {
      completeFlag = true;
    }
  }
}

void initializeHomingMotion() { // sets homing speed and acceleration and sets target homing position

  for(i = 0; i<NUM_AXES; i++) {

    steppers[i].setMaxSpeed(homeSpeed[i]);
    steppers[i].setAcceleration(homeAccel[i]);
    steppers[i].setCurrentPosition(0);

    if(i == 0) // Special case for axis 1
    {
      if((axisDir[0]*steppers[0].currentPosition() < -homeCompSteps[0]))
      {
        steppers[0].move(-homePos[i]);
      }
      else
      {
        steppers[0].move(homePos[i]);
      }
    }

    else 
    {
      steppers[i].move(homePos[i]);
    }
    
  }
}


void initializeMotion() { // sets main program speeds for each axis and zeros position

  for(i = 0; i<NUM_AXES; i++) {
    steppers[i].setMaxSpeed(speedVals[maxSpeedIndex][i]);
    steppers[i].setAcceleration(maxAccel[i]);
    steppers[i].setCurrentPosition(0);
  }
}

void waitForHome()
{
  String inData;
  char recieved = '\0';

  while(!initFlag)
  {
    inData = "";
    if(Serial.available()>0)
    {
      do {
        recieved = Serial.read();
        inData += String(recieved);
      } while(recieved != '\n');
    }
  
    if(recieved == '\n')
    {
      if(inData.substring(0, 2) == "HM")
        homeArm();
        initFlag = true;
    }
  }
}


// OLD FUNCTIONS THAT MAY BE USED IN THE FIRMWARE //

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

// void changeSpeed(char speedVal) { // changes speed of all axes based on user input
  
//   if(speedVal == faster){
//     if(speedIndex < maxSpeedIndex) {
//       speedIndex++;
//       for(i=0;i<NUM_AXES;i++) {
//         steppers[i].setMaxSpeed(speedVals[speedIndex][i]);
//       }
//     }
//   }

//   else if(speedVal == slower) {
//     if(speedIndex > 0) {
//       speedIndex--;
//       for(i=0;i<NUM_AXES;i++) {
//         steppers[i].setMaxSpeed(speedVals[speedIndex][i]);
//       }
//     }
//   }
// }