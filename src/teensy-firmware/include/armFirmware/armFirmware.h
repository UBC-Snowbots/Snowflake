/*
Created By: Tate Kolton, Graeme Dockrill
Created On: November 11, 2022
Updated On: January 10, 2023
Description: Header file for firmware for driving a 6 axis arm via ROS on a teensy 4.1 MCU
*/

#include <AccelStepper.h>
#include <Encoder.h>

// general parameters
#define NUM_AXES 6
#define ON 0
#define OFF 1
#define SW_ON 0
#define SW_OFF 1
#define FWD 1
#define REV 0

static const char release = 'R';
static const char move = 'M';
static const char right = 'R';
static const char left = 'L';
static const char up = 'U';
static const char down = 'D';
static const char open = 'O';
static const char close = 'C';
static const char EEval = 'E';
static const char homeValEE = 'H';

// Motor pins
int stepPins[6] =   {6, 8, 2, 10, 12, 25}; 
int dirPins[6] =    {5, 7, 1, 9, 11, 24}; 

// Encoder pins
int encPinA[6] = {17, 38, 40, 36, 13, 15};
int encPinB[6] = {16, 37, 39, 35, 41, 14};

// limit switch pins
int limPins[6] = {18, 19, 20, 21, 23, 22};

// pulses per revolution for motors
long ppr[6] = {400, 400, 400, 400, 400, 400};

// Gear Reductions
float red[6] = {50.0, 160.0, 92.3077, 43.936, 57.0, 14.0};

// End effector variables
const int closePos = 0;
const int openPos = 500; 
const int EEstepPin = 4;
const int EEdirPin = 3;
const int speedEE = 500;
const int accEE = 500;
const int MOTOR_DIR_EE = 1;
const int forcePin = 12;


// Encoder Variables
int curEncSteps[NUM_AXES], cmdEncSteps[NUM_AXES];
const int pprEnc = 512;
const float ENC_MULT[] = {5.12, 5.12, 5.12, 5.12, 5.12, 5.12};
float ENC_STEPS_PER_DEG[NUM_AXES];

// Motor speeds and accelerations
const int maxSpeed[6] = {1000, 1800, 2000, 2000, 3000, 2000};
const int maxAccel[6] = {2000, 3000, 3000, 3300, 4000, 3000};
const int homeSpeed[6] = {1500, 2000, 2000, 2500, 1500, 1000}; 
const int homeAccel[6] = {1500, 2000, 2000, 2500, 1500, 1000}; 

// Stepper motor objects for AccelStepper library
AccelStepper steppers[6];
AccelStepper endEff(1, EEstepPin, EEdirPin);

// Encoder objects for Encoder library
Encoder enc1(encPinA[0], encPinB[0]);
Encoder enc2(encPinA[1], encPinB[1]);
Encoder enc3(encPinA[2], encPinB[2]);
Encoder enc4(encPinA[3], encPinB[3]);
Encoder enc5(encPinA[4], encPinB[4]);
Encoder enc6(encPinA[5], encPinB[5]);

// General Global Variable declarations
const int axisDir[6] = {1, -1, -1, 1, -1, -1};  
const int IK_DIR[6] = {1, 1, 1, 1, 1, 1};
int i;
bool initFlag = false;
bool jointFlag = false; 
bool IKFlag = false;
bool resetEE = false;

// Variables for homing / arm calibration
long homePosConst = -99000;
long homePos[] = {axisDir[0]*homePosConst, axisDir[1]*homePosConst, axisDir[2]*homePosConst, axisDir[3]*homePosConst, axisDir[4]*homePosConst, axisDir[5]*homePosConst};
long homeCompAngles[] = {50, 0, 0, 100, 100, 90};
long homeCompConst[] = {500, 1000, 1000, 500, 500, 200};
long homeComp[] = {axisDir[0]*homeCompConst[0], axisDir[1]*homeCompConst[1], axisDir[2]*homeCompConst[2], axisDir[3]*homeCompConst[3], axisDir[4]*homeCompConst[4], axisDir[5]*homeCompConst[5]};
long homeCompSteps[] = {axisDir[0]*homeCompAngles[0]*red[0]*ppr[0]/360.0, axisDir[1]*homeCompAngles[1]*red[1]*ppr[1]/360.0, axisDir[2]*homeCompAngles[2]*red[2]*ppr[2]/360.0, axisDir[3]*homeCompAngles[3]*red[3]*ppr[3]/360.0, axisDir[4]*homeCompAngles[4]*red[4]*ppr[4]/360.0, axisDir[5]*homeCompAngles[5]*red[5]*ppr[5]/360.0};

// Range of motion (degrees) for each axis
int maxAngles[6] = {200, 100, 180, 180, 180, 290};
long max_steps[] = {axisDir[0]*red[0]*maxAngles[0]/360.0*ppr[0], axisDir[1]*red[1]*maxAngles[1]/360.0*ppr[1], axisDir[2]*red[2]*maxAngles[2]/360.0*ppr[2], axisDir[3]*red[3]*maxAngles[3]/360.0*ppr[3], axisDir[4]*red[4]*maxAngles[4]/360.0*ppr[4], axisDir[5]*red[5]*maxAngles[5]/360.0*ppr[5]};
long min_steps[NUM_AXES]; 
char value;

// Cartesian mode speed settings
float IKspeeds[] = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
float IKaccs[] = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3};


//****// FUNCTION PROTOTYPES //****//

// Waits for a message to be published to the serial port.
void recieveCommand();

// Parses the message passed to it and determines which subfunction to call for control of arm.
void parseMessage(String inMsg);

 //For closed loop control. Reads and sends current position. Sets new position based on feedback while the arm is in cartesian mode.
void cartesianCommands(String inMsg);

// Sets the max speed and acceleration for each joint and motor to its Inverse Kinematic speed.
void setCartesianSpeed();

// Parses which commands to execute when in joint space mode.
void jointCommands(String inMsg);

// Sets the max speed and acceleration for each joint and motor to its Joint Mode speed.
void setJointSpeed();

// Takes in the axis to move and the direction, and calls moveAxis in FWD or REV.
void moveArm(char axis, char dir);

// Stops the passed axis and sets its run flag to 0.
void relArm(char axis);

// Changes the axis target position and sets run flag to 1 or -1 depending on dir.
void moveAxis(int axis, int dir);

// Opens, closes, releases, or homes the end effector depending on the data in inMsg [2].
void endEffectorCommands(String inMsg);

// Selects which drill function to execute based on inMsg[2].
void drillCommands(String inMsg);

// Prints the current encoder position for each axis to serial.
void sendArmFeedback();

// Reads each encoder's position and stores them in encPos[].
void readEncPos(int* encPos);

// Zeroes each encoder at its current position.
void zeroEncoders();

// For closed loop movement (Inverse Kin Mode)
int cmdArm();

// Main function for homing the entire arm.
void homeArm();

// Runs through and checks if each axis has reached its limit switch, then runs it a couple steps away from switch for 0 position.
void homeAxes();

// sets homing speed and acceleration for axes 1-6 and sets target homing position.
void initializeHomingMotion();

// sets main program speeds for each axis.
void initializeMotion();

// Runs all stepper motors to target position (if no target defined, motors will not move).
void runSteppers();

// Waits for the user to press the home button before continuing with other functions. Reads serial port and homes the arm if "HM___" is read.
void waitForHome();

// Reads end effector gripper force
int readGripperForce();

// Move arm to GUI specified pose
void executePose(String inMsg);