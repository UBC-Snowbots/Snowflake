/*
Created By: Tate Kolton, Graeme Dockrill
Created On: November 11, 2022
Updated On: January 10, 2023
Description: Header file for firmware for driving a 6 axis arm via ROS on a teensy 4.1 MCU
*/

#include <AccelStepper.h>
#include <HX711.h>
#include <Encoder.h>

// general parameters
#define NUM_AXES 6
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
float red[6] = {50.0, 160.0, 93.07, 43.08, 19, 14};

// End effector variables
const float calibrationFactor = -111.25;
float force;
HX711 scale;
const int dataPin = 34;
const int clkPin = 33;
int calPos = 0;
int closePos = 0;
int openPos = 500; 
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

// Encoder Variables
int curEncSteps[NUM_AXES], cmdEncSteps[NUM_AXES];
int pprEnc = 512;
int ENC_DIR[6] = {-1, -1, -1, -1, -1, -1};
const float ENC_MULT[] = {5.12, 5.12, 5.12, 5.12, 5.12, 5.12};
float ENC_STEPS_PER_DEG[NUM_AXES];

// Motor speeds and accelerations
int maxSpeed[6] = {1200, 1800, 3000, 2500, 2200, 2200};
int maxAccel[6] = {1300, 3500, 4600, 3300, 5000, 5000};
int homeSpeed[6] = {1200, 1500, 1500, 2000, 500, 500}; 
int homeAccel[6] = {1500, 2000, 2000, 2000, 500, 500}; 

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

// Stepper motor objects for AccelStepper library
AccelStepper steppers[6];
AccelStepper endEff(1, EEstepPin, EEdirPin);
AccelStepper steppersIK[6];

// Encoder objects for Encoder library
Encoder enc1(encPinA[0], encPinB[0]);
Encoder enc2(encPinA[1], encPinB[1]);
Encoder enc3(encPinA[2], encPinB[2]);
Encoder enc4(encPinA[3], encPinB[3]);
Encoder enc5(encPinA[4], encPinB[4]);
Encoder enc6(encPinA[5], encPinB[5]);

// General Global Variable declarations

int axisDir[6] = {1, -1, -1, 1, 1, 1};  
int axisDirIK[6] = {-1, -1, -1, 1, -1, -1};
int runFlags[] = {0, 0, 0, 0, 0, 0};
int currentAxis = 1;
int i;
bool initFlag = false;
bool jointFlag = false;
bool IKFlag = false;
bool resetEE = false;
bool vertFlag = false;
bool horizFlag = false;

// Variables for homing / arm calibration
long homePosConst = -99000;
long homePos[] = {axisDir[0]*homePosConst, axisDir[1]*homePosConst, axisDir[2]*homePosConst, axisDir[3]*homePosConst, axisDir[4]*homePosConst, axisDir[5]*homePosConst};
long homeCompAngles[] = {50, 0, 0, 110, 30, 30};
long homeCompConst[] = {500, 500, 200, 500, 500, 500};
long homeComp[] = {axisDir[0]*homeCompConst[0], axisDir[1]*homeCompConst[1], axisDir[2]*homeCompConst[2], axisDir[3]*homeCompConst[3], axisDir[4]*homeCompConst[4], axisDir[5]*homeCompConst[5]};
long homeCompSteps[] = {axisDir[0]*homeCompAngles[0]*red[0]*ppr[0]/360.0, axisDir[1]*homeCompAngles[1]*red[1]*ppr[1]/360.0, axisDir[2]*homeCompAngles[2]*red[2]*ppr[2]/360.0, axisDir[3]*homeCompAngles[3]*red[3]*ppr[3]/360.0, axisDir[4]*homeCompAngles[4]*red[4]*ppr[4]/360.0, axisDir[5]*homeCompAngles[5]*red[5]*ppr[5]/360.0};

// Range of motion (degrees) for each axis
int maxAngles[6] = {160, 160, 180, 180, 180, 340};
long max_steps[] = {axisDir[0]*red[0]*maxAngles[0]/360.0*ppr[0], axisDir[1]*red[1]*maxAngles[1]/360.0*ppr[1], axisDir[2]*red[2]*maxAngles[2]/360.0*ppr[2], axisDir[3]*red[3]*maxAngles[3]/360.0*ppr[3], red[4]*maxAngles[4]/360.0*ppr[4], red[5]*maxAngles[5]/360.0*ppr[5]};
long min_steps[NUM_AXES]; 
char value;

// Values for changing speed
const int maxSpeedIndex = 2;
int speedVals[maxSpeedIndex+1][NUM_AXES] = {{600, 900, 1500, 1250, 1050, 1050}, {900, 1200, 2000, 1665, 1460, 1460}, {900, 1600, 2500, 2200, 2000, 2000}};
int speedIndex = maxSpeedIndex;

// Cartesian mode speed settings
float IKspeeds[] = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
float IKaccs[] = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3};


//****// FUNCTION PROTOTYPES //****//

// Waits for a message to be published to the serial port.
void recieveCommand();

// Parses the message passed to it and determines which subfunction to call for control of arm.
void parseMessage(String inMsg);

// Takes a character and prints it to the serial port.
void sendMessage(char outChar);

// If the arm is in joint mode, the encoder positions are read and sent over serial.
void sendFeedback(String inMsg);

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

// Measures the end effector force.
void getEEForce();

// Prints end effector force to serial.
void sendEEForce();

// Selects which drill function to execute based on inMsg[2].
void drillCommands(String inMsg);

// Manually spins drill based on user input.
void manualDrill(char dir);

// Stops the drill.
void stopDrill();

// Moves drill to max travel.
void spinDrill();

// Not yet implemented
void collectSample();

// Not yet implemented.
void depositSample();

// Prints the current encoder position for each axis to serial.
void sendCurrentPosition();

// Reads each encoder's position and stores them in encPos[].
void readEncPos(int* encPos);

// Zeroes each encoder at its current position.
void zeroEncoders();

// For closed loop movement (Inverse Kin Mode)
void cmdArm();

// changes speed of all axes based on user input.
void changeSpeed(char speedVal);

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