



/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the Uno and
  Leonardo, it is attached to digital pin 13. If you're unsure what
  pin the on-board LED is connected to on your Arduino model, check
  the documentation at http://www.arduino.cc

  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald
 */


#include "MatrixMath.h"
#include "Servo.h"
#include "DueTimer.h"


//#define ServiceServoUpdateWithInterrupt
//#define CheckKinematicsProcessingWeight
//#define FreeComForDebug 
//#define SlowDownTransmitionsForDebug
//#define TransmitPackageInCompactModeForDebug
//#define DebugTrigonometry
//#define DebugMatrixCalculations 





float dummy[20];

//General Definitions
const int obLED = 13; //Onboard LED Pin
const int iBaudRate = 250000; //Serial port baudrate
const int iBaudRateSerial3 = 250000; //Serial port baudrate


#define en 6 //Number of end effector degrees of freedom 
#define qn 6 //Number of joints (or actuators) on robot 



//Declarations for joystick
int ijsCoordSys = 0; //Coordinate system of manual control (0->JointSystem; 1->Global Coordinates; 2->End effector coordinates)
int ijsAxisControlled[2]; //Index of the controlled coordinate
int ijsInputMode; //Input mode for joystick (0->manual axis control; 1-Coordinate System Select; 2-Coordinate Select)
float fjsJoyStickDirCart[]= { -1.0, 1.0,  1.0,  1.0,  1.0,  1.0, 1.0}; //Used to invert direction of joystick if convenient
float fjsJoyStickDirJoint[]= {1.0, 1.0,  1.0,  1.0,  1.0,  1.0, 1.0}; //Used to invert direction of joystick if convenient
const int jsX = A8; //Joystick X Analogue Input
const int jsY = A9; //Joystick Y Analogue Input
const int jsBut = 53; //Joystick Pushbutton
const int jsSupplyVcc = 50; //Joystick Supply Pin Vcc
const int jsSupplyVss = 52; //Joystick Supply Pin Vcc
int ijsButDebounce = 1; // Debounce counter variable
boolean jsButDB = false; // Debounced value of input 
boolean jsButLS = false; // Last scan value of input (usefull to detect rising or falling edges)
int ijsPosition[2]; //Joystick position
float fjsQuadraticSignal[2]; // A Quadratic Function is softer at low speeds and faster at high speeds, it gives the user more controlability
const float fjsQuadraticDivider = 0x20000; // = 512^2
const int ijsNeutralPosition = 512; //Analogue input value in central/neutral position
const int ijsDeadRange = 15; //Maximum deviation to ignore
const float fjsThreshold = 0.5; //To use the joystick as a digital scroll input 
const float fjsHysteresis = 0.2; //Since it's an analogue input, it's better to set a hysteresis then a debounce
boolean jsScrollUp[2]; //Logic signal activated when joystick is pushed over the defined threshold
boolean jsScrollUpLS[2]; //Last scan value of variable (usefull to detect rising or falling edges)
boolean jsScrollDown[2]; //Logic signal activated when joystick is pushed over the defined threshold
boolean jsScrollDownLS[2]; //Last scan value of variable (usefull to detect rising or falling edges)
int ijsHighestAxInd; //Highst possible axis index depending on coordinate system
long ljsLastButPushedTime;
#if (en > qn)
float fjsDelta[en];
#else
float fjsDelta[qn];
#endif

//Declarations for cyclic scan definitions
long currentTime; //Current time in milliseconds
const long lUserInputScan = 100; //Scan user inputs every 100ms
long lUserInputScan_L; //Last time user inputs were scaned 
const long lKineReCalc = 5; //Recalc kinematics every XXms
long lKineReCalc_L; //Last time kinematics were recalulated
#ifdef SlowDownTransmitionsForDebug
const long lUserSerialInfoScan = 500; //Update info on Serial port every half a second
#else
const long lUserSerialInfoScan = 50; //Update info on Serial port every half a second
#endif
long lUserSerialInfoScan_L; //Last time info was updated on serial port
const long lUpdateServos = 5;  //Update servo position every 1 ms (1ms/servo)
long lUpdateServos_L = 1;  //Last time Servos were updated
bool EndEffectorUpToDate; //True if e is calculated with most recent q data

//declarations for robot joints
Servo servos[6];// 0, q1, q2, q3, q4, q5; //servo robot joints
int servoPin[] = { 22, 23, 24, 25, 26, 27 };//Joint servo pin numbers
int iServoToUpdate = 1; //Interrupt routine updates only one servo per cycle

int iTransmitVar;

//Declarations for kinematics
float q[qn]; //joint state vector considering theoretical kinematics
float qServos[qn]; //Real Servo positions with fisical offsets and inversions
float qServos_L[qn]; //Real Servo positions on the Last Scan
float qZeros[] =      {  98.0, -110.0,  88.0,    88.0, 107.0,  71.0,    0.0 };// q ->qJoints Zero Positions (if servo direction is negative, subtract 180º)
float qDirections[] = {   1.0,  -1.0,   1.0,      1.0,   1.0,   1.0,    0.0 };//q ->Use this vector to invert servo directions
float qMins[] =       { -98.0, -70.0, -88.0,    -88.0,-107.0, -62.0,    0.0 };
float qMaxs[] =       {  82.0,  70.0,  92.0,     92.0,  73.0, 109.0,    0.0 };
float qAvStrokes[qn]; //Available stroke remaining on side that has least available stroke (If negativive, shortest distance on negative way)
float eAvStrokes[en]; //Available stroke remaining on side that has least available stroke (If negativive, shortest distance on negative way)
float e[en]; //End effector current position and orientation
float er[en]; //End effector required position and orientation
float eUsOff[] = {  -35.0,  -35.0,  77.0,  0.0,  00.0,  0.0, 0.0 }; //User friendly offsets
float fDistToTargetPos; //Distance between e and er (position)
float fDistToTargetOri;  //Distance between e and er (orientation)
float d_e_norm;
float d_e[en]; //End effector direction of movement
float d_q[qn]; //Joint vector direction of movement
float delta_q[qn]; //Joint position increment per scan depending on d_q and velocity
float cq[qn];
float sq[qn];
float q1q2; // q2 - q1
float q22;
float cq1q2;
float sq1q2;
float cq22;
float sq22;
float q1q22; // q1 + q22
float cq1q22;
float sq1q22;
float dsq22_dq1q2;
float dcq22_dsq22;
float dq22_dsq22;
float dq22_dq1q2;
float J[en][qn];
float Jtrans[qn][en]; //In transposed matrixes dimentions ({qn][en]) are reversed
float J_Jtrans_inv[en][en]; //This becomes a square matrix
float Jinv[qn][en]; //Jinv dimentions are also inverted when compared to J
float JinvTemp[3][3]; //Temporary J inverse;
float Jp[3][3];
float JpInv[3][3];
float JpInvTemp[3][3];
float Jo[3][3];
float JoInv[3][3];
float JoInvTemp[3][3];
float JoTrans[3][3];
float JoJoTransInv[3][3];
float rx;
float ry;
float rz;
float srx;
float sry;
float srz;
float drx_dsrx;
float dry_dsry;
float drz_dsrz;



float Teegc[4][4];  //End effector coordinates to global coordinates transformation
float Tgcee[4][4];  //Global coordinates to end effector coordinates transformation
bool bLastJpInversionSuccessful;
bool bLastJoInversionSuccessful;


//Declarations for dinamics
//Joint Feed rates
const float fMaxManualJointVelocity = 30.0; //Maximum rotational velociy when on manual joystick º/s 
const float fManualJointAccel = 1.0; //Manual rotational acceleration rpm/s
float fMaxManualJointFeedPerServoCycle;
float fManualJointFeedVariationPerServoCycle; //Manual rotational acceleration rpm/s
//Cartesian Position feedrate
const float fMaxManualLinVelocity  = 50.0; //Maximum linear velociy when on manual joystick mm/s
const float fManualLinAccel = 200.0; //Manual linear acceleration mm/s^2
float fMaxManualLinFeedPerServoCycle;
float fManualLinFeedVariationPerServoCycle; //Manual linear acceleration mm/s^2
//Cartesian orientation feedrate
const float fMaxManualRotVelocity = 30.0; //Maximum rotational velociy when on manual joystick º/s 
const float fManualRotAccel = 1.0; //Manual rotational acceleration rpm/s
float fMaxManualRotFeedPerServoCycle;
float fManualRotFeedVariationPerServoCycle; //Manual rotational acceleration rpm/s
//Currentfeedrates on each axis
float fManualCurrentJointFeedRate[qn]; //In units per servo cycle
float fManualCurrentCartFeedRate[en];  //In units per servo cycle

