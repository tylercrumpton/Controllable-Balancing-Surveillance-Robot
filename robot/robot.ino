// Notes on tuning from Lauszus:
/* 1. Select typical operating setting for desired speed, turn off integral and derivative part, and then increase Kp to max or until oscillation occurs.
   2. If system oscillates, divide Kp by 2.
   3. Increase Kd and observe behaviour when changing desired speed by about 5% and choose a value of Kd that gives a fast damped response.
   4. Slowly increase Ki until oscillation starts. Then divide Ki by 2 or 3.
*/

//INCLUDES----------------------------------------------------------
#include <PinChangeInt.h> 
#include <AdaEncoder.h>
#include <PID_v1.h>
#include <TimerOne.h>
#include <math.h>
#include <KalmanFilter.h>

//Constants---------------------------------------------------------
//Define Motor Controller Pins
#define LEFT_EN_PIN  5
#define LEFT_L4_PIN  10 
#define LEFT_L3_PIN  7 
#define RIGHT_EN_PIN 11
#define RIGHT_L2_PIN 12
#define RIGHT_L1_PIN 6

//Define Motor Encoder Pins
#define L_PINA A0
#define L_PINB A1
#define R_PINA A2
#define R_PINB A3

//Define Sensor Pins
#define gyroPin A4
#define accelPin A5

//Define Constants
#define pi 3.14159;
#define samplePeriod      20000     //Time in uS for sampling loop period.
#define MAX_TILT          10       // Maximum controllable tilt in degrees
#define MAX_MOTOR_SPEED   255      // Maximum PWM for motor control

//Global Variables---------------------------------------------------
//Encoder Counts
int L_Clicks, R_Clicks = 0;

//Encoder Pointers (set by genie fuction in AdaEncoder library)
int8_t clicks=0;
char id=0;

//Balance Variables                              
double angle = 0;            //Filtered angled value        

double accelVal = 0;        //Raw accel value from ADC 0-1023
double gyroVal = 0;         //Raw gyro value from ADC 0-1023

int accelOffset = 0;        //Raw offset for accelerometer
int gyroOffset = -8;       //Raw offset for gyro

double L_clicks_last = 0;      //Encoder clicks last cycle
double R_clicks_last = 0;

// PID Coeffs
double KP = 70;   //100
double KI = 400;   //22
double KD = .006;  //.05
// Control Coeffs
double CP = .0; //.0010
double CI = 0   ;   //.75
double CD = 0.000  ;   //.0
// Turn PID Coeffs
double TP = 0; //250
double TI = 00; //500
double TD = 0;  //0.09

double desiredTilt;  // Desired tilt of robot (+/- MAX_TILT)
double desiredSpeed;  // Desired speed of robot (+/- MAX_MOTOR_SPEED)
double balanceSpeed; // Base motor speed for balance (+/- MAX_MOTOR_SPEED)
int turnSpeed;    // Turn speed differential of robot (+/- MAX_TURN_SPEED)


// Setup for PID computation
double setpointPID, inputPID, outputPID;
PID bPID(&inputPID, &outputPID, &setpointPID, KP, KI, KD, DIRECT);
 
double setpointCPID, inputCPID, outputCPID;
PID cPID(&inputCPID, &outputCPID, &setpointCPID, CP, CI, CD, DIRECT);

double setpointTPID, inputTPID, outputTPID;
PID TPID(&inputTPID, &outputTPID, &setpointTPID, TP, TI, TD, DIRECT); //turn correction pid function

// Setup Kalman Filter
KalmanFilter kmf;


void setup()
{
  Serial.begin(115200);
  
  // Initialize variables.
  turnSpeed    = 0;
  desiredTilt  = 0;
  desiredSpeed = 0;
  balanceSpeed = 0;
  
  // Setup pin IO:
  pinMode(LEFT_EN_PIN, OUTPUT);
  pinMode(LEFT_L3_PIN, OUTPUT);
  pinMode(LEFT_L4_PIN, OUTPUT);
  pinMode(RIGHT_EN_PIN, OUTPUT);
  pinMode(RIGHT_L1_PIN, OUTPUT);
  pinMode(RIGHT_L2_PIN, OUTPUT);
  
  // Turn on the PID.
  bPID.SetMode(AUTOMATIC);
  cPID.SetMode(AUTOMATIC);
  
  TPID.SetMode(AUTOMATIC);
  
  // Force PID to the range of motor speeds. (+/- MAX_MOTOR_SPEED - MAX_TURN_SPEED)
  //bPID.SetOutputLimits(-(MAX_MOTOR_SPEED - MAX_TURN_SPEED), MAX_MOTOR_SPEED - MAX_TURN_SPEED); 
  bPID.SetOutputLimits(-MAX_MOTOR_SPEED,MAX_MOTOR_SPEED);
  bPID.SetSampleTime(2);
  cPID.SetOutputLimits(-MAX_TILT,MAX_TILT);
  cPID.SetSampleTime(2);
  
  TPID.SetOutputLimits(-15,15);
  TPID.SetSampleTime(2);
  
  initSensors();
  
  //Setup motor encoders
  AdaEncoder::addEncoder('a', L_PINA, L_PINB);
  AdaEncoder::addEncoder('b', R_PINA, R_PINB);
  	
}

void loop()
{
  encoder *thisEncoder;
  thisEncoder=AdaEncoder::genie(&clicks, &id);
  if (thisEncoder != NULL) {
    //thisEncoder=AdaEncoder::getFirstEncoder();
    
    if (id =='a')
    {
      L_Clicks += clicks;
    }
    if (id =='b')
    {
      R_Clicks += clicks;
    }
    thisEncoder->clicks = 0;
  }
  /*Serial.print(L_Clicks);
  Serial.print("\t");
  Serial.println(R_Clicks);*/
}


// Converts the value between +/- MAX_MOTOR_SPEED to an actual PWM signal and direction
void updateMotors()
{
 
  double tempLeftSpeed = (balanceSpeed - turnSpeed-outputTPID);    // Left motor speed from -255 to 255
  double tempRightSpeed = (balanceSpeed + turnSpeed+outputTPID);   // Reft motor speed from -255 to 255
  
  analogWrite(LEFT_EN_PIN, byte(abs(tempLeftSpeed)));   // Set PWM as magnitude of left speed (0 to 255)
  analogWrite(RIGHT_EN_PIN, byte(abs(tempRightSpeed))*.85); // Set PWM as magnitude of right speed (0 to 255)
  
  if (tempLeftSpeed < 0) 
  {
    digitalWrite(LEFT_L3_PIN, LOW);   // If negative, direction = LOW;
    digitalWrite(LEFT_L4_PIN, HIGH);   // If negative, direction = LOW;
  }
  else
  {
    digitalWrite(LEFT_L3_PIN, HIGH);  // If positive, direction = HIGH
    digitalWrite(LEFT_L4_PIN, LOW);   // If negative, direction = LOW;
  
  }
  
  if (tempRightSpeed < 0) 
  {
    digitalWrite(RIGHT_L1_PIN, HIGH); // If negative, direction = LOW;
    digitalWrite(RIGHT_L2_PIN, LOW);   // If negative, direction = LOW;
  }
  else 
  {
    digitalWrite(RIGHT_L1_PIN, LOW); // If positive, direction = HIGH
    digitalWrite(RIGHT_L2_PIN, HIGH);   // If negative, direction = LOW;
  }
}

