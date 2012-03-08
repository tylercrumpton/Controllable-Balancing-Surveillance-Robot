#include <PID_v1.h>

#define LEFT_EN_PIN  5
#define LEFT_L4_PIN  10 
#define LEFT_L3_PIN  7 
#define RIGHT_EN_PIN 11
#define RIGHT_L2_PIN 12
#define RIGHT_L1_PIN 6 
  


#define MAX_TILT          30  // Maximum controllable tilt 
#define MAX_MOTOR_SPEED   255 // Maximum PWM for motor control
#define MAX_TURN_SPEED    25  // Maximum PWM differntial for turning
                              // (NOTE: MAX_TURN_SPEED determines max balance speed.)


                              
double angle = 0;            //Filtered angled value                              

// PID Coeffs
double KP = 100.0;
double KI = 22.0;
double KD = 0.05;
// Control Coeffs
double CP = 0.0010;
double CI = 0.75;
double CD = 0.0 ;

double desiredTilt;  // Desired tilt of robot (+/- MAX_TILT)
int desiredSpeed;  // Desired speed of robot (+/- MAX_MOTOR_SPEED)
double balanceSpeed; // Base motor speed for balance (+/- MAX_MOTOR_SPEED)
int turnSpeed;    // Turn speed differential of robot (+/- MAX_TURN_SPEED)


// Setup for PID computation
double setpointPID, inputPID, outputPID;
PID bPID(&inputPID, &outputPID, &setpointPID, KP, KI, KD, DIRECT);

double setpointCPID, inputCPID, outputCPID;
PID cPID(&inputCPID, &outputCPID, &setpointCPID, CP, CI, CD, DIRECT);


void setup()
{
  Serial.begin(9600);
  
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
  // Force PID to the range of motor speeds. (+/- MAX_MOTOR_SPEED - MAX_TURN_SPEED)
  //bPID.SetOutputLimits(-(MAX_MOTOR_SPEED - MAX_TURN_SPEED), MAX_MOTOR_SPEED - MAX_TURN_SPEED); 
  bPID.SetOutputLimits(-255,255);
  bPID.SetSampleTime(2);
  cPID.SetOutputLimits(-25,25);
  cPID.SetSampleTime(2);
  initSensors();
  	
}

void loop()
{
  
  //balance();
}

// Calculate balanceSpeed given desired speed.
int calcPID(double input, int target)
{
  inputPID = input;   // Set PID input to tilt angle.
  setpointPID = (balanceSpeed - target) * CP;
  bPID.Compute();     // Compute correction, store in outputPID.
  balanceSpeed = outputPID;
}


void balance()
{
  // Read the sensors and calculate a tilt angle.
  //double angle = getCurrentTilt();
  
  // Try to balance the robot at the desired angle.
  

  // Update the motor PWM values.
  updateMotors();
}

// Converts the value between +/- MAX_MOTOR_SPEED to an actual PWM signal and direction
void updateMotors()
{
 
  double tempLeftSpeed = (balanceSpeed - turnSpeed);    // Left motor speed from -255 to 255
  double tempRightSpeed = (balanceSpeed + turnSpeed)*0.8;   // Reft motor speed from -255 to 255
  
  analogWrite(LEFT_EN_PIN, byte(abs(tempLeftSpeed)));   // Set PWM as magnitude of left speed (0 to 255)
  analogWrite(RIGHT_EN_PIN, byte(abs(tempRightSpeed))); // Set PWM as magnitude of right speed (0 to 255)
  
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
    digitalWrite(RIGHT_L1_PIN, LOW); // If negative, direction = LOW;
    digitalWrite(RIGHT_L2_PIN, HIGH);   // If negative, direction = LOW;
  }
  else 
  {
    digitalWrite(RIGHT_L1_PIN, HIGH); // If positive, direction = HIGH
    digitalWrite(RIGHT_L2_PIN, LOW);   // If negative, direction = LOW;
  }
}

