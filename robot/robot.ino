#include <PID_v1.h>

#define LEFT_PWM_PIN   10
#define LEFT_DIR_PIN   11
#define RIGHT_PWM_PIN  12
#define RIGHT_DIR_PIN  13

#define MAX_TILT          30  // Maximum controllable tilt 
#define MAX_MOTOR_SPEED   255 // Maximum PWM for motor control
#define MAX_TURN_SPEED    25  // Maximum PWM differntial for turning
                              // (NOTE: MAX_TURN_SPEED determines max balance speed.)

// PID Coeffs
double KP = 1;
double KI = 0.05;
double KD = 0.25;

int desiredTilt;  // Desired tilt of robot (+/- MAX_TILT)
int balanceSpeed; // Base motor speed for balance (+/- MAX_MOTOR_SPEED)
int turnSpeed;    // Turn speed differential of robot (+/- MAX_TURN_SPEED)


// Setup for PID computation
double setpointPID, inputPID, outputPID;
PID bPID(&inputPID, &outputPID, &setpointPID, KP, KI, KD, DIRECT);

void setup()
{
  // Initialize variables.
  turnSpeed    = 0;
  desiredTilt  = 0;
  balanceSpeed = 0;
  
  // Setup pin IO:
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  
  // Turn on the PID.
  bPID.SetMode(AUTOMATIC);
  // Force PID to the range of motor speeds. (+/- MAX_MOTOR_SPEED - MAX_TURN_SPEED)
  bPID.SetOutputLimits(-(MAX_MOTOR_SPEED - MAX_TURN_SPEED), MAX_MOTOR_SPEED - MAX_TURN_SPEED); 
  	
}

void loop()
{
  balance();
}

// Calculate balanceSpeed given tilt angle.
int calcPID(double input, int target)
{
  inputPID = input;   // Set PID input to tilt angle.
  bPID.Compute();     // Compute correction, store in outputPID.
  balanceSpeed = outputPID;
}


void balance()
{
  // Read the sensors and calculate a tilt angle.
  double angle = getCurrentTilt();
  
  // Try to balance the robot at the desired angle.
  balanceSpeed = calcPID(angle, desiredTilt);
  
  // Update the motor PWM values.
  updateMotors();
}

// Converts the value between +/- MAX_MOTOR_SPEED to an actual PWM signal and direction
void updateMotors()
{
  int tempLeftSpeed = balanceSpeed - turnSpeed;    // Left motor speed from -255 to 255
  int tempRightSpeed = balanceSpeed + turnSpeed;   // Reft motor speed from -255 to 255
  
  analogWrite(LEFT_PWM_PIN, byte(abs(tempLeftSpeed)));   // Set PWM as magnitude of left speed (0 to 255)
  analogWrite(RIGHT_PWM_PIN, byte(abs(tempRightSpeed))); // Set PWM as magnitude of right speed (0 to 255)
  
  if (tempLeftSpeed < 0) {digitalWrite(LEFT_DIR_PIN, LOW);}   // If negative, direction = LOW;
  else {digitalWrite(LEFT_DIR_PIN, HIGH);}  // If positive, direction = HIGH
  
  if (tempRightSpeed < 0) {digitalWrite(RIGHT_DIR_PIN, LOW);} // If negative, direction = LOW;
  else {digitalWrite(RIGHT_DIR_PIN, HIGH);} // If positive, direction = HIGH
}

