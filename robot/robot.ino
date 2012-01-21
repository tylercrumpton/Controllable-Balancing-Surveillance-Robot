#define MOTOR_ONE_PIN	9
#define MOTOR_TWO_PIN	10

int motorOneDelta;
int motorTwoDelta;
int balanceDelta;

void setup()
{
  motorOneDelta = 0;
  motorTwoDelta = 0;
  balanceDelta  = 0;	
}

void loop()
{
  balance();
}


int calcPID(int value, int target)
{
  //TODO: Calculate PID error.
}

void balance()
{
  int gyro  = readGyro();
  int accel = readAccel();
  int angle = getAngle(gyro, accel);
  int balanceDelta = calcPID(angle, 0);
  motorOneSpeed = forwardSpeed;
  motorTwoSpeed = forwardSpeed;
}

int readGyro()
{
  //TODO: Read gyro data (analog)
}

int readAccel()
{
  //TODO: Read accelerometer data (analog?)
}

int getAngle(int gyroSpeed, int acceleration)
{
  //TODO: Use some sort of filter to get actual angle.
}
