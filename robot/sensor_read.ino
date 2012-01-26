//WORK IN PROGRESS
//Implements a complimetary filter to combine accelerometer and gyro readings into to one 
//angle in degrees/

/*BUGS
-Fails at extreme degrees--I think this is do to an interrupt timing issue.
-Cannot print multiple values for compariosn--Also I believe timing issue.
*/

/*Constraints
-Must allow time for initial stabilization of filtered angle 3-4 sec
-Extremem Rapid horizonal acceleraton will result in an incorrect angle for a small period of time 
*/


#include <TimerOne.h>//<-------THIRD PARTY LIBRARY.  DOWNLOAD FROM ARDUINO WEBSITE TO RUN

//Pin definitions and constants
const int pi=3.14159;
const int gyroPin = A1;
const int accelPin = 2;
const int samplePeriod = 20000;  //Time in uS for sampling loop period.


int accelVal = 0;            //Raw accel value in width of high pulse from PWM
int gyroVal = 0;             //Raw gyro value from ADC 0-1023
int highTime,lowTime = 0;    //Used for measuring the accelVal, Must be global for interrupt.
double angle = 0;            //Filtered angled value

int accelOffset = 0;        //Raw offset for accelerometer
int gyroOffset = -3;        //Raw offset for gyro



//Variables used for debugging--CAN DELETE
double angleOnlyG, angleOnlyA = 0;
int timeNow,prevTime = 0;


void setup()
{
  Serial.begin(9600);
  attachInterrupt(0,accelGetValue, CHANGE);         //Interrupt setup on pin 2--triggers on both edges for accel
  Timer1.initialize(samplePeriod);                  //Time one overflow defines sample period
  Timer1.attachInterrupt(sampling_loop);            //Interrupr on overflow calls sampling_loop function
}

void loop()
{
  //Do nothing.  Interrput Driven Program
}

//Currently 50HZ
void sampling_loop()
{
  double timeScale = 1000000/samplePeriod;                          //Compute scalefactor for gyro from degrees per sec.
  double gyroAngle = gyroToDegreesPerSec()/timeScale;
  double accelAngle = accelToAngle();
  angle = 0.97*(angle + (gyroAngle)) +(0.03*accelAngle);            //Compute new angl measurement.
  
  //DEBUG STATEMENTS
  angleOnlyG = 1.0*(angleOnlyG + (gyroAngle)) +(0.00*accelAngle);
  angleOnlyA = 0.0*(angle + (gyroAngle)) +(1.0*accelAngle);
  //Serial.println(angle);
  //Serial.print("\t");
  //Serial.print(angleOnlyG);
  //Serial.print("\t");
  //Serial.println(angleOnlyA);
  Serial.println(angle);
  
  
}

//Takes the raw acceleration PWM signal and converts to a degree.  Vertical is 0 degrees.
double accelToAngle()
{
   double accelMilliG = (((accelVal-accelOffset)/10) - 500) * 8;
   double accelAngle = asin(accelMilliG/1000)*180/pi;
   return accelAngle;
}

//Takes the raw analog value from the acelerometer and converts to degrees per sec.
double gyroToDegreesPerSec()
{
  double gyroAnglePerSec = 0;
  gyroVal = analogRead(gyroPin)-gyroOffset;
  gyroAnglePerSec = (gyroVal - 512)/1.023;
  return gyroAnglePerSec;
  
  //Possible idea to filter out noise.
  /*if (gyroAnglePerSec < 2.0 && gyroAnglePerSec > -2.0)
  {
    return 0;
  }
  else
  {
    return gyroAnglePerSec;
  }*/
}


//Interrupt driven function to capture the raw accel value.  Interrputs on rising and falling edges of the accel Pin
void accelGetValue()
{
  
  if (digitalRead(accelPin) == HIGH)        //Read time of rising edge
  {
    highTime = micros(); 
  }
  else                                      //Read time of falling edge
  {
    lowTime = micros();
    accelVal = lowTime - highTime;          //Calculate width of pulse
  }
}


