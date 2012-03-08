//WORK IN PROGRESS
//Implements a complimetary filter to combine accelerometer and gyro readings into to one 
//angle in degrees/

/*Constraints
-Must allow time for initial stabilization of filtered angle 3-4 sec
-Extremem Rapid horizonal acceleraton will result in an incorrect angle for a small period of time 
*/


#include <TimerOne.h>//<-------THIRD PARTY LIBRARY.  DOWNLOAD FROM ARDUINO WEBSITE TO RUN

//Pin definitions and constants
const double pi=3.14159;
const int gyroPin = A4;
const int accelPin = 3;
const double samplePeriod = 20000;  //Time in uS for sampling loop period.
int tiltOffset = 0;


int accelVal = 0;            //Raw accel value in width of high pulse from PWM
int gyroVal = 0;             //Raw gyro value from ADC 0-1023
int highTime,lowTime = 0;    //Used for measuring the accelVal, Must be global for interrupt.
//double angle = 0;            //Filtered angled value

int accelOffset = 0;        //Raw offset for accelerometer
int gyroOffset = 1;        //Raw offset for gyro



//Variables used for debugging--CAN DELETE
double angleOnlyG, angleOnlyA = 0;
int timeNow,prevTime = 0;


void initSensors()
{
  attachInterrupt(1,accelGetValue, CHANGE);         //Interrupt setup on pin 2--triggers on both edges for accel
  Timer1.initialize(samplePeriod);                  //Time one overflow defines sample period
  Timer1.attachInterrupt(sampling_loop);            //Interrupr on overflow calls sampling_loop function
}


//Currently 50HZ
void sampling_loop()
{
  cli(); 
  double timeScale = 1000000 /samplePeriod;                          //Compute scalefactor for gyro from degrees per sec.
  double gyroAngle = -gyroToDegreesPerSec()/timeScale;
  double accelAngle = -accelToAngle();
  angle = 0.97*(angle + (gyroAngle)) +(0.03*accelAngle);            //Compute new angl measurement.
  
  //DEBUG STATEMENTS
  //angleOnlyG = 1.0*(angleOnlyG + (gyroAngle)) +(0.00*accelAngle);
  //angleOnlyA = 0.0*(angle + (gyroAngle)) +(1.0*accelAngle);
  /*Serial.print(angle);
  Serial.print("\t");
  Serial.print(angleOnlyG);
  Serial.print("\t");
  Serial.print(angleOnlyA);
  Serial.print("\t");
  Serial.println(gyroVal);*/
  
  //Serial.print(angle);
  //Serial.print("\t");
  //Serial.print(balanceSpeed);
  //Serial.print("\t");
  //Serial.println(setpointPID);
  
  
  
//  if (balanceSpeed > 0)
//    setpointPID = setpointPID + 0.75;
//  else if (balanceSpeed < 0)
//    setpointPID = setpointPID - 0.75;
//  else
//    setpointPID = setpointPID;
//    
//  if (setpointPID > 25)
//  {
//    setpointPID = 25;
//  }
//  else if(setpointPID < -25)
//  {
//    setpointPID = -25;
//  }
  //setpointPID = (balanceSpeed - 0) * CP;
  
  //setpointPID = 0;
  inputCPID = balanceSpeed;
  setpointCPID = desiredSpeed;
  cPID.Compute();
  
  inputPID = angle;   // Set PID input to tilt angle.
  setpointPID = -outputCPID;
  bPID.Compute();     // Compute correction, store in outputPID.
  balanceSpeed = outputPID;
  
  updateMotors();
  
  sei();
  
  
}

double getCurrentTilt()
{
  return angle;
}

//Takes the raw acceleration PWM signal and converts to a degree.  Vertical is 0 degrees.
double accelToAngle()
{
   double accelMilliG = (((accelVal-accelOffset)/10) - 500) * 8;
   if (accelMilliG > 1000)
   {
     return 90;
   }
   else if (accelMilliG < -1000)
   {
     return -90;
   }
   else
   {
     double accelAngle = asin(accelMilliG/1000)*180/pi;
     return accelAngle;
   }
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
  cli();
  
  if (digitalRead(accelPin) == HIGH)        //Read time of rising edge
  {
    highTime = micros(); 
  }
  else                                      //Read time of falling edge
  {
    lowTime = micros();
    accelVal = lowTime - highTime;          //Calculate width of pulse
  }
  sei();
}

/*void GyroCalibrate()
{
   boolean complete = false;
   int indexNum = 100;
   long val[indexNum] = {0};
   int percentError = 100;
   int index1, index2 = 0;
   double sum, avg = 0;
   while(complete == false)
   {
     if index1 > indexNum;
     {
       index2 = index1 % indexNum;
     }
     
     if (index1 < indexNum || percentError > 5)
     {
        val[index2] = analogRead(A1);
        
        for (int = 0; i < 100; i++)
        {
           sum = sum + val[i];
        }
        avg = sum/indexNum;
        
        
     }
     
     sum = 0;
     index ++;
   }
}*/


