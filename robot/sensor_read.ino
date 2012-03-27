




//Variables used for debugging--CAN DELETE
double angleOnlyG, angleOnlyA = 0;
int timeNow,prevTime = 0;


void initSensors()
{
  //attachInterrupt(1,accelGetValue, CHANGE);         //Interrupt setup on pin 2--triggers on both edges for accel
  Timer1.initialize(samplePeriod);                  //Time one overflow defines sample period
  Timer1.attachInterrupt(sampling_loop);            //Interrupr on overflow calls sampling_loop function
}


//Currently 50HZ
void sampling_loop()
{
  double L_speed = L_Clicks - L_clicks_last;
  double R_speed = R_Clicks - R_clicks_last;
  L_clicks_last = L_Clicks;
  R_clicks_last = R_Clicks;
  double avg_speed = ((L_speed+R_speed)/2.0)*.10776;                  //RPS
  double timeScale = 1000000 /samplePeriod;                          //Compute scalefactor for gyro from degrees per sec.
  double gyroAngleRate = gyroToDegreesPerSec();
  double accelAngle = accelToAngle();
  angle = kmf.calculate(accelAngle, gyroAngleRate, timeScale);            //Compute new angl measurement. (sampleRate is 20ms)
  
  //DEBUG STATEMENTS
  /*angleOnlyG = 1.0*(angleOnlyG + (gyroAngle)) +(0.00*accelAngle);
  angleOnlyA = 0.0*(angle + (gyroAngle)) +(1.0*accelAngle);
  Serial.print(angle);
  Serial.print("\t");
  Serial.print(angleOnlyG);
  Serial.print("\t");
  Serial.print(angleOnlyA);
  Serial.print("\t");
  Serial.println(gyroVal);*/
  
  
 
  //inputCPID = balanceSpeed;
  
  inputCPID = avg_speed;
  setpointCPID = desiredSpeed;
  cPID.Compute();
  
  setpointPID = outputCPID; 
  inputPID = angle;   // Set PID input to tilt angle.
  bPID.Compute();     // Compute correction, store in outputPID.
  balanceSpeed = outputPID;
  
  /*if (balanceSpeed > 0 && balanceSpeed < 40)
  {
    balanceSpeed = 40;
  }
  if (balanceSpeed < 0 && balanceSpeed > -40)
  {
    balanceSpeed = -40;
  }*/
  
  setpointTPID = 0;
  inputTPID = L_speed - R_speed;
  TPID.Compute(); 
  
  updateMotors();
  
  Serial.print(angle);
  Serial.print("\t");
  Serial.print(setpointPID);
  Serial.print("\t");
  Serial.print(angle-setpointPID);
  Serial.print("\t");
  /*Serial.print(outputTPID);
  Serial.println("\t");*/
  Serial.print(balanceSpeed);
  Serial.print("\t");
  Serial.println(avg_speed-desiredSpeed);
  /*Serial.print("\t");
  Serial.println(desiredSpeed);*/
  
  
  
}

double getCurrentTilt()
{
  return angle;
}

//Takes the raw acceleration PWM signal and converts to a degree.  Vertical is 0 degrees.
double accelToAngle()
{
  int avg[15] = {0};
  double sum = 0;
  for(int i = 0;i<15;i++)
  {
    sum += analogRead(accelPin)-accelOffset;
  }
   accelVal = sum/15.0;
   double accelMilliG = (accelVal-338)/.164;
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
}





