/*
  KalmanFilter.h - Library for using a KalmanFilter with Gyro/Accel inputs.
  Put together into library form by Tyler Crumpton.
  
  (Based on code from "www.x-firm.com/?page_id=191")
  
  March 25, 2012 - v0.1: Initial commit. 
  
  Example usage:
  
  KalmanFilter km; //Do this in setup or pre-init
  ...
  filteredAngle = km.calculate(accelAngle, gyroAngleRate, sampleRate); // Call this to get your good, filtered angle.
*/

#ifndef KalmanFilter_h
#define KalmanFilter_h

#include "Arduino.h"

class KalmanFilter
{
  public:
    KalmanFilter();
    float calculate(float newAngle, float newRate,int looptime);
  private:
    float _Q_angle;
    float _Q_gyro;
    float _R_angle;
    
    float _x_angle;
    float _x_bias;
    float _P_00;
    float _P_01;
    float _P_10;
    float _P_11;
    float _dt;
    float _y;
    float _S;
    float _K_0;
    float _K_1;
};

#endif


