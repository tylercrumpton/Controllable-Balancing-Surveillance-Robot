/*
  KalmanFilter.h - Library for using a KalmanFilter with Gyro/Accel inputs.
  Put together into library form by Tyler Crumpton.
  
  (Based on code from "www.x-firm.com/?page_id=191")
  
  March 25, 2012 - v0.1: Initial commit. 
*/

#include "Arduino.h"
#include "KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
    _Q_angle  =  0.001;
    _Q_gyro   =  0.003;
    _R_angle  =  0.03;

    _x_angle = 0;
    _x_bias = 0;
    _P_00 = 0;
    _P_01 = 0;
    _P_10 = 0;
    _P_11 = 0;
}

float KalmanFilter::calculate(float newAngle, float newRate,int looptime) {
  _dt = float(looptime)/1000;
  _x_angle += _dt * (newRate - _x_bias);
  _P_00 +=  - _dt * (_P_10 + _P_01) + _Q_angle * _dt;
  _P_01 +=  - _dt * _P_11;
  _P_10 +=  - _dt * _P_11;
  _P_11 +=  + _Q_gyro * _dt;

  _y = newAngle - _x_angle;
  _S = _P_00 + _R_angle;
  _K_0 = _P_00 / _S;
  _K_1 = _P_10 / _S;

  _x_angle +=  _K_0 * _y;
  _x_bias  +=  _K_1 * _y;
  _P_00 -= _K_0 * _P_00;
  _P_01 -= _K_0 * _P_01;
  _P_10 -= _K_1 * _P_00;
  _P_11 -= _K_1 * _P_01;

  return _x_angle;
}
