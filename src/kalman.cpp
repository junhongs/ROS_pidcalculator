#include <ros/ros.h>

#include <iostream>
#include <stdio.h>
#include <time.h>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "kalman.h"
using namespace Eigen;


PV_kalman::PV_kalman(Matrix<float, 2, 1> _X, Matrix<float, 2, 2> _P, Matrix<float, 2, 2> _R)
   : X(_X), P(_P), R(_R), dt(0)
{
   current_position = 0.0f;
   last_position = 0.0f;
   last_time = 0.0l;
   measure = 0.0f;
}
PV_kalman::PV_kalman()
   : dt(0)
{
   R << 1, 0, 0, 2;
   current_position = 0.0f;
   last_position = 0.0f;
   last_time = 0.0l;
   measure = 0.0f;
}
Matrix<float, 2, 1> PV_kalman::getKalman(float _position) {
   if (cycle_time()) {
      InitK(_position);
   }
   else {
      Measure(_position);
      Predict();
      Correct();
   }
   return X;
}
float PV_kalman::getKalman_1(float _position) {
   if (cycle_time()) {
      InitK(_position);
   }
   else {
      Measure(_position);
      Predict();
      Correct();
   }
   return X(0, 0);
}

void PV_kalman::Predict() {
   X_estimated = A * X;
   P_estimated = A * P * A.transpose() + Q;
}

void PV_kalman::InitK(float _position) {
   X(0, 0) = _position;
   X(1, 0) = 0;
   P << 10, 0, 0, 10;
}
void PV_kalman::Correct() {
   Kalman_gain = P_estimated * ( P_estimated + R ).inverse();
   X = X_estimated + Kalman_gain * (X_measured - X_estimated);
   current_position = X(0, 0);
   P = P_estimated - Kalman_gain * P_estimated;
}

void PV_kalman::Measure(float _position) {
   // if(cycle_time()){
   //    //first time to call

   // }
   X_measured(0, 0) = _position;
   X_measured(1, 0) = calc_vel(_position, X(0, 0) );

}

void PV_kalman::Compare(float _position) {

}
float PV_kalman::calc_vel(float _position, float _last_position) {

   float vel = (_position - _last_position) / dt;
   return vel;
}

int PV_kalman::cycle_time() {
   int ret = 0;
   double cur_time = ros::Time::now().toSec();
   if (last_time == 0.0l) {
      ret = 1;
   }
   dt = cur_time - last_time;
   last_time = cur_time;
   A << 1, dt, 0, 1;

   Q << pow(dt, 4) / 4, pow(dt, 3) / 2 , pow(dt, 3) / 2 , pow(dt, 2);
   Q *= 100;
   return ret;
}

