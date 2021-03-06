#include <ros/ros.h>

#include <iostream>
#include <stdio.h>
#include <time.h>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "kalman.h"
using namespace Eigen;


PV_kalman::PV_kalman( Matrix<float, 2, 2> _P, Matrix<float, 2, 2> _R)
   : P(_P), R(_R), dt(0), H(1)
{
   current_position = 0.0f;
   last_position = 0.0f;
   last_time = 0.0l;
}

PV_kalman::PV_kalman()
   : dt(0), H(1), initial_P(10), initial_Q(100), initial_R(1)
{
   R << 1, 0, 0, 2;
   R *= initial_R;
   current_position = 0.0f;
   last_position = 0.0f;
   last_time = 0.0l;
}

PV_kalman::PV_kalman(float _P, float _Q, float _R)
   : dt(0), H(1), initial_P(_P), initial_Q(_Q), initial_R(_R)
{
   R << 1, 0, 0, 2;
   R *= initial_R;
   current_position = 0.0f;
   last_position = 0.0f;
   last_time = 0.0l;
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
   P << 1, 0, 0, 1;
   P *= initial_P;
}

void PV_kalman::Correct() {
   Kalman_gain = P_estimated * H * ( H * P_estimated * H + R ).inverse();
   X = X_estimated + Kalman_gain * (X_measured - X_estimated);
   current_position = X(0, 0);
   P = P_estimated - Kalman_gain * H * P_estimated;
}

void PV_kalman::Measure(float _position) {
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
   Q *= initial_Q;
   return ret;
}





PV3_kalman::PV3_kalman(Matrix<float, 6, 1> _X, Matrix<float, 6, 6> _P, Matrix<float, 3, 3> _R)
   : X(_X), P(_P), R(_R), dt(0)
{
   last_time = 0.0l;
}

PV3_kalman::PV3_kalman()
   : dt(0)
{
   R <<
     1, 0, 0,
     0, 1, 0,
     0, 0, 1;

   R *= 2;
   last_time = 0.0l;
}

Matrix<float, 6, 1> PV3_kalman::getKalman(Matrix<float, 3, 1> _Z_measured) {
   if (cycle_time()) {
      InitK(_Z_measured);
   }
   else {
      Measure(_Z_measured);
      Predict();
      Correct();
   }
   return X;
}

void PV3_kalman::Predict() {
   X_estimated = A * X;
   P_estimated = A * P * A.transpose() + Q;
}

void PV3_kalman::InitK(Matrix<float, 3, 1> _Z_measured) {
   // X(0, 0) = _position;
   // X(1, 0) = 0;
   X = stereo_system_model::calc_init_3d_pos(_Z_measured);
   P <<
     1, 0, 0, 0, 0, 0,
     0, 1, 0, 0, 0, 0,
     0, 0, 1, 0, 0, 0,
     0, 0, 0, 2, 0, 0,
     0, 0, 0, 0, 2, 0,
     0, 0, 0, 0, 0, 2
     ;
   P *= 30;
}

void PV3_kalman::Correct() {
   Kalman_gain = P_estimated * H.transpose() * ( H * P_estimated * H.transpose() + R ).inverse();

   Matrix<float, 3, 1> _tmp_Z;
   _tmp_Z(0, 0) = stereo_system_model::H_convert_leftx(X_estimated(0, 0), X_estimated(1, 0), X_estimated(2, 0));
   _tmp_Z(1, 0) = stereo_system_model::H_convert_rightx(X_estimated(0, 0), X_estimated(1, 0), X_estimated(2, 0));
   _tmp_Z(2, 0) = stereo_system_model::H_convert_y(X_estimated(0, 0), X_estimated(1, 0), X_estimated(2, 0));
   X = X_estimated + Kalman_gain * (Z_measured - _tmp_Z);
   P = P_estimated - Kalman_gain * H * P_estimated;
}

void PV3_kalman::Measure(Matrix<float, 3, 1> _Z_measured) {
   Z_measured = _Z_measured;

   float hax = stereo_system_model::leftx_diff_X(X_estimated(0, 0), X_estimated(1, 0), X_estimated(2, 0));
   float haz = stereo_system_model::leftx_diff_Z(X_estimated(0, 0), X_estimated(1, 0), X_estimated(2, 0));
   float hbx = stereo_system_model::rightx_diff_X(X_estimated(0, 0), X_estimated(1, 0), X_estimated(2, 0));
   float hbz = stereo_system_model::rightx_diff_Z(X_estimated(0, 0), X_estimated(1, 0), X_estimated(2, 0));
   float hcy = stereo_system_model::y_diff_Y(X_estimated(0, 0), X_estimated(1, 0), X_estimated(2, 0));
   float hcz = stereo_system_model::y_diff_Z(X_estimated(0, 0), X_estimated(1, 0), X_estimated(2, 0));

   H <<   hax, 0, haz, 0, 0, 0,
   hbx, 0, hbz, 0, 0, 0,
   0, hcy, hcz, 0, 0, 0;

}

void PV3_kalman::Compare(Matrix<float, 3, 1> _Z_measured) {

}

int PV3_kalman::cycle_time() {
   int ret = 0;
   double cur_time = ros::Time::now().toSec();
   if (last_time == 0.0l) {
      ret = 1;
   }
   dt = cur_time - last_time;
   last_time = cur_time;
   A <<
     1, 0, 0, dt, 0, 0,
     0, 1, 0, 0, dt, 0,
     0, 0, 1, 0, 0, dt,
     0, 0, 0, 1, 0, 0,
     0, 0, 0, 0, 1, 0,
     0, 0, 0, 0, 0, 1;

   float p4 = pow(dt, 4) / 4;
   float p3 = pow(dt, 3) / 2;
   float p2 = pow(dt, 2);

   Q <<
     p4 , 0 , 0 , p3 , 0 , 0 ,
     0 , p4 , 0 , 0 , p3 , 0 ,
     0 , 0 , p4 , 0 , 0 , p3 ,
     p3 , 0 , 0 , p2 , 0 , 0 ,
     0 , p3 , 0 , 0 , p2 , 0 ,
     0 , 0 , p3 , 0 , 0 , p2;

   Q *= 20;
   return ret;
}

