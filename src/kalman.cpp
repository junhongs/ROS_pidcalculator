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
   H = 1;
}
PV_kalman::PV_kalman()
   : dt(0)
{
   R << 1, 0, 0, 2;
   H = 1;
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
   P << 10, 0, 0, 10;
}
void PV_kalman::Correct() {
   Kalman_gain = P_estimated * ( P_estimated + R ).inverse();
   X = X_estimated + Kalman_gain * (X_measured - X_estimated);
   current_position = X(0, 0);
   P = P_estimated - Kalman_gain * P_estimated;
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
   Q *= 100;
   return ret;
}









class stereo_system_model {

public:
   static const float constant_ab = 0.00375f / 2.97f / 2.0f;
   static const float constant_depth = ( 2.97f * 450.0f / 0.00375f );
   static const float width = 1280.0f;
   static const float height = 960.0f;


   static inline float H_convert_leftx(float x, float y, float z) {

      return 1.0f / 2.0f / z * ( constant_depth + x / constant_ab ) + width / 2.0f;

   }
   static inline float H_convert_rightx(float x, float y, float z) {

      return 1.0f / 2.0f / z * ( -constant_depth + x / constant_ab  ) + width / 2.0f;
   }
   static inline float H_convert_y(float x, float y, float z) {
      return height / 2.0f - y / z * constant_ab / 2.0f;
   }


   static inline float leftx_diff_X(float x, float y, float z ) {
      return 1.0f / z / 2.0f / constant_ab;
   }
   static inline float leftx_diff_Z(float x, float y, float z) {
      return - 1.0f / 2.0f * (constant_depth + x / constant_ab) / z / z;
   }
   static inline float rightx_diff_X(float x, float y, float z) {
      return 1.0f / 2.0f / z / constant_ab;
   }
   static inline float rightx_diff_Z(float x, float y, float z) {
      return - 1.0f / 2.0f * (x / constant_ab - constant_depth) / z / z;
   }
   static inline float y_diff_Y(float x, float y, float z) {
      return - 1.0f / z / constant_ab / 2.0f;
   }
   static inline float y_diff_Z(float x, float y, float z) {
      return 1.0f / constant_ab / 2.0f * y / z / z;
   }
   static Matrix<float, 3, 1> calc_3d_pos(Matrix<float, 3, 1> _Z) {
      Matrix<float, 3, 1> X;

      float depth =  constant_depth / (_Z(0, 0) - _Z(1, 0));
      float tx = depth * constant_ab * ( (_Z(0, 0) + _Z(1, 0)) - width);
      float ty = - depth * constant_ab * ( 2 * _Z(2, 0) - height);

      X(0, 0) = tx;
      X(1, 0) = ty;
      X(2, 0) = depth;

      return X;
   }

   static Matrix<float, 6, 1> calc_init_3d_pos(Matrix<float, 3, 1> _Z) {
      Matrix<float, 6, 1> X;
      Matrix<float, 3, 1> tmp_X;
      tmp_X = calc_3d_pos(_Z);

      X << tmp_X(0, 0), tmp_X(1, 0), tmp_X(2, 0), 0, 0, 0;

      return X;
   }
};



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

