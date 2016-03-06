#ifndef _KALMAN_H
#define _KALMAN_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <vector>
#include <cmath>
using namespace Eigen;



class PV_kalman {
public:
   double dt;
   Matrix<float, 2, 2> A;
   Matrix<float, 2, 2> Q;
   Matrix<float, 2, 2> R;
   float H;

   Matrix<float, 2, 1> X;
   Matrix<float, 2, 1> X_measured;
   Matrix<float, 2, 1> X_estimated;
   Matrix<float, 2, 2> P;
   Matrix<float, 2, 2> P_estimated;
   Matrix<float, 2, 2> Kalman_gain;

   float measure;
   float last_position;
   float current_position;

   double last_time;

   PV_kalman(Matrix<float, 2, 1> _X, Matrix<float, 2, 2> _P, Matrix<float, 2, 2> _R);
 
   PV_kalman();

   Matrix<float, 2, 1> getKalman(float _position);

   float getKalman_1(float _position);
 
   void Predict();

   void InitK(float _position);
 
   void Correct();

   void Measure(float _position);
 

   void Compare(float _position) ;
   float calc_vel(float _position, float _last_position);

   int cycle_time();


private:
};



#endif