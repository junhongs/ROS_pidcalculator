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

	static void calc_3d_pos(float lx, float rx, float y, float *ox, float *oy, float *oz) {

		float depth =  constant_depth / (lx - rx);
		float tx = depth * constant_ab * ( (lx + rx) - width);
		float ty = - depth * constant_ab * ( 2 * y - height);

		*ox = tx;
		*oy = ty;
		*oz = depth;

	}

	static Matrix<float, 6, 1> calc_init_3d_pos(Matrix<float, 3, 1> _Z) {
		Matrix<float, 6, 1> X;
		Matrix<float, 3, 1> tmp_X;
		tmp_X = calc_3d_pos(_Z);

		X << tmp_X(0, 0), tmp_X(1, 0), tmp_X(2, 0), 0, 0, 0;

		return X;
	}
};

class PV_kalman {
public:
	double dt;
	Matrix<float, 2, 2> A;
	Matrix<float, 2, 2> Q;
	Matrix<float, 2, 2> R;
	float H;
	float initial_P, initial_Q, initial_R;
	Matrix<float, 2, 1> X;
	Matrix<float, 2, 1> X_measured;
	Matrix<float, 2, 1> X_estimated;
	Matrix<float, 2, 2> P;
	Matrix<float, 2, 2> P_estimated;
	Matrix<float, 2, 2> Kalman_gain;

	float last_position;
	float current_position;

	double last_time;

	PV_kalman(Matrix<float, 2, 2> _P, Matrix<float, 2, 2> _R);
	PV_kalman(float _P, float _Q, float _R);
	PV_kalman();

	Matrix<float, 2, 1> getKalman(float _position);

	float getKalman_1(float _position);

	float getpos() {return X(0, 0);};
	float getvel() {return X(1, 0);};

	void Predict();

	void InitK(float _position);

	void Correct();

	void Measure(float _position);


	void Compare(float _position) ;
	float calc_vel(float _position, float _last_position);

	int cycle_time();


private:
};

class PV3_kalman {
public:
	double dt;
	Matrix<float, 6, 6> A;
	Matrix<float, 6, 6> Q;
	Matrix<float, 3, 3> R;

	Matrix<float, 6, 1> X;
	Matrix<float, 3, 6> H;
	Matrix<float, 3, 1> Z_measured;
	Matrix<float, 6, 1> X_estimated;
	Matrix<float, 6, 6> P;
	Matrix<float, 6, 6> P_estimated;
	Matrix<float, 6, 3> Kalman_gain;

	double last_time;
	PV3_kalman(Matrix<float, 6, 1> _X, Matrix<float, 6, 6> _P, Matrix<float, 3, 3> _R);
	PV3_kalman();
	Matrix<float, 6, 1> getKalman(Matrix<float, 3, 1> _Z_measured);
	void Predict();
	void InitK(Matrix<float, 3, 1> _Z_measured);
	void Correct();
	void Measure(Matrix<float, 3, 1> _Z_measured);
	void Compare(Matrix<float, 3, 1> _Z_measured) ;
	int cycle_time();

private:
};

#endif