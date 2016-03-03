#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Point.h"

#include "geometry_msgs/PointStamped.h"
#include "pcl_msgs/Vertices.h"
#include "iostream"
#include <sstream>
#include "math.h"
#include <string>


#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <iostream>
#include <fstream>


#include <Eigen/Dense>

#include "calculation.h"


#if 0
typedef struct pos_vel_t {
	float cur_pos;
	float cur_vel;
	float last_pos;
	float last_vel;
	double cur_time;
	double last_time;
	double cycle_time;

	float last_vel_raw;
	float cur_vel_raw;
} pos_vel_t;

void calc_velocity(pos_vel_t* pos_vel) {
	int is_lpf = 1;
	if (pos_vel->last_time && pos_vel->cur_time) {
		pos_vel->cycle_time = pos_vel->cur_time - pos_vel->last_time;

		//if ros's cycle period is fast (in my case, about 1000hz), sometime the cycle period might have some noize over 30%. it must be corrected.
		// if(pos_vel->cycle_time > 0.0013 || pos_vel->cycle_time < 0.0007){
		//         pos_vel->last_vel = pos_vel->cur_vel;
		//         pos_vel->last_time = pos_vel->cur_time;
		//         pos_vel->last_pos = pos_vel->cur_pos;
		//     return;
		// }
	}
	if (pos_vel->cur_pos && pos_vel->last_pos)
		pos_vel->cur_vel = pos_vel->cur_pos - pos_vel->last_pos;

	if (pos_vel->cycle_time)
		pos_vel->cur_vel /= pos_vel->cycle_time;

	pos_vel->cur_vel_raw = pos_vel->cur_vel;

	if (pos_vel->last_vel && is_lpf)
		pos_vel->cur_vel = (pos_vel->cur_vel + pos_vel->last_vel) / 2;

	pos_vel->last_vel = pos_vel->cur_vel;
	pos_vel->last_time = pos_vel->cur_time;
	pos_vel->last_pos = pos_vel->cur_pos;
	pos_vel->last_vel_raw = pos_vel->cur_vel_raw;

}
#endif


using namespace std;
using namespace Eigen;



class position_PV_kalman {
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

	position_PV_kalman(
	    Matrix<float, 2, 1> _X,
	    Matrix<float, 2, 2> _P,
	    Matrix<float, 2, 2> _R
	) : X(_X), P(_P), dt(0), R(_R)
	{
		// H << 1, 0;
		R << 1, 0, 0, 2;
		current_position = 0;
		last_position = 0;
		// X << 0,0;
		// P << 10,0,0,10;
	}

	void Predict() {
		X_estimated = A * X;
		P_estimated = A * P * A.transpose() + Q;
	}

	void Correct(float _position) {
		Kalman_gain = P_estimated * ( P_estimated + R ).inverse();

		X = X_estimated + Kalman_gain * (X_measured - X_estimated);
		current_position = X(0, 0);

		P = P_estimated - Kalman_gain * P_estimated;
	}

	void Measure(float _position) {
		// if(cycle_time()){
		// 	//first time to call

		// }
		X_measured(0, 0) = _position;
		X_measured(1, 0) = calc_vel(_position, X(0, 0) );

	}

	void Compare(float _position) {

	}

	float calc_vel(float _position, float _last_position) {
		float vel = (_position - _last_position) / dt;
		return vel;
	}

	int cycle_time() {
		int ret = 0;
		double cur_time = ros::Time::now().toSec();
		if (!last_time)
			ret = 1;
		dt = cur_time - last_time;
		last_time = cur_time;

		A << 1, dt, 0, 1;
		Q << pow(dt, 4) / 4, pow(dt, 3) / 2 , pow(dt, 3) / 2 , pow(dt, 2);
		Q *= 2;

		return ret;
	}
private:
};


void calc_3d() {
	float lx, rx, y;

	float a1, b1, c1;
	float tx, ty, tz;

	lx = 502;
	rx = 623;
	y = 101;

	c1 =  2.97f * 450 / ((lx - rx) * 0.00375f);
	a1 = (2.97f * 450 / ((lx - rx) * 0.00375f)) * (0.00375f * (lx + rx - 1280)) / (2 * 2.97f);
	b1 =  (-0.00375f) * ((2 * y) / 2 - 480.0f) * (2.97f * 450 / ((lx - rx) * 0.00375f)) / 2.97f;

	cout << "first : " << a1 << "," << b1 << "," << c1 << endl;


// depth =  ( 2.97 * 450 / 0.00375 ) / (left_x - right_x);
// b1 = - depth * 0.00375 / 2.97 / 2 * ( (left_y + right_y) - 960);
// a1 = depth * 0.00375 / 2.97 / 2 * ( (left_x + right_x) - 1280);

	float constant_ab = 0.00375f / 2.97f / 2.0f;
	float constant_depth = ( 2.97f * 450.0f / 0.00375f );
	float width = 1280.0f;
	float height = 960.0f;
	// cout << "constant : " << constant_ab << ", " << constant_depth << endl;
// depth =  constant_depth / (left_x - right_x);
// b1 = - depth * constant_ab * ( (left_y + right_y) - 960);
// a1 = depth * constant_ab * ( (left_x + right_x) - 1280);
	tz =  constant_depth / (lx - rx);
	tx = tz * constant_ab * ( (lx + rx) - width);
	ty = - tz * constant_ab * ( 2 * y - height);

	cout << "second : " << tx  << "," << ty  << "," << tz << endl;
}




/*
cycle_time() -> Predict() -> {multiple of} Measure() -> Correct()


*/


// depth =  2.97 * 450 / ((left_x - right_x) * 0.00375);



// a1 = depth * ( 0.00375) * ( (left_x + right_x) / 2 - 1280 / 2) / 2.97;

// a1 = depth * ( 0.00375 / 2.97 / 2) * ( (left_x + right_x) - 1280) ;


// b1 = depth * (-0.00375) * ( (left_y + right_y) / 2 - 960 / 2)  /2.97);

// final
// depth =  ( 2.97 * 450 / 0.00375 ) / (left_x - right_x);
// deriv_depth_left_x = -(2.97 * 450 / 0.00375 ) / pow(left_x - right_x,2)

// deriv_depth_left_y =  (2.97 * 450 / 0.00375 ) / pow(left_x - right_x,2)



// b1 = depth * (-0.00375) / 2.97 / 2 * ( (left_y + right_y) - 960);
// a1 = depth * ( 0.00375) / 2.97 / 2 * ( (left_x + right_x) - 1280);



int main(int argc, char **argv) {
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 100);
	ros::Publisher chat_float_pub = n.advertise<std_msgs::Float64>("generate_sin_pulse", 100);
	// ros::Publisher chat_point_pub = n.advertise<geometry_msgs::Point32>("potition1", 100);
	ros::Publisher chat_point_pub = n.advertise<geometry_msgs::PointStamped>("potition1", 100);
	ros::Publisher float_pub = n.advertise<std_msgs::Float32>("calculated_pid_talker", 100);
	ros::Publisher position_pub = n.advertise<geometry_msgs::Point>("/FIRST/CURRENT_POS", 100);
	ros::Rate loop_rate(30);

	int count = 0;


	// int fd = open("/tmp/save", O_RDWR | O_CREAT | O_TRUNC, 0777);


	// char buf[100] = "hello";
	double buff = 1.21234l;
	// sprintf(buf,"%lf",buff);
	// write(fd,buf,sizeof(buf));
	// printf("%d\n",fd);


	// std::ofstream myfile;
	// myfile.open ("/tmp/example.txt");
	// //myfile << "buff\n";
	// myfile.close();

// #define MAX_SIZE 1000
// 	char inputString[MAX_SIZE];


	// std::ofstream outFile("/tmp/output.txt");

	// for (int i = 0 ; i < 10 ; i++) {
	// 	outFile << i << std::endl;
	// }

	// outFile.close();




	// Matrix2f mat, mat2, mat3;
	// Matrix<float, 2, 2> mat4;
	// mat << 1, 2, 3, 4;
	// mat2 = Matrix2f::Identity(2, 2);
	// Vector2f vec;
	// vec = mat.diagonal();
	// cout << "diagonal" << endl << vec << endl << endl << endl;
	// cout << "transpose" << endl << mat.transpose() << endl << endl;
	// cout << "inverse" << endl << mat.inverse() << endl << endl;
	// cout << "*" << endl << mat * mat2 << endl << endl;
	// cout << "/" << endl << mat * mat2.inverse() << endl << endl;
	calc_3d();



	double dt = 0.1;
	Matrix<float, 2, 2> A, Q;
	Matrix<float, 1, 2> H;

	Matrix<float, 2, 1> X;
	Matrix<float, 2, 1> Kalman_gain;

	Matrix<float, 2, 1> X_estimated;


	Matrix<float, 2, 2> P;


	float R;
	float position_k, velocity_k;


	X << 1, 0;

	A << 1, dt, 0, 1;
	H << 1, 0;
	Q << pow(dt, 4) / 4, pow(dt, 3) / 2 , pow(dt, 3) / 2 , pow(dt, 2);
	R = 2;


	X_estimated = A * X;


	// cout << Kalman_gain << endl;
	// cout << H * A << endl;


	// // cout << "dig" << endl << mat * mat2.inverse() << endl<<endl;
	// std::ifstream inFile("/tmp/pidparam1");
	// cout << "FILE TEST" << inFile.is_open() << endl;
	// std::string tmp_st("/tmp/pidparam");
	// tmp_st += "1";
	// cout << tmp_st << endl;
	while (ros::ok()) {




		if (argv[1] != NULL)
			std::cout  <<  argv[1]  << std::endl;
		//std::cout  <<argv[0]  << std::endl;
		// static ros::Time ros_time_last;
		// ros::Time ros_time = ros::Time::now();
		// std::cout  <<  ros_time - ros_time_last << std::endl;
		// ros_time_last = ros_time;
		geometry_msgs::Point pt_msg;

		pt_msg.x = 0;
		pt_msg.y = 500;
		pt_msg.z = -3000;
		position_pub.publish(pt_msg);

		std_msgs::String msg;
		// std_msgs::Float32 float_msg;

		std_msgs::Float64 float_msg;

		pcl_msgs::Vertices param_msg;

		float_msg.data = 0;
		static float float_msg_tmp = 0;
		float_msg_tmp += 0.01;
		//float_msg.data = sin((double)float_msg_tmp);
		static double curt = ros::Time::now().toSec();
		float_msg.data = ros::Time::now().toSec();
		// std::cout << float_msg.data - curt << std::endl;
		ros::Duration one_hour = ros::Duration(0.5);
#if 0
		static pos_vel_t msg_pos_vel = {0,};
		msg_pos_vel.cur_time = ros::Time::now().toSec();
		msg_pos_vel.cur_pos = float_msg.data;
		calc_velocity(&msg_pos_vel);

		std_msgs::Float32 msg_float;

		msg_float.data = msg_pos_vel.cur_vel / 10 ;
		//msg_float.data = msg_pos_vel.cur_vel * 100/30 ;
		float_pub.publish(msg_float);
#endif

		geometry_msgs::Point32 position1;
		geometry_msgs::PointStamped position_stamp;


		position1.x = 0;
		position1.y = 0;
		position1.z = -3000;


		position_stamp.point.x = 0;
		position_stamp.point.y = 0;
		position_stamp.point.z = 0;

		chatter_pub.publish(msg);
		chat_float_pub.publish(float_msg);
		chat_point_pub.publish(position_stamp);
		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}