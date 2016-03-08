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

using namespace std;
using namespace Eigen;


ros::Publisher target_pub;


void position__Callback2(const geometry_msgs::Point& msg) {
	static float x = msg.x, y = msg.y, z = msg.z;
	static float sphere_x = x;

	static int a = 20;

	if ( x > 0 )
		a = -20;

	if ( x < -1000)
		a = 20;
	x += a;
	cout << x << endl;
	geometry_msgs::Quaternion target_msgs;
	target_msgs.x = x;
	target_msgs.y = y;
	target_msgs.z = z;
	target_msgs.w = MISSION_AUTO_N;
	target_pub.publish(target_msgs);
}
void position__Callback(const geometry_msgs::Point& msg) {
	float radius = 400.0f;
	static float x = msg.x, y = msg.y + radius, z = msg.z;
	static float sphere_x = x, sphere_y = y;

	static float angle_r = 0.0f;

	float linear_speed = 20.0f;
	float angular_speed = linear_speed / radius;

	angle_r += angular_speed;
	int tmpx = x, tmpy = y;

	x = sphere_x + radius * sin(angle_r);
	y = sphere_y - radius * cos(angle_r);

	cout << sqrt((x - tmpx) * (x - tmpx) + (y - tmpy) * (y - tmpy)) << endl;

	cout << x << "," << y <<  endl;

	geometry_msgs::Quaternion target_msgs;
	target_msgs.x = x;
	target_msgs.y = y;
	target_msgs.z = z;
	target_msgs.w = MISSION_AUTO_N;
	target_pub.publish(target_msgs);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 100);
	// ros::Publisher chat_float_pub = n.advertise<std_msgs::Float64>("generate_sin_pulse", 100);
	// ros::Publisher chat_point_pub = n.advertise<geometry_msgs::Point32>("potition1", 100);
	ros::Publisher chat_point_pub = n.advertise<geometry_msgs::PointStamped>("potition1", 100);
	ros::Publisher float_pub = n.advertise<std_msgs::Float32>("calculated_pid_talker", 100);
	ros::Publisher position_pub = n.advertise<geometry_msgs::Point>("/FIRST/CURRENT_POS", 100);
	ros::Rate loop_rate(30);
	target_pub = n.advertise<geometry_msgs::Quaternion>("/FIRST/TARGET_POS", 100);
	// ros::Subscriber position_sub = n.subscribe("/FIRST/CURRENT_POS/", 1, &position__Callback);
	ros::Subscriber position_sub;

	if (argc > 1 && !strcmp(argv[1], "sin")) {
		cout << "sin target pos" << endl;
		position_sub = n.subscribe("/FIRST/CURRENT_POS/", 1, &position__Callback);
	}
	else {
		cout << "line target pos" << endl;
		position_sub = n.subscribe("/FIRST/CURRENT_POS/", 1, &position__Callback2);
	}


	int count = 0;

	ros::spin();

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
		// position_pub.publish(pt_msg);

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
		// chat_float_pub.publish(float_msg);
		chat_point_pub.publish(position_stamp);
		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}



// int fd = open("/tmp/save", O_RDWR | O_CREAT | O_TRUNC, 0777);


// char buf[100] = "hello";
// double buff = 1.21234l;
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
// calc_3d();



// double dt = 0.1;
// Matrix<float, 2, 2> A, Q;
// Matrix<float, 1, 2> H;

// Matrix<float, 2, 1> X;
// Matrix<float, 2, 1> Kalman_gain;

// Matrix<float, 2, 1> X_estimated;


// Matrix<float, 2, 2> P;


// float R;
// float position_k, velocity_k;


// X << 1, 0;

// A << 1, dt, 0, 1;
// H << 1, 0;
// Q << pow(dt, 4) / 4, pow(dt, 3) / 2 , pow(dt, 3) / 2 , pow(dt, 2);
// R = 2;


// X_estimated = A * X;


// cout << Kalman_gain << endl;
// cout << H * A << endl;


// // cout << "dig" << endl << mat * mat2.inverse() << endl<<endl;
// std::ifstream inFile("/tmp/pidparam1");
// cout << "FILE TEST" << inFile.is_open() << endl;
// std::string tmp_st("/tmp/pidparam");
// tmp_st += "1";
// cout << tmp_st << endl;
