#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Point.h"
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


int main(int argc, char **argv) {
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 100);
	ros::Publisher chat_float_pub = n.advertise<std_msgs::Float32>("generate_sin_pulse", 100);
	ros::Publisher chat_point_pub = n.advertise<geometry_msgs::Point32>("potition1", 100);
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


	// // cout << "dig" << endl << mat * mat2.inverse() << endl<<endl;
	std::ifstream inFile("/tmp/pidparam1");
	cout <<"FILE TEST"<<inFile.is_open() << endl;

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
		std_msgs::Float32 float_msg;
		pcl_msgs::Vertices param_msg;

		float_msg.data = 0;
		static float float_msg_tmp = 0;
		float_msg_tmp += 0.01;
		float_msg.data = sin((double)float_msg_tmp);

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
		position1.x = 0;
		position1.y = 0;
		position1.z = -3000;

		chatter_pub.publish(msg);
		chat_float_pub.publish(float_msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}