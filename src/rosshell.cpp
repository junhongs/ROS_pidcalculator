#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point.h"
#include "pcl_msgs/ModelCoefficients.h"

#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include <iostream>
#include <sstream>
#include <string>
#include "math.h"
#include "param.h"




static int kfd = 0;
static int is_loop = 1;
static struct termios cooked, raw;
static std::string current_program;
static std::stringstream ss;
static std::vector<std::string> argvector;


static ros::Publisher param_pub;

static int is_received_float = 0;
static float float_data = 0;

static int is_received_velocity = 0;
static geometry_msgs::Point velocity_data;

static int is_received_position = 0;
static geometry_msgs::Point position_data;


#define USELESS1 0x1B
#define USELESS2 0x5B
#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_SPACE 0x20
#define KEYCODE_ENTER 0x0A
#define KEYCODE_TAB 0x09
#define KEYCODE_ESC 0x1B
#define KEYCODE_BSP 0x7F


void print_cur() {
	std::cout << current_program << "$ ";
}
void print_cur(std::string str) {
	std::cout << current_program << "$ " << str;
}

void quit(int sig) {
	tcsetattr(kfd, TCSANOW, &cooked);
	is_loop = 0;
	std::cout << "SIGINT is occured\n" << std::endl;
	current_program = "DRONE_SHELL";
	//ros::shutdown();
	//exit(0);
}

void key_debug(std::string str) {
	std::cout << "DEBUG-" << current_program << ":" <<  str << std::endl;
}

void key_debug(std::string str, int n) {
	std::cout << "DEBUG-" << current_program << ":" << str <<  n << std::endl;
}
void key_debug(int n) {
	std::cout << "DEBUG-" << current_program << ":" <<  n << std::endl;
}
void key_debug(std::string str, double n) {
	std::cout << "DEBUG-" << current_program << ":" << str <<  n << std::endl;
}




void keyLoop() {
	char c;
	bool dirty = false;
	print_cur();

	static int pr = 0, xyz = 0, pidi = 0;

	static float scale = 0.001;

	// get the console in raw mode
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &= ~ (ICANON | ECHO);
	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);
	puts("Reading from keyboard");
	//printf("\033[K"); // Erase to end of line
	//print_cur();
	//printf("\033[2J"); // Clear the screen, move to (0,0)
	//printf("\033[1B"); //  Move the cursor down N lines
	is_loop = 1;
	std::cout << param_list[get_param_n(pr, xyz, pidi)] << std::endl;
	double *db_pt = get_param( pr,  xyz, pidi);


	while (is_loop) {
		// get the next event from the keyboard
		if (read(kfd, &c, 1) < 0) {
			perror("read():");
			exit(-1);
		}
		//printf("\033[1A"); // UP 1 lines
		//printf("\033[K"); // erase to end of line
		if (!is_loop)
			break;

		switch (c) {
		case KEYCODE_L:
			scale *= 10;
			key_debug("::SCALE:", scale);
			break;
		case KEYCODE_R:
			scale /= 10;
			key_debug("::SCALE:", scale);
			break;
		case KEYCODE_U:

			*db_pt += scale;
			key_debug(param_list[get_param_n(pr, xyz, pidi)], *db_pt);
			set_param_n(pr, xyz, pidi, *db_pt);
			break;
		case KEYCODE_D:
			*db_pt -= scale;
			key_debug(param_list[get_param_n(pr, xyz, pidi)], *db_pt);
			set_param_n(pr, xyz, pidi, *db_pt);
			break;


		case 'u'://pid up
			// key_debug("DOWN");
			break;
		case 'j':
			// key_debug("DOWN");
			break;
		case 'i':
			// key_debug("DOWN");
			break;
		case 'k':
			// key_debug("DOWN");
			break;
		case 'o':
			// key_debug("DOWN");
			break;
		case 'l':
			// key_debug("DOWN");
			break;


		case 'p':
			key_debug("::POSE is selected");
			pr = 0;
			break;
		case 'r'://pid up
			key_debug("::RATE is selected");
			pr = 1;
			break;

//rate or pos :: r p
//



		case 'z':
			key_debug("::X is selected");
			xyz = 0;
			break;
		case 'x':
			key_debug("::Y is selected");
			xyz = 1;
			break;
		case 'c':
			key_debug("::Z is selected");
			xyz = 2;
			break;
//X or Y or Z :: z x c


		case 'a':
			key_debug("P");
			pidi = 0;
			break;
		case 's':
			key_debug("I");
			pidi = 1;
			break;
		case 'd':
			key_debug("D");
			pidi = 2;
			break;
		case 'f':
			key_debug("Imax");
			pidi = 3;
			break;

//P or I or D or Imax :: a s d f


//scale :: left or right
//0.001 0.01 0.1 1 10 100

		case KEYCODE_Q:
			key_debug("QUIT THE PROGRAM\n");
			is_loop = 0;
			break;
		case USELESS2: case USELESS1:
			break;
		default:
			//printf("\033[K"); // Erase to end of line
			//printf("value: %c \t 0x%02X\n", c, c);
			break;
		}
		db_pt = get_param( pr,  xyz, pidi);
		std::cout << param_list[get_param_n(pr, xyz, pidi)] << ":" << *db_pt << std::endl;

	}
	tcsetattr(kfd, TCSANOW, &cooked);
	key_debug("QUIT THE PROGRAM\n");
	return;
}

void positionCallback(const std_msgs::Float32& msg) {
	float_data = msg.data;
	is_received_float = 1;
}

void position_Callback(const geometry_msgs::Point& msg) {
	position_data = msg;
	is_received_position = 1;
}





void velocityCallback(const geometry_msgs::Point& msg) {
	velocity_data = msg;
	is_received_velocity = 1;
}



int main(int argc, char **argv) {
	ros::init(argc, argv, "shell");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("generate_sin_pulse", 100, positionCallback);

	ros::Subscriber velocity_sub = n.subscribe("/FIRST/CURRENT_VEL", 100, velocityCallback);

	ros::Subscriber position_sub = n.subscribe("/FIRST/CURRENT_POS", 100, position_Callback);



// <geometry_msgs::Point>
	param_pub = n.advertise<pcl_msgs::ModelCoefficients>("param_talker", 100);

	ros::Rate loop_rate(1000);
	//ros::Rate loop_rate(30);

	signal(SIGINT, quit);

	tcgetattr(kfd, &cooked);

	int count = 0;

	init_param(&pid_param);
	printf("\033[K"); // Erase to end of line
	printf("\033[2J\n"); // Clear the screen, move to (0,0)


	while (ros::ok()) {

		if ( argv[1] != NULL)
			std::cout  <<  argv[1]  << std::endl;
		//std::cout  <<argv[0]  << std::endl;

		// static ros::Time ros_time_last;

		// ros::Time ros_time = ros::Time::now();
		// std::cout  <<  ros_time - ros_time_last << std::endl;

		// ros_time_last = ros_time;
		std::string name, name2;




		current_program = "DRONE_SHELL";
		print_cur();


		argvector.clear();
		if (std::getline(std::cin, name, '\n')) {
			int i = 0;
			std::stringstream input_stream(name);
			while ( std::getline(input_stream, name2, ' ') ) {
				argvector.push_back(name2);

				std::string sstr;
				ss.clear();
				ss << i++ << ":" << name2 << std::endl;
				ss >> sstr;


			}
			if ( !argvector.size() )
				continue;

			// key_debug(sstr);
			if ( argvector[0] == "pid") {
				current_program = "PID_CONTROLLER";
				keyLoop();
			}
			else if ( argvector[0] == "quit" || argvector[0] == "q" || argvector[0] == "exit") {

				key_debug("SHUT DOWN THE DRONE_SHELL\n");
				ros::shutdown();
				exit(0);
			}
			else if ( argvector[0] == "dat" ) {


				int is_recursive = 0;
				if ( argvector.size() > 1 && argvector[1] == "-r" ) {
					argvector.erase(argvector.begin() + 1);
					is_recursive = 1;
				}
				else if (argvector.size() > 2 && argvector[2] == "-r" ) {
					argvector.erase(argvector.begin() + 2);
					is_recursive = 1;
				}


				if ( argvector.size() > 1 && argvector[1] == "vel" ) {
					is_loop = 1;
					ros::Rate data_rate(30);
					while (is_loop) {

						is_received_float = 0;
						is_received_velocity = 0;

						ros::spinOnce(); // receive the topic. It will cause the callback function
						// if (is_received_float)
						// 	std::cout << float_data << std::endl;
						if (is_received_velocity)
							std::cout << "VEL X:Y:Z=" << velocity_data.x << ":" << velocity_data.y << ":" << velocity_data.z << std::endl;
						if (!is_recursive)
							break;
						data_rate.sleep();
					}

					continue;
				}


				is_loop = 1;
				ros::Rate data_rate(30);
				while (is_loop) {

					is_received_float = 0;
					is_received_velocity = 0;

					ros::spinOnce(); // receive the topic. It will cause the callback function
					// if (is_received_float)
					// 	std::cout << float_data << std::endl;

					if (is_received_velocity)
						std::cout << "VEL X:Y:Z=" << velocity_data.x << ":" << velocity_data.y << ":" << velocity_data.z << std::endl;

					if (is_received_position)
						std::cout << "POS X:Y:Z=" << position_data.x << ":" << position_data.y << ":" << position_data.z << std::endl;
					if (!is_recursive)
						break;
					data_rate.sleep();
				}
			}
			else
				std::cout << std::endl;




		}

		std_msgs::String msg;
		std_msgs::Float32 float_msg;
		

		pcl_msgs::ModelCoefficients param_msg;

		float_msg.data = 0;
		static float float_msg_tmp = 0;
		float_msg_tmp += 0.01;
		float_msg.data = sin((double)float_msg_tmp);


		loop_rate.sleep();
		++count;
	}


	return 0;
}
