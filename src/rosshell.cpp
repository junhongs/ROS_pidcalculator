#include "ros/ros.h"
#include "unistd.h"

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "pcl_msgs/ModelCoefficients.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt16MultiArray.h"

#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include "math.h"
#include "param.h"
#include "calculation.h"

static int kfd = 0;
static int is_loop = 1;
static struct termios cooked, raw;
static std::string current_program;
static std::stringstream ss;
static std::vector<std::string> argvector;

static ros::Publisher param_pub;
static ros::Publisher target_pub;
static ros::Publisher target2_pub;
static ros::Publisher target3_pub;
static ros::Publisher target4_pub;

static int is_received_float = 0;
static float float_data = 0.0f;

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

void print_param(int nav) {
	printf("wsx:XYZ  |  yuio hjkl:PPID UP,DN  |  rf:I  |  ");
	printf("az:SCALE  |  12:NAV\nPARAM: 3:SAVE,4:LOAD,5:DELETE,6:LOAD STARTED\n");
	if (nav)
		printf("--------NAV-------\n");
	else
		printf("--------POS-------\n");
	printf(":::  P   :   I   :   P   :   I   :   D\n");

	printf("X::%4.3lf : %4.3lf : %4.3lf : %4.3lf : %4.3lf\n", *get_param_n(nav, 0, 0, 0), *get_param_n(nav, 0, 0, 1), *get_param_n(nav, 1, 0, 0), *get_param_n(nav, 1, 0, 1), *get_param_n(nav, 1, 0, 2));
	printf("Y::%4.3lf : %4.3lf : %4.3lf : %4.3lf : %4.3lf\n", *get_param_n(nav, 0, 1, 0), *get_param_n(nav, 0, 1, 1), *get_param_n(nav, 1, 1, 0), *get_param_n(nav, 1, 1, 1), *get_param_n(nav, 1, 1, 2));
	printf("Z::%4.3lf : %4.3lf : %4.3lf : %4.3lf : %4.3lf\n\n", *get_param_n(nav, 0, 2, 0), *get_param_n(nav, 0, 2, 1), *get_param_n(nav, 1, 2, 0), *get_param_n(nav, 1, 2, 1), *get_param_n(nav, 1, 2, 2));
}


void keyLoop(std::string param_file_name) {
	char c;
	bool dirty = false;
	print_cur();

	static int pr = 0, xyz = 0, pidi = 0, nav = 0;

	static double scale = 0.001l;
	static std_msgs::Int32 param_msg;

	std::cout << std::endl << param_file_name.c_str() << std::endl;

	std::string param_start_str("tmp_");
	param_start_str += param_file_name;

	// get the console in raw mode
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &= ~ (ICANON | ECHO);
	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);
	// puts("Reading from keyboard");
	//printf("\033[K"); // Erase to end of line
	//print_cur();
	//printf("\033[2J"); // Clear the screen, move to (0,0)
	//printf("\033[1B"); //  Move the cursor down N lines
	is_loop = 1;
	std::cout << param_list[get_param_num(pr, xyz, pidi)] << std::endl;
	double *db_pt = get_param_n(pr,  xyz, pidi);



	print_param(nav);
	// printf("wsx:XYZ  |  yuio hjkl:PPID UP,DN  |  rf:I  |  ");
	// printf("az:SCALE  |  12:NAV\nPARAM: 3:SAVE,4:LOAD,5:DELETE,6:LOAD STARTED\n\n");
	// if (nav)
	// 	printf("--------NAV-------\n");
	// else
	// 	printf("--------POS-------\n");
	// printf(":::  P   :   I   :   P   :   I   :   D\n");

	// printf("X::%4.3lf : %4.3lf : %4.3lf : %4.3lf : %4.3lf\n", *get_param_n(nav, 0, 0, 0), *get_param_n(nav, 0, 0, 1), *get_param_n(nav, 1, 0, 0), *get_param_n(nav, 1, 0, 1), *get_param_n(nav, 1, 0, 2));
	// printf("Y::%4.3lf : %4.3lf : %4.3lf : %4.3lf : %4.3lf\n", *get_param_n(nav, 0, 1, 0), *get_param_n(nav, 0, 1, 1), *get_param_n(nav, 1, 1, 0), *get_param_n(nav, 1, 1, 1), *get_param_n(nav, 1, 1, 2));
	// printf("Z::%4.3lf : %4.3lf : %4.3lf : %4.3lf : %4.3lf\n", *get_param_n(nav, 0, 2, 0), *get_param_n(nav, 0, 2, 1), *get_param_n(nav, 1, 2, 0), *get_param_n(nav, 1, 2, 1), *get_param_n(nav, 1, 2, 2));



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
		case 'a':
			scale *= 10.0l;
			key_debug("::SCALE:", scale);
			break;
		case 'z':
		case KEYCODE_R:
			scale /= 10.0l;
			key_debug("::SCALE:", scale);
			break;
		case KEYCODE_U:
			*db_pt += scale;
			key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
			set_param_n(nav, pr, xyz, pidi, *db_pt);
			param_msg.data = get_param_num(nav, pr, xyz, pidi);
			param_pub.publish(param_msg);
			break;
		case KEYCODE_D:
			*db_pt -= scale;
			key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
			set_param_n(nav, pr, xyz, pidi, *db_pt);
			param_msg.data = get_param_num(nav, pr, xyz, pidi);
			param_pub.publish(param_msg);
			break;
		case 'y':
			pr = 0;
			pidi = 0;
			db_pt = get_param_n(nav, pr,  xyz, pidi);
			*db_pt += scale;
			set_param_n(nav, pr, xyz, pidi, *db_pt);
			param_msg.data = get_param_num(nav, pr, xyz, pidi);
			param_pub.publish(param_msg);
			key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
			break;
		case 'h':
			pr = 0;
			pidi = 0;
			db_pt = get_param_n(nav, pr,  xyz, pidi);
			*db_pt -= scale;
			set_param_n(nav, pr, xyz, pidi, *db_pt);
			param_msg.data = get_param_num(nav, pr, xyz, pidi);
			param_pub.publish(param_msg);
			key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
			break;
		case 'u':
			pr = 1;
			pidi = 0;
			db_pt = get_param_n(nav, pr,  xyz, pidi);
			*db_pt += scale;
			set_param_n(nav, pr, xyz, pidi, *db_pt);
			param_msg.data = get_param_num(nav, pr, xyz, pidi);
			param_pub.publish(param_msg);
			key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
			break;
		case 'j':
			pr = 1;
			pidi = 0;
			db_pt = get_param_n(nav, pr,  xyz, pidi);
			*db_pt -= scale;
			set_param_n(nav, pr, xyz, pidi, *db_pt);
			param_msg.data = get_param_num(nav, pr, xyz, pidi);
			param_pub.publish(param_msg);
			key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
			break;
		case 'i':
			pr = 1;
			pidi = 1;
			db_pt = get_param_n(nav, pr,  xyz, pidi);
			*db_pt += scale;
			set_param_n(nav, pr, xyz, pidi, *db_pt);
			param_msg.data = get_param_num(nav, pr, xyz, pidi);
			param_pub.publish(param_msg);
			key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
			break;
		case 'k':
			pr = 1;
			pidi = 1;
			db_pt = get_param_n(nav, pr,  xyz, pidi);
			*db_pt -= scale;
			set_param_n(nav, pr, xyz, pidi, *db_pt);
			param_msg.data = get_param_num(nav, pr, xyz, pidi);
			param_pub.publish(param_msg);
			key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
			break;
		case 'o':
			pr = 1;
			pidi = 2;
			db_pt = get_param_n(nav, pr,  xyz, pidi);
			*db_pt += scale;
			set_param_n(nav, pr, xyz, pidi, *db_pt);
			param_msg.data = get_param_num(nav, pr, xyz, pidi);
			param_pub.publish(param_msg);
			key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
			break;
		case 'l':
			pr = 1;
			pidi = 2;
			db_pt = get_param_n(nav, pr,  xyz, pidi);
			*db_pt -= scale;
			set_param_n(nav, pr, xyz, pidi, *db_pt);
			param_msg.data = get_param_num(nav, pr, xyz, pidi);
			param_pub.publish(param_msg);
			key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
			break;
		case 'r':
			pr = 0;
			pidi = 1;
			db_pt = get_param_n(nav, pr,  xyz, pidi);
			*db_pt += scale;
			set_param_n(nav, pr, xyz, pidi, *db_pt);
			param_msg.data = get_param_num(nav, pr, xyz, pidi);
			param_pub.publish(param_msg);
			key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
			break;
		case 'f':
			pr = 0;
			pidi = 1;
			db_pt = get_param_n(nav, pr,  xyz, pidi);
			*db_pt -= scale;
			set_param_n(nav, pr, xyz, pidi, *db_pt);
			param_msg.data = get_param_num(nav, pr, xyz, pidi);
			param_pub.publish(param_msg);
			key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
			break;

		// case 'p':
		// 	key_debug("::POSE is selected");
		// 	pr = 0;
		// 	break;
		// case 'r'://pid up
		// 	key_debug("::RATE is selected");
		// 	pr = 1;
		// 	break;

		case 'w':
			key_debug("::X is selected");
			xyz = 0;
			break;
		case 's':
			key_debug("::Y is selected");
			xyz = 1;
			break;
		case 'x':
			key_debug("::Z is selected");
			xyz = 2;
			break;

		case '1':
			key_debug("::POS is selected");
			nav = 0;
			break;
		case '2':
			key_debug("::NAV is selected");
			nav = 1;
			break;

// "/tmp/pidparam"
		case '3':
			key_debug("::SAVE THE PARAMETER");
			save_param(param_file_name.c_str());
			break;
		case '4':
			key_debug("::LOAD THE PARAMETER");
			load_param(param_file_name.c_str());
			param_msg.data = LOAD_PARAMFILE;
			param_pub.publish(param_msg);

			break;
		case '5':
			key_debug("::DELETE THE PARAMETER");
			delete_file_param();
			load_param(param_file_name.c_str());
			param_msg.data = LOAD_PARAMFILE;
			param_pub.publish(param_msg);
			break;
		case '6':
			key_debug("::LOAD THE STARTED PARAMETER");
			load_param(param_start_str.c_str());
			param_msg.data = LOAD_PARAMFILE;
			param_pub.publish(param_msg);
			break;


//X or Y or Z :: z x c
		// case 'a':
		// 	key_debug("P");
		// 	pidi = 0;
		// 	break;
		// case 's':
		// 	key_debug("I");
		// 	pidi = 1;
		// 	break;
		// case 'd':
		// 	key_debug("D");
		// 	pidi = 2;
		// 	break;
		// case 'f':
		// 	key_debug("Imax");
		// 	pidi = 3;
		// 	break;

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
		db_pt = get_param_n(pr,  xyz, pidi);

		print_param(nav);
		// //printf("%d,%d,%d\n", pr,  xyz, pidi);
		// printf("wsx:XYZ  |  yuio hjkl:PPID UP,DN  |  rf:I  |  ");
		// printf("az:SCALE  |  12:NAV\nPARAM: 3:SAVE,4:LOAD,5:DELETE,6:LOAD STARTED\n");
		// if (nav)
		// 	printf("--------NAV-------\n");
		// else
		// 	printf("--------POS-------\n");
		// printf(":::  P   :   I   :   P   :   I   :   D\n");

		// printf("X::%4.3lf : %4.3lf : %4.3lf : %4.3lf : %4.3lf\n", *get_param_n(nav, 0, 0, 0), *get_param_n(nav, 0, 0, 1), *get_param_n(nav, 1, 0, 0), *get_param_n(nav, 1, 0, 1), *get_param_n(nav, 1, 0, 2));
		// printf("Y::%4.3lf : %4.3lf : %4.3lf : %4.3lf : %4.3lf\n", *get_param_n(nav, 0, 1, 0), *get_param_n(nav, 0, 1, 1), *get_param_n(nav, 1, 1, 0), *get_param_n(nav, 1, 1, 1), *get_param_n(nav, 1, 1, 2));
		// printf("Z::%4.3lf : %4.3lf : %4.3lf : %4.3lf : %4.3lf\n\n", *get_param_n(nav, 0, 2, 0), *get_param_n(nav, 0, 2, 1), *get_param_n(nav, 1, 2, 0), *get_param_n(nav, 1, 2, 1), *get_param_n(nav, 1, 2, 2));
//		std::cout << param_list[get_param_num(pr, xyz, pidi)] << ":" << *db_pt << std::endl;

	}
	tcsetattr(kfd, TCSANOW, &cooked);
	key_debug("\nQUIT THE PROGRAM\n");
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

	param_pub = n.advertise<std_msgs::Int32>("/PARAM_CHANGE", 100);
	target_pub = n.advertise<geometry_msgs::Quaternion>("/FIRST/TARGET_POS", 100);
	target2_pub = n.advertise<geometry_msgs::Quaternion>("/SECOND/TARGET_POS", 100);
	target3_pub = n.advertise<geometry_msgs::Quaternion>("/THIRD/TARGET_POS", 100);
	target4_pub = n.advertise<geometry_msgs::Quaternion>("/FOURTH/TARGET_POS", 100);



	ros::Rate loop_rate(1000);

	signal(SIGINT, quit);
	tcgetattr(kfd, &cooked);
	int count = 0;
	init_param();

	printf("\033[K"); // Erase to end of line
	printf("\033[2J\n"); // Clear the screen, move to (0,0)
	std::cout << "COMMAND :: pid quit dat mod" << std::endl;
	while (ros::ok()) {
		if (argv[1] != NULL)
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
			while (std::getline(input_stream, name2, ' ') ) {
				argvector.push_back(name2);
				std::string sstr;
				ss.clear();
				ss << i++ << ":" << name2 << std::endl;
				ss >> sstr;
			}
			if (!argvector.size() )
				continue;
			if (argvector[0] == "pid") {
				current_program = "PID_CONTROLLER";
				keyLoop("/tmp/pidparam");
			}
			else if (argvector[0] == "quit" || argvector[0] == "q" || argvector[0] == "exit") {
				key_debug("SHUT DOWN THE DRONE_SHELL\n");
				ros::shutdown();
				exit(0);
			}
			else if (argvector[0] == "dat" ) {
				int is_recursive = 0;
				if (argvector.size() > 1 && argvector[1] == "-r" ) {
					argvector.erase(argvector.begin() + 1);
					is_recursive = 1;
				}
				else if (argvector.size() > 2 && argvector[2] == "-r" ) {
					argvector.erase(argvector.begin() + 2);
					is_recursive = 1;
				}
				if (argvector.size() > 1 && argvector[1] == "vel" ) {
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
			else if (argvector[0] == "mod" ) {
				geometry_msgs::Quaternion target_msgs;
				target_msgs.x = 0.0f;
				target_msgs.y = 0.0f;
				target_msgs.z = 0.0f;
				target_msgs.w = MISSION_AUTO;
				while (argvector.size() > 1 ) {
					std::string mod_command = argvector.back();
					argvector.pop_back();
					if ((mod_command == "1" || mod_command == "T" || mod_command == "t" ) ) {
						target_msgs.x = 0.0f;
						target_msgs.y = 0.0f;
						target_msgs.z = 0.0f;
						target_msgs.w = MISSION_TAKEOFF;
					}
					if ((mod_command == "2" || mod_command == "M" ) ) {
						target_msgs.x = 0.0f;
						target_msgs.y = 0.0f;
						target_msgs.z = 0.0f;
						target_msgs.w = MISSION_MANUAL;
					}
					if ((mod_command == "3" || mod_command == "A" ) ) {
						target_msgs.x = 0.0f;
						target_msgs.y = 0.0f;
						target_msgs.z = 0.0f;
						target_msgs.w = MISSION_AUTO;
					}
					if ((mod_command == "4" || mod_command == "L" || mod_command == "l") ) {
						target_msgs.x = 0.0f;
						target_msgs.y = 0.0f;
						target_msgs.z = 0.0f;
						target_msgs.w = MISSION_LANDING;
					}
					if ((mod_command == "W" || mod_command == "w")) {
						target_msgs.x = 500.0f;
						target_msgs.w = MISSION_AUX;
					}
					if ((mod_command == "E" || mod_command == "e")) {
						target_msgs.x = -500.0f;
						target_msgs.w = MISSION_AUX;
					}
					if ((mod_command == "N" || mod_command == "n")) {
						target_msgs.y = -500.0f;
						target_msgs.w = MISSION_AUX;
					}
					if ((mod_command == "S" || mod_command == "s")) {

						target_msgs.y = 500.0f;
						target_msgs.w = MISSION_AUX;
					}
					if ((mod_command == "U" || mod_command == "u")) {

						target_msgs.z = 500.0f;
						target_msgs.w = MISSION_AUX;
					}
					if ((mod_command == "D" || mod_command == "d")) {
						target_msgs.z = -500.0f;
						target_msgs.w = MISSION_AUX;
					}
					if ((mod_command == "G" || mod_command == "g")) {
						target_msgs.w = MISSION_GROUND;
					}
					if ((mod_command == "RESET" || mod_command == "reset" )) {
						target_msgs.x = 500.0f;
						target_msgs.w = MISSION_RESET;
					}
				}
				// std::cout << target_msgs.x << "," << target_msgs.y << "," << target_msgs.z <<std::endl;
				target_pub.publish(target_msgs);
				target2_pub.publish(target_msgs);
				target3_pub.publish(target_msgs);
				target4_pub.publish(target_msgs);
			}
			else
				std::cout << std::endl;
		}
		std_msgs::String msg;
		std_msgs::Float32 float_msg;
//std_msgs::Float32MultiArray
		//std_msgs::Float32MultiArray param_msg;
		//pcl_msgs::ModelCoefficients param_msg;
		float_msg.data = 0.0f;
		static float float_msg_tmp = 0.0f;
		float_msg_tmp += 0.01f;
		float_msg.data = sin((double)float_msg_tmp);
		loop_rate.sleep();
		++count;
	}
	return 0;
}
