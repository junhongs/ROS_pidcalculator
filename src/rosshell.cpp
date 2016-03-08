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
#include <algorithm>

using namespace std;

static int kfd = 0;
static int is_loop = 1;
static struct termios cooked, raw;
static std::string current_program;
static std::stringstream ss;
static std::vector<std::string> argvector;

static ros::Publisher param_pub;
static ros::Publisher target_pub_arr[4];

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
	std::cout << "   " << current_program << ":" <<  str << std::endl;
}

void key_debug(std::string str, int n) {
	std::cout << "   " << current_program << ":" << str << "," <<  n << std::endl;
}

void key_debug(int n) {
	std::cout << "   " << current_program << ":" <<  n << std::endl;
}

void key_debug(std::string str, double n) {
	std::cout << "   " << current_program << ":" << str << "," <<  n << std::endl;
}

void print_param(int nav, int drone_num) {
	printf("   wsx:XYZ  |  yuio hjkl:PPID UP,DN  |  rf:I  |  ");
	printf("   az:SCALE  |  12:NAV\n   PARAM: 3:SAVE,4:LOAD,5:DELETE,6:LOAD STARTED\n");
	if (nav)
		printf("   --------NAV-------%d\n", drone_num);
	else
		printf("   --------POS-------%d\n", drone_num);
	printf("   :::  P   :   I   :   D   :   P   :   I   :   D\n");

	printf("   X::%4.3lf : %4.3lf : %4.3lf : %4.3lf : %4.3lf : %4.3lf\n",   *get_param_n(nav, 0, 0, 0), *get_param_n(nav, 0, 0, 1), *get_param_n(nav, 0, 0, 2), *get_param_n(nav, 1, 0, 0), *get_param_n(nav, 1, 0, 1), *get_param_n(nav, 1, 0, 2));
	printf("   Y::%4.3lf : %4.3lf : %4.3lf : %4.3lf : %4.3lf : %4.3lf\n", 	 *get_param_n(nav, 0, 1, 0), *get_param_n(nav, 0, 1, 1), *get_param_n(nav, 0, 1, 2), *get_param_n(nav, 1, 1, 0), *get_param_n(nav, 1, 1, 1), *get_param_n(nav, 1, 1, 2));
	printf("   Z::%4.3lf : %4.3lf : %4.3lf : %4.3lf : %4.3lf : %4.3lf\n", *get_param_n(nav, 0, 2, 0), *get_param_n(nav, 0, 2, 1), *get_param_n(nav, 0, 2, 2), *get_param_n(nav, 1, 2, 0), *get_param_n(nav, 1, 2, 1), *get_param_n(nav, 1, 2, 2));

//b: heading(50)   n: takeoff_x(48)    m: takeoff_y(49)
	std::cout << "   B,HEADING:" << *get_param_n(50) << "   N,TAKEOFF_X:" << *get_param_n(48) << "   M,TAKEOFF_Y:" << *get_param_n(49) << std::endl;
}


void keyLoop(std::string param_file_name, int drone_num) {
	char c;
	bool dirty = false;
	static int pr = 0, xyz = 0, pidi = 0, nav = 0;
	static int param_n = 0;;

	static double scale = 0.001l;
	static std_msgs::Int32 param_msg;

	std::cout << "   " << std::endl << "   " << param_file_name.c_str() << std::endl;
	int is_original = 1;
	if ( param_file_name == "pidparam" ) {
		is_original = 0;
	}
	std::string param_start_str("tmp_");
	param_start_str = tmp_dir + param_start_str + param_file_name;
	std::string param_str = tmp_dir + param_file_name;


	if ( !load_param(param_str.c_str()) ) {
		save_param(param_str.c_str());
	}
	param_msg.data = LOAD_PARAMFILE + drone_num;
	param_pub.publish(param_msg);
	//if exist loadparam
	// if not exist save the current param
	//
	// set the param
	// *getparam() = content


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
	// std::cout<< "   " << param_list[get_param_num(pr, xyz, pidi)] << std::endl;
	double *db_pt = get_param_n(pr,  xyz, pidi);
	save_param(param_start_str.c_str());


	print_param(nav, drone_num);

	while (is_loop) {
		// get the next event from the keyboard
		if (!is_loop)
			break;
		if (read(kfd, &c, 1) < 0) {
			perror("read():");
			exit(-1);
		}
		//printf("\033[1A"); // UP 1 lines
		//printf("\033[K"); // erase to end of line

//b: heading(50)   n: takeoff_x(48)    m: takeoff_y(49)
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

		case 'b':
			param_n = 50;
			break;
		case 'n':
			param_n = 48;
			break;
		case 'm':
			param_n = 49;
			break;

		case KEYCODE_U:
			if (param_n >= 48 && param_n <= 50) {
				(*get_param_n(param_n))++;
				set_param_n(param_n, *get_param_n(param_n));
				key_debug(param_list[ param_n], *get_param_n(param_n));
				param_msg.data = param_n + drone_num * 100;
				param_pub.publish(param_msg);

				if (param_n == 50) {
					geometry_msgs::Quaternion target_msgs;
					target_msgs.x = 0.0f;
					target_msgs.z = 0.0f;
					target_msgs.y = *get_param_n(param_n) + 600;
					target_msgs.w = MISSION_MAGHOLD;
					target_pub_arr[drone_num - 1].publish(target_msgs);
				}

			}
			else
			{
				*db_pt += scale;
				key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
				set_param_n(nav, pr, xyz, pidi, *db_pt);
				param_msg.data = param_n + drone_num * 100;
				param_pub.publish(param_msg);
			}
			break;
		case KEYCODE_D:
			if (param_n >= 48 && param_n <= 50) {
				(*get_param_n(param_n))--;
				set_param_n(param_n, *get_param_n(param_n));
				key_debug(param_list[ param_n], *get_param_n(param_n));
				param_msg.data = param_n + drone_num * 100;
				param_pub.publish(param_msg);

				if (param_n == 50) {
					geometry_msgs::Quaternion target_msgs;
					target_msgs.x = 0.0f;
					target_msgs.z = 0.0f;
					target_msgs.y = *get_param_n(param_n) + 600;
					target_msgs.w = MISSION_MAGHOLD;
					target_pub_arr[drone_num - 1].publish(target_msgs);
				}

			}
			else
			{
				*db_pt -= scale;
				key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
				set_param_n(nav, pr, xyz, pidi, *db_pt);
				param_msg.data = get_param_num(nav, pr, xyz, pidi) + drone_num * 100;
				param_pub.publish(param_msg);
			}
			break;
		case 'y':
			pr = 0;
			pidi = 0;
			db_pt = get_param_n(nav, pr,  xyz, pidi);
			*db_pt += scale;
			set_param_n(nav, pr, xyz, pidi, *db_pt);
			param_msg.data = get_param_num(nav, pr, xyz, pidi) + drone_num * 100;
			param_pub.publish(param_msg);
			key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
			break;
		case 'h':
			pr = 0;
			pidi = 0;
			db_pt = get_param_n(nav, pr,  xyz, pidi);
			*db_pt -= scale;
			set_param_n(nav, pr, xyz, pidi, *db_pt);
			param_msg.data = get_param_num(nav, pr, xyz, pidi) + drone_num * 100;
			param_pub.publish(param_msg);
			key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
			break;
		case 'u':
			pr = 1;
			pidi = 0;
			db_pt = get_param_n(nav, pr,  xyz, pidi);
			*db_pt += scale;
			set_param_n(nav, pr, xyz, pidi, *db_pt);
			param_msg.data = get_param_num(nav, pr, xyz, pidi) + drone_num * 100;
			param_pub.publish(param_msg);
			key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
			break;
		case 'j':
			pr = 1;
			pidi = 0;
			db_pt = get_param_n(nav, pr,  xyz, pidi);
			*db_pt -= scale;
			set_param_n(nav, pr, xyz, pidi, *db_pt);
			param_msg.data = get_param_num(nav, pr, xyz, pidi) + drone_num * 100;
			param_pub.publish(param_msg);
			key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
			break;
		case 'i':
			pr = 1;
			pidi = 1;
			db_pt = get_param_n(nav, pr,  xyz, pidi);
			*db_pt += scale;
			set_param_n(nav, pr, xyz, pidi, *db_pt);
			param_msg.data = get_param_num(nav, pr, xyz, pidi) + drone_num * 100;
			param_pub.publish(param_msg);
			key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
			break;
		case 'k':
			pr = 1;
			pidi = 1;
			db_pt = get_param_n(nav, pr,  xyz, pidi);
			*db_pt -= scale;
			set_param_n(nav, pr, xyz, pidi, *db_pt);
			param_msg.data = get_param_num(nav, pr, xyz, pidi) + drone_num * 100;
			param_pub.publish(param_msg);
			key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
			break;
		case 'o':
			pr = 1;
			pidi = 2;
			db_pt = get_param_n(nav, pr,  xyz, pidi);
			*db_pt += scale;
			set_param_n(nav, pr, xyz, pidi, *db_pt);
			param_msg.data = get_param_num(nav, pr, xyz, pidi) + drone_num * 100;
			param_pub.publish(param_msg);
			key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
			break;
		case 'l':
			pr = 1;
			pidi = 2;
			db_pt = get_param_n(nav, pr,  xyz, pidi);
			*db_pt -= scale;
			set_param_n(nav, pr, xyz, pidi, *db_pt);
			param_msg.data = get_param_num(nav, pr, xyz, pidi) + drone_num * 100;
			param_pub.publish(param_msg);
			key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
			break;
		case 'r':
			pr = 0;
			pidi = 1;
			db_pt = get_param_n(nav, pr,  xyz, pidi);
			*db_pt += scale;
			set_param_n(nav, pr, xyz, pidi, *db_pt);
			param_msg.data = get_param_num(nav, pr, xyz, pidi) + drone_num * 100;
			param_pub.publish(param_msg);
			key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
			break;
		case 'f':
			pr = 0;
			pidi = 1;
			db_pt = get_param_n(nav, pr,  xyz, pidi);
			*db_pt -= scale;
			set_param_n(nav, pr, xyz, pidi, *db_pt);
			param_msg.data = get_param_num(nav, pr, xyz, pidi) + drone_num * 100;
			param_pub.publish(param_msg);
			key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
			break;

		case 't':
			pr = 0;
			pidi = 2;
			db_pt = get_param_n(nav, pr,  xyz, pidi);
			*db_pt += scale;
			set_param_n(nav, pr, xyz, pidi, *db_pt);
			param_msg.data = get_param_num(nav, pr, xyz, pidi) + drone_num * 100;
			param_pub.publish(param_msg);
			key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
			break;
		case 'g':
			pr = 0;
			pidi = 2;
			db_pt = get_param_n(nav, pr,  xyz, pidi);
			*db_pt -= scale;
			set_param_n(nav, pr, xyz, pidi, *db_pt);
			param_msg.data = get_param_num(nav, pr, xyz, pidi) + drone_num * 100;
			param_pub.publish(param_msg);
			key_debug(param_list[ get_param_num(nav, pr, xyz, pidi)], *db_pt);
			break;
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

		case '3':
			std::cout << "   " << param_str << ":::";
			key_debug("::SAVE THE PARAMETER");
			save_param(param_str.c_str());
			param_msg.data = LOAD_PARAMFILE + drone_num;
			param_pub.publish(param_msg);
			break;
		case '4':
			std::cout << "   " << param_str << ":::";
			key_debug("::LOAD THE PARAMETER");
			load_param(param_str.c_str());
			param_msg.data = LOAD_PARAMFILE + drone_num;
			param_pub.publish(param_msg);

			break;
		case '5':
			std::cout << "   " << param_str << ":::";
			key_debug("::DELETE THE PARAMETER");
			delete_file_param(param_str.c_str());
			if (!drone_num) {
				load_param(param_start_str.c_str());
				save_param(param_str.c_str());
			}
			param_msg.data = LOAD_PARAMFILE + drone_num;
			param_pub.publish(param_msg);
			break;
		case '6':
			std::cout << "   " << param_str << ":::";
			key_debug("::LOAD THE STARTED PARAMETER");
			load_param(param_start_str.c_str());
			param_msg.data = LOAD_PARAMFILE + drone_num;
			param_pub.publish(param_msg);
			break;
		case KEYCODE_Q:
			save_param(param_str.c_str());
			param_msg.data = LOAD_PARAMFILE + drone_num;
			param_pub.publish(param_msg);
			// key_debug("QUIT THE PROGRAM\n");
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
		if (is_loop)
			print_param(nav, drone_num);

	}
	tcsetattr(kfd, TCSANOW, &cooked);
	// key_debug("\nQUIT THE PROGRAM\n");
	return;
}


void keyLoop_nav(std::string str = "null") {
	char c;
	bool dirty = false;
	// print_cur();
	geometry_msgs::Quaternion target_msgs;

	std::cout << std::endl << "   " << "::::NAIGATION::::" << std::endl;

	std::cout << "   " << "WASD:direction   RF:up&down  12:takeoff&landing OP:magnetic" << std::endl;;
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

	short maghold = 5 + 600;
	int is_magchange = 0;

	std::vector<int> msg_num;
	std::vector<int>::iterator iter_msg;
	int maghold_value[4] = {0, };

	if (str == "null") {
		msg_num.push_back(0);
		msg_num.push_back(1);
		msg_num.push_back(2);
		msg_num.push_back(3);
	}
	else {
		std::string tmp_str(str);
		std::sort(tmp_str.begin(), tmp_str.end());

		for (int i = 0; i < tmp_str.length(); i++) {
			int n = tmp_str[i] - '0';
			if (n > 0 && n <= 4) {
				char num_tmp[2] = "0";
				num_tmp[0] += n;
				msg_num.push_back(n - 1);
				std::string param_file("pidparam");
				std::string param_str = tmp_dir + param_file + num_tmp;

				if ( !load_param(param_str.c_str()) ) {
					cout << "   " << "NO PARAM FILE" << endl;
				}
				maghold_value[n - 1] = *get_param_n(50) + 600;
				cout << "   " << "MAG" <<  n  << "PARAM " << maghold_value[n - 1] - 600 << endl;
			}
		}
	}




	while (is_loop) {
		is_magchange = 0;
		if (!is_loop)
			break;
		if (read(kfd, &c, 1) < 0) {
			perror("read():");
			exit(-1);
		}
		//printf("\033[1A"); // UP 1 lines
		//printf("\033[K"); // erase to end of line
		target_msgs.x = 0.0f;
		target_msgs.y = 0.0f;
		target_msgs.z = 0.0f;
		target_msgs.w = MISSION_MANUAL;




		switch (c) {
		case 'w': case 'W':
			target_msgs.y = -10.0f;
			target_msgs.w = MISSION_AUX;
			cout << "   " << "Y- DIRECTION ";
			break;
		case 'a': case 'A':
			target_msgs.x = 10.0f;
			target_msgs.w = MISSION_AUX;
			cout << "   " << "X+ DIRECTION ";
			break;
		case 's': case 'S':
			target_msgs.y = 10.0f;
			target_msgs.w = MISSION_AUX;
			cout << "   " << "Y+ DIRECTION ";
			break;
		case 'd': case 'D':
			target_msgs.x = -10.0f;
			target_msgs.w = MISSION_AUX;
			cout << "   " << "Y- DIRECTION ";
			break;
		case 'r': case 'R':
			target_msgs.z = 10.0f;
			target_msgs.w = MISSION_AUX;
			cout << "   " << "Z+ DIRECTION ";
			break;
		case 'f': case 'F':
			target_msgs.z = -10.0f;
			target_msgs.w = MISSION_AUX;
			cout << "   " << "Z- DIRECTION ";
			break;

		case 'o': case 'O':
			if (str == "null") {
				std::cout << "   " << "CANNOT MESSAGE TO ALL DRONE" << std::endl;
			}
			else {
				iter_msg = msg_num.begin();
				while (iter_msg != msg_num.end()) {

					maghold_value[*iter_msg] -= 1;
					target_msgs.y = maghold_value[*iter_msg];
					target_msgs.w = MISSION_MAGHOLD;
					target_pub_arr[*iter_msg].publish(target_msgs);
					cout << "   " << "MAG " << (*iter_msg + 1) << " : " << (maghold_value[*iter_msg] - 600);
					*get_param_n(50) = maghold_value[*iter_msg] - 600;
					char num_tmp[2] = "1";
					num_tmp[0] += *iter_msg;
					std::string param_file("pidparam");
					std::string param_str = tmp_dir + param_file + num_tmp;
					save_param(param_str.c_str());

					iter_msg++;
				}
				cout << endl;
			}
			is_magchange = 1;
			break;

		case 'p': case 'P':
			if (str == "null") {
				std::cout << "   " << "CANNOT MESSAGE TO ALL DRONE" << std::endl;
			}
			else {
				iter_msg = msg_num.begin();
				while (iter_msg != msg_num.end()) {

					maghold_value[*iter_msg] += 1;
					target_msgs.y = maghold_value[*iter_msg];
					target_msgs.w = MISSION_MAGHOLD;
					target_pub_arr[*iter_msg].publish(target_msgs);
					cout << "   " << "MAG " << (*iter_msg + 1) << " : " << (maghold_value[*iter_msg] - 600);
					*get_param_n(50) = maghold_value[*iter_msg] - 600;
					char num_tmp[2] = "1";
					num_tmp[0] += *iter_msg;
					std::string param_file("pidparam");
					std::string param_str = tmp_dir + param_file + num_tmp;
					save_param(param_str.c_str());

					iter_msg++;
				}
				cout << endl;

			}
			is_magchange = 1;
			break;

		case '1':
		case 't':
			target_msgs.w = MISSION_TAKEOFF;
			cout << "   " << "TAKEOFF ";
			break;
		case '2':
		case 'l':
			target_msgs.w = MISSION_LANDING;
			cout << "   " << "LANDING ";
			break;


		case KEYCODE_Q:
			// key_debug("QUIT THE PROGRAM\n");
			is_loop = 0;
			break;
		default:
			//printf("\033[K"); // Erase to end of line
			//printf("value: %c \t 0x%02X\n", c, c);
			break;
		}
		if (is_loop)
			if (!is_magchange)
				if (str == "null") {
					std::cout << "   " << "MESSAGE TO ALL DRONE" << std::endl;
					for (int i = 0; i < 4; i++) {
						target_pub_arr[i].publish(target_msgs);
					}
				}
				else {
					std::cout << "   " << "MESSAGE TO DRONE : ";

					std::string tmp_str(str);
					std::sort(tmp_str.begin(), tmp_str.end());

					for (int i = 0; i < tmp_str.length(); i++) {
						int n = tmp_str[i] - '0';
						if (n > 0 && n <= 4) {
							std::cout << n;
							target_pub_arr[n - 1].publish(target_msgs);
						}
					}
					std::cout << "   " << std::endl;
				}
	}
	tcsetattr(kfd, TCSANOW, &cooked);
	return;
}


void positionCallback(const std_msgs::Float32 & msg) {
	float_data = msg.data;
	is_received_float = 1;
}

void position_Callback(const geometry_msgs::Point & msg) {
	position_data = msg;
	is_received_position = 1;
}

void velocityCallback(const geometry_msgs::Point & msg) {
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

	target_pub_arr[0] = n.advertise<geometry_msgs::Quaternion>("/FIRST/TARGET_POS", 100);
	target_pub_arr[1] = n.advertise<geometry_msgs::Quaternion>("/SECOND/TARGET_POS", 100);
	target_pub_arr[2] = n.advertise<geometry_msgs::Quaternion>("/THIRD/TARGET_POS", 100);
	target_pub_arr[3] = n.advertise<geometry_msgs::Quaternion>("/FOURTH/TARGET_POS", 100);

	ros::Rate loop_rate(1000);

	signal(SIGINT, quit);
	tcgetattr(kfd, &cooked);
	int count = 0;
	init_param();

	printf("\033[K"); // Erase to end of line
	printf("\033[2J\n"); // Clear the screen, move to (0,0)
	std::cout << "COMMAND :: pid quit dat mod nav" << std::endl;
	while (ros::ok()) {
		if (argv[1] != NULL)
			std::cout  <<  argv[1]  << std::endl;
		//std::cout  <<argv[0]  << std::endl;
		// static ros::Time ros_time_last;
		// ros::Time ros_time = ros::Time::now();
		// std::cout  <<  ros_time - ros_time_last << std::endl;
		// ros_time_last = ros_time;
		std::string name, name2;
		std::string param_file("pidparam");
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

				if (argvector.size() > 1 && argvector[1] == "1" ) {
					param_file += "1";
					keyLoop(param_file, 1);
				}
				else if (argvector.size() > 1 && argvector[1] == "2" ) {
					param_file += "2";
					keyLoop(param_file, 2);
				}
				else if (argvector.size() > 1 && argvector[1] == "3" ) {
					param_file += "3";
					keyLoop(param_file, 3);
				}
				else if (argvector.size() > 1 && argvector[1] == "4" ) {
					param_file += "4";
					keyLoop(param_file, 4);
				}
				else
					std::cout << "SELECT THE DRONE NUMBER" << std::endl;
				// keyLoop(param_file, 0);
			}
			else if (argvector[0] == "nav") {
				int arg = 0;
				if (argvector.size() > 1 ) {
					// std::string mod_command = argvector.back();
					// argvector.pop_back();
					keyLoop_nav(argvector[1]);
				}
				else
					// DRONE1
					keyLoop_nav();
			}
			else if (argvector[0] == "quit" || argvector[0] == "q" || argvector[0] == "exit") {
				// key_debug("SHUT DOWN THE DRONE_SHELL\n");
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
				target_pub_arr[0].publish(target_msgs);
				target_pub_arr[1].publish(target_msgs);
				target_pub_arr[2].publish(target_msgs);
				target_pub_arr[3].publish(target_msgs);
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
