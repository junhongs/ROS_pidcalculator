#ifndef _CALCULATION_H
#define _CALCULATION_H

#include "ros/ros.h"
#include "param.h"
#include "geometry_msgs/Inertia.h"
#include "std_msgs/String.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Inertia.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Quaternion.h"

#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "pcl_msgs/Vertices.h"
#include "pcl_msgs/ModelCoefficients.h"

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt16MultiArray.h"

#include <string>

class lpf_t {
public:
   lpf_t() {
      input = 0;
      cur_time = 0;
      last_input = 0;
      last_time = 0;
      cycle_time = 0;
      lpf_filter = 0;
      lpf_hz = 0;
   }
   double input;
   double cur_time;

   double last_input;
   double last_time;
   double cycle_time;
   double lpf_filter;
   int lpf_hz;
} ;

class pid_calc_t {
public:
   pid_calc_t() {
      error = 0;
      cycle_time = 0;
      derivative = 0;

      integrator = 0;          // integrator value
      last_error = 0;
      last_derivative = 0;     // last derivative for low-pass filter

      inner_p = 0;
      inner_i = 0;
      inner_d = 0;
      output = 0;

   }
   float error;
   double cycle_time;
   float derivative;

   float integrator;          // integrator value
   float last_error;
   float last_derivative;     // last derivative for low-pass filter

   float inner_p;
   float inner_i;
   float inner_d;

   float output;

};

class pos_vel_t {
public:
   pos_vel_t() {
      cur_pos = 0;
      cur_vel = 0;
      last_pos = 0;
      last_vel = 0;
      cur_time = 0;
      last_time = 0;
      cycle_time = 0;

      cur_vel_raw = 0; 
   }
   float cur_pos;
   float cur_vel;
   float last_pos;
   float last_vel;
   double cur_time;
   double last_time;
   double cycle_time;

   float cur_vel_raw;

   lpf_t lpf;
} ;

class target_pos_vel_t {
public:
   target_pos_vel_t(){
      target_pos = 0;  
      target_vel = 0;
   }
   float target_pos;
   float target_vel;
} ;

enum flight_mode_e {
   MODE_TAKEOFF,
   MODE_NAV,
   MODE_MANUAL,
   MODE_LANDING,
   MODE_POSHOLD,
   MODE_GROUND
};

enum flight_mission_e {
   MISSION_TAKEOFF,
   MISSION_AUTO,
   MISSION_MANUAL,
   MISSION_LANDING,
   MISSION_AUTO_N,
   MISSION_GROUND,
   MISSION_RESET,
   MISSION_AUX
};

enum {
   GET,
   SET,
   SET_TARGET,
   SET_TIMER,
};

extern std::string DRONE[4];
extern std::string mission_str[MISSION_AUX + 1];
extern std::string mode_str[MODE_GROUND + 1];
void reset_PID(pid_calc_t *, double);
void reset_I(pid_calc_t *, double);
double calc_dist(double, double, double, double, double, double);
void calc_velocity( pos_vel_t* pos_vel);
void calc_pos_error(pid_calc_t *pid, target_pos_vel_t *target, pos_vel_t *current);
void calc_rate_error(pid_calc_t *pid, target_pos_vel_t *target, pos_vel_t *current);
void calc_pid(pid_calc_t* pid, pid_parameter_t* pid_param);
int constrain(int amt, int low, int high);
float constrain(float amt, float low, float high);
void pos_hold(pid_calc_t *, pid_calc_t *, target_pos_vel_t *, pos_vel_t *, float, ros::Publisher *, pid_parameter_t *, pid_parameter_t *);
void manual(pid_calc_t *, pid_calc_t *, target_pos_vel_t *, pos_vel_t *, float, ros::Publisher *, pid_parameter_t *, pid_parameter_t *, float);


void navi_rate(pid_calc_t *, pid_calc_t *, target_pos_vel_t *, pos_vel_t *, float, ros::Publisher *, pid_parameter_t *, pid_parameter_t *, int);
void calc_navi_set_target(target_pos_vel_t *, pos_vel_t *, target_pos_vel_t *, pos_vel_t *, float);
void calc_navi_set_target(target_pos_vel_t *, pos_vel_t *, target_pos_vel_t *, pos_vel_t *, target_pos_vel_t *, pos_vel_t *, float);
void calc_takeoff_altitude(pid_calc_t *);
void calc_takeoff_altitude_once(pid_calc_t *, int);


int get_lpf(lpf_t *, int);




class PIDCONTROLLER
{
public:
   PIDCONTROLLER(int x_off, int y_off) :
      x_offset(x_off), y_offset(y_off), limited_target_vel(400), max_vel(200)
   {
      pid_output_msg.data.resize(5, 1000);

      std::string drone;
      drone_num = making_drone();
      if (drone_num <= 4)
         drone = DRONE[drone_num];
      std::cout << "initializing.." << drone << std::endl;
      std::string current_vel = drone + "/CURRENT_VEL";
      std::string output_pid = drone +  "/OUTPUT_PID";
      std::string output_inner_pid_x = drone + "/OUTPUT_INNER_PID/X";
      std::string output_inner_pid_y = drone + "/OUTPUT_INNER_PID/Y";
      std::string output_inner_pid_z = drone + "/OUTPUT_INNER_PID/Z";
      std::string current_pos = drone + "/CURRENT_POS";
      std::string target_pos = drone + "/TARGET_POS";

      velocity_pub = nod.advertise<geometry_msgs::Point>(current_vel, 100);
      // timer = nod.createTimer(ros::Duration(0.08), &PIDCONTROLLER::timerCallback, this);
      pid_out_pub     = nod.advertise<std_msgs::UInt16MultiArray>(output_pid, 100);
      pid_inner_x_pub = nod.advertise<geometry_msgs::Inertia>(output_inner_pid_x, 100);
      pid_inner_y_pub = nod.advertise<geometry_msgs::Inertia>(output_inner_pid_y, 100);
      pid_inner_z_pub = nod.advertise<geometry_msgs::Inertia>(output_inner_pid_z, 100);
      position_sub = nod.subscribe(current_pos, 100, &PIDCONTROLLER::position_Callback, this);
      target_sub = nod.subscribe(target_pos, 100, &PIDCONTROLLER::targetCallback, this);



      current_mode = MODE_GROUND;
      is_changed_manage_mode = 0;

      current_target_x = 0;
      current_target_y = 0;
      current_target_z = 0;
      is_changed_manage_target = 0;


      current_position_x = 0;
      current_position_y = 500;
      current_position_z = -2000;
      is_changed_manage_current_pos = 0;


      tim1_timer = 0, tim2_timer = 0;

      // std::cout << current_X.cur_pos << std::endl;
      // current_X = {0,};
      // current_Y = {0,};
      // current_Z = {0,};
      flight_mode_position_callback = MODE_GROUND;

      // pid_pos_X = {0, };
      // pid_rate_X = {0, };
      // target_X = {0, };
      // //Y
      // pid_pos_Y = {0, };
      // pid_rate_Y = {0, };
      // target_Y = {0, };
      // //Z
      // pid_pos_Z = {0, };
      // pid_rate_Z = {0, 0, 0, -500, 0, 0, 0, 0, 0, 0};
      pid_rate_Z.integrator = -500;
      // target_Z = {0, };
      node_cur_time = 0;
      node_last_time = 0;

      target_pos_x = 0;
      target_pos_y = 0;
      target_pos_z = 0;
   }
private:
   ros::Timer timer;
   ros::Publisher velocity_pub;
   ros::Publisher pid_out_pub;
   ros::Publisher pid_inner_x_pub;
   ros::Publisher pid_inner_y_pub;
   ros::Publisher pid_inner_z_pub;
   ros::Subscriber position_sub;
   ros::Subscriber target_sub;
   ros::NodeHandle nod;
   int x_offset;
   int y_offset;
   int drone_num;
   float limited_target_vel;
   float max_vel;
   double node_cur_time;
   std_msgs::UInt16MultiArray pid_output_msg;

   unsigned int current_mode;
   int is_changed_manage_mode;

   float current_target_x;
   float current_target_y;
   float current_target_z;
   int is_changed_manage_target;

   float current_position_x;
   float current_position_y;
   float current_position_z;
   int is_changed_manage_current_pos;

   int tim1_timer, tim2_timer;

   pos_vel_t current_X;
   pos_vel_t current_Y;
   pos_vel_t current_Z;
   unsigned int flight_mode_position_callback;

   pid_calc_t pid_pos_X;
   pid_calc_t pid_rate_X;
   target_pos_vel_t target_X;
   //Y
   pid_calc_t pid_pos_Y;
   pid_calc_t pid_rate_Y;
   target_pos_vel_t target_Y;
   //Z
   pid_calc_t pid_pos_Z;
   pid_calc_t pid_rate_Z;
   //pid_rate_Z.integrator;
   target_pos_vel_t target_Z;

   double node_last_time;

   float target_pos_x;
   float target_pos_y;
   float target_pos_z;






   static int making_drone();

   int manage_mode(unsigned int getset, unsigned int *state);
   int manage_target(unsigned int getset, float *x, float *y, float *z );
   int manage_current_pos(unsigned int getset, float *x, float *y, float *z );
// target_pos_vel_t *target;
// pos_vel_t *current;
   void timerCallback(const ros::TimerEvent&);
   void position_Callback(const geometry_msgs::Point& msg);

   void reboot_drone();

   void targetCallback(const geometry_msgs::Quaternion& msg);
};



#endif