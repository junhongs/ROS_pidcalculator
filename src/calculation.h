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

#define GROUND_ALTITUDE -2700
#define MANAGE_MODE_ERROR -1
#define MANAGE_TARGET_ERROR -1

static const float TAKEOFF_SPEED = 200;
static const float LANDING_SPEED = -200;

struct lpf_t {
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
   float input;
   double cur_time;
   float last_input;
   double last_time;
   double cycle_time;
   float lpf_filter;
   float lpf_hz;
} ;

class lpf_c {
public:
   lpf_c() {
      cur_time = ros::Time::now().toSec();
      last_input = 0;
      last_time = 0;
      cycle_time = 0;
      lpf_hz = 15;
      lpf_filter = (1.0f / (2.0f * M_PI * (float)lpf_hz));
      is_first = 1;
   }
   void set_cutoff_freq(float hz){
      lpf_filter = (1.0f / (2.0f * M_PI * hz));
   }
   void set_cycletime(){
      last_time = cur_time;
      cur_time = ros::Time::now().toSec();
      cycle_time = cur_time - last_time;
   }
   float get_lpf(float input) {
      set_cycletime();
      if(is_first){
         is_first = 0;
         last_input = input;   
      }
      input = last_input + (cycle_time / (lpf_filter + cycle_time)) * (input - last_input);
      last_input = input;
      return input;
   }
   int is_first;
   double cur_time;
   float last_input;
   double last_time;
   double cycle_time;
   float lpf_filter;
   int lpf_hz;
};


struct pid_calc_t {
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

struct pos_vel_t {
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
   double cur_time;

   float last_pos;
   float last_vel;
   double last_time;
   
   double cycle_time;
   float cur_vel_raw;
   lpf_t lpf;
};

struct target_pos_vel_t {
public:
   target_pos_vel_t() {
      target_pos = 0;
      target_vel = 0;
   }
   float target_pos;
   float target_vel;
};

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
void  reset_PID(pid_calc_t *, float);
void  reset_I(pid_calc_t *, float);

float calc_dist(float, float, float, float, float, float);
void  calc_velocity( pos_vel_t* pos_vel);
void  calc_pos_error(pid_calc_t *pid, target_pos_vel_t *target, pos_vel_t *current);
void  calc_rate_error(pid_calc_t *pid, target_pos_vel_t *target, pos_vel_t *current);
void  calc_pid(pid_calc_t* pid, pid_parameter_t* pid_param);

void  calc_navi_set_target(target_pos_vel_t *, pos_vel_t *, target_pos_vel_t *, pos_vel_t *, float);
void  calc_navi_set_target(target_pos_vel_t *, pos_vel_t *, target_pos_vel_t *, pos_vel_t *, target_pos_vel_t *, pos_vel_t *, float);
void  calc_takeoff_altitude(pid_calc_t *);
void  calc_takeoff_altitude_once(pid_calc_t *, int);

int   constrain(int amt, int low, int high);
float constrain(float amt, float low, float high);

void  pos_hold(pid_calc_t *, pid_calc_t *, target_pos_vel_t *, pos_vel_t *, float, ros::Publisher *, pid_parameter_t *, pid_parameter_t *);
void  manual(pid_calc_t *, pid_calc_t *, target_pos_vel_t *, pos_vel_t *, float, ros::Publisher *, pid_parameter_t *, pid_parameter_t *, float);
void  navi_rate(pid_calc_t *, pid_calc_t *, target_pos_vel_t *, pos_vel_t *, float, ros::Publisher *, pid_parameter_t *, pid_parameter_t *, int);

float get_lpf(lpf_t *, int);

class PIDCONTROLLER
{
public:
   PIDCONTROLLER(std::string DRONE,float x_off, float y_off);
   ~PIDCONTROLLER();
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

   std::string drone;

   int drone_num;

   float x_offset;
   float y_offset;
   
   float max_vel;
   float limited_target_vel;
   
   double node_cur_time;
   double node_last_time;

   std_msgs::UInt16MultiArray pid_output_msg;

   unsigned int current_mode;
   unsigned int flight_mode_position_callback;

   int ground_altitude;

   int is_changed_manage_mode;
   int is_changed_manage_target;
   int is_changed_manage_current_pos;

   float current_target_x;
   float current_target_y;
   float current_target_z;

   float current_position_x;
   float current_position_y;
   float current_position_z;

   int tim1_timer;
   int tim2_timer;

   pos_vel_t current_X;
   pos_vel_t current_Y;
   pos_vel_t current_Z;


   pid_calc_t pid_pos_X;
   pid_calc_t pid_pos_Y;
   pid_calc_t pid_pos_Z;

   pid_calc_t pid_rate_X;
   pid_calc_t pid_rate_Y;
   pid_calc_t pid_rate_Z;

   target_pos_vel_t target_X;
   target_pos_vel_t target_Y;
   target_pos_vel_t target_Z;


   float target_pos_x;
   float target_pos_y;
   float target_pos_z;

   static int making_drone();

   int manage_mode(unsigned int getset, unsigned int *state);
   int manage_target(unsigned int getset, float *x, float *y, float *z );
   int manage_current_pos(unsigned int getset, float *x, float *y, float *z );

   void reboot_drone();

   void timerCallback(const ros::TimerEvent&);
   void position_Callback(const geometry_msgs::Point& msg);
   void targetCallback(const geometry_msgs::Quaternion& msg);
};



#endif