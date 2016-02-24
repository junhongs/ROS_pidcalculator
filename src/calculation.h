//#include "pidcontroller.h"
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



#define GROUND_ALTITUDE -2700.0f
#define MANAGE_MODE_ERROR -1
#define MANAGE_TARGET_ERROR -1

// static const float TAKEOFF_SPEED = 200;
// static const float LANDING_SPEED = -200;

struct lpf_t {
public:
   lpf_t():
      input(0.0f),
      cur_time(0.0l),
      last_input(0.0f),
      last_time(0.0l),
      cycle_time(0.0l),
      lpf_filter(0.0f),
      lpf_hz(0.0f)
   {}
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
   void set_cutoff_freq(float hz) {
      lpf_filter = (1.0f / (2.0f * M_PI * hz));
   }
   void set_cycletime() {
      last_time = cur_time;
      cur_time = ros::Time::now().toSec();
      cycle_time = cur_time - last_time;
   }
   float get_lpf(float input) {
      set_cycletime();
      if (is_first) {
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
   pid_calc_t():
      error(0.0f),
      cycle_time(0.0l),
      derivative(0.0f),
      integrator(0.0f),          // integrator value
      last_error(0.0f),
      last_derivative(0.0f),     // last derivative for low-pass filter
      inner_p(0.0f),
      inner_i(0.0f),
      inner_d(0.0f),
      output(0.0f)
   {};
   pid_calc_t(float integrator):
      error(0.0f),
      cycle_time(0.0l),
      derivative(0.0f),
      integrator(integrator),          // integrator value
      last_error(0.0f),
      last_derivative(0.0f),     // last derivative for low-pass filter
      inner_p(0.0f),
      inner_i(0.0f),
      inner_d(0.0f),
      output(0.0f)
   {};

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
   pos_vel_t():
      cur_pos(0.0f),
      cur_vel(0.0f),
      last_pos(0.0f),
      last_vel(0.0f),
      cur_time(0.0l),
      last_time(0.0l),
      cycle_time(0.0l),
      cur_vel_raw(0.0f)
   {
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
   target_pos_vel_t() :
      target_pos(0.0f),
      target_vel(0.0f)
   {
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
void  calc_velocity(pos_vel_t* pos_vel);
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

#endif