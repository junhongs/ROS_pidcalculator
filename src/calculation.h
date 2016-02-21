#ifndef _CALCULATION_H
#define _CALCULATION_H

#include "ros/ros.h"
#include "param.h"
#include "geometry_msgs/Inertia.h"
#include <string>

typedef struct lpf_t {
   double input;
   double cur_time;

   double last_input;
   double last_time;
   double cycle_time;
   double lpf_filter;
   int lpf_hz;
} lpf_t;




typedef struct {
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

} pid_calc_t;

typedef struct pos_vel_t {
   float cur_pos;
   float cur_vel;
   float last_pos;
   float last_vel;
   double cur_time;
   double last_time;
   double cycle_time;

   float cur_vel_raw;

   lpf_t lpf;
} pos_vel_t;

typedef struct target_pos_vel_t {
   float target_pos;
   float target_vel;
} target_pos_vel_t;

enum flight_mode_e{
   MODE_TAKEOFF,
   MODE_NAV,
   MODE_MANUAL,
   MODE_LANDING,   
   MODE_POSHOLD,
   MODE_GROUND   
};

enum flight_mission_e{
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


void reset_PID(pid_calc_t *,double);
void reset_I(pid_calc_t *, double);
double calc_dist(double,double,double,double,double,double);
void calc_velocity( pos_vel_t* pos_vel);
void calc_pos_error(pid_calc_t *pid, target_pos_vel_t *target, pos_vel_t *current);
void calc_rate_error(pid_calc_t *pid, target_pos_vel_t *target, pos_vel_t *current);
void calc_pid(pid_calc_t* pid, pid_parameter_t* pid_param);
int constrain(int amt, int low, int high);
float constrain(float amt, float low, float high);
void pos_hold(pid_calc_t *, pid_calc_t *, target_pos_vel_t *, pos_vel_t *, float, ros::Publisher *, pid_parameter_t *, pid_parameter_t *);
void manual(pid_calc_t *, pid_calc_t *, target_pos_vel_t *, pos_vel_t *, float, ros::Publisher *, pid_parameter_t *, pid_parameter_t *, float);


void navi_rate(pid_calc_t *, pid_calc_t *, target_pos_vel_t *, pos_vel_t *, float, ros::Publisher *, pid_parameter_t *,pid_parameter_t *, int);
void calc_navi_set_target(target_pos_vel_t *, pos_vel_t *, target_pos_vel_t *, pos_vel_t *, float);
void calc_navi_set_target(target_pos_vel_t *, pos_vel_t *, target_pos_vel_t *, pos_vel_t *, target_pos_vel_t *, pos_vel_t *, float);
void calc_takeoff_altitude(pid_calc_t *);
void calc_takeoff_altitude_once(pid_calc_t *, int);


int get_lpf(lpf_t *, int);

#endif