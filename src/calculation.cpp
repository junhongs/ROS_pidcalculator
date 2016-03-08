#include "ros/ros.h"
#include "calculation.h"
#include "math.h"
#include <iostream>
using namespace std;
std::string mission_str[SIZEOFMISSION] =
{
   "MISSION_TAKEOFF",
   "MISSION_AUTO",
   "MISSION_MANUAL",
   "MISSION_LANDING",
   "MISSION_AUTO_N",
   "MISSION_GROUND",
   "MISSION_RESET",
   "MISSION_MAGHOLD",
   "MISSION_AUX"
};
std::string mode_str[SIZEOFMODE] =
{
   "MODE_TAKEOFF",
   "MODE_NAV",
   "MODE_NAV_N",
   "MODE_MANUAL",
   "MODE_LANDING",
   "MODE_POSHOLD",
   "MODE_GROUND",
   "MODE_FAILSAFE",
   "MODE_NOT_DETECTED"
};

static float get_P(pid_calc_t *pid, pid_parameter_t *pid_param) {
   return pid->error * pid_param->pid_P;
}

static float get_I(pid_calc_t *pid, pid_parameter_t *pid_param) {
   pid->integrator += (pid->error * pid_param->pid_I) * pid->cycle_time;
   pid->integrator = constrain(pid->integrator, -pid_param->pid_Imax, pid_param->pid_Imax);
   return pid->integrator;
}

static float get_D(pid_calc_t *pid, pid_parameter_t *pid_param) {
   if (pid->cycle_time) {
      pid->derivative = (pid->error - pid->last_error) / pid->cycle_time;
   }
#define PID_FILTER       (1.0f / (2.0f * M_PI * (float)3.0f))
   pid->derivative = pid->last_derivative + (pid->cycle_time / (PID_FILTER + pid->cycle_time)) * (pid->derivative - pid->last_derivative);
   // update state
   pid->last_error = pid->error;
   pid->last_derivative = pid->derivative;
   // add in derivative component
   return pid_param->pid_D * pid->derivative;
}

static float get_D_L(pid_calc_t *pid, pid_parameter_t *pid_param) {
   if (pid->cycle_time)
      pid->derivative = (pid->error - pid->last_error) / pid->cycle_time;
#define PID_FILTER_L       (1.0f / (2.0f * M_PI * (float)0.7f))
   pid->derivative = pid->last_derivative + (pid->cycle_time / (PID_FILTER_L + pid->cycle_time)) * (pid->derivative - pid->last_derivative);
   pid->last_error = pid->error;
   pid->last_derivative = pid->derivative;
   return pid_param->pid_D * pid->derivative;
}

void reset_PID(pid_calc_t *pid, float integrator) {
   pid->integrator = integrator;
   pid->last_derivative = 0.0f;
   pid->last_error = 0.0f;
}

void reset_I(pid_calc_t *pid, float integrator) {
   pid->integrator = integrator;
}

float calc_dist(float x, float y, float z, float xx, float yy, float zz) {
   float tmp = 0.0f;
   float sum = 0.0f;
   tmp = (x - xx);
   sum += tmp * tmp;
   tmp = (y - yy);
   sum += tmp * tmp;
   tmp = (z - zz);
   sum += tmp * tmp;
   return sqrt(sum);
}

int constrain(int amt, int low, int high) {
   if (amt < low)
      return low;
   else if (amt > high)
      return high;
   else
      return amt;
}

float constrain(float amt, float low, float high) {
   if (amt < low)
      return low;
   else if (amt > high)
      return high;
   else
      return amt;
}

void calc_pos_error(pid_calc_t *pid, target_pos_vel_t *target, pos_vel_t *current) {
   pid->error = target->target_pos - current->cur_pos;
   pid->cycle_time = current->cycle_time;
}

void calc_rate_error(pid_calc_t *pid, target_pos_vel_t *target, pos_vel_t *current) {
   pid->error = target->target_vel - current->cur_vel;
   pid->cycle_time = current->cycle_time;
}

void calc_pid(pid_calc_t* pid, pid_parameter_t* pid_param) {
   pid->inner_p = get_P(pid, pid_param);
   pid->inner_i = get_I(pid, pid_param);
   pid->inner_d = constrain(get_D(pid, pid_param), -100.0f , 100.0f) + constrain(get_D_L(pid, pid_param), -100.0f , 100.0f);

   pid->output = pid->inner_p + pid->inner_i + pid->inner_d;
}

void calc_velocity(pos_vel_t* current) {
   const int is_lpf = 0;
   if (current->last_time && current->cur_time) {
      current->cycle_time = current->cur_time - current->last_time;
   }
   if (current->cycle_time) {
      current->cur_vel = current->cur_pos - current->last_pos;
      current->cur_vel /= current->cycle_time;
   }
   current->cur_vel_raw = current->cur_vel;
   // if( abs(current->last_vel - current->cur_vel) > 500 ){
   //       current->cur_vel = current->last_vel;
   // }

   if (is_lpf )
      current->cur_vel = (current->cur_vel + current->last_vel) / 2.0f;
   current->last_vel = current->cur_vel;
   current->last_time = current->cur_time;
   current->last_pos = current->cur_pos;
}

void calc_navi_set_target(target_pos_vel_t *target_x, pos_vel_t *cur_x, target_pos_vel_t *target_y, pos_vel_t *cur_y, float nav_target_vel) {
   float vector_x = target_x->target_pos - cur_x->cur_pos;
   float vector_y = target_y->target_pos - cur_y->cur_pos;
   float norm_xy = sqrt(vector_x * vector_x + vector_y * vector_y);
   if (norm_xy) {
      target_x->target_vel = vector_x / norm_xy * nav_target_vel;
      target_y->target_vel = vector_y / norm_xy * nav_target_vel;
   }
}

void calc_navi_set_target(target_pos_vel_t *target_x, pos_vel_t *cur_x, target_pos_vel_t *target_y, pos_vel_t *cur_y, target_pos_vel_t *target_z, pos_vel_t *cur_z, float nav_target_vel) {
   float vector_x = target_x->target_pos - cur_x->cur_pos;
   float vector_y = target_y->target_pos - cur_y->cur_pos;
   float vector_z = target_z->target_pos - cur_z->cur_pos;
   float norm_xyz = sqrt(vector_x * vector_x + vector_y * vector_y + vector_z * vector_z);

   if (norm_xyz > 0) {
      target_x->target_vel = vector_x / norm_xyz * nav_target_vel;
      target_y->target_vel = vector_y / norm_xyz * nav_target_vel;
      target_z->target_vel = vector_z / norm_xyz * nav_target_vel;
   }
}
void calc_navi_proportional_set_target(target_pos_vel_t *target_x, pos_vel_t *cur_x, target_pos_vel_t *target_y, pos_vel_t *cur_y, target_pos_vel_t *target_z, pos_vel_t *cur_z, float max_vel, pid_calc_t *pid_pos, pid_parameter_t *pos_param) {
   float vector_x = target_x->target_pos - cur_x->cur_pos;
   float vector_y = target_y->target_pos - cur_y->cur_pos;
   float vector_z = target_z->target_pos - cur_z->cur_pos;
   float norm_xyz = sqrt(vector_x * vector_x + vector_y * vector_y + vector_z * vector_z);

   pid_pos->error = norm_xyz;
   pid_pos->cycle_time = cur_x->cycle_time;
   calc_pid(pid_pos, pos_param);
   pid_pos->output = constrain(pid_pos->output, -max_vel, max_vel);
   if (norm_xyz > 0) {
      target_x->target_vel = vector_x / norm_xyz * pid_pos->output;
      target_y->target_vel = vector_y / norm_xyz * pid_pos->output;
      target_z->target_vel = vector_z / norm_xyz * pid_pos->output;
   }
}
//if the mode is not changed, the changed poshold is not return to the navi_rate
int navi_rate(pid_calc_t *pid_pos, pid_calc_t *pid_rate, target_pos_vel_t *target, pos_vel_t *current, float limited_target_vel, ros::Publisher *pid_inner_pub , pid_parameter_t *pos_param, pid_parameter_t *rate_param, int changed_target, pid_parameter_t *ph_pos_param, pid_parameter_t *ph_rate_param, int *changed_to_poshold, float *offset, lpf_c *offset_lpf) {

   float err_pos = target->target_pos - current->cur_pos;

   target->target_vel = target->target_lpf.get_lpf(target->target_vel);

   if (changed_target)
      *changed_to_poshold = 0;

   // If current position is in a 50mm target range, change the mode to the pos_hold
   if ((err_pos < 30.0f && err_pos > -30.0f) || *changed_to_poshold ) {
      *changed_to_poshold = 1;
      // target->target_lpf.set_cutoff_freq(3.5f);
      pos_hold(pid_pos, pid_rate, target, current, limited_target_vel, pid_inner_pub, ph_pos_param, ph_rate_param, offset, offset_lpf);
      return 1;
   }
   else {
      // (0)target_vel, (1)rateP, (2)rateI, (3)rateD, (4)res
      calc_rate_error(pid_rate, target, current);
      calc_pid(pid_rate, rate_param);

      geometry_msgs::Inertia pid_inner_msg;
      pid_inner_msg.m = target->target_vel;
      pid_inner_msg.ixx = pid_rate->inner_p;
      pid_inner_msg.ixy = pid_rate->inner_i;
      pid_inner_msg.ixz = pid_rate->inner_d;
      pid_inner_msg.izz =  pid_rate->output;
      pid_inner_pub->publish(pid_inner_msg);
      return 0;
   }
}
int navi_rate_proportional(pid_calc_t *pid_pos, pid_calc_t *pid_rate, target_pos_vel_t *target, pos_vel_t *current, float limited_target_vel, ros::Publisher *pid_inner_pub , pid_parameter_t *pos_param, pid_parameter_t *rate_param, int changed_target, pid_parameter_t *ph_pos_param, pid_parameter_t *ph_rate_param, int *changed_to_poshold, float *offset, lpf_c *offset_lpf) {

   calc_pos_error(pid_pos, target, current);
   target->target_vel = target->target_lpf.get_lpf(target->target_vel);

   if (changed_target)
      *changed_to_poshold = 0;

   // If current position is in a 50mm target range, change the mode to the pos_hold
   if ((pid_pos->error < 30.0f && pid_pos->error > -30.0f) || *changed_to_poshold ) {
      // target->target_lpf.set_cutoff_freq(6.0f);
      *changed_to_poshold = 1;
      pos_hold(pid_pos, pid_rate, target, current, limited_target_vel, pid_inner_pub, ph_pos_param, ph_rate_param, offset, offset_lpf);
      return 1;
   }
   else {
      // (0)target_vel, (1)rateP, (2)rateI, (3)rateD, (4)res
      calc_rate_error(pid_rate, target, current);
      calc_pid(pid_rate, rate_param);

      geometry_msgs::Inertia pid_inner_msg;
      pid_inner_msg.m = target->target_vel;
      pid_inner_msg.ixx = pid_rate->inner_p;
      pid_inner_msg.ixy = pid_rate->inner_i;
      pid_inner_msg.ixz = pid_rate->inner_d;
      pid_inner_msg.izz =  pid_rate->output;
      pid_inner_pub->publish(pid_inner_msg);
      return 0;
   }
}
//if the mode is not changed, the changed poshold is not return to the navi_rate
void navi_rate_next(pid_calc_t *pid_pos, pid_calc_t *pid_rate, target_pos_vel_t *target, pos_vel_t *current, float limited_target_vel, ros::Publisher *pid_inner_pub , pid_parameter_t *pos_param, pid_parameter_t *rate_param) {

   // (0)target_vel, (1)rateP, (2)rateI, (3)rateD, (4)res
   calc_rate_error(pid_rate, target, current);
   calc_pid(pid_rate, rate_param);

   geometry_msgs::Inertia pid_inner_msg;
   pid_inner_msg.m = target->target_vel;
   pid_inner_msg.ixx = pid_rate->inner_p;
   pid_inner_msg.ixy = pid_rate->inner_i;
   pid_inner_msg.ixz = pid_rate->inner_d;
   pid_inner_msg.izz =  pid_rate->output;
   pid_inner_pub->publish(pid_inner_msg);
}


void manual(pid_calc_t *pid_pos, pid_calc_t *pid_rate, target_pos_vel_t *target, pos_vel_t *current, float limited_target_vel, ros::Publisher *pid_inner_pub , pid_parameter_t *pos_param, pid_parameter_t *rate_param, float max_vel) {
   // (0)target_vel, (1)rateP, (2)rateI, (3)rateD, (4)res
   target->target_vel = target->target_pos * max_vel;
   calc_rate_error(pid_rate, target, current);
   calc_pid(pid_rate, rate_param);

   geometry_msgs::Inertia pid_inner_msg;
   pid_inner_msg.m = target->target_vel;
   pid_inner_msg.ixx = pid_rate->inner_p;
   pid_inner_msg.ixy = pid_rate->inner_i;
   pid_inner_msg.ixz = pid_rate->inner_d;
   pid_inner_msg.izz =  pid_rate->output;
   pid_inner_pub->publish(pid_inner_msg);
}

void pos_hold(pid_calc_t *pid_pos, pid_calc_t *pid_rate, target_pos_vel_t *target, pos_vel_t *current, float limited_target_vel, ros::Publisher *pid_inner_pub , pid_parameter_t *pos_param, pid_parameter_t *rate_param, float *offset, lpf_c *offset_lpf) {
   //calculate the target velocity
   calc_pos_error(pid_pos, target, current);
   // calc_pid(pid_pos, pos_param);
   // target->target_vel = pid_pos->output;

   target->target_vel = get_P(pid_pos, pos_param) + get_D(pid_pos, pos_param);
   target->target_vel = constrain(target->target_vel, -limited_target_vel, limited_target_vel);
   // (0)target_vel, (1)rateP, (2)rateI, (3)rateD, (4)res
   target->target_vel = target->target_lpf.get_lpf(target->target_vel);
   float tmp_I = 0, tmp_D = 0;


   calc_rate_error(pid_rate, target, current);
   calc_pid(pid_rate, rate_param);

   *offset = offset_lpf->get_lpf(pid_rate->output);

   pid_rate->output += tmp_I = get_I(pid_pos, pos_param);
   // pid_rate->output += tmp_D = get_D(pid_pos, pos_param);

   geometry_msgs::Inertia pid_inner_msg;
   pid_inner_msg.m = target->target_vel;
   pid_inner_msg.ixx = pid_rate->inner_p;
   pid_inner_msg.ixy = pid_rate->inner_i;
   pid_inner_msg.ixz = pid_rate->inner_d;
   pid_inner_msg.iyy = tmp_I;
   pid_inner_msg.iyz = tmp_D;
   pid_inner_msg.izz = pid_rate->output;
   pid_inner_pub->publish(pid_inner_msg);
}

int calc_takeoff_altitude(pid_calc_t *pid) {
   if (pid->integrator < 0.0f ) {
      pid->integrator += 200.0f * pid->cycle_time;
      return 1;
   }
   else
      return 0;
}

void calc_landing_altitude(pid_calc_t *pid) {
   if (pid->integrator < 100.0f ) {
      pid->integrator -= 200.0f * pid->cycle_time;
   }
}

int calc_takeoff_altitude_once(pid_calc_t *pid, int is_changed_to_takeoff, int takeoff_throttle, int *is_takeoff) {
   if (is_changed_to_takeoff)
      *is_takeoff = 1;
   if (pid->integrator >= takeoff_throttle ) {
      *is_takeoff = 0;
   }
   if (pid->integrator < takeoff_throttle && *is_takeoff ) {
      pid->integrator += 200.0f * pid->cycle_time;
      return 1;
   }
   else return 0;
}

lpf_c::lpf_c() :
   last_input (0.0l),
   last_time (0.0l),
   cycle_time (0.0l),
   lpf_hz (0.0f),
   lpf_filter(0.0f),
   cur_time(0.0l) {
   set_cutoff_freq(lpf_hz);
}

lpf_c::lpf_c(float hz) :
   last_input (0.0l),
   last_time (0.0l),
   cycle_time (0.0l),
   lpf_filter(0.0f),
   lpf_hz (hz),
   cur_time(0.0l) {
   set_cutoff_freq(hz);
}

void lpf_c::set_cutoff_freq(float hz) {
   if (lpf_hz == hz)
      return;
   lpf_hz = hz;
   if (hz == 0.0f) {
      lpf_filter = 0;
   }
   else
      lpf_filter = (1.0f / (2.0f * M_PI * hz));

}

void lpf_c::set_cycletime() {
   last_time = cur_time;
   cur_time = ros::Time::now().toSec();
   cycle_time = cur_time - last_time;
}

void lpf_c::set_cycletime(double cur) {
   last_time = cur_time;
   cur_time = cur;
   cycle_time = cur_time - last_time;
}

float lpf_c::get_lpf(float input) {
   set_cycletime();
   if (last_time == 0.0l) {
      last_input = input;
      return input;
   }
   input = last_input + (cycle_time / (lpf_filter + cycle_time)) * (input - last_input);
   last_input = input;
   return input;
}

float lpf_c::get_lpf(float input, double cur) {
   set_cycletime(cur);
   if (last_time == 0.0l) {
      last_input = input;
      return input;
   }
   input = last_input + (cycle_time / (lpf_filter + cycle_time)) * (input - last_input);
   last_input = input;
   return input;
}

pid_calc_t::pid_calc_t(float integrator):
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
{}

pid_calc_t::pid_calc_t():
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
{}

pos_vel_t::pos_vel_t():
   cur_pos(0.0f),
   cur_vel(0.0f),
   last_pos(0.0f),
   last_vel(0.0f),
   cur_time(0.0l),
   last_time(0.0l),
   cycle_time(0.0l),
   cur_vel_raw(0.0f)
{}

target_pos_vel_t::target_pos_vel_t() :
   target_pos(0.0f),
   target_vel(0.0f),
   target_lpf(5.0f)
{}