#include "ros/ros.h"
#include "calculation.h"
#include "math.h"

#include <iostream>
using namespace std;


int get_lpf(lpf_t *lpf, int lpf_hz = 15) {
   if (!lpf->last_time || !lpf_hz) {
      lpf->last_time = lpf->cur_time;
      return 0;
   }
   if (!lpf->lpf_filter || lpf_hz != lpf->lpf_hz) {
      lpf->lpf_filter = (1.0f / (2.0f * M_PI * (float)lpf_hz));
      lpf->lpf_hz = lpf_hz;
   }

   lpf->cycle_time = lpf->cur_time - lpf->last_time;
   lpf->last_time = lpf->cur_time;

   lpf->input = lpf->last_input + (lpf->cycle_time / (lpf->lpf_filter + lpf->cycle_time)) * (lpf->input - lpf->last_input);
   lpf->last_input = lpf->input;
   return lpf->input;
}




// pid_calc_t -> error
static float get_P(pid_calc_t *pid, pid_parameter_t *pid_param) {
   return pid->error * pid_param->pid_P;
}

// pid_calc_t -> error
// pid_calc_t -> cycle_time
static float get_I(pid_calc_t *pid, pid_parameter_t *pid_param) {
   pid->integrator += (pid->error * pid_param->pid_I) * pid->cycle_time;
   pid->integrator = constrain(pid->integrator, -pid_param->pid_Imax, pid_param->pid_Imax);
   return pid->integrator;
}


// pid_calc_t -> error
// pid_calc_t -> cycle_time
// pid_calc_t -> derivative = pos_vel_t -> cur_vel_raw
static float get_D(pid_calc_t *pid, pid_parameter_t *pid_param) {
   if (pid->cycle_time)
      pid->derivative = (pid->error - pid->last_error) / pid->cycle_time;

// #define DEBUG
#ifdef DEBUG
   cout << " Deriv:" << pid->derivative;
#endif

   // Low pass filter cut frequency for derivative calculation
   // Set to  "1 / ( 2 * PI * gps_lpf )"
#define PID_FILTER       (1.0f / (2.0f * M_PI * (float)5))
   // discrete low pass filter, cuts out the
   // high frequency noise that can drive the controller crazy
   pid->derivative = pid->last_derivative + (pid->cycle_time / (PID_FILTER + pid->cycle_time)) * (pid->derivative - pid->last_derivative);

#ifdef DEBUG
   cout << "  Deriv:" << pid->derivative;
#endif
   // update state

   pid->last_error = pid->error;
   pid->last_derivative = pid->derivative;
   // add in derivative component

#ifdef DEBUG
   cout << "  Param_D:" << pid_param->pid_D;
#endif

   return pid_param->pid_D * pid->derivative;
}

void reset_PID(pid_calc_t *pid) {
   pid->integrator = 0;
   //pid->last_derivative = 0;
   //pid->last_error = 0;
}

double calc_dist(double x, double y, double z, double xx, double yy, double zz) {

   double tmp = 0;
   double sum = 0;
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
   if (current->cycle_time)
      pid->cycle_time = current->cycle_time;
}

void calc_rate_error(pid_calc_t *pid, target_pos_vel_t *target, pos_vel_t *current) {
   pid->error = target->target_vel - current->cur_vel;
   pid->derivative = current->cur_vel_raw;
   if (current->cycle_time)
      pid->cycle_time = current->cycle_time;
}
// pid_calc_t -> error
// pid_calc_t -> cycle_time
// pid_calc_t -> derivative = pos_vel_t -> cur_vel_raw

void calc_pid(pid_calc_t* pid, pid_parameter_t* pid_param) {
   pid->output = pid->inner_p  = get_P(pid, pid_param);
   pid->output += pid->inner_i = get_I(pid, pid_param);
   pid->output += pid->inner_d = constrain( get_D(pid, pid_param), -200.0 , 200.0);


}

void calc_velocity( pos_vel_t* pos_vel) {
   const int is_lpf = 1;
   if (pos_vel->last_time && pos_vel->cur_time) {
      pos_vel->cycle_time = pos_vel->cur_time - pos_vel->last_time;

      //if ros's cycle period is fast (in my case, about 1000hz), sometime the cycle period might have some noize over 30% of the period time. it must be corrected.
      // if( pos_vel->cycle_time > 0.0013 || pos_vel->cycle_time < 0.0007){
      //         pos_vel->last_vel = pos_vel->cur_vel;
      //         pos_vel->last_time = pos_vel->cur_time;
      //         pos_vel->last_pos = pos_vel->cur_pos;
      //     return;
      // }
   }
   pos_vel->cur_vel = pos_vel->cur_pos - pos_vel->last_pos;

   if (pos_vel->cycle_time)
      pos_vel->cur_vel /= pos_vel->cycle_time;

   pos_vel->cur_vel_raw = pos_vel->cur_vel;

   if ( is_lpf )
      pos_vel->cur_vel = ( pos_vel->cur_vel + pos_vel->last_vel) / 2;

   pos_vel->last_vel = pos_vel->cur_vel;
   pos_vel->last_time = pos_vel->cur_time;
   pos_vel->last_pos = pos_vel->cur_pos;
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

   if (norm_xyz) {
      target_x->target_vel = vector_x / norm_xyz * nav_target_vel;
      target_y->target_vel = vector_y / norm_xyz * nav_target_vel;
      target_z->target_vel = vector_z / norm_xyz * nav_target_vel;
   }
}


void navi_rate(pid_calc_t *pid_pos, pid_calc_t *pid_rate, target_pos_vel_t *target, pos_vel_t *current, float limited_target_vel, ros::Publisher *pid_inner_pub ) {

   float err_pos = target->target_pos - current->cur_pos;

   // If current position is in a 50mm target range, change the mode to the pos_hold
   if ( err_pos < 50 || err_pos > -50 ) {
      pos_hold(pid_pos, pid_rate, target, current, limited_target_vel, pid_inner_pub);
   }
   else {
      // (0)target_vel, (1)rateP, (2)rateI, (3)rateD, (4)res
      calc_rate_error(pid_rate, target, current);
      calc_pid(pid_rate, &pid_rate_param_X);

      geometry_msgs::Inertia pid_inner_msg;
      pid_inner_msg.m = target->target_vel;
      pid_inner_msg.ixx = pid_rate->inner_p;
      pid_inner_msg.ixy = pid_rate->inner_i;
      pid_inner_msg.ixz = pid_rate->inner_d;
      pid_inner_msg.izz =  pid_rate->output;
      pid_inner_pub->publish(pid_inner_msg);
   }
}

void pos_hold(pid_calc_t *pid_pos, pid_calc_t *pid_rate, target_pos_vel_t *target, pos_vel_t *current, float limited_target_vel, ros::Publisher *pid_inner_pub ) {
   //calculate the target velocity
   calc_pos_error(pid_pos, target, current);
   // pid_pos_p->output = get_P(pid_pos_p, &pid_pos_param_X);
   calc_pid(pid_pos, &pid_pos_param_X);
   target->target_vel = pid_pos->output;
   target->target_vel = constrain(target->target_vel, -limited_target_vel, limited_target_vel);
   // (0)target_vel, (1)rateP, (2)rateI, (3)rateD, (4)res

   calc_rate_error(pid_rate, target, current);
   calc_pid(pid_rate, &pid_rate_param_X);

   geometry_msgs::Inertia pid_inner_msg;
   pid_inner_msg.m = target->target_vel;
   pid_inner_msg.ixx = pid_rate->inner_p;
   pid_inner_msg.ixy = pid_rate->inner_i;
   pid_inner_msg.ixz = pid_rate->inner_d;
   pid_inner_msg.izz = pid_rate->output;
   pid_inner_pub->publish(pid_inner_msg);


}



void calc_takeoff_altitude(pid_calc_t *pid) {
   if ( pid->integrator < 50 ) {
      pid->integrator += 400 * pid->cycle_time;
   }
}