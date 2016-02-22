#include "ros/ros.h"
#include "calculation.h"
#include "math.h"
#include <iostream>
using namespace std;
std::string mission_str[MISSION_AUX + 1] =
{
   "MISSION_TAKEOFF",
   "MISSION_AUTO",
   "MISSION_MANUAL",
   "MISSION_LANDING",
   "MISSION_AUTO_N",
   "MISSION_GROUND",
   "MISSION_RESET",
   "MISSION_AUX"
};
std::string mode_str[MODE_GROUND + 1] =
{
   "MODE_TAKEOFF",
   "MODE_NAV",
   "MODE_MANUAL",
   "MODE_LANDING",
   "MODE_POSHOLD",
   "MODE_GROUND"
};
std::string DRONE[4] = {
   "/FIRST",
   "/SECOND",
   "/THIRD",
   "/FOURTH"
};

float get_lpf(lpf_t *lpf, int lpf_hz = 15) {
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

// #define DEBUG
// pid_calc_t -> error
// pid_calc_t -> cycle_time
// pid_calc_t -> derivative = pos_vel_t -> cur_vel_raw
static float get_D(pid_calc_t *pid, pid_parameter_t *pid_param) {
   if (pid->cycle_time) {
      pid->derivative = (pid->error - pid->last_error) / pid->cycle_time;
   }
   #define PID_FILTER       (1.0f / (2.0f * M_PI * (float)3))
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
   #define PID_FILTER_L       (1.0f / (2.0f * M_PI * (float)0.5))
   pid->derivative = pid->last_derivative + (pid->cycle_time / (PID_FILTER_L + pid->cycle_time)) * (pid->derivative - pid->last_derivative);
   pid->last_error = pid->error;
   pid->last_derivative = pid->derivative;
   return pid_param->pid_D * pid->derivative;
}

void reset_PID(pid_calc_t *pid, float integrator) {
   pid->integrator = integrator;
   pid->last_derivative = 0;
   pid->last_error = 0;
}

void reset_I(pid_calc_t *pid, float integrator) {
   pid->integrator = integrator;
}

float calc_dist(float x, float y, float z, float xx, float yy, float zz) {
   float tmp = 0;
   float sum = 0;
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
   // if (current->cycle_time)
   pid->cycle_time = current->cycle_time;
}

void calc_rate_error(pid_calc_t *pid, target_pos_vel_t *target, pos_vel_t *current) {
   pid->error = target->target_vel - current->cur_vel;
//   pid->derivative = current->cur_vel_raw;
   // if (current->cycle_time)
   pid->cycle_time = current->cycle_time;
}
// pid_calc_t -> error
// pid_calc_t -> cycle_time
// pid_calc_t -> derivative = pos_vel_t -> cur_vel_raw

void calc_pid(pid_calc_t* pid, pid_parameter_t* pid_param) {
   pid->output = pid->inner_p  = get_P(pid, pid_param);
   pid->output += pid->inner_i = get_I(pid, pid_param);
   pid->output += pid->inner_d = constrain( get_D(pid, pid_param), -200.0 , 200.0);
   pid->output += pid->inner_d = constrain( get_D_L(pid, pid_param), -200.0 , 200.0);   
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
   if (pos_vel->cycle_time){
      pos_vel->cur_vel = pos_vel->cur_pos - pos_vel->last_pos;
      pos_vel->cur_vel /= pos_vel->cycle_time;
   }
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

//if the mode is not changed, the changed poshold is not return to the navi_rate
void navi_rate(pid_calc_t *pid_pos, pid_calc_t *pid_rate, target_pos_vel_t *target, pos_vel_t *current, float limited_target_vel, ros::Publisher *pid_inner_pub , pid_parameter_t *pos_param, pid_parameter_t *rate_param, int changed_target) {

   float err_pos = target->target_pos - current->cur_pos;

   static int changed_to_poshold = 0;

   if (changed_target)
      changed_to_poshold = 0;

   // If current position is in a 50mm target range, change the mode to the pos_hold
   if ( (err_pos < 50 && err_pos > -50) || changed_to_poshold ) {
      changed_to_poshold = 1;
      pos_hold(pid_pos, pid_rate, target, current, limited_target_vel, pid_inner_pub, pos_param, rate_param);
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
   }
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

void pos_hold(pid_calc_t *pid_pos, pid_calc_t *pid_rate, target_pos_vel_t *target, pos_vel_t *current, float limited_target_vel, ros::Publisher *pid_inner_pub , pid_parameter_t *pos_param, pid_parameter_t *rate_param) {
   //calculate the target velocity
   calc_pos_error(pid_pos, target, current);
   // pid_pos_p->output = get_P(pid_pos_p, &pid_pos_param_X);
   calc_pid(pid_pos, pos_param);
   target->target_vel = pid_pos->output;
   target->target_vel = constrain(target->target_vel, -limited_target_vel, limited_target_vel);
   // (0)target_vel, (1)rateP, (2)rateI, (3)rateD, (4)res

   calc_rate_error(pid_rate, target, current);
   calc_pid(pid_rate, rate_param);

   geometry_msgs::Inertia pid_inner_msg;
   pid_inner_msg.m = target->target_vel;
   pid_inner_msg.ixx = pid_rate->inner_p;
   pid_inner_msg.ixy = pid_rate->inner_i;
   pid_inner_msg.ixz = pid_rate->inner_d;
   pid_inner_msg.iyy = pid_rate->error;
   pid_inner_msg.iyz = pid_rate->inner_d;

   pid_inner_msg.izz = pid_rate->output;
   pid_inner_pub->publish(pid_inner_msg);
}



void calc_takeoff_altitude(pid_calc_t *pid) {
   if ( pid->integrator < 100 ) {
      pid->integrator += 400 * pid->cycle_time;
   }
}


void calc_takeoff_altitude_once(pid_calc_t *pid, int is_changed_to_takeoff) {
   static int is_takeoff = 0;

   static int takeoff_throttle = 170;
   if (is_changed_to_takeoff)
      is_takeoff = 1;
   if ( pid->integrator >= takeoff_throttle ) {
      is_takeoff = 0;
   }
   if ( pid->integrator < takeoff_throttle && is_takeoff ) {
      pid->integrator += 400 * pid->cycle_time;
   }
}

int PIDCONTROLLER::making_drone() {
   static int making_drone_num = 0;
   return making_drone_num++;
}

int PIDCONTROLLER::manage_mode(unsigned int getset, unsigned int *state) {
   // static unsigned int current_mode = MODE_GROUND;
   // static int is_changed_manage_mode = 0;
   int ret = 0;

   // if( current_mode <= MODE_GROUND && *state <= MODE_GROUND)
   // std::cout<< getset << ":" << mode_str[current_mode] << "  " <<mode_str[*state] << std::endl;
   if (current_mode == *state)
      return ret;

   if (getset == GET) {
      ret = is_changed_manage_mode;
      is_changed_manage_mode = 0;
      *state = current_mode;
   }
   else if (getset == SET || getset == SET_TIMER) {
      std::cout << "CURRENT MODE : " << mode_str[current_mode] << std::endl;
      std::cout << "   TO   " << mode_str[*state] << std::endl;

      if ( (*state == MODE_TAKEOFF && current_mode != MODE_GROUND) ) {
         std::cout << "NO PERMISSION to TAKEOFF" << std::endl;
         return MANAGE_MODE_ERROR;
      }
      if ( current_mode == MODE_GROUND && *state != MODE_TAKEOFF ) {
         std::cout << "NO PERMISSION" << std::endl;
         return MANAGE_MODE_ERROR;
      }
      if ( current_mode != *state)
         is_changed_manage_mode = 1;
      current_mode = *state;
   }
   return ret;
}
int PIDCONTROLLER::manage_target(unsigned int getset, float *x, float *y, float *z ) {
   // static float current_target_x = 0;
   // static float current_target_y = 0;
   // static float current_target_z = 0;
   // static int is_changed_manage_target = 0;
   float current_x = 0;
   float current_y = 0;
   float current_z = 0;
   manage_current_pos(GET, &current_x, &current_y, &current_z);

   int ret = 0;
   if (getset == GET) {
      // std::cout << "GET the TARGET" <<current_target_x<<","<<current_target_x<<","<<current_target_x << std::endl;
      ret = is_changed_manage_target;
      *x = current_target_x;
      *y = current_target_y;
      *z = current_target_z;
      is_changed_manage_target = 0;
   }
   else if (getset == SET || getset == SET_TARGET ) {
      if (*y == 0 || *z == 0) {
         current_target_x = current_x;
         current_target_y = current_y;
         current_target_z = current_z;
         std::cout << "ERROR TARGET. SET the CURRENT POSITION" << std::endl;
         return MANAGE_TARGET_ERROR;
      }
      //do i consider the mode???
      if (getset == SET_TARGET)
         is_changed_manage_target = 1;
      current_target_x = *x;
      current_target_y = *y;
      current_target_z = *z;
      std::cout << "SET the TARGET" << current_target_x << "," << current_target_y << "," << current_target_z << std::endl;
   }
   return ret;
}
int PIDCONTROLLER::manage_current_pos(unsigned int getset, float *x, float *y, float *z ) {
   // static float current_position_x = 0;
   // static float current_position_y = 500;
   // static float current_position_z = -2000;
   // static int is_changed_manage_current_pos = 0;

   int ret = 0;

   if (getset == GET) {
      ret = is_changed_manage_current_pos;
      *x = current_position_x;
      *y = current_position_y;
      *z = current_position_z;
      is_changed_manage_current_pos = 0;
   }
   else if (getset == SET) {
      //do i consider the mode???
      is_changed_manage_current_pos = 1;
      current_position_x = *x;
      current_position_y = *y;
      current_position_z = *z;
   }
   return ret;
}
// target_pos_vel_t *target;
// pos_vel_t *current;
void PIDCONTROLLER::timerCallback(const ros::TimerEvent&) {

   // static int tim1_timer = 0, tim2_timer = 0;

   if ( ros::Time::now().toSec() - node_cur_time > 2 ) {
      if (!tim2_timer && node_cur_time) {
         std::cout << drone_num <<"," <<tim2_timer <<":No Coordinate after 2 min" << std::endl;
         tim2_timer = 1;
         unsigned int flight_mode = MODE_GROUND;
         manage_mode(SET_TIMER, &flight_mode);
         //Write the pid_output
         pid_output_msg.data[0] = 1500; // ROLL
         pid_output_msg.data[1] = 1500;
         pid_output_msg.data[3] = 1500;
         pid_output_msg.data[2] = 1500;   // YAW
         pid_output_msg.data[4] = 1050;
         pid_out_pub.publish(pid_output_msg);
      }
   }
   else if ( ros::Time::now().toSec() - node_cur_time > 1 ) {
      if (!tim1_timer && node_cur_time) {
         std::cout << drone_num <<"," <<tim2_timer<< ":No Coordinate after 1 min" << std::endl;
         tim1_timer = 1;
         unsigned int flight_mode = MODE_LANDING;
         manage_mode(SET_TIMER, &flight_mode);
      }
   }
   else {
      std::cout << drone_num <<"," <<tim2_timer<<  ":release" << std::endl;
      tim1_timer = 0;
      tim2_timer = 0;
   }

   // update_param();
}
void PIDCONTROLLER::position_Callback(const geometry_msgs::Point& msg) {
   // static pos_vel_t current_X = {0,};
   // static pos_vel_t current_Y = {0,};
   // static pos_vel_t current_Z = {0,};
   // static unsigned int flight_mode_position_callback = MODE_GROUND;

   // static pid_calc_t pid_pos_X = {0, };
   // static pid_calc_t pid_rate_X = {0, };
   // static target_pos_vel_t target_X = {0, };
   // //Y
   // static pid_calc_t pid_pos_Y = {0, };
   // static pid_calc_t pid_rate_Y = {0, };
   // static target_pos_vel_t target_Y = {0, };
   // //Z
   // static pid_calc_t pid_pos_Z = {0, };
   // static pid_calc_t pid_rate_Z = {0, 0, 0, -500, 0, 0, 0, 0, 0, 0};
   // //pid_rate_Z.integrator = -500;
   // static target_pos_vel_t target_Z = {0, };

   // static double node_last_time = 0;

   // static float target_pos_x = 0;
   // static float target_pos_y = 0;
   // static float target_pos_z = 0;


   /*
    *       Check and save the time.
    *       Calculate the velocity
    *       Publish the velocity
    */
   node_cur_time = ros::Time::now().toSec();
   int is_arm = 1000;
   // DECLARE the pid output

   // DECLARE the inner pid message
   geometry_msgs::Inertia pid_inner_y_msg;
   geometry_msgs::Inertia pid_inner_z_msg;
   // DECLARE the X, Y, Z pid calculation variables.
   //X
   /*
         1. restrict the target velocity by 200.   OK
         2. set the target_position.               OK
   */
   //JUST ADD MY TARGET VELOCITY. PLEASE CHANGE LATER

   //JUST ADD MY TARGET POSITION. PLEASE CHANGE LATER

   int ground_altitude = GROUND_ALTITUDE;

   current_X.cur_time = current_Y.cur_time = current_Z.cur_time = node_cur_time;
   current_X.lpf.cur_time = current_Y.lpf.cur_time = current_Z.lpf.cur_time = node_cur_time;

   current_X.cur_pos = msg.x;
   current_Y.cur_pos = msg.y;
   current_Z.cur_pos = msg.z;

   calc_velocity(&current_X);
   calc_velocity(&current_Y);
   calc_velocity(&current_Z);

   /* velocity Low Pass Filter*/
   // current_X.lpf.input = current_X.cur_vel;
   // current_Y.lpf.input = current_Y.cur_vel;
   // current_Z.lpf.input = current_Z.cur_vel;
   // current_X.cur_vel = get_lpf(&(current_X.lpf), 2);
   // current_Y.cur_vel = get_lpf(&(current_Y.lpf), 2);
   // current_Z.cur_vel = get_lpf(&(current_Z.lpf), 5);

   /* Position Low Pass Filter*/
   current_X.lpf.input = current_X.cur_pos;
   current_Y.lpf.input = current_Y.cur_pos;
   current_Z.lpf.input = current_Z.cur_pos;
   current_X.cur_pos = get_lpf(&(current_X.lpf), 4);
   current_Y.cur_pos = get_lpf(&(current_Y.lpf), 3);
   current_Z.cur_pos = get_lpf(&(current_Z.lpf), 5);


   geometry_msgs::Point velocity_msg;
   velocity_msg.x = current_X.cur_vel;
   velocity_msg.y = current_Y.cur_vel;
   velocity_msg.z = current_Z.cur_vel;
   velocity_pub.publish(velocity_msg);

   manage_current_pos(SET, &(current_X.cur_pos), &(current_Y.cur_pos), &(current_Z.cur_pos));
   // if ( node_cur_time - node_last_time > 3.0 ) {
   //    flight_mode_position_callback = MODE_GROUND;
   //    manage_mode(SET, &flight_mode_position_callback);
   //    static int is_first = 0;
   //    if (is_first) {
   //       std::cout << "Position Input has been delayed more than 3 seconds." << std::endl;
   //    }
   //    is_first = 1;
   // }
   node_last_time = node_cur_time;

   int is_changed_target = manage_target(GET, &target_pos_x, &target_pos_y, &target_pos_z);
   if ( target_pos_x == 0 && target_pos_y == 0 && target_pos_z == 0) {
      manage_target(SET, &current_X.cur_pos, &current_Y.cur_pos, &current_Z.cur_pos);
   }
   int is_changed_mode = manage_mode(GET, &flight_mode_position_callback);

   target_X.target_pos = target_pos_x;
   target_Y.target_pos = target_pos_y;
   target_Z.target_pos = target_pos_z;

   if (flight_mode_position_callback == MODE_NAV) {
      calc_navi_set_target(&target_X, &current_X, &target_Y, &current_Y, &target_Z, &current_Z , limited_target_vel);
      navi_rate(&pid_pos_Z, &pid_rate_Z, &target_Z, &current_Z, limited_target_vel, &pid_inner_z_pub, &pid_pos_param_Z, &pid_rate_param_Z, is_changed_target);
      if (pid_rate_Z.output < 0) {
         reset_I(&pid_rate_X, 0);
         reset_I(&pid_rate_Y, 0);
      }
      navi_rate(&pid_pos_X, &pid_rate_X, &target_X, &current_X, limited_target_vel, &pid_inner_x_pub, &pid_pos_param_X, &pid_rate_param_X, is_changed_target);
      navi_rate(&pid_pos_Y, &pid_rate_Y, &target_Y, &current_Y, limited_target_vel, &pid_inner_y_pub, &pid_pos_param_Y, &pid_rate_param_Y, is_changed_target);
      is_arm = 1950;
   }

   else if (flight_mode_position_callback == MODE_MANUAL) {
      calc_takeoff_altitude(&pid_rate_Z);
      manual(&pid_pos_Z, &pid_rate_Z, &target_Z, &current_Z, limited_target_vel, &pid_inner_z_pub, &pid_pos_param_Z, &pid_rate_param_Z, max_vel);
      if (pid_rate_Z.output < 0) {
         reset_I(&pid_rate_X, 0);
         reset_I(&pid_rate_Y, 0);
      }
      manual(&pid_pos_X, &pid_rate_X, &target_X, &current_X, limited_target_vel, &pid_inner_x_pub, &pid_pos_param_X, &pid_rate_param_X, max_vel);
      manual(&pid_pos_Y, &pid_rate_Y, &target_Y, &current_Y, limited_target_vel, &pid_inner_y_pub, &pid_pos_param_Y, &pid_rate_param_Y, max_vel);
      manage_target(SET, &current_X.cur_pos, &current_Y.cur_pos, &current_Z.cur_pos);
      is_arm = 1950;
   }

   else if (flight_mode_position_callback == MODE_POSHOLD) {
      //Calculate the pos_hold mod
      calc_takeoff_altitude(&pid_rate_Z);
      pos_hold(&pid_pos_Z, &pid_rate_Z, &target_Z, &current_Z, limited_target_vel, &pid_inner_z_pub, &pid_pos_param_Z, &pid_rate_param_Z);
      if (pid_rate_Z.output < 0) {
         reset_I(&pid_rate_X, 0);
         reset_I(&pid_rate_Y, 0);
      }
      pos_hold(&pid_pos_X, &pid_rate_X, &target_X, &current_X, limited_target_vel, &pid_inner_x_pub, &pid_pos_param_X, &pid_rate_param_X);
      pos_hold(&pid_pos_Y, &pid_rate_Y, &target_Y, &current_Y, limited_target_vel, &pid_inner_y_pub, &pid_pos_param_Y, &pid_rate_param_Y);
      is_arm = 1950;
   }
   else if (flight_mode_position_callback == MODE_TAKEOFF) {
      calc_takeoff_altitude(&pid_rate_Z);
      calc_takeoff_altitude_once(&pid_rate_Z, is_changed_mode);
      target_Z.target_vel = TAKEOFF_SPEED;
      navi_rate(&pid_pos_Z, &pid_rate_Z, &target_Z, &current_Z, limited_target_vel, &pid_inner_z_pub, &pid_pos_param_Z, &pid_rate_param_Z, is_changed_target);
      if (pid_rate_Z.output < 0) {
         reset_I(&pid_rate_X, 0);
         reset_I(&pid_rate_Y, 0);
      }
      pos_hold(&pid_pos_X, &pid_rate_X, &target_X, &current_X, limited_target_vel, &pid_inner_x_pub, &pid_pos_param_X, &pid_rate_param_X);
      pos_hold(&pid_pos_Y, &pid_rate_Y, &target_Y, &current_Y, limited_target_vel, &pid_inner_y_pub, &pid_pos_param_Y, &pid_rate_param_Y);
      is_arm = 1950;
   }
   else if (flight_mode_position_callback == MODE_LANDING) {
      target_Z.target_vel = LANDING_SPEED;
      navi_rate(&pid_pos_Z, &pid_rate_Z, &target_Z, &current_Z, limited_target_vel, &pid_inner_z_pub, &pid_pos_param_Z, &pid_rate_param_Z, is_changed_target);
      if (pid_rate_Z.output < 0) {
         reset_I(&pid_rate_X, 0);
         reset_I(&pid_rate_Y, 0);
      }
      pos_hold(&pid_pos_X, &pid_rate_X, &target_X, &current_X, limited_target_vel, &pid_inner_x_pub, &pid_pos_param_X, &pid_rate_param_X);
      pos_hold(&pid_pos_Y, &pid_rate_Y, &target_Y, &current_Y, limited_target_vel, &pid_inner_y_pub, &pid_pos_param_Y, &pid_rate_param_Y);

      is_arm = 1950;
      if ( current_Z.cur_pos < ground_altitude + 60) {
         flight_mode_position_callback = MODE_GROUND;
         manage_mode(SET, &flight_mode_position_callback);
      }
   }
   if (flight_mode_position_callback == MODE_GROUND) {
      reset_PID(&pid_pos_X, 0.0);
      reset_PID(&pid_rate_X, 0.0);
      reset_PID(&pid_pos_Y, 0.0);
      reset_PID(&pid_rate_Y, 0.0);
      reset_PID(&pid_pos_Z, 0.0);
      reset_PID(&pid_rate_Z, -500.0);
      pid_rate_X.output = 0;
      pid_rate_Y.output = 0;
      pid_rate_Z.output = -500;
      is_arm = 1050;
   }



   //Write the pid_output
   pid_output_msg.data[0] = 1500 - x_offset - (unsigned short)constrain(pid_rate_X.output, -500.0, 500.0); // ROLL
   pid_output_msg.data[1] = 1500 - y_offset - (unsigned short)constrain(pid_rate_Y.output, -500.0, 500.0); // PITCH
   pid_output_msg.data[3] = 1500 + (unsigned short)constrain(pid_rate_Z.output, -500.0, 500.0); // THROTTLE
   pid_output_msg.data[2] = 1500;   // YAW
   pid_output_msg.data[4] = is_arm;
   pid_out_pub.publish(pid_output_msg);

}

void PIDCONTROLLER::reboot_drone() {
   std::cout << "RESET_VALUE" << std::endl;
   pid_output_msg.data[0] = 1500; // ROLL
   pid_output_msg.data[1] = 1500; // PITCH
   pid_output_msg.data[3] = 1000; // THROTTLE
   pid_output_msg.data[2] = 1500;   // YAW
   pid_output_msg.data[4] = 500;
   pid_out_pub.publish(pid_output_msg);
}

void PIDCONTROLLER::targetCallback(const geometry_msgs::Quaternion& msg) {
   std::cout << "::TARGET MESSAGE::" << std::endl;;
   float target_x = msg.x;
   float target_y = msg.y;
   float target_z = msg.z;

   float current_x = 0;
   float current_y = 0;
   float current_z = 0;

   manage_current_pos(GET, &current_x, &current_y, &current_z);

   unsigned int mission = (unsigned int)msg.w;
   unsigned int tmp_mod = MODE_GROUND;

   if ( mission <= MISSION_AUX )
      std::cout << "MISSION MESSAGE IS :: " << mission_str[mission] << std::endl;

   if (mission == MISSION_TAKEOFF) {
      tmp_mod = MODE_TAKEOFF;

      if (current_z > GROUND_ALTITUDE + 100 ) {
         std::cout << "NOT THE GROUND" << std::endl;
      }

      if ( manage_mode(SET, &tmp_mod) != MANAGE_MODE_ERROR )
         if (target_z)
            manage_target(SET_TARGET, &current_x, &current_y, &target_z);
         else {
            current_z += 500;
            manage_target(SET_TARGET, &current_x, &current_y, &current_z);
         }
   }
   else if (mission == MISSION_AUTO) {
      tmp_mod = MODE_NAV;
      if ( manage_mode(SET, &tmp_mod) != MANAGE_MODE_ERROR )
         if (manage_target(SET_TARGET, &target_x, &target_y, &target_z) == MANAGE_TARGET_ERROR) {
            tmp_mod = MODE_POSHOLD;
            manage_mode(SET, &tmp_mod);
            //manage_target(SET_TARGET, &current_x, &current_y, &current_z);
         }
   }
   else if (mission == MISSION_MANUAL) {
      tmp_mod = MODE_MANUAL;
      if ( manage_mode(SET, &tmp_mod) != MANAGE_MODE_ERROR )
         manage_target(SET_TARGET, &target_x, &target_y, &target_z);
   }
   else if (mission == MISSION_LANDING) {
      tmp_mod = MODE_LANDING;
      if ( manage_mode(SET, &tmp_mod) != MANAGE_MODE_ERROR ) {
         target_z = -3000;
         manage_target(SET_TARGET, &current_x, &current_y, &target_z);
      }
   }
   else if (mission == MISSION_AUX ) {
      tmp_mod = MODE_NAV;
      if ( manage_mode(SET, &tmp_mod) != MANAGE_MODE_ERROR ) {
         current_x += target_x;
         current_y += target_y;
         current_z += target_z;
         manage_target(SET_TARGET, &current_x, &current_y, &current_z);
      }
   }
   else if (mission == MISSION_GROUND) {
      tmp_mod = MODE_GROUND;
      manage_mode(SET, &tmp_mod);
      manage_target(SET_TARGET, &current_x, &current_y, &current_z);
   }
   else if (mission == MISSION_RESET)
   {
      reboot_drone();
   }
}