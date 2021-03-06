#include "ros/ros.h"
#include "pidcontroller.h"
#include "math.h"
#include <iostream>
using namespace std;


PIDCONTROLLER::~PIDCONTROLLER() {

}
PIDCONTROLLER::PIDCONTROLLER(std::string DRONE) :
   drone(DRONE),
   limited_target_vel(1000.0f),
   max_vel(300.0f),
   max_proportional_vel(600.0f),

   is_changed_manage_mode(0),
   is_changed_manage_target(0),
   is_changed_manage_current_pos(0),

   target_pos_x(0.0f),
   target_pos_y(0.0f),
   target_pos_z(0.0f),

   current_target_x(0.0f),
   current_target_y(0.0f),
   current_target_z(0.0f),

   seq(0),
   seq2(0),

   current_position_x(0.0f),
   current_position_y(500.0f),
   current_position_z(-3500.0f),

   last_target_x(0.0f),
   last_target_y(0.0f),
   last_target_z(0.0f),

   offset_roll(1500.0f),
   offset_pitch(1500.0f),
   offset_throttle(1500.0f),

   takeoff_altitude(-3000.0f),

   is_landing_range(0),

   tim1_timer(0.0f),
   tim2_timer(0.0f),

   mag_counter(1),

   current_mode(MODE_NOT_DETECTED),

   flight_mode_position_callback(MODE_GROUND),

   node_cur_time(0.0l),
   node_last_time(0.0l),
   reboot_time(0.0l),
   manual_time(0.0l),
   failsafe_time(0.0l),
   mode_change_time_to_smooth_target_velocity(0.0l),
   ground_altitude(GROUND_ALTITUDE),
//teste
   lpf_offset_x(0.1f),
   lpf_offset_y(0.1f),
   lpf_offset_z(0.1f),

   lpf_vel_x(1.5f),
   lpf_vel_y(1.5f),
   lpf_vel_z(1.5f),

   lpf_pos_x(2.0f),
   lpf_pos_y(2.0f),
   lpf_pos_z(2.0f),

   lpf_target_x(0.0f),
   lpf_target_y(0.0f),
   lpf_target_z(0.0f),

   kalman_x(10.0f, 100.0f, 1.0f),
   kalman_y(10.0f, 100.0f, 1.0f),
   kalman_z(10.0f, 100.0f, 1.0f),

   is_arm(1000),
   is_first_get_position(0),
   changed_to_poshold_x(0),
   changed_to_poshold_y(0),
   changed_to_poshold_z(0),

   pid_poshold_pos_param_X(__pid_poshold_pos_param_X),
   pid_poshold_rate_param_X(__pid_poshold_rate_param_X),
   pid_poshold_pos_param_Y(__pid_poshold_pos_param_Y),
   pid_poshold_rate_param_Y(__pid_poshold_rate_param_Y),
   pid_poshold_pos_param_Z(__pid_poshold_pos_param_Z),
   pid_poshold_rate_param_Z(__pid_poshold_rate_param_Z),
   pid_nav_pos_param_X(__pid_nav_pos_param_X),
   pid_nav_rate_param_X(__pid_nav_rate_param_X),
   pid_nav_pos_param_Y(__pid_nav_pos_param_Y),
   pid_nav_rate_param_Y(__pid_nav_rate_param_Y),
   pid_nav_pos_param_Z(__pid_nav_pos_param_Z),
   pid_nav_rate_param_Z(__pid_nav_rate_param_Z),

   flight_param(__flight_param),
   pid_rate_Z(-450.0f)
{
   pid_output_msg.data.resize(5, 1000);
   drone_num = making_drone();
   ;


   // std::string tmp_dir("/tmp/");
   std::string tmp_st("pidparam");
   std::string tmp_st_start;

   char dnum[2];
   dnum[0] = '1' + drone_num;
   dnum[1] = 0;
   tmp_st += dnum;
   tmp_st_start = "tmp_" + tmp_st;

   tmp_st = tmp_dir + tmp_st;
   tmp_st_start = tmp_dir + tmp_st_start;

   std::cout << "initializing.." << drone;
   set_self_param();
   if ( load_param(tmp_st.c_str(), &pid_param_c) ) {
      std::cout << "::" << "Saved Parameter loaded!" << tmp_st  << std::endl;
      save_param(tmp_st_start.c_str(), &pid_param_c);
   }

   else {
      set_common_param();
      std::cout << std::endl;
   }

   std::string current_vel = drone + "/CURRENT_VEL";
   std::string output_pid = drone +  "/OUTPUT_PID";
   std::string output_inner_pid_x = drone + "/OUTPUT_INNER_PID/X";
   std::string output_inner_pid_y = drone + "/OUTPUT_INNER_PID/Y";
   std::string output_inner_pid_z = drone + "/OUTPUT_INNER_PID/Z";
   std::string current_pos = drone + "/CURRENT_POS";
   std::string target_pos = drone + "/TARGET_POS";

   velocity_pub = nod.advertise<geometry_msgs::Point>(current_vel, 10);
   // timer = nod.createTimer(ros::Duration(0.08), &PIDCONTROLLER::timerCallback, this);
   pid_out_pub     = nod.advertise<std_msgs::UInt16MultiArray>(output_pid, 10);
   pid_inner_x_pub = nod.advertise<geometry_msgs::Inertia>(output_inner_pid_x, 10);
   pid_inner_y_pub = nod.advertise<geometry_msgs::Inertia>(output_inner_pid_y, 10);
   pid_inner_z_pub = nod.advertise<geometry_msgs::Inertia>(output_inner_pid_z, 10);

   position_sub = nod.subscribe(current_pos, 10, &PIDCONTROLLER::position_Callback, this);
// position_sub = nod.subscribe(current_pos, 1, &PIDCONTROLLER::seq_Callback, this);

   target_sub = nod.subscribe(target_pos, 10, &PIDCONTROLLER::targetCallback, this);

   pid_rate_Z.integrator = -500.0f;
}

int PIDCONTROLLER::making_drone() {
   static int making_drone_num = 0;
   return making_drone_num++;
}

int PIDCONTROLLER::manage_mode(unsigned int getset, unsigned int *state) {
   int ret = 0;
   // if(current_mode <= MODE_GROUND && *state <= MODE_GROUND)
   // std::cout<< getset << ":" << mode_str[current_mode] << "  " <<mode_str[*state] << std::endl;
   if (current_mode == *state)
      return ret;
   if (current_mode == MODE_NOT_DETECTED && *state != MODE_GROUND) {
      return MANAGE_MODE_ERROR;
   }

   if (getset == GET) {
      ret = is_changed_manage_mode;
      is_changed_manage_mode = 0;
      *state = current_mode;
   }
   else if (getset == SET || getset == SET_TIMER) {
      std::cout << drone << ":" << mode_str[current_mode] << " TO " << mode_str[*state];
      if ((*state == MODE_TAKEOFF && current_mode != MODE_GROUND) ) {
         std::cout << "     NO PERMISSION to TAKEOFF" << std::endl;
         return MANAGE_MODE_ERROR;
      }
      if (current_mode == MODE_GROUND && *state != MODE_TAKEOFF ) {
         std::cout << "     NO PERMISSION" << std::endl;
         return MANAGE_MODE_ERROR;
      }

      if ( !( current_mode == MODE_TAKEOFF || *state == MODE_TAKEOFF ) )
         if ( !( current_mode == MODE_LANDING || *state == MODE_LANDING ) )
            if ( current_mode == MODE_POSHOLD || *state == MODE_POSHOLD || current_mode == MODE_NAV || *state == MODE_NAV ) {
               mode_change_time_to_smooth_target_velocity = ros::Time::now().toSec();
            }


      if (current_mode != *state)
         is_changed_manage_mode = 1;
      current_mode = *state;
      std::cout << std::endl;
   }
   return ret;
}

int PIDCONTROLLER::manage_target(unsigned int getset, float *x, float *y, float *z ) {
   float current_x = 0.0f;
   float current_y = 0.0f;
   float current_z = 0.0f;
   manage_current_pos(GET, &current_x, &current_y, &current_z);
   int ret = 0;
   if (getset == GET) {
      // std::cout << drone << ":" <<"GET the TARGET" <<current_target_x<<","<<current_target_x<<","<<current_target_x << std::endl;
      ret = is_changed_manage_target;
      *x = current_target_x;
      *y = current_target_y;
      *z = current_target_z;
      is_changed_manage_target = 0;
   }
   else if (getset == SET_MANUAL) {
      current_target_x = *x;
      current_target_y = *y;
      current_target_z = *z;
      std::cout << drone << ":" << "MANUAL TARGET VELOCITY: " << current_target_x << "," << current_target_y << "," << current_target_z << std::endl;
   }
   else if (getset == SET || getset == SET_TARGET ) {
      if (*y == 0.0f || *z == 0.0f) {
         current_target_x = current_x;
         current_target_y = current_y;
         current_target_z = current_z;
         std::cout << drone << ":" << "ERROR TARGET. SET the CURRENT POSITION" << std::endl;
         return MANAGE_TARGET_ERROR;
      }
      //do i consider the mode???
      if (getset == SET_TARGET)
         is_changed_manage_target = 1;
      current_target_x = *x;
      current_target_y = *y;
      current_target_z = *z;
      std::cout << drone << ":" << "SET the TARGET" << current_target_x << "," << current_target_y << "," << current_target_z << std::endl;
   }
   return ret;
}

int PIDCONTROLLER::manage_current_pos(unsigned int getset, float *x, float *y, float *z ) {
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

void PIDCONTROLLER::timerCallback(const ros::TimerEvent&) {
   if (ros::Time::now().toSec() - node_cur_time > 2 ) {
      if (!tim2_timer && node_cur_time) {
         std::cout << drone << ":" << drone_num << "," << tim2_timer << ":No Coordinate after 2 min" << std::endl;
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
   else if (ros::Time::now().toSec() - node_cur_time > 1 ) {
      if (!tim1_timer && node_cur_time) {
         std::cout << drone << ":" << drone_num << "," << tim2_timer << ":No Coordinate after 1 min" << std::endl;
         tim1_timer = 1;
         unsigned int flight_mode = MODE_LANDING;
         manage_mode(SET_TIMER, &flight_mode);
      }
   }
   else {
      std::cout << drone << ":" << drone_num << "," << tim2_timer <<  ":release" << std::endl;
      tim1_timer = 0;
      tim2_timer = 0;
   }
}
void PIDCONTROLLER::seq_Callback(const geometry_msgs::Point& msg){
   seq++;
   std::cout << "sequence : " << seq << std::endl;
}


void PIDCONTROLLER::position_Callback(const geometry_msgs::Point& msg) {
   seq++;
   node_cur_time = ros::Time::now().toSec();

   current_X.cur_time = current_Y.cur_time = current_Z.cur_time = node_cur_time;

   current_X.cur_pos = msg.x;
   current_Y.cur_pos = msg.y;
   current_Z.cur_pos = msg.z;

   if ( current_Y.cur_pos == 0 || current_Z.cur_pos == 0) {
      std::cout << drone << ": NO COORDINATE INFO SET LAST POSITION" << std::endl;
      unsigned int tmp_flight_mode;
      int is_changed_mode_tmp = manage_mode(GET, &tmp_flight_mode);

      current_X.cur_pos = last_target_x;
      current_Y.cur_pos = last_target_y;
      current_Z.cur_pos = last_target_z;

      // In flight
      if ( tmp_flight_mode != MODE_GROUND && tmp_flight_mode != MODE_NOT_DETECTED ) {
         if (!failsafe_time) {
            std::cout << drone << ": GET IN FAILSAFE" << std::endl;
            failsafe_time = ros::Time::now().toSec();
         }
         else if (failsafe_time > 0) {
            if ( ros::Time::now().toSec() - failsafe_time > 4.0l ) {
               std::cout << drone << ": FAILSAFE TO GROUND" << std::endl;
               unsigned int tmp_mod = MODE_GROUND;
               manage_mode(SET, &tmp_mod);
            }
            else if ( ros::Time::now().toSec() - failsafe_time > 2.0l ) {
               std::cout << drone << ": FAILSAFE TO LANDING" << std::endl;
               unsigned int tmp_mod = MODE_FAILSAFE;
               manage_mode(SET, &tmp_mod);
            }
         }
      }
      else
         failsafe_time = 0;
   }
   else
      failsafe_time = 0;

   last_target_x = current_X.cur_pos;
   last_target_y = current_Y.cur_pos;
   last_target_z = current_Z.cur_pos;

   if (!is_first_get_position) {
      is_first_get_position = 1;
      std::cout << drone << ": GET THE FIRST POSITION" << std::endl;
      unsigned int flight_mode = MODE_GROUND;
      manage_mode(SET, &flight_mode);
   }

   calc_velocity(&current_X);
   calc_velocity(&current_Y);
   calc_velocity(&current_Z);

   /* velocity Low Pass Filter*/
   // lpf_vel_x.set_cutoff_freq(1.5f);
   // lpf_vel_y.set_cutoff_freq(1.5f);
   // lpf_vel_z.set_cutoff_freq(1.5f);
   kalman_x.getKalman(current_X.cur_pos);
   kalman_y.getKalman(current_Y.cur_pos);
   kalman_z.getKalman(current_Z.cur_pos);

   // current_X.cur_vel = kalman_x.getvel();
   // current_Y.cur_vel = kalman_y.getvel();
   // current_Z.cur_vel = kalman_z.getvel();

   // current_X.cur_pos = kalman_x.getpos();
   // current_Y.cur_pos = kalman_y.getpos();
   // current_Z.cur_pos = kalman_z.getpos();

   current_X.cur_vel = lpf_vel_x.get_lpf(current_X.cur_vel);
   current_Y.cur_vel = lpf_vel_y.get_lpf(current_Y.cur_vel);
   current_Z.cur_vel = lpf_vel_z.get_lpf(current_Z.cur_vel);

   /* Position Low Pass Filter*/
   // lpf_pos_x.set_cutoff_freq(2.0f);
   // lpf_pos_y.set_cutoff_freq(2.0f);
   // lpf_pos_z.set_cutoff_freq(2.0f);
   current_X.cur_pos = lpf_pos_x.get_lpf(current_X.cur_pos);
   current_Y.cur_pos = lpf_pos_y.get_lpf(current_Y.cur_pos);
   current_Z.cur_pos = lpf_pos_z.get_lpf(current_Z.cur_pos);

   geometry_msgs::Point velocity_msg;
   velocity_msg.x = current_X.cur_vel;
   velocity_msg.y = current_Y.cur_vel;
   velocity_msg.z = current_Z.cur_vel;
   velocity_pub.publish(velocity_msg);

   manage_current_pos(SET, &(current_X.cur_pos), &(current_Y.cur_pos), &(current_Z.cur_pos));

   node_last_time = node_cur_time;

   int is_changed_target = manage_target(GET, &target_pos_x, &target_pos_y, &target_pos_z);
   if (target_pos_x == 0.0f && target_pos_y == 0.0f && target_pos_z == 0.0f) {
      manage_target(SET, &current_X.cur_pos, &current_Y.cur_pos, &current_Z.cur_pos);
   }
   int is_changed_mode = manage_mode(GET, &flight_mode_position_callback);

   target_X.target_pos = target_pos_x;
   target_Y.target_pos = target_pos_y;
   target_Z.target_pos = target_pos_z;


   if ( ros::Time::now().toSec() - mode_change_time_to_smooth_target_velocity < 0.5f) {
      lpf_target_x.set_cutoff_freq(2.0f);
      lpf_target_y.set_cutoff_freq(2.0f);
   }
   else {
      lpf_target_x.set_cutoff_freq(0.0f);
      lpf_target_y.set_cutoff_freq(0.0f);
   }


   if (flight_mode_position_callback == MODE_NAV_P) {

      int sum_nav = 0;
      calc_navi_set_target(&target_X, &current_X, &target_Y, &current_Y, &target_Z, &current_Z , max_vel);
      sum_nav += navi_rate(&pid_pos_Z, &pid_rate_Z, &target_Z, &current_Z, limited_target_vel, &pid_inner_z_pub, pid_param_c.pos_nav_pid_Z, pid_param_c.rate_nav_pid_Z, is_changed_target, pid_param_c.pos_pid_Z, pid_param_c.rate_pid_Z, &changed_to_poshold_z, &offset_throttle, &lpf_offset_z, &lpf_target_z);
      if (pid_rate_Z.output < 50.0f) {
         reset_I(&pid_rate_X, 0.0f);
         reset_I(&pid_rate_Y, 0.0f);
         reset_I(&pid_pos_X, 0.0f);
         reset_I(&pid_pos_Y, 0.0f);
      }
      sum_nav += navi_rate(&pid_pos_X, &pid_rate_X, &target_X, &current_X, limited_target_vel, &pid_inner_x_pub, pid_param_c.pos_nav_pid_X, pid_param_c.rate_nav_pid_X, is_changed_target, pid_param_c.pos_pid_X, pid_param_c.rate_pid_X, &changed_to_poshold_x, &offset_roll, &lpf_offset_x, &lpf_target_x);
      sum_nav += navi_rate(&pid_pos_Y, &pid_rate_Y, &target_Y, &current_Y, limited_target_vel, &pid_inner_y_pub, pid_param_c.pos_nav_pid_Y, pid_param_c.rate_nav_pid_Y, is_changed_target, pid_param_c.pos_pid_Y, pid_param_c.rate_pid_Y, &changed_to_poshold_y, &offset_pitch, &lpf_offset_y, &lpf_target_y);

      if (sum_nav == 3) {
         unsigned int tmp_mod = MODE_POSHOLD;
         manage_mode(SET, &tmp_mod);
      }
      is_arm = 1950;
   }
   else if (flight_mode_position_callback == MODE_NAV) {

      int sum_nav = 0;
      calc_navi_proportional_set_target(&target_X, &current_X, &target_Y, &current_Y, &target_Z, &current_Z , max_proportional_vel, &pid_navi_proprotion_vel, pid_param_c.pos_nav_pid_X);
      sum_nav += navi_rate_proportional(&pid_pos_Z, &pid_rate_Z, &target_Z, &current_Z, limited_target_vel, &pid_inner_z_pub, pid_param_c.pos_nav_pid_Z, pid_param_c.rate_nav_pid_Z, is_changed_target, pid_param_c.pos_pid_Z, pid_param_c.rate_pid_Z, &changed_to_poshold_z, &offset_throttle, &lpf_offset_z, &lpf_target_z);
      if (pid_rate_Z.output < 50.0f) {
         reset_I(&pid_rate_X, 0.0f);
         reset_I(&pid_rate_Y, 0.0f);
         reset_I(&pid_pos_X, 0.0f);
         reset_I(&pid_pos_Y, 0.0f);
      }
      sum_nav += navi_rate_proportional(&pid_pos_X, &pid_rate_X, &target_X, &current_X, limited_target_vel, &pid_inner_x_pub, pid_param_c.pos_nav_pid_X, pid_param_c.rate_nav_pid_X, is_changed_target, pid_param_c.pos_pid_X, pid_param_c.rate_pid_X, &changed_to_poshold_x, &offset_roll, &lpf_offset_x, &lpf_target_x);
      sum_nav += navi_rate_proportional(&pid_pos_Y, &pid_rate_Y, &target_Y, &current_Y, limited_target_vel, &pid_inner_y_pub, pid_param_c.pos_nav_pid_Y, pid_param_c.rate_nav_pid_Y, is_changed_target, pid_param_c.pos_pid_Y, pid_param_c.rate_pid_Y, &changed_to_poshold_y, &offset_pitch, &lpf_offset_y, &lpf_target_y);

      if (sum_nav == 3) {
         unsigned int tmp_mod = MODE_POSHOLD;
         manage_mode(SET, &tmp_mod);
      }
      is_arm = 1950;
   }   
   else if (flight_mode_position_callback == MODE_NAV_N) {
      calc_navi_set_target(&target_X, &current_X, &target_Y, &current_Y, &target_Z, &current_Z , max_vel);
      navi_rate_next(&pid_pos_Z, &pid_rate_Z, &target_Z, &current_Z, limited_target_vel, &pid_inner_z_pub, pid_param_c.pos_nav_pid_Z, pid_param_c.rate_nav_pid_Z);
      if (pid_rate_Z.output < 50.0f) {
         reset_I(&pid_rate_X, 0.0f);
         reset_I(&pid_rate_Y, 0.0f);
         reset_I(&pid_pos_X, 0.0f);
         reset_I(&pid_pos_Y, 0.0f);
      }
      navi_rate_next(&pid_pos_X, &pid_rate_X, &target_X, &current_X, limited_target_vel, &pid_inner_x_pub, pid_param_c.pos_nav_pid_X, pid_param_c.rate_nav_pid_X);
      navi_rate_next(&pid_pos_Y, &pid_rate_Y, &target_Y, &current_Y, limited_target_vel, &pid_inner_y_pub, pid_param_c.pos_nav_pid_Y, pid_param_c.rate_nav_pid_Y);
      is_arm = 1950;
   }
   else if (flight_mode_position_callback == MODE_MANUAL) {
      manual(&pid_pos_Z, &pid_rate_Z, &target_Z, &current_Z, limited_target_vel, &pid_inner_z_pub, pid_param_c.pos_nav_pid_Z, pid_param_c.rate_nav_pid_Z, max_vel, &lpf_target_z);
      if (pid_rate_Z.output < 50.0f) {
         reset_I(&pid_rate_X, 0.0f);
         reset_I(&pid_rate_Y, 0.0f);
         reset_I(&pid_pos_X, 0.0f);
         reset_I(&pid_pos_Y, 0.0f);
      }
      manual(&pid_pos_X, &pid_rate_X, &target_X, &current_X, limited_target_vel, &pid_inner_x_pub, pid_param_c.pos_nav_pid_X, pid_param_c.rate_nav_pid_X, max_vel, &lpf_target_x);
      manual(&pid_pos_Y, &pid_rate_Y, &target_Y, &current_Y, limited_target_vel, &pid_inner_y_pub, pid_param_c.pos_nav_pid_Y, pid_param_c.rate_nav_pid_Y, max_vel, &lpf_target_y);
      if ( ros::Time::now().toSec() - manual_time > 0.3l ) {
         manage_target(SET, &current_X.cur_pos, &current_Y.cur_pos, &current_Z.cur_pos);
         unsigned int flight_mode_tmp = MODE_POSHOLD;
         manage_mode(SET, &flight_mode_tmp);
      }
      is_arm = 1950;
   }
   else if (flight_mode_position_callback == MODE_POSHOLD) {
      //Calculate the pos_hold mod
      calc_takeoff_altitude(&pid_rate_Z);
      pos_hold(&pid_pos_Z, &pid_rate_Z, &target_Z, &current_Z, limited_target_vel, &pid_inner_z_pub, pid_param_c.pos_pid_Z, pid_param_c.rate_pid_Z, &offset_throttle, &lpf_offset_z, &lpf_target_z);
      if (pid_rate_Z.output < 50.0f) {
         reset_I(&pid_rate_X, 0.0f);
         reset_I(&pid_rate_Y, 0.0f);
         reset_I(&pid_pos_X, 0.0f);
         reset_I(&pid_pos_Y, 0.0f);
      }
      pos_hold(&pid_pos_X, &pid_rate_X, &target_X, &current_X, limited_target_vel, &pid_inner_x_pub, pid_param_c.pos_pid_X, pid_param_c.rate_pid_X, &offset_roll, &lpf_offset_x, &lpf_target_x);
      pos_hold(&pid_pos_Y, &pid_rate_Y, &target_Y, &current_Y, limited_target_vel, &pid_inner_y_pub, pid_param_c.pos_pid_Y, pid_param_c.rate_pid_Y, &offset_pitch, &lpf_offset_y, &lpf_target_y);
      is_arm = 1950;
   }
   else if (flight_mode_position_callback == MODE_TAKEOFF) {
      mag_counter++;
      if ( mag_counter % 10 == 0 )
         maghold_drone(pid_param_c.flight_param->pid_D + 600);
      calc_takeoff_altitude(&pid_rate_Z);
      calc_takeoff_altitude_once(&pid_rate_Z, is_changed_mode, 130, &is_takeoff);
      target_Z.target_vel = TAKEOFF_SPEED;

      pid_parameter_t tmp_pid_poshold_rate_param_Z = *pid_param_c.rate_nav_pid_Z;

      if (current_Z.cur_pos < takeoff_altitude + 30.0f) {
         target_Z.target_vel = TAKEOFF_SPEED + 100;
         tmp_pid_poshold_rate_param_Z.pid_I *= 11;
      }
      else if (current_Z.cur_pos < takeoff_altitude + 100.0f) {
         target_Z.target_vel = TAKEOFF_SPEED + 50;
         tmp_pid_poshold_rate_param_Z.pid_I *= 4;
      }

      if ( navi_rate(&pid_pos_Z, &pid_rate_Z, &target_Z, &current_Z, limited_target_vel, &pid_inner_z_pub, pid_param_c.pos_nav_pid_Z, &tmp_pid_poshold_rate_param_Z, is_changed_target, pid_param_c.pos_pid_Z, pid_param_c.rate_pid_Z, &changed_to_poshold_z, &offset_throttle, &lpf_offset_z, &lpf_target_z)) {
         unsigned int tmp_mod = MODE_POSHOLD;
         manage_mode(SET, &tmp_mod);
      }

      if (pid_rate_Z.output < 50.0f) {
         reset_I(&pid_rate_X, 0.0f);
         reset_I(&pid_rate_Y, 0.0f);
         reset_I(&pid_pos_X, 0.0f);
         reset_I(&pid_pos_Y, 0.0f);
      }
      pos_hold(&pid_pos_X, &pid_rate_X, &target_X, &current_X, limited_target_vel, &pid_inner_x_pub, pid_param_c.pos_pid_X, pid_param_c.rate_pid_X, &offset_roll, &lpf_offset_x, &lpf_target_x);
      pos_hold(&pid_pos_Y, &pid_rate_Y, &target_Y, &current_Y, limited_target_vel, &pid_inner_y_pub, pid_param_c.pos_pid_Y, pid_param_c.rate_pid_Y, &offset_pitch, &lpf_offset_y, &lpf_target_y);
      if ( is_arm == 1050 )
         pid_rate_Z.output = -500;

      is_arm = 1950;
   }
   else if (flight_mode_position_callback == MODE_LANDING) {
      target_Z.target_vel = LANDING_SPEED;

      pid_parameter_t tmp_pid_poshold_rate_param_Z = *pid_param_c.rate_nav_pid_Z;
      if (current_Z.cur_pos < takeoff_altitude + 70.0f || is_landing_range) {
         // calc_landing_altitude(&pid_rate_Z);
         is_landing_range = 1;
         tmp_pid_poshold_rate_param_Z.pid_I *= 10;
         tmp_pid_poshold_rate_param_Z.pid_P *= 5;
         std::cout << drone << "IN LANDING ZONE" << std::endl;
      }
      //TEST

      navi_rate(&pid_pos_Z, &pid_rate_Z, &target_Z, &current_Z, limited_target_vel, &pid_inner_z_pub, pid_param_c.pos_nav_pid_Z, &tmp_pid_poshold_rate_param_Z, is_changed_target, pid_param_c.pos_pid_Z, pid_param_c.rate_pid_Z, &changed_to_poshold_z, &offset_throttle, &lpf_offset_z, &lpf_target_z);


      if (pid_rate_Z.output < 50.0f) {
         reset_I(&pid_rate_X, 0.0f);
         reset_I(&pid_rate_Y, 0.0f);
         reset_I(&pid_pos_X, 0.0f);
         reset_I(&pid_pos_Y, 0.0f);
      }
      pos_hold(&pid_pos_X, &pid_rate_X, &target_X, &current_X, limited_target_vel, &pid_inner_x_pub, pid_param_c.pos_pid_X, pid_param_c.rate_pid_X, &offset_roll, &lpf_offset_x, &lpf_target_x);
      pos_hold(&pid_pos_Y, &pid_rate_Y, &target_Y, &current_Y, limited_target_vel, &pid_inner_y_pub, pid_param_c.pos_pid_Y, pid_param_c.rate_pid_Y, &offset_pitch, &lpf_offset_y, &lpf_target_y);

      is_arm = 1950;
      if (current_Z.cur_pos < takeoff_altitude + 30.0f) {
         flight_mode_position_callback = MODE_GROUND;
         manage_mode(SET, &flight_mode_position_callback);
      }
   }
   else if (flight_mode_position_callback == MODE_FAILSAFE) {
      pid_rate_X.output = offset_roll;
      pid_rate_Y.output = offset_pitch;
      pid_rate_Z.output = offset_throttle - 50;
   }



   if (flight_mode_position_callback == MODE_GROUND) {
      reset_PID(&pid_pos_X, 0.0f);
      reset_PID(&pid_rate_X, 0.0f);
      reset_PID(&pid_pos_Y, 0.0f);
      reset_PID(&pid_rate_Y, 0.0f);
      reset_PID(&pid_pos_Z, 0.0f);
      reset_PID(&pid_rate_Z, -500.0f);
      reset_PID(&pid_navi_proprotion_vel, 0.0f);
      pid_rate_X.output = 0;
      pid_rate_Y.output = 0;
      pid_rate_Z.output = -500;
      is_arm = 1050;
      is_landing_range = 0;
      is_takeoff = 0;
      changed_to_poshold_x = 0;
      changed_to_poshold_y = 0;
      changed_to_poshold_z = 0;
   }
   //Write the pid_output
   pid_output_msg.data[0] = (unsigned short)(1500.0f - pid_param_c.flight_param->pid_P - (unsigned short)constrain(pid_rate_X.output, -500.0, 500.0)); // ROLL
   pid_output_msg.data[1] = (unsigned short)(1500.0f - pid_param_c.flight_param->pid_I - (unsigned short)constrain(pid_rate_Y.output, -500.0, 500.0)); // PITCH
   pid_output_msg.data[3] = (unsigned short)(1500.0f + (unsigned short)constrain(pid_rate_Z.output, -500.0, 500.0)); // THROTTLE
   pid_output_msg.data[2] = (unsigned short)1500;   // YAW
   pid_output_msg.data[4] = is_arm;
   pid_out_pub.publish(pid_output_msg);
   seq2++;
}

void PIDCONTROLLER::reboot_drone() {
   std::cout << drone << ":" << "RESET_VALUE" << std::endl;
   pid_output_msg.data[0] = (unsigned short)1500; // ROLL
   pid_output_msg.data[1] = (unsigned short)1500; // PITCH
   pid_output_msg.data[3] = (unsigned short)1000; // THROTTLE
   pid_output_msg.data[2] = (unsigned short)1500;   // YAW
   pid_output_msg.data[4] = (unsigned short)500;
   pid_out_pub.publish(pid_output_msg);
}

void PIDCONTROLLER::maghold_drone(unsigned short maghold) {
   std::cout << drone << ":" << "SEND MAGHOLD" << (maghold - 600) << std::endl;
   pid_output_msg.data[0] = (unsigned short)1500; // ROLL
   pid_output_msg.data[1] = (unsigned short)1500; // PITCH
   pid_output_msg.data[3] = (unsigned short)1000; // THROTTLE
   pid_output_msg.data[2] = (unsigned short)maghold;   // YAW
   pid_output_msg.data[4] = (unsigned short)600;
   pid_out_pub.publish(pid_output_msg);
}

void PIDCONTROLLER::targetCallback(const geometry_msgs::Quaternion& msg) {
   float target_x = msg.x;
   float target_y = msg.y;
   float target_z = msg.z;

   float current_x = 0.0f;
   float current_y = 0.0f;
   float current_z = 0.0f;

   manage_current_pos(GET, &current_x, &current_y, &current_z);
   unsigned int mission = (unsigned int)msg.w;
   unsigned int tmp_mod = MODE_GROUND;

   if (mission == MISSION_TAKEOFF) {
      std::cout << "T";
      tmp_mod = MODE_TAKEOFF;
      if (current_z > GROUND_ALTITUDE + 150.0f ) {
         std::cout << drone << ":" << "NOT THE GROUND" << std::endl;
         return;
      }
      if ( ros::Time::now().toSec() - reboot_time < 7.0l) {
         std::cout << drone << ":" << "Wait for reboot" << std::endl;
         return;
      }
      if (manage_mode(SET, &tmp_mod) != MANAGE_MODE_ERROR ) {
         takeoff_altitude = current_z;
         if (target_z < 0)
            manage_target(SET_TARGET, &current_x, &current_y, &target_z);
         else if (target_z > 0) {
            current_z += target_z;
            manage_target(SET_TARGET, &current_x, &current_y, &target_z);
         }
         else {
            current_z += 750.0f;
            manage_target(SET_TARGET, &current_x, &current_y, &current_z);
         }
      }
   }
   else if (mission == MISSION_MANUAL) {
      tmp_mod = MODE_MANUAL;
      manual_time = ros::Time::now().toSec();
      if (manage_mode(SET, &tmp_mod) != MANAGE_MODE_ERROR )
         if (manage_target(SET_MANUAL, &target_x, &target_y, &target_z) == MANAGE_TARGET_ERROR) {
            tmp_mod = MODE_POSHOLD;
            manage_mode(SET, &tmp_mod);
         }
   }
   else if (mission == MISSION_AUTO) {
      std::cout << "T";
      tmp_mod = MODE_NAV;
      if (manage_mode(SET, &tmp_mod) != MANAGE_MODE_ERROR )
         if (manage_target(SET_TARGET, &target_x, &target_y, &target_z) == MANAGE_TARGET_ERROR) {
            tmp_mod = MODE_POSHOLD;
            manage_mode(SET, &tmp_mod);
         }
   }
   else if (mission == MISSION_AUTO_N) {
      std::cout << "T";
      tmp_mod = MODE_NAV_N;
      if (manage_mode(SET, &tmp_mod) != MANAGE_MODE_ERROR )
         if (manage_target(SET_TARGET, &target_x, &target_y, &target_z) == MANAGE_TARGET_ERROR) {
            tmp_mod = MODE_POSHOLD;
            manage_mode(SET, &tmp_mod);
         }
   }
   else if (mission == MISSION_LANDING) {
      std::cout << "T";
      tmp_mod = MODE_LANDING;
      if (manage_mode(SET, &tmp_mod) != MANAGE_MODE_ERROR ) {
         target_z = -4000.0f;
         manage_target(SET_TARGET, &current_x, &current_y, &target_z);
      }
   }
   else if (mission == MISSION_AUX ) {
      std::cout << "T";
      tmp_mod = MODE_NAV;
      if (manage_mode(SET, &tmp_mod) != MANAGE_MODE_ERROR ) {
         current_x += target_x;
         current_y += target_y;
         current_z += target_z;
         manage_target(SET_TARGET, &current_x, &current_y, &current_z);
      }
   }
   else if (mission == MISSION_GROUND) {
      std::cout << "T";
      tmp_mod = MODE_GROUND;
      manage_mode(SET, &tmp_mod);
      manage_target(SET_TARGET, &current_x, &current_y, &current_z);
   }
   else if (mission == MISSION_RESET) {
      reboot_drone();
      reboot_time = ros::Time::now().toSec();
   }
   else if (mission == MISSION_MAGHOLD) {
      maghold_drone((unsigned short)msg.y);
      // reboot_time = ros::Time::now().toSec();
   }
}





void PIDCONTROLLER::set_self_param() {
   pid_param_c.set_param(
      &pid_poshold_pos_param_X,
      &pid_poshold_rate_param_X,
      &pid_poshold_pos_param_Y,
      &pid_poshold_rate_param_Y,
      &pid_poshold_pos_param_Z,
      &pid_poshold_rate_param_Z,
      &pid_nav_pos_param_X,
      &pid_nav_rate_param_X,
      &pid_nav_pos_param_Y,
      &pid_nav_rate_param_Y,
      &pid_nav_pos_param_Z,
      &pid_nav_rate_param_Z
      , &flight_param
   );

   param_location = PARAM_INNER;

}
void PIDCONTROLLER::set_common_param() {
   pid_param_c.set_param(
      &__pid_poshold_pos_param_X,
      &__pid_poshold_rate_param_X,
      &__pid_poshold_pos_param_Y,
      &__pid_poshold_rate_param_Y,
      &__pid_poshold_pos_param_Z,
      &__pid_poshold_rate_param_Z,
      &__pid_nav_pos_param_X,
      &__pid_nav_rate_param_X,
      &__pid_nav_pos_param_Y,
      &__pid_nav_rate_param_Y,
      &__pid_nav_pos_param_Z,
      &__pid_nav_rate_param_Z
      , &__flight_param
   );
   param_location = PARAM_OUTER;
}


