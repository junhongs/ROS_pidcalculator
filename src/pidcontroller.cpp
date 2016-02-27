#include "ros/ros.h"
#include "pidcontroller.h"
#include "math.h"
#include <iostream>
using namespace std;


PIDCONTROLLER::~PIDCONTROLLER() {

}
PIDCONTROLLER::PIDCONTROLLER(std::string DRONE, float x_off, float y_off) :
   x_offset(x_off),
   y_offset(y_off),
   limited_target_vel(400.0f),
   max_vel(200.0f),

   is_changed_manage_mode(0),
   is_changed_manage_target(0),
   is_changed_manage_current_pos(0),

   target_pos_x(0.0f),
   target_pos_y(0.0f),
   target_pos_z(0.0f),

   current_target_x(0.0f),
   current_target_y(0.0f),
   current_target_z(0.0f),

   current_position_x(0.0f),
   current_position_y(500.0f),
   current_position_z(-3000.0f),

   tim1_timer(0.0f),
   tim2_timer(0.0f),

   current_mode(MODE_GROUND),

   flight_mode_position_callback(MODE_GROUND),

   node_cur_time(0.0l),
   node_last_time(0.0l),
   reboot_time(0.0l),
   ground_altitude(GROUND_ALTITUDE),

   pid_rate_Z(-450.0f)
{
   pid_output_msg.data.resize(5, 1000);
   drone_num = making_drone();
   drone = DRONE;

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
   if (getset == GET) {
      ret = is_changed_manage_mode;
      is_changed_manage_mode = 0;
      *state = current_mode;
   }
   else if (getset == SET || getset == SET_TIMER) {
      std::cout << drone << ":" << mode_str[current_mode] << ":" << "   TO   " << mode_str[*state];
      if ((*state == MODE_TAKEOFF && current_mode != MODE_GROUND) ) {
         std::cout << "     NO PERMISSION to TAKEOFF" << std::endl;
         return MANAGE_MODE_ERROR;
      }
      if (current_mode == MODE_GROUND && *state != MODE_TAKEOFF ) {
         std::cout << "     NO PERMISSION" << std::endl;
         return MANAGE_MODE_ERROR;
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

void PIDCONTROLLER::position_Callback(const geometry_msgs::Point& msg) {
   node_cur_time = ros::Time::now().toSec();

   unsigned short is_arm = 1000;
   current_X.cur_time = current_Y.cur_time = current_Z.cur_time = node_cur_time;
//   current_X.lpf.cur_time = current_Y.lpf.cur_time = current_Z.lpf.cur_time = node_cur_time;

   current_X.cur_pos = msg.x;
   current_Y.cur_pos = msg.y;
   current_Z.cur_pos = msg.z;

   calc_velocity(&current_X);
   calc_velocity(&current_Y);
   calc_velocity(&current_Z);

   /* velocity Low Pass Filter*/
   lpf_vel_x.set_cutoff_freq(1.5f);
   lpf_vel_y.set_cutoff_freq(1.5f);
   lpf_vel_z.set_cutoff_freq(1.5f);
   current_X.cur_vel = lpf_vel_x.get_lpf(current_X.cur_vel);
   current_Y.cur_vel = lpf_vel_y.get_lpf(current_Y.cur_vel);
   current_Z.cur_vel = lpf_vel_z.get_lpf(current_Z.cur_vel);

   /* Position Low Pass Filter*/
   lpf_pos_x.set_cutoff_freq(2.0f);
   lpf_pos_y.set_cutoff_freq(2.0f);
   lpf_pos_z.set_cutoff_freq(2.0f);
   current_X.cur_pos = lpf_pos_x.get_lpf(current_X.cur_pos);
   current_Y.cur_pos = lpf_pos_y.get_lpf(current_Y.cur_pos);
   current_Z.cur_pos = lpf_pos_z.get_lpf(current_Z.cur_pos);

   geometry_msgs::Point velocity_msg;
   velocity_msg.x = current_X.cur_vel;
   velocity_msg.y = current_Y.cur_vel;
   velocity_msg.z = current_Z.cur_vel;
   velocity_pub.publish(velocity_msg);

   manage_current_pos(SET, &(current_X.cur_pos), &(current_Y.cur_pos), &(current_Z.cur_pos));
   // if (node_cur_time - node_last_time > 3.0 ) {
   //    flight_mode_position_callback = MODE_GROUND;
   //    manage_mode(SET, &flight_mode_position_callback);
   //    static int is_first = 0;
   //    if (is_first) {
   //       std::cout << drone << ":" <<"Position Input has been delayed more than 3 seconds." << std::endl;
   //    }
   //    is_first = 1;
   // }
   node_last_time = node_cur_time;

   int is_changed_target = manage_target(GET, &target_pos_x, &target_pos_y, &target_pos_z);
   if (target_pos_x == 0.0f && target_pos_y == 0.0f && target_pos_z == 0.0f) {
      manage_target(SET, &current_X.cur_pos, &current_Y.cur_pos, &current_Z.cur_pos);
   }
   int is_changed_mode = manage_mode(GET, &flight_mode_position_callback);

   target_X.target_pos = target_pos_x;
   target_Y.target_pos = target_pos_y;
   target_Z.target_pos = target_pos_z;

   if (flight_mode_position_callback == MODE_NAV) {
      calc_navi_set_target(&target_X, &current_X, &target_Y, &current_Y, &target_Z, &current_Z , limited_target_vel);
      navi_rate(&pid_pos_Z, &pid_rate_Z, &target_Z, &current_Z, limited_target_vel, &pid_inner_z_pub, &pid_poshold_pos_param_Z, &pid_poshold_rate_param_Z, is_changed_target);
      if (pid_rate_Z.output < 0.0f) {
         reset_I(&pid_rate_X, 0.0f);
         reset_I(&pid_rate_Y, 0.0f);
      }
      navi_rate(&pid_pos_X, &pid_rate_X, &target_X, &current_X, limited_target_vel, &pid_inner_x_pub, &pid_poshold_pos_param_X, &pid_poshold_rate_param_X, is_changed_target);
      navi_rate(&pid_pos_Y, &pid_rate_Y, &target_Y, &current_Y, limited_target_vel, &pid_inner_y_pub, &pid_poshold_pos_param_Y, &pid_poshold_rate_param_Y, is_changed_target);
      is_arm = 1950;
   }

   else if (flight_mode_position_callback == MODE_MANUAL) {
      calc_takeoff_altitude(&pid_rate_Z);
      manual(&pid_pos_Z, &pid_rate_Z, &target_Z, &current_Z, limited_target_vel, &pid_inner_z_pub, &pid_poshold_pos_param_Z, &pid_poshold_rate_param_Z, max_vel);
      if (pid_rate_Z.output < 0.0f) {
         reset_I(&pid_rate_X, 0.0f);
         reset_I(&pid_rate_Y, 0.0f);
      }
      manual(&pid_pos_X, &pid_rate_X, &target_X, &current_X, limited_target_vel, &pid_inner_x_pub, &pid_poshold_pos_param_X, &pid_poshold_rate_param_X, max_vel);
      manual(&pid_pos_Y, &pid_rate_Y, &target_Y, &current_Y, limited_target_vel, &pid_inner_y_pub, &pid_poshold_pos_param_Y, &pid_poshold_rate_param_Y, max_vel);
      manage_target(SET, &current_X.cur_pos, &current_Y.cur_pos, &current_Z.cur_pos);
      is_arm = 1950;
   }

   else if (flight_mode_position_callback == MODE_POSHOLD) {
      //Calculate the pos_hold mod
      calc_takeoff_altitude(&pid_rate_Z);
      pos_hold(&pid_pos_Z, &pid_rate_Z, &target_Z, &current_Z, limited_target_vel, &pid_inner_z_pub, &pid_poshold_pos_param_Z, &pid_poshold_rate_param_Z);
      if (pid_rate_Z.output < 0.0f) {
         reset_I(&pid_rate_X, 0.0f);
         reset_I(&pid_rate_Y, 0.0f);
      }
      pos_hold(&pid_pos_X, &pid_rate_X, &target_X, &current_X, limited_target_vel, &pid_inner_x_pub, &pid_poshold_pos_param_X, &pid_poshold_rate_param_X);
      pos_hold(&pid_pos_Y, &pid_rate_Y, &target_Y, &current_Y, limited_target_vel, &pid_inner_y_pub, &pid_poshold_pos_param_Y, &pid_poshold_rate_param_Y);
      is_arm = 1950;
   }
   else if (flight_mode_position_callback == MODE_TAKEOFF) {
      calc_takeoff_altitude(&pid_rate_Z);
      calc_takeoff_altitude_once(&pid_rate_Z, is_changed_mode);
      target_Z.target_vel = TAKEOFF_SPEED;
      navi_rate(&pid_pos_Z, &pid_rate_Z, &target_Z, &current_Z, limited_target_vel, &pid_inner_z_pub, &pid_poshold_pos_param_Z, &pid_poshold_rate_param_Z, is_changed_target);
      if (pid_rate_Z.output < 0.0f) {
         reset_I(&pid_rate_X, 0.0f);
         reset_I(&pid_rate_Y, 0.0f);
      }
      pos_hold(&pid_pos_X, &pid_rate_X, &target_X, &current_X, limited_target_vel, &pid_inner_x_pub, &pid_poshold_pos_param_X, &pid_poshold_rate_param_X);
      pos_hold(&pid_pos_Y, &pid_rate_Y, &target_Y, &current_Y, limited_target_vel, &pid_inner_y_pub, &pid_poshold_pos_param_Y, &pid_poshold_rate_param_Y);
      is_arm = 1950;
   }
   else if (flight_mode_position_callback == MODE_LANDING) {
      target_Z.target_vel = LANDING_SPEED;
      navi_rate(&pid_pos_Z, &pid_rate_Z, &target_Z, &current_Z, limited_target_vel, &pid_inner_z_pub, &pid_poshold_pos_param_Z, &pid_poshold_rate_param_Z, is_changed_target);
      if (pid_rate_Z.output < 0.0f) {
         reset_I(&pid_rate_X, 0.0f);
         reset_I(&pid_rate_Y, 0.0f);
      }
      pos_hold(&pid_pos_X, &pid_rate_X, &target_X, &current_X, limited_target_vel, &pid_inner_x_pub, &pid_poshold_pos_param_X, &pid_poshold_rate_param_X);
      pos_hold(&pid_pos_Y, &pid_rate_Y, &target_Y, &current_Y, limited_target_vel, &pid_inner_y_pub, &pid_poshold_pos_param_Y, &pid_poshold_rate_param_Y);

      is_arm = 1950;
      if (current_Z.cur_pos < ground_altitude + 60.0f) {
         flight_mode_position_callback = MODE_GROUND;
         manage_mode(SET, &flight_mode_position_callback);
      }
   }
   if (flight_mode_position_callback == MODE_GROUND) {
      reset_PID(&pid_pos_X, 0.0f);
      reset_PID(&pid_rate_X, 0.0f);
      reset_PID(&pid_pos_Y, 0.0f);
      reset_PID(&pid_rate_Y, 0.0f);
      reset_PID(&pid_pos_Z, 0.0f);
      reset_PID(&pid_rate_Z, -500.0f);
      pid_rate_X.output = 0;
      pid_rate_Y.output = 0;
      pid_rate_Z.output = -500;
      is_arm = 1050;
   }

   //Write the pid_output
   pid_output_msg.data[0] = (unsigned short)(1500.0f - x_offset - (unsigned short)constrain(pid_rate_X.output, -500.0, 500.0)); // ROLL
   pid_output_msg.data[1] = (unsigned short)(1500.0f - y_offset - (unsigned short)constrain(pid_rate_Y.output, -500.0, 500.0)); // PITCH
   pid_output_msg.data[3] = (unsigned short)(1500.0f + (unsigned short)constrain(pid_rate_Z.output, -500.0, 500.0)); // THROTTLE
   pid_output_msg.data[2] = (unsigned short)1500;   // YAW
   pid_output_msg.data[4] = is_arm;
   pid_out_pub.publish(pid_output_msg);
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

void PIDCONTROLLER::targetCallback(const geometry_msgs::Quaternion& msg) {
   // std::cout << drone << ":" << "::TARGET MESSAGE::" << std::endl;;
   float target_x = msg.x;
   float target_y = msg.y;
   float target_z = msg.z;

   float current_x = 0.0f;
   float current_y = 0.0f;
   float current_z = 0.0f;

   manage_current_pos(GET, &current_x, &current_y, &current_z);
   unsigned int mission = (unsigned int)msg.w;
   unsigned int tmp_mod = MODE_GROUND;
   // if (mission <= MISSION_AUX )
   //    std::cout << drone << ":             " << "MISSION MESSAGE IS :: " << mission_str[mission] << std::endl;

   if (mission == MISSION_TAKEOFF) {
      tmp_mod = MODE_TAKEOFF;

      if (current_z > GROUND_ALTITUDE + 100.0f ) {
         std::cout << drone << ":" << "NOT THE GROUND" << std::endl;
         return;
      }
      if( ros::Time::now().toSec() - reboot_time < 5.0l){
         std::cout << drone << ":" << "Wait for reboot" << std::endl;
         return;
      }
      if (manage_mode(SET, &tmp_mod) != MANAGE_MODE_ERROR )
         if (target_z)
            manage_target(SET_TARGET, &current_x, &current_y, &target_z);
         else {
            current_z += 500.0f;
            manage_target(SET_TARGET, &current_x, &current_y, &current_z);
         }
   }
   else if (mission == MISSION_AUTO) {
      tmp_mod = MODE_NAV;
      if (manage_mode(SET, &tmp_mod) != MANAGE_MODE_ERROR )
         if (manage_target(SET_TARGET, &target_x, &target_y, &target_z) == MANAGE_TARGET_ERROR) {
            tmp_mod = MODE_POSHOLD;
            manage_mode(SET, &tmp_mod);
            //manage_target(SET_TARGET, &current_x, &current_y, &current_z);
         }
   }
   else if (mission == MISSION_MANUAL) {
      tmp_mod = MODE_MANUAL;
      if (manage_mode(SET, &tmp_mod) != MANAGE_MODE_ERROR )
         manage_target(SET_TARGET, &target_x, &target_y, &target_z);
   }
   else if (mission == MISSION_LANDING) {
      tmp_mod = MODE_LANDING;
      if (manage_mode(SET, &tmp_mod) != MANAGE_MODE_ERROR ) {
         target_z = -3000.0f;
         manage_target(SET_TARGET, &current_x, &current_y, &target_z);
      }
   }
   else if (mission == MISSION_AUX ) {
      tmp_mod = MODE_NAV;
      if (manage_mode(SET, &tmp_mod) != MANAGE_MODE_ERROR ) {
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
   else if (mission == MISSION_RESET) {
      reboot_drone();
      reboot_time = ros::Time::now().toSec();
   }
}