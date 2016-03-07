#ifndef _PIDCONTROLLER_H
#define _PIDCONTROLLER_H

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
#include "kalman.h"

#include <string>
#include "calculation.h"

#define GROUND_ALTITUDE -2650.0f
#define MANAGE_MODE_ERROR -1
#define MANAGE_TARGET_ERROR -1

static const float TAKEOFF_SPEED = 200.0f;
static const float LANDING_SPEED = -200.0f;

#define PARAM_OUTER 1
#define PARAM_INNER 2

class PIDCONTROLLER
{
public:
   PIDCONTROLLER(std::string DRONE);
   ~PIDCONTROLLER();
   pos_pid_parameter_t pid_param_c;
   int param_location;
   int seq, seq2;
   std::string drone;

   void set_self_param();
   void set_common_param();
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



   int drone_num;

   float max_vel;
   float limited_target_vel;

   double node_cur_time;
   double node_last_time;
   double reboot_time;
   double manual_time;
   double failsafe_time;
   double mode_change_time_to_smooth_target_velocity;

   unsigned short is_arm;
   unsigned int is_first_get_position;
   int is_takeoff;
   int changed_to_poshold_x;
   int changed_to_poshold_y;
   int changed_to_poshold_z;

   std_msgs::UInt16MultiArray pid_output_msg;

   unsigned int current_mode;
   unsigned int flight_mode_position_callback;

   int ground_altitude;

   int is_changed_manage_mode;
   int is_changed_manage_target;
   int is_changed_manage_current_pos;
   int is_landing_range;

   float offset_roll;
   float offset_pitch;
   float offset_throttle;

   float current_target_x;
   float current_target_y;
   float current_target_z;

   float last_target_x;
   float last_target_y;
   float last_target_z;

   float current_position_x;
   float current_position_y;
   float current_position_z;

   float takeoff_altitude;

   int tim1_timer;
   int tim2_timer;

   int mag_counter;

   lpf_c lpf_vel_x;
   lpf_c lpf_vel_y;
   lpf_c lpf_vel_z;

   lpf_c lpf_pos_x;
   lpf_c lpf_pos_y;
   lpf_c lpf_pos_z;

   PV_kalman kalman_x;
   PV_kalman kalman_y;
   PV_kalman kalman_z;

   lpf_c lpf_offset_x;
   lpf_c lpf_offset_y;
   lpf_c lpf_offset_z;

   lpf_c lpf_target_x;
   lpf_c lpf_target_y;
   lpf_c lpf_target_z;   

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

//POSHOLD PARAMETER
   pid_parameter_t pid_poshold_pos_param_X;
   pid_parameter_t pid_poshold_rate_param_X;

   pid_parameter_t pid_poshold_pos_param_Y;
   pid_parameter_t pid_poshold_rate_param_Y;

   pid_parameter_t pid_poshold_pos_param_Z;
   pid_parameter_t pid_poshold_rate_param_Z;

//NAVIGATION PARAMETER
   pid_parameter_t pid_nav_pos_param_X;
   pid_parameter_t pid_nav_rate_param_X;

   pid_parameter_t pid_nav_pos_param_Y;
   pid_parameter_t pid_nav_rate_param_Y;

   pid_parameter_t pid_nav_pos_param_Z;
   pid_parameter_t pid_nav_rate_param_Z;

   pid_parameter_t flight_param;



   static int making_drone();

   int manage_mode(unsigned int, unsigned int *);
   int manage_target(unsigned int, float *, float *, float * );
   int manage_current_pos(unsigned int, float *, float *, float * );

   void reboot_drone();
   void maghold_drone(unsigned short);
   void timerCallback(const ros::TimerEvent&);
   void position_Callback(const geometry_msgs::Point&);
   void targetCallback(const geometry_msgs::Quaternion&);
   void seq_Callback(const geometry_msgs::Point&);

};
#endif