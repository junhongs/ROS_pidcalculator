#include "ros/ros.h"

#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Inertia.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "pcl_msgs/Vertices.h"
#include "pcl_msgs/ModelCoefficients.h"

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt16MultiArray.h"

#include <iostream>

#include "param.h"
#include "calculation.h"

static const float TAKEOFF_SPEED = 200;
static const float LANDING_SPEED = -200;

// cfg.P8[PIDNAVR] = 14; // NAV_P * 10;
// cfg.I8[PIDNAVR] = 20; // NAV_I * 100;
// cfg.D8[PIDNAVR] = 80; // NAV_D * 1000;
/*
PARAM{

      "FIRST/RATE/X/PID_P"
   PID Position            X,Y,Z  * (P,I,D,IMAX)     (12)
   PID velocity            X,Y,Z  * (P,I,D,IMAX)     (12)

   Maximum velocity        X,Y,Z                     (3)
}
TOPIC{
   in::
         "FIRST/TARGET_POS/X/"
         "FIRST/CURRENT_POS/X/"
      Target Position      X,Y,Z                     (3)
      Current Position     X,Y,Z                     (3)

         (random access? or just get this by param_sever?)
         (but we don't have to get those parameter values at the same time)
         "FIRST/PARAM_POS/X/PID_P"
      PIDparam Pos         X,Y,Z  * (P,I,D,IMAX)     (12)
      PIDparam rate        X,Y,Z  * (P,I,D,IMAX)     (12)

   out::
         "FIRST/OUTPUT_PID/"
         pcl_msgs::ModelCoefficients
      Output_PID           X,Y,Z,Y,A                 (5)

         "FIRST/OUTPUT_INNER_PID/X"
         pcl_msgs::ModelCoefficients
      Output_inner_PPID         X,Y,Z * (res,target_vel,P,I,D)

}
SERVICE{
}
*/
static ros::Publisher float_pub;
static ros::Publisher velocity_pub;
static ros::Publisher pid_out_pub;
static ros::Publisher pid_inner_x_pub;
static ros::Publisher pid_inner_y_pub;
static ros::Publisher pid_inner_z_pub;



// target_pos_vel_t *target;
// pos_vel_t *current;

void timerCallback(const ros::TimerEvent&) {
   update_param(&pid_param);
}

void paramCallback(const std_msgs::Int32& msg) {
   update_param(msg.data);
   std::cout << "number param ::" << msg.data << std::endl << "data::" << *get_param_n(msg.data)<< std::endl;
}

void position_Callback(const geometry_msgs::Point& msg) {
   static pos_vel_t current_X = {0,};
   static pos_vel_t current_Y = {0,};
   static pos_vel_t current_Z = {0,};
   // static int flight_mode = GROUND;
   static int flight_mode = MISSION_POSHOLD;




   /*
    *       Check and save the time.
    *       Calculate the velocity
    *       Publish the velocity
    */
   current_X.cur_time = current_Y.cur_time = current_Z.cur_time = ros::Time::now().toSec();

   current_X.lpf.cur_time = current_Y.lpf.cur_time = current_Z.lpf.cur_time = ros::Time::now().toSec();

   current_X.cur_pos = msg.x;
   current_Y.cur_pos = msg.y;
   current_Z.cur_pos = msg.z;

   calc_velocity(&current_X);
   calc_velocity(&current_Y);
   calc_velocity(&current_Z);

   current_X.lpf.input = current_X.cur_vel;
   current_Y.lpf.input = current_Y.cur_vel;
   current_Z.lpf.input = current_Z.cur_vel;


   current_X.cur_vel = get_lpf(&(current_X.lpf), 5);
   current_Y.cur_vel = get_lpf(&(current_Y.lpf), 5);
   current_Z.cur_vel = get_lpf(&(current_Z.lpf), 5);



   geometry_msgs::Point velocity_msg;
   velocity_msg.x = current_X.cur_vel;
   velocity_msg.y = current_Y.cur_vel;
   velocity_msg.z = current_Z.cur_vel;
   velocity_pub.publish(velocity_msg);
   //


   /*
         1. restrict the target velocity by 200.   OK
         2. set the target_position.               OK
   */
   //JUST ADD MY TARGET VELOCITY. PLEASE CHANGE LATER
   float limited_target_vel = 200;
   //JUST ADD MY TARGET POSITION. PLEASE CHANGE LATER
   double target_pos_x = -500;
   double target_pos_y = 700;
   double target_pos_z = -1700;

   int is_arm = 1000;

   // DECLARE the pid output
   std_msgs::UInt16MultiArray pid_output_msg;
   pid_output_msg.data.resize(5, 1000);

   // DECLARE the inner pid message
   geometry_msgs::Inertia pid_inner_y_msg;
   geometry_msgs::Inertia pid_inner_z_msg;

   // DECLARE the X, Y, Z pid calculation variables.
   //X
   static pid_calc_t pid_pos_X = {0, };
   static pid_calc_t pid_rate_X = {0, };
   static target_pos_vel_t target_X = {0, };
   //Y
   static pid_calc_t pid_pos_Y = {0, };
   static pid_calc_t pid_rate_Y = {0, };
   static target_pos_vel_t target_Y = {0, };
   //Z
   static pid_calc_t pid_pos_Z = {0, };
   static pid_calc_t pid_rate_Z = {0, 0, 0, -500, 0, 0, 0, 0, 0, 0};
   //pid_rate_Z.integrator = -500;
   static target_pos_vel_t target_Z = {0, };

   //JUST ADD MY TARGET POSITION. PLEASE CHANGE LATER
   target_X.target_pos = target_pos_x;
   target_Y.target_pos = target_pos_y;
   target_Z.target_pos = target_pos_z;


   if (flight_mode == GROUND) {

      reset_PID(&pid_pos_X);
      reset_PID(&pid_rate_X);

      reset_PID(&pid_pos_Y);
      reset_PID(&pid_rate_Y);

      reset_PID(&pid_pos_Z);
      reset_PID(&pid_rate_Z);

      pid_rate_X.output = 0;
      pid_rate_Y.output = 0;
      pid_rate_Z.output = -500;
      is_arm = 1000;
   }
   else if (flight_mode == MISSION_NAV) {
      calc_navi_set_target(&target_X, &current_X, &target_Y, &current_Y, &target_Z, &current_Z , limited_target_vel);
// navi_rate(pid_calc_t *pid_rate, target_pos_vel_t *target, pos_vel_t *current, ros::Publisher *pid_inner_pub )
      navi_rate(&pid_pos_X, &pid_rate_X, &target_X, &current_X, limited_target_vel, &pid_inner_x_pub);
      navi_rate(&pid_pos_Y, &pid_rate_Y, &target_Y, &current_Y, limited_target_vel, &pid_inner_y_pub);
      navi_rate(&pid_pos_Z, &pid_rate_Z, &target_Z, &current_Z, limited_target_vel, &pid_inner_z_pub);
      is_arm = 1950;
   }
   else if (flight_mode == MISSION_POSHOLD) {
      //Calculate the pos_hold mod
      pos_hold(&pid_pos_X, &pid_rate_X, &target_X, &current_X, limited_target_vel, &pid_inner_x_pub);
      pos_hold(&pid_pos_Y, &pid_rate_Y, &target_Y, &current_Y, limited_target_vel, &pid_inner_y_pub);
      pos_hold(&pid_pos_Z, &pid_rate_Z, &target_Z, &current_Z, limited_target_vel, &pid_inner_z_pub);
      is_arm = 1950;
   }
   else if (flight_mode == MISSION_TAKEOFF) {
      calc_takeoff_altitude(&pid_rate_Z);
      pos_hold(&pid_pos_X, &pid_rate_X, &target_X, &current_X, limited_target_vel, &pid_inner_x_pub);
      pos_hold(&pid_pos_Y, &pid_rate_Y, &target_Y, &current_Y, limited_target_vel, &pid_inner_y_pub);
      target_Z.target_vel = TAKEOFF_SPEED;
      navi_rate(&pid_pos_Z, &pid_rate_Z, &target_Z, &current_Z, limited_target_vel, &pid_inner_z_pub);
      is_arm = 1950;
   }
   else if (flight_mode == MISSION_LANDING) {
      pos_hold(&pid_pos_X, &pid_rate_X, &target_X, &current_X, limited_target_vel, &pid_inner_x_pub);
      pos_hold(&pid_pos_Y, &pid_rate_Y, &target_Y, &current_Y, limited_target_vel, &pid_inner_y_pub);
      target_Z.target_vel = LANDING_SPEED;
      navi_rate(&pid_pos_Z, &pid_rate_Z, &target_Z, &current_Z, limited_target_vel, &pid_inner_z_pub);
      is_arm = 1950;
   }

   //Write the pid_output
   pid_output_msg.data[0] = 1500 - (unsigned short)constrain(pid_rate_X.output, -500.0, 500.0); // ROLL
   pid_output_msg.data[1] = 1500 - (unsigned short)constrain(pid_rate_Y.output, -500.0, 500.0); // PITCH
   pid_output_msg.data[3] = 1500 + (unsigned short)constrain(pid_rate_Z.output, -500.0, 500.0); // THROTTLE
   pid_output_msg.data[2] = 1500;   // YAW
   pid_output_msg.data[4] = is_arm;
   pid_out_pub.publish(pid_output_msg);
   

//target->target_pos - current->cur_pos
   std::cout << target_Z.target_pos << "::::" <<  current_Z.cur_pos << std::endl;;
   std::cout << "inner_I   " <<pid_rate_Z.inner_i << "  :::target_vel " << target_Z.target_vel << std::endl;
   // //
   int distance = calc_dist(target_pos_x, target_pos_y, target_pos_z, msg.x, msg.y, msg.z);
   // static int is_start = 0;

   // if (distance < 50.0)
   //    is_start = 1;

   // if (is_start == 1) {
   //    pid_output_msg.data[4] = 1950;
   // }
   // else {
   //    pid_output_msg.data[4] = 1000;
   //    pid_output_msg.data[3] = 1000;
   //    reset_PID(&pid_rate_Z);
   //    reset_PID(&pid_pos_Z);
   // }
   // std_msgs::Float32 float_msg;
   // float_msg.data = distance ;
   // //float_msg.data = msg_pos_vel.cur_vel * 100/30 ;
   // float_pub.publish(float_msg);


}


void positionCallback(const std_msgs::Float32& msg) {
   static pos_vel_t msg_pos_vel = {0,};
   msg_pos_vel.cur_time = ros::Time::now().toSec();
   msg_pos_vel.cur_pos = msg.data;
   calc_velocity(&msg_pos_vel);

   std_msgs::Float32 float_msg;
   float_msg.data = msg_pos_vel.cur_vel / 10 ;
   //float_msg.data = msg_pos_vel.cur_vel * 100/30 ;
   float_pub.publish(float_msg);


   geometry_msgs::Point velocity_msg;
   velocity_msg.x = 0;
   velocity_msg.y = 0;
   velocity_msg.z = msg_pos_vel.cur_vel;
   velocity_pub.publish(velocity_msg);




   static pid_calc_t pid_pos = {0, };
   static pid_calc_t pid_rate = {0, };
   static target_pos_vel_t target_pos_vel = {0, };

   pid_calc_t *tmp_pid_pos = &pid_pos;
   pid_calc_t *tmp_pid_rate = &pid_rate;
   target_pos_vel_t *tmp_target_pos_vel = &target_pos_vel;

   calc_pos_error(tmp_pid_pos, tmp_target_pos_vel , &msg_pos_vel);
   // pid_pos_p->output = get_P(pid_pos_p, &pid_pos_param_X);
   calc_pid(tmp_pid_pos, &pid_pos_param_X);
   tmp_target_pos_vel->target_vel = tmp_pid_pos->output;
   calc_rate_error(tmp_pid_rate, tmp_target_pos_vel , &msg_pos_vel);
   calc_pid(tmp_pid_rate, &pid_rate_param_X);


   // pid_rate_p->output = get_P(pid_rate_p, &pid_rate_param_X);
   // pid_rate_p->output += get_I(pid_rate_p, &pid_rate_param_X);
   // pid_rate_p->output += constrain( get_D(pid_rate_p, &pid_rate_param_X), -2000, 2000);
   //pid_rate_p is the final output.

   // typedef struct target_pos_vel_t {
   //     float target_pos;
   //     float target_vel;
   // } target_pos_vel_t;
   // calc_pos_error(pid_calc_t *pid, target_pos_vel_t *target, pos_vel_t *current) {

   // int tmp = msg_pos_vel.cur_vel * 10000;
   //std::cout   <<  msg_pos_vel.cycle_time <<  std::endl;
   // if(tmp > 200000 || tmp < -200000)
   // std::cout   << tmp   << "cycle time is :: " << msg_pos_vel.cycle_time <<  std::endl;

   //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
   ros::init(argc, argv, "listener");
   ros::NodeHandle n;
   init_param(&pid_param);


   // Creat Timer to update the parameter.
   // parameter server uses disk io, so it causes some delay.
   // ros::Timer timer = n.createTimer(ros::Duration(1), timerCallback);

   float_pub = n.advertise<std_msgs::Float32>("calculated_distance", 100);

   velocity_pub = n.advertise<geometry_msgs::Point>("/FIRST/CURRENT_VEL", 100);

   //pcl_msgs/ModelCoefficients
   pid_out_pub       = n.advertise<std_msgs::UInt16MultiArray>("/FIRST/OUTPUT_PID", 100);
   pid_inner_x_pub   = n.advertise<geometry_msgs::Inertia>("/FIRST/OUTPUT_INNER_PID/X", 100);
   pid_inner_y_pub   = n.advertise<geometry_msgs::Inertia>("/FIRST/OUTPUT_INNER_PID/Y", 100);
   pid_inner_z_pub   = n.advertise<geometry_msgs::Inertia>("/FIRST/OUTPUT_INNER_PID/Z", 100);


   // std_msgs/UInt16MultiArray

   /*
   "FIRST/OUTPUT_PID/"
   "FIRST/OUTPUT_INNER_PID/X"
   */
   //n.advertise<geometry_msgs::Point>("/FIRST/POSDATA", 100);

   pcl_msgs::ModelCoefficients v_msgs;
   v_msgs.values.clear();
   v_msgs.values.push_back(10);
   v_msgs.values.push_back(20);
   v_msgs.values.push_back(30);
   v_msgs.values.push_back(40);


   ros::Subscriber sub = n.subscribe("generate_sin_pulse", 100, positionCallback);

   ros::Subscriber param_sub = n.subscribe("/PARAM_CHANGE", 100, paramCallback);

   ros::Subscriber position_sub = n.subscribe("/FIRST/CURRENT_POS", 100, position_Callback);


   ros::spin();

   return 0;
}
