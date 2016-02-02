#include "ros/ros.h"

#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point.h"
#include <std_msgs/Float32.h>
#include "pcl_msgs/Vertices.h"

#include <iostream>

#include "param.h"
#include "calculation.h"

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
         "FIRST/OUTPUT_PID/X/"
      Output_Pid           X,Y,Z,Y                   (3)

      Output_
}
SERVICE{
}
*/
static ros::Publisher float_pub;


static ros::Publisher velocity_pub;

// target_pos_vel_t *target;
// pos_vel_t *current;
typedef struct lpf_t {
   double input;
   double last_input;
   double cycle_time;
} lpf_t;

static int get_lpf(lpf_t *lpf) {
#define LPF_FILTER       (1.0f / (2.0f * M_PI * (float)50))
   lpf->input = lpf->last_input + (lpf->cycle_time / (LPF_FILTER + lpf->cycle_time)) * (lpf->input - lpf->last_input);

   lpf->last_input = lpf->input;
   return lpf->input;
}

void timerCallback(const ros::TimerEvent&) {
   update_param(&pid_param);
}

void paramCallback(const pcl_msgs::Vertices& msg) {
   msg.vertices[0];
}
void position_Callback(const geometry_msgs::Point& msg) {
   static pos_vel_t msg_pos_vel_X = {0,};
   msg_pos_vel_X.cur_time = ros::Time::now().toSec();
   msg_pos_vel_X.cur_pos = msg.x;
   calc_velocity(&msg_pos_vel_X);

   static pos_vel_t msg_pos_vel_Y = {0,};
   msg_pos_vel_Y.cur_time = msg_pos_vel_X.cur_time;
   msg_pos_vel_Y.cur_pos = msg.y;
   calc_velocity(&msg_pos_vel_Y);

   static pos_vel_t msg_pos_vel_Z = {0,};
   msg_pos_vel_Z.cur_time = msg_pos_vel_X.cur_time;
   msg_pos_vel_Z.cur_pos = msg.z;
   calc_velocity(&msg_pos_vel_Z);





   geometry_msgs::Point velocity_msg;
   velocity_msg.x = msg_pos_vel_X.cur_vel;
   velocity_msg.y = msg_pos_vel_Y.cur_vel;
   velocity_msg.z = msg_pos_vel_Z.cur_vel;
   velocity_pub.publish(velocity_msg);



   static pid_calc_t pid_pos_X = {0, };
   static pid_calc_t pid_rate_X = {0, };
   static target_pos_vel_t target_pos_vel_X = {0, };

   pid_calc_t *pid_pos_p = &pid_pos_X;
   pid_calc_t *pid_rate_p = &pid_rate_X;
   target_pos_vel_t *target_pos_vel_p = &target_pos_vel_X;

   calc_pos_error(pid_pos_p, target_pos_vel_p , &msg_pos_vel_X);
   // pid_pos_p->output = get_P(pid_pos_p, &pid_pos_param_X);
   calc_pid(pid_pos_p, &pid_pos_param_X);
   target_pos_vel_p->target_vel = pid_pos_p->output;
   calc_rate_error(pid_rate_p, target_pos_vel_p , &msg_pos_vel_X);
   calc_pid(pid_rate_p, &pid_rate_param_X);



   static pid_calc_t pid_pos_Y = {0, };
   static pid_calc_t pid_rate_Y = {0, };
   static target_pos_vel_t target_pos_vel_Y = {0, };

   pid_pos_p = &pid_pos_Y;
   pid_rate_p = &pid_rate_Y;
   target_pos_vel_p = &target_pos_vel_Y;

   calc_pos_error(pid_pos_p, target_pos_vel_p , &msg_pos_vel_Y);
   // pid_pos_p->output = get_P(pid_pos_p, &pid_pos_param_X);
   calc_pid(pid_pos_p, &pid_pos_param_X);
   target_pos_vel_p->target_vel = pid_pos_p->output;
   calc_rate_error(pid_rate_p, target_pos_vel_p , &msg_pos_vel_Y);
   calc_pid(pid_rate_p, &pid_rate_param_X);




   static pid_calc_t pid_pos_Z = {0, };
   static pid_calc_t pid_rate_Z = {0, };
   static target_pos_vel_t target_pos_vel_Z = {0, };

   pid_pos_p = &pid_pos_Z;
   pid_rate_p = &pid_rate_Z;
   target_pos_vel_p = &target_pos_vel_Z;

   calc_pos_error(pid_pos_p, target_pos_vel_p , &msg_pos_vel_Z);
   // pid_pos_p->output = get_P(pid_pos_p, &pid_pos_param_X);
   calc_pid(pid_pos_p, &pid_pos_param_Z);
   target_pos_vel_p->target_vel = pid_pos_p->output;
   calc_rate_error(pid_rate_p, target_pos_vel_p , &msg_pos_vel_Z);
   calc_pid(pid_rate_p, &pid_rate_param_Z);
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

   pid_calc_t *pid_pos_p = &pid_pos;
   pid_calc_t *pid_rate_p = &pid_rate;
   target_pos_vel_t *target_pos_vel_p = &target_pos_vel;

   calc_pos_error(pid_pos_p, target_pos_vel_p , &msg_pos_vel);
   // pid_pos_p->output = get_P(pid_pos_p, &pid_pos_param_X);
   calc_pid(pid_pos_p, &pid_pos_param_X);
   target_pos_vel_p->target_vel = pid_pos_p->output;
   calc_rate_error(pid_rate_p, target_pos_vel_p , &msg_pos_vel);
   calc_pid(pid_rate_p, &pid_rate_param_X);


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

   float_pub = n.advertise<std_msgs::Float32>("calculated_pid", 100);

   velocity_pub = n.advertise<geometry_msgs::Point>("calculated_velocity", 100);

   //n.advertise<geometry_msgs::Point>("/FIRST/POSDATA", 100);


   ros::Subscriber sub = n.subscribe("generate_sin_pulse", 100, positionCallback);

   ros::Subscriber param_sub = n.subscribe("param_talker", 100, paramCallback);

   ros::Subscriber position_sub = n.subscribe("/FIRST/CURRENT_POS", 100, position_Callback);


   ros::spin();

   return 0;
}
