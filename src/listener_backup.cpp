#include "ros/ros.h"

#include "std_msgs/String.h"
#include <std_msgs/Float32.h>
#include "pcl_msgs/Vertices.h"

#include <iostream>

#include "param.h"
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
      Target Position      X,Y,Z                     (3)
      Current Position     X,Y,Z                     (3)

         (random access? or just get this by param_sever?)
         (but we don't have to get those parameter values at the same time)
         "FIRST/PARAM_POS/X/PID_P"
      PIDparam Pos         X,Y,Z  * (P,I,D,IMAX)     (12)
      PIDparam rate        X,Y,Z  * (P,I,D,IMAX)     (12)

   out::
         "FIRST/OUTPUT_PID/X/"
      Output_Pid           X,Y,Z                     (3)
}
SERVICE{
}
*/


typedef struct {
   float error;
   double cycle_time;
   float derivative;

   float integrator;          // integrator value
   float last_error;
   float last_derivative;     // last derivative for low-pass filter
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

   float last_vel_raw;
   float cur_vel_raw;
} pos_vel_t;

typedef struct target_pos_vel_t {
   float target_pos;
   float target_vel;
} target_pos_vel_t;




static pid_parameter_t pid_pos_param = {0, };
static pid_parameter_t pid_rate_param = {0, };
static pos_pid_parameter_t pid_param = {
   &pid_pos_param,
   &pid_rate_param
};

static ros::Publisher float_pub;





void calc_velocity( pos_vel_t* pos_vel);
void calc_pos_error(pid_calc_t *pid, target_pos_vel_t *target, pos_vel_t *current);
void calc_rate_error(pid_calc_t *pid, target_pos_vel_t *target, pos_vel_t *current);
void calc_pid(pid_calc_t* pid, pid_parameter_t* pid_param);

int constrain(int amt, int low, int high) {
   if (amt < low)
      return low;
   else if (amt > high)
      return high;
   else
      return amt;
}

// pid_calc_t -> error
static int get_P(pid_calc_t *pid, pid_parameter_t *pid_param) {
   return pid->error * pid_param->pid_P;
}

// pid_calc_t -> error
// pid_calc_t -> cycle_time
static int get_I(pid_calc_t *pid, pid_parameter_t *pid_param) {
   pid->integrator += (pid->error * pid_param->pid_I) * pid->cycle_time;
   pid->integrator = constrain(pid->integrator, -pid_param->pid_Imax, pid_param->pid_Imax);
   return pid->integrator;
}


// pid_calc_t -> error
// pid_calc_t -> cycle_time
// pid_calc_t -> derivative = pos_vel_t -> cur_vel_raw
static int get_D(pid_calc_t *pid, pid_parameter_t *pid_param) {
   pid->derivative = (pid->error - pid->last_error) / pid->cycle_time;

   // Low pass filter cut frequency for derivative calculation
   // Set to  "1 / ( 2 * PI * gps_lpf )"
#define PID_FILTER       (1.0f / (2.0f * M_PI * (float)50))
   // discrete low pass filter, cuts out the
   // high frequency noise that can drive the controller crazy
   pid->derivative = pid->last_derivative + (pid->cycle_time / (PID_FILTER + pid->cycle_time)) * (pid->derivative - pid->last_derivative);
   // update state

   pid->last_error = pid->error;

   pid->last_derivative = pid->derivative;
   // add in derivative component
   return pid_param->pid_D * pid->derivative;
}

static void reset_PID(pid_calc_t *pid) {
   pid->integrator = 0;
   pid->last_derivative = 0;
}
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












void calc_pos_error(pid_calc_t *pid, target_pos_vel_t *target, pos_vel_t *current) {
// if (target->target_pos && current->cur_pos )
   pid->error = target->target_pos - current->cur_pos;
   if (current->cycle_time)
      pid->cycle_time = current->cycle_time;
}

void calc_rate_error(pid_calc_t *pid, target_pos_vel_t *target, pos_vel_t *current) {
// if ( target->target_vel && current->cur_vel )
   pid->error = target->target_vel - current->cur_vel;
// if ( current->cur_vel_raw )
   pid->derivative = current->cur_vel_raw;
   if (current->cycle_time)
      pid->cycle_time = current->cycle_time;
}
// pid_calc_t -> error
// pid_calc_t -> cycle_time
// pid_calc_t -> derivative = pos_vel_t -> cur_vel_raw

void calc_pid(pid_calc_t* pid, pid_parameter_t* pid_param) {
   pid->output = get_P(pid, pid_param);
   pid->output += get_I(pid, pid_param);
   pid->output += constrain( get_D(pid, pid_param), -2000, 2000);
}

void calc_velocity( pos_vel_t* pos_vel) {
   const int is_lpf = 1;
   if (pos_vel->last_time && pos_vel->cur_time) {
      pos_vel->cycle_time = pos_vel->cur_time - pos_vel->last_time;

      //if ros's cycle period is fast (in my case, about 1000hz), sometime the cycle period might have some noize over 30%. it must be corrected.
      // if( pos_vel->cycle_time > 0.0013 || pos_vel->cycle_time < 0.0007){
      //         pos_vel->last_vel = pos_vel->cur_vel;
      //         pos_vel->last_time = pos_vel->cur_time;
      //         pos_vel->last_pos = pos_vel->cur_pos;
      //     return;
      // }
   }
   // if (pos_vel->cur_pos && pos_vel->last_pos)
   pos_vel->cur_vel = pos_vel->cur_pos - pos_vel->last_pos;

   if (pos_vel->cycle_time)
      pos_vel->cur_vel /= pos_vel->cycle_time;

   pos_vel->cur_vel_raw = pos_vel->cur_vel;

   // if (pos_vel->last_vel && is_lpf)
   if ( is_lpf )
      pos_vel->cur_vel = ( pos_vel->cur_vel + pos_vel->last_vel) / 2;

   pos_vel->last_vel = pos_vel->cur_vel;
   pos_vel->last_time = pos_vel->cur_time;
   pos_vel->last_pos = pos_vel->cur_pos;
   pos_vel->last_vel_raw = pos_vel->cur_vel_raw;

}


void paramCallback(const pcl_msgs::Vertices& msg) {
   msg.vertices[0];
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

   static pid_calc_t pid_pos = {0, };
   static pid_calc_t pid_rate = {0, };
   static target_pos_vel_t target_pos_vel = {0, };

   pid_calc_t *pid_pos_p = &pid_pos;
   pid_calc_t *pid_rate_p = &pid_rate;
   target_pos_vel_t *target_pos_vel_p = &target_pos_vel;

   calc_pos_error(pid_pos_p, target_pos_vel_p , &msg_pos_vel);
   // pid_pos_p->output = get_P(pid_pos_p, &pid_pos_param);
   calc_pid(pid_pos_p, &pid_pos_param);
   target_pos_vel_p->target_vel = pid_pos_p->output;
   calc_rate_error(pid_rate_p, target_pos_vel_p , &msg_pos_vel);
   calc_pid(pid_rate_p, &pid_rate_param);
   // pid_rate_p->output = get_P(pid_rate_p, &pid_rate_param);
   // pid_rate_p->output += get_I(pid_rate_p, &pid_rate_param);
   // pid_rate_p->output += constrain( get_D(pid_rate_p, &pid_rate_param), -2000, 2000);
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
void timerCallback(const ros::TimerEvent&) {
   update_param(&pid_param);
}

void update_param(  pos_pid_parameter_t *pid_param) {
   pid_parameter_t *pid_param_tmp;
   pid_param_tmp = pid_param->pos_pid;
   ros::param::param("FIRST/RATE/X/PID_P", pid_param_tmp->pid_P);
   ros::param::param("FIRST/RATE/X/PID_I", pid_param_tmp->pid_I);
   ros::param::param("FIRST/RATE/X/PID_D", pid_param_tmp->pid_D);
   ros::param::param("FIRST/RATE/X/PID_IMAX", pid_param_tmp->pid_Imax);

   pid_param_tmp = pid_param->rate_pid;
   ros::param::param("FIRST/POS/X/PID_P", pid_param_tmp->pid_P);
   ros::param::param("FIRST/POS/X/PID_I", pid_param_tmp->pid_I);
   ros::param::param("FIRST/POS/X/PID_D", pid_param_tmp->pid_D);
   ros::param::param("FIRST/POS/X/PID_IMAX", pid_param_tmp->pid_Imax);
}

void init_param(  pos_pid_parameter_t *pid_param) {
   pid_parameter_t default_param_pos = {
      0.11,
      0,
      0,
      2000
   };

   pid_parameter_t default_param_rate = {
      2,
      0.08,
      0.045,
      2000
   };

   pid_parameter_t *pid_param_tmp;
   pid_parameter_t *default_param_tmp;
   /*
      if params are not exist, make and set the parameters.
      if exist, get the parameters.
   */
   pid_param_tmp = pid_param->pos_pid;
   default_param_tmp = &default_param_pos;
   if ( !ros::param::param("FIRST/RATE/X/PID_P", pid_param_tmp->pid_P)  )
      ros::param::set("FIRST/RATE/X/PID_P", pid_param_tmp->pid_P = default_param_tmp->pid_P);
   if ( !ros::param::param("FIRST/RATE/X/PID_I", pid_param_tmp->pid_I) )
      ros::param::set("FIRST/RATE/X/PID_I", pid_param_tmp->pid_I = default_param_tmp->pid_I);
   if ( !ros::param::param("FIRST/RATE/X/PID_D", pid_param_tmp->pid_D) )
      ros::param::set("FIRST/RATE/X/PID_D", pid_param_tmp->pid_D = default_param_tmp->pid_D);
   if ( !ros::param::param("FIRST/RATE/X/PID_IMAX", pid_param_tmp->pid_Imax) )
      ros::param::set("FIRST/RATE/X/PID_IMAX", pid_param_tmp->pid_Imax = default_param_tmp->pid_Imax);

   pid_param_tmp = pid_param->rate_pid;
   default_param_tmp = &default_param_rate;
   if ( !ros::param::param("FIRST/POS/X/PID_P", pid_param_tmp->pid_P)  )
      ros::param::set("FIRST/POS/X/PID_P", pid_param_tmp->pid_P = default_param_tmp->pid_P);
   if ( !ros::param::param("FIRST/POS/X/PID_I", pid_param_tmp->pid_I) )
      ros::param::set("FIRST/POS/X/PID_I", pid_param_tmp->pid_I = default_param_tmp->pid_I);
   if ( !ros::param::param("FIRST/POS/X/PID_D", pid_param_tmp->pid_D) )
      ros::param::set("FIRST/POS/X/PID_D", pid_param_tmp->pid_D = default_param_tmp->pid_D);
   if ( !ros::param::param("FIRST/POS/X/PID_IMAX", pid_param_tmp->pid_Imax) )
      ros::param::set("FIRST/POS/X/PID_IMAX", pid_param_tmp->pid_Imax = default_param_tmp->pid_Imax);
}

// pos_P = 11 / 100
// pos_I = 0
// pos_D = 0
// pos_rate_P = 20 / 10;
// pos_rate_I = 8 / 100;
// pos_rate_d = 45 / 1000;

void delete_param( ) {
   ros::param::del("FIRST/POS/X/PID_P");
   ros::param::del("FIRST/POS/X/PID_I");
   ros::param::del("FIRST/POS/X/PID_D");
   ros::param::del("FIRST/POS/X/PID_IMAX");
   ros::param::del("FIRST/RATE/X/PID_P");
   ros::param::del("FIRST/RATE/X/PID_I");
   ros::param::del("FIRST/RATE/X/PID_D");
   ros::param::del("FFIRST/RATE/X/PID_IMAX");
}









int main(int argc, char **argv) {
   ros::init(argc, argv, "listener");
   ros::NodeHandle n;
   init_param(&pid_param);

   // Creat Timer to update the parameter.
   // parameter server uses disk io, so it causes some delay.
   // ros::Timer timer = n.createTimer(ros::Duration(1), timerCallback);

   float_pub = n.advertise<std_msgs::Float32>("calculated_pid", 100);
   ros::Subscriber sub = n.subscribe("generate_sin_pulse", 100, positionCallback);


   ros::Subscriber param_sub = n.subscribe("param_talker", 100, paramCallback);



   ros::spin();

   return 0;
}
