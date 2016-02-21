#include "ros/ros.h"

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

#include <string>


#include <iostream>

#include "param.h"
#include "calculation.h"


#define GROUND_ALTITUDE -2700

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
// static ros::Publisher velocity_pub;
// static ros::Publisher pid_out_pub;
// static ros::Publisher pid_inner_x_pub;
// static ros::Publisher pid_inner_y_pub;
// static ros::Publisher pid_inner_z_pub;
int making_drone() {
   static int n = 0;
   return n++;
}

std::string DRONE[4] = {
   "/FIRST",
   "/SECOND",
   "/THIRD",
   "/FOURTH"
};

class PIDCALCULATION
{
public:
   PIDCALCULATION() :
         x_offset(30),y_offset(30),limited_target_vel(300),max_vel(200)
    {
      std::string drone;
      drone_num = making_drone();
      drone = DRONE[drone_num];
      std::string current_vel = drone + "/CURRENT_VEL";
      std::string output_pid = drone +  "/OUTPUT_PID";
      std::string output_inner_pid_x = drone + "/OUTPUT_INNER_PID/X";
      std::string output_inner_pid_y = drone + "/OUTPUT_INNER_PID/Y";
      std::string output_inner_pid_z = drone + "/OUTPUT_INNER_PID/Z";
      std::string current_pos = drone + "/CURRENT_POS";
      std::string target_pos = drone + "/TARGET_POS";

      velocity_pub = nod.advertise<geometry_msgs::Point>("/FIRST/CURRENT_VEL", 100);
      // timer = n.createTimer(ros::Duration(1), timerCallback);
      pid_out_pub     = nod.advertise<std_msgs::UInt16MultiArray>("/FIRST/OUTPUT_PID", 100);
      pid_inner_x_pub = nod.advertise<geometry_msgs::Inertia>("/FIRST/OUTPUT_INNER_PID/X", 100);
      pid_inner_y_pub = nod.advertise<geometry_msgs::Inertia>("/FIRST/OUTPUT_INNER_PID/Y", 100);
      pid_inner_z_pub = nod.advertise<geometry_msgs::Inertia>("/FIRST/OUTPUT_INNER_PID/Z", 100);
      position_sub = nod.subscribe("/FIRST/CURRENT_POS", 100, &PIDCALCULATION::position_Callback, this);
      target_sub = nod.subscribe("/FIRST/TARGET_POS", 100, &PIDCALCULATION::targetCallback, this);

   }
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
   int x_offset;
   int y_offset;
   int drone_num;
   float limited_target_vel;
   float max_vel;
   double node_cur_time;
   int manage_mode(unsigned int getset, unsigned int *state) {
      static unsigned int current_state = GROUND;
      static int is_changed = 0;
      int ret = 0;
      if (getset == GET) {
         ret = is_changed;
         is_changed = 0;
         *state = current_state;
      }
      else if (getset == SET) {
         if ( current_state != *state)
            is_changed = 1;
#define PR_STATE(N) std::cout << "STATE :: " << #N;
         if ( current_state == MODE_TAKEOFF) PR_STATE(MODE_TAKEOFF);
         if ( current_state == MODE_NAV) PR_STATE(MODE_NAV);
         if ( current_state == MODE_MANUAL) PR_STATE(MODE_MANUAL);
         if ( current_state == MODE_LANDING) PR_STATE(MODE_LANDING);
         if ( current_state == GROUND) PR_STATE(GROUND);
         if ( current_state == MODE_POSHOLD) PR_STATE(MODE_POSHOLD);


         if ( (*state == MODE_TAKEOFF && current_state != GROUND) ) {
            std::cout << "NO PERMISSION to TAKEOFF" << std::endl;
            return -1;
         }


         
         current_state = *state;
#define PR_STATE2(N) std::cout << "   TO    " << #N << std::endl;
         if ( current_state == MODE_TAKEOFF) PR_STATE2(MODE_TAKEOFF);
         if ( current_state == MODE_NAV) PR_STATE2(MODE_NAV);
         if ( current_state == MODE_MANUAL) PR_STATE2(MODE_MANUAL);
         if ( current_state == MODE_LANDING) PR_STATE2(MODE_LANDING);
         if ( current_state == GROUND) PR_STATE2(GROUND);
         if ( current_state == MODE_POSHOLD) PR_STATE2(MODE_POSHOLD);
      }
      return ret;
   }

   int manage_target(unsigned int getset, float *x, float *y, float *z ) {
      static float current_target_x = 0;
      static float current_target_y = 0;
      static float current_target_z = 0;
      static int is_changed = 0;
      int ret = 0;
      if (getset == GET) {
         // std::cout << "GET the TARGET" <<current_target_x<<","<<current_target_x<<","<<current_target_x << std::endl;
         ret = is_changed;
         *x = current_target_x;
         *y = current_target_y;
         *z = current_target_z;
         is_changed = 0;
      }
      else if (getset == SET || getset == SET_TARGET ) {
         if (*y == 0 || *z == 0) {
            std::cout << "SET the CURRENT TARGET" << std::endl;
            return -1;
         }
         //do i consider the mode???
         if (getset == SET_TARGET)
            is_changed = 1;
         current_target_x = *x;
         current_target_y = *y;
         current_target_z = *z;
         std::cout << "SET the TARGET" << current_target_x << "," << current_target_y << "," << current_target_z << std::endl;
      }
      return ret;
   }
   int manage_current_pos(unsigned int getset, float *x, float *y, float *z ) {
      static float current_position_x = 0;
      static float current_position_y = 0;
      static float current_position_z = 0;
      static int is_changed = 0;

      int ret = 0;

      if (getset == GET) {
         ret = is_changed;
         *x = current_position_x;
         *y = current_position_y;
         *z = current_position_z;
         is_changed = 0;
      }
      else if (getset == SET) {
         //do i consider the mode???
         is_changed = 1;
         current_position_x = *x;
         current_position_y = *y;
         current_position_z = *z;
      }
      return ret;
   }

// target_pos_vel_t *target;
// pos_vel_t *current;

   void timerCallback(const ros::TimerEvent&) {
      // update_param();
   }
   void position_Callback(const geometry_msgs::Point& msg) {
      static pos_vel_t current_X = {0,};
      static pos_vel_t current_Y = {0,};
      static pos_vel_t current_Z = {0,};
      static unsigned int flight_mode = GROUND;

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

      static double node_last_time = 0;

      static float target_pos_x = 0;
      static float target_pos_y = 0;
      static float target_pos_z = 0;


      /*
       *       Check and save the time.
       *       Calculate the velocity
       *       Publish the velocity
       */
      node_cur_time = ros::Time::now().toSec();
      int is_arm = 1000;
      // DECLARE the pid output
      std_msgs::UInt16MultiArray pid_output_msg;
      pid_output_msg.data.resize(5, 1000);
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

      manage_current_pos(SET, &(current_X.cur_pos), &(current_Y.cur_pos), &(current_Z.cur_pos));
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

      geometry_msgs::Point velocity_msg;
      velocity_msg.x = current_X.cur_vel;
      velocity_msg.y = current_Y.cur_vel;
      velocity_msg.z = current_Z.cur_vel;
      velocity_pub.publish(velocity_msg);

      if ( node_cur_time - node_last_time > 3.0 ) {
         flight_mode = MODE_LANDING;
         manage_mode(SET, &flight_mode);
         std::cout << "Position Input has been delayed more than 3 seconds.";
      }
      node_last_time = node_cur_time;

      int is_changed_target = manage_target(GET, &target_pos_x, &target_pos_y, &target_pos_z);
      if ( target_pos_x == 0 && target_pos_y == 0 && target_pos_z == 0) {
         manage_target(SET, &current_X.cur_pos, &current_Y.cur_pos, &current_Z.cur_pos);
      }
      int is_changed_mode = manage_mode(GET, &flight_mode);

      target_X.target_pos = target_pos_x;
      target_Y.target_pos = target_pos_y;
      target_Z.target_pos = target_pos_z;

      if (flight_mode == MODE_NAV) {
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

      else if (flight_mode == MODE_MANUAL) {
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

      else if (flight_mode == MODE_POSHOLD) {
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
      else if (flight_mode == MODE_TAKEOFF) {
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
      else if (flight_mode == MODE_LANDING) {
         navi_rate(&pid_pos_Z, &pid_rate_Z, &target_Z, &current_Z, limited_target_vel, &pid_inner_z_pub, &pid_pos_param_Z, &pid_rate_param_Z, is_changed_target);
         if (pid_rate_Z.output < 0) {
            reset_I(&pid_rate_X, 0);
            reset_I(&pid_rate_Y, 0);
         }
         pos_hold(&pid_pos_X, &pid_rate_X, &target_X, &current_X, limited_target_vel, &pid_inner_x_pub, &pid_pos_param_X, &pid_rate_param_X);
         pos_hold(&pid_pos_Y, &pid_rate_Y, &target_Y, &current_Y, limited_target_vel, &pid_inner_y_pub, &pid_pos_param_Y, &pid_rate_param_Y);
         target_Z.target_vel = LANDING_SPEED;
         is_arm = 1950;
         if ( current_Z.cur_pos < ground_altitude + 60) {
            flight_mode = GROUND;
            manage_mode(SET, &flight_mode);
         }
      }
      if (flight_mode == GROUND) {
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
   void targetCallback(const geometry_msgs::Quaternion& msg) {
      std::cout << "TARGET::" << std::endl;;
      float target_x = msg.x;
      float target_y = msg.y;
      float target_z = msg.z;

      float current_x = 0;
      float current_y = 0;
      float current_z = 0;

      manage_current_pos(GET, &current_x, &current_y, &current_z);

      unsigned int mod = msg.w;
      unsigned int tmp_mod = GROUND;

#define PR_MOD(N) std::cout << "MSG MODE :: " << #N << std::endl;
      std::cout << ":: " << mod << std::endl;
      if ( mod == TAKEOFF) PR_MOD(TAKEOFF);
      if ( mod == MISSION_AUTO) PR_MOD(MISSION_AUTO);
      if ( mod == MISSION_MANUAL) PR_MOD(MISSION_MANUAL);
      if ( mod == LANDING) PR_MOD(LANDING);

      if (mod == TAKEOFF) {
         tmp_mod = MODE_TAKEOFF;
         if ( manage_mode(SET, &tmp_mod) != -1 );
         if (target_z)
            manage_target(SET_TARGET, &current_x, &current_y, &target_z);
         else {
            current_z += 500;
            manage_target(SET_TARGET, &current_x, &current_y, &current_z);
         }
      }
      else if (mod == MISSION_AUTO) {
         tmp_mod = MODE_NAV;
         manage_mode(SET, &tmp_mod);
         if (manage_target(SET_TARGET, &target_x, &target_y, &target_z) == -1) {
            tmp_mod = MODE_POSHOLD;
            manage_mode(SET, &tmp_mod);
            manage_target(SET_TARGET, &current_x, &current_y, &current_z);
         }
      }
      else if (mod == MISSION_MANUAL) {
         tmp_mod = MODE_MANUAL;
         manage_mode(SET, &tmp_mod);
         manage_target(SET_TARGET, &target_x, &target_y, &target_z);
      }
      else if (mod == LANDING) {
         tmp_mod = MODE_LANDING;
         manage_mode(SET, &tmp_mod);
         target_z = -3000;
         manage_target(SET_TARGET, &current_x, &current_y, &target_z);
      }
      else if (mod == 111 || mod == 112) {
         tmp_mod = MODE_NAV;
         manage_mode(SET, &tmp_mod);
         current_x += target_x;
         current_y += target_y;
         current_z += target_z;

         std::cout << current_x << "," << current_y << "," << current_z << std::endl;
         manage_target(SET_TARGET, &current_x, &current_y, &current_z);
      }
   }
};

void positionCallback(const std_msgs::Float32& msg) {
   // static pos_vel_t msg_pos_vel = {0,};

   // static pid_calc_t pid_pos = {0, };
   // static pid_calc_t pid_rate = {0, };
   // static target_pos_vel_t target_pos_vel = {0, };

   // msg_pos_vel.cur_time = ros::Time::now().toSec();
   // msg_pos_vel.cur_pos = msg.data;
   // calc_velocity(&msg_pos_vel);

   // std_msgs::Float32 float_msg;
   // float_msg.data = msg_pos_vel.cur_vel / 10 ;
   // //float_msg.data = msg_pos_vel.cur_vel * 100/30 ;
   // float_pub.publish(float_msg);

   // geometry_msgs::Point velocity_msg;
   // velocity_msg.x = 0;
   // velocity_msg.y = 0;
   // velocity_msg.z = msg_pos_vel.cur_vel;
   // velocity_pub.publish(velocity_msg);


   // pid_calc_t *tmp_pid_pos = &pid_pos;
   // pid_calc_t *tmp_pid_rate = &pid_rate;
   // target_pos_vel_t *tmp_target_pos_vel = &target_pos_vel;

   // calc_pos_error(tmp_pid_pos, tmp_target_pos_vel , &msg_pos_vel);
   // calc_pid(tmp_pid_pos, &pid_pos_param_X);
   // tmp_target_pos_vel->target_vel = tmp_pid_pos->output;
   // calc_rate_error(tmp_pid_rate, tmp_target_pos_vel , &msg_pos_vel);
   // calc_pid(tmp_pid_rate, &pid_rate_param_X);

}
void paramCallback(const std_msgs::Int32& msg) {
   update_param(msg.data);
   std::cout << param_list[msg.data] << " :: " << *get_param_n(msg.data) << std::endl;
}


int main(int argc, char **argv) {
   ros::init(argc, argv, "listener");
   ros::NodeHandle n;
   // init_param(&pid_param);
   init_param();

   // Creat Timer to update the parameter.
   // parameter server uses disk io, so it causes some delay.
   // ros::Timer timer = n.createTimer(ros::Duration(1), timerCallback);
   float_pub = n.advertise<std_msgs::Float32>("calculated_distance", 100);
   // velocity_pub = n.advertise<geometry_msgs::Point>("/FIRST/CURRENT_VEL", 100);

   // pid_out_pub     = n.advertise<std_msgs::UInt16MultiArray>("/FIRST/OUTPUT_PID", 100);
   // pid_inner_x_pub = n.advertise<geometry_msgs::Inertia>("/FIRST/OUTPUT_INNER_PID/X", 100);
   // pid_inner_y_pub = n.advertise<geometry_msgs::Inertia>("/FIRST/OUTPUT_INNER_PID/Y", 100);
   // pid_inner_z_pub = n.advertise<geometry_msgs::Inertia>("/FIRST/OUTPUT_INNER_PID/Z", 100);

   ros::Subscriber sub = n.subscribe("generate_sin_pulse", 100, positionCallback);
   ros::Subscriber param_sub = n.subscribe("/PARAM_CHANGE", 100, paramCallback);
   // ros::Subscriber position_sub = n.subscribe("/FIRST/CURRENT_POS", 100, position_Callback);
   // ros::Subscriber target_sub = n.subscribe("/FIRST/TARGET_POS", 100, targetCallback);
   PIDCALCULATION first();
   PIDCALCULATION second();
   PIDCALCULATION third();
   PIDCALCULATION fourth();

   ros::MultiThreadedSpinner spinner(4); // Use 4 threads
   spinner.spin(); // spin() will not return until the node has been shutdown

   // ros::spin();
   return 0;
}
