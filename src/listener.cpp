#include "ros/ros.h"

#include <string>
#include <iostream>
#include "param.h"
#include "pidcontroller.h"


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
PIDCONTROLLER *drone_controller[4];



void positionCallback(const std_msgs::Float32& msg) {
}

void paramCallback(const std_msgs::Int32& msg) {

   std::string param_name("pidparam");

   // std::cout << msg.data << std::endl;
   if ( msg.data >= LOAD_PARAMFILE && msg.data <= LOAD_PARAMFILE4) {

      param_name = tmp_dir + param_name;
      int drone_tmp_number = msg.data - LOAD_PARAMFILE;
      // std::cout << "drone_num:" << drone_tmp_number << std::endl;
      char num[2] = "0";
      num[0] += drone_tmp_number;
      if (num[0] != '0') {
         drone_controller[drone_tmp_number - 1]->set_self_param();
         param_name += num;
      }
      std::cout << "LOAD the Param file:::" << param_name  << std::endl;

      if (!drone_tmp_number) {
         if (!load_param(param_name.c_str() ) ) {
            std::cout << "FAIL to load:::" << param_name << std::endl;
         }

      }
      else if (!load_param(param_name.c_str(), &(drone_controller[drone_tmp_number - 1]->pid_param_c) ) ) {
         std::cout << "FAIL to load:::" << param_name << std::endl;
      }

      // std::cout << *get_param_n(0, &(drone_controller[drone_tmp_number-1]->pid_param_c)) << std::endl;
   }
   else {
      int param_data = msg.data % 100;
      int drone_number = msg.data / 100;
      if (!drone_number) {
         update_param(param_data);
         std::cout << "COMMON :: " << param_list[param_data] << " :: " << *get_param_n(param_data) << std::endl;
      }
      else {
         update_param(param_data, &(drone_controller[drone_number - 1]->pid_param_c) );
         std::cout << drone_controller[drone_number - 1]->drone << " :: " << param_list[param_data] << " :: " << *get_param_n(param_data, &(drone_controller[drone_number - 1]->pid_param_c) ) << std::endl;
      }
   }
}

int main(int argc, char **argv) {
   ros::init(argc, argv, "listener");
   ros::NodeHandle n;
   init_param();

   // Creat Timer to update the parameter.
   // parameter server uses disk io, so it causes some delay.
   // ros::Timer timer = n.createTimer(ros::Duration(1), timerCallback);
   //float_pub = n.advertise<std_msgs::Float32>("calculated_distance", 100);
   ros::Subscriber sub = n.subscribe("generate_sin_pulse", 100, positionCallback);
   ros::Subscriber param_sub = n.subscribe("/PARAM_CHANGE", 100, paramCallback);

   PIDCONTROLLER first("/FIRST", 0.0f, 0.0f);
   drone_controller[0] = &first;
   PIDCONTROLLER second("/SECOND", 0.0f, 0.0f);
   drone_controller[1] = &second;
   PIDCONTROLLER third("/THIRD", 0.0f, 0.0f);
   drone_controller[2] = &third;
   PIDCONTROLLER fourth("/FOURTH", 0.0f, 0.0f);
   drone_controller[3] = &fourth;

   // ros::MultiThreadedSpinner spinner(4); // Use 4 threads
   // spinner.spin(); // spin() will not return until the node has been shutdown

   ros::spin();
   return 0;
}
