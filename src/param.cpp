#include "ros/ros.h"
#include "param.h"


char param_list[][50] =
{
   "FIRST/POS/X/PID_P",    //0
   "FIRST/POS/X/PID_I",    //1
   "FIRST/POS/X/PID_D",    //2
   "FIRST/POS/X/PID_IMAX",    //3

   "FIRST/RATE/X/PID_P",      //4
   "FIRST/RATE/X/PID_I",      //5
   "FIRST/RATE/X/PID_D",      //6
   "FIRST/RATE/X/PID_IMAX",   //7

   "FIRST/POS/Y/PID_P",    //8
   "FIRST/POS/Y/PID_I",    //9
   "FIRST/POS/Y/PID_D",    //10
   "FIRST/POS/Y/PID_IMAX",    //11

   "FIRST/RATE/Y/PID_P",      //12
   "FIRST/RATE/Y/PID_I",      //13
   "FIRST/RATE/Y/PID_D",      //14
   "FIRST/RATE/Y/PID_IMAX",   //15

   "FIRST/POS/Z/PID_P",    //16
   "FIRST/POS/Z/PID_I",    //17
   "FIRST/POS/Z/PID_D",    //18
   "FIRST/POS/Z/PID_IMAX",    //19

   "FIRST/RATE/Z/PID_P",      //20
   "FIRST/RATE/Z/PID_I",      //21
   "FIRST/RATE/Z/PID_D",      //22
   "FIRST/RATE/Z/PID_IMAX"    //23
};






pid_parameter_t pid_pos_param_X = {0, };
pid_parameter_t pid_rate_param_X = {0, };

pid_parameter_t pid_pos_param_Y = {0, };
pid_parameter_t pid_rate_param_Y = {0, };

pid_parameter_t pid_pos_param_Z = {0, };
pid_parameter_t pid_rate_param_Z = {0, };



pid_parameter_t default_param_pos_X = {
   1.5l,
   0.0l,
   0.0l,
   200.0l
};


pid_parameter_t default_param_rate_X = {
   0.12l,
   0.018l,
   0.06l,
   300.0l
};
pid_parameter_t default_param_pos_Y = default_param_pos_X;
pid_parameter_t default_param_rate_Y = default_param_rate_X;
pid_parameter_t default_param_pos_Z = {
   2.5l,
   0.0l,
   0.0l,
   200.0l
};



pid_parameter_t default_param_rate_Z = {
   0.1l,
   0.04l,
   0.04l,
   500.0l
};


pos_pid_parameter_t pid_default_param = {
   &default_param_pos_X,
   &default_param_rate_X,
   &default_param_pos_Y,
   &default_param_rate_Y,
   &default_param_pos_Z,
   &default_param_rate_Z
};


pos_pid_parameter_t pid_param = {
   &pid_pos_param_X,
   &pid_rate_param_X,
   &pid_pos_param_Y,
   &pid_rate_param_Y,
   &pid_pos_param_Z,
   &pid_rate_param_Z
};






double *get_param_n(int n) {
   return &((&(((&(pid_param.pos_pid_X))[n / 4])->pid_P))[n % 4]);
}
double *get_param_n(int pr, int xyz, int pidi) {
   int n = get_param_num(pr, xyz, pidi);
   return &((&(((&(pid_param.pos_pid_X))[n / 4])->pid_P))[n % 4]);
}

double *get_default_param_n(int n) {
   return &((&(((&(pid_default_param.pos_pid_X))[n / 4])->pid_P))[n % 4]);
}
double *get_default_param_n(int pr, int xyz, int pidi) {
   int n = get_param_num(pr, xyz, pidi);
   return &((&(((&(pid_default_param.pos_pid_X))[n / 4])->pid_P))[n % 4]);
}

int get_param_num(int pr, int xyz, int pidi) {
   return pr * 4 + xyz * 8 + pidi;
}


void set_param_n(int n, double data) {
   ros::param::set(param_list[n], data);
}
void set_param_n(int pr, int xyz, int pidi, double data) {
   int n = get_param_num(pr, xyz, pidi);
   ros::param::set(param_list[n], data);
}



void update_param(int n) {
   *get_param_n(n) = ros::param::param( (param_list[n]), *get_param_n(n));
}
void update_param(){
   int i = 0;
   while (i < sizeof(param_list) / sizeof(param_list[0])) {
      update_param(i++);
   }

}

void reset_param() {
   int i = 0;
   while (i < sizeof(param_list) / sizeof(param_list[0])) {
      //init_param(param_list[i], get_param_n(i), get_default_param_n(i));
      ros::param::set(param_list[i], *get_default_param_n(i));

      i++;
   }

}
void init_param(const std::string& param_name, double *param_ptr, double *default_ptr) {

   if (ros::param::has(param_name)) {
      ros::param::get(param_name, *param_ptr);
      *default_ptr = *param_ptr;
   }
   else {
      std::cout << *default_ptr;
      ros::param::set(param_name, *default_ptr);
      
      *param_ptr = *default_ptr;
   }

}



void init_param() {
   int i = 0;
   //std::cout << sizeof(param_list) / sizeof(param_list[0]) << std::endl;
   while (i < sizeof(param_list) / sizeof(param_list[0])) {
      init_param(param_list[i], get_param_n(i), get_default_param_n(i));
      // std::cout << i << " " << param_list[i] << " : " << *get_param_n(i) << ":" << *get_default_param_n(i) << std::endl;
      i++;
   }
}

// pos_P = 11 / 100
// pos_I = 0
// pos_D = 0
// pos_rate_P = 20 / 10;
// pos_rate_I = 8 / 100;
// pos_rate_d = 45 / 1000;

void delete_param() {

   int i = 0;
   while (i < sizeof(param_list) / sizeof(param_list[0])) {
      //init_param(param_list[i], get_param_n(i), get_default_param_n(i));
      ros::param::del(param_list[i]);

      i++;
   }


}