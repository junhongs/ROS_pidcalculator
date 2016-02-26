#include "ros/ros.h"
#include "param.h"

// RATE = 24
char param_list[][50] =
{
   "FIRST/POSHOLD/POS/X/PID_P",    //0
   "FIRST/POSHOLD/POS/X/PID_I",    //1
   "FIRST/POSHOLD/POS/X/PID_D",    //2
   "FIRST/POSHOLD/POS/X/PID_IMAX",    //3

   "FIRST/POSHOLD/RATE/X/PID_P",      //4
   "FIRST/POSHOLD/RATE/X/PID_I",      //5
   "FIRST/POSHOLD/RATE/X/PID_D",      //6
   "FIRST/POSHOLD/RATE/X/PID_IMAX",   //7

   "FIRST/POSHOLD/POS/Y/PID_P",    //8
   "FIRST/POSHOLD/POS/Y/PID_I",    //9
   "FIRST/POSHOLD/POS/Y/PID_D",    //10
   "FIRST/POSHOLD/POS/Y/PID_IMAX",    //11

   "FIRST/POSHOLD/RATE/Y/PID_P",      //12
   "FIRST/POSHOLD/RATE/Y/PID_I",      //13
   "FIRST/POSHOLD/RATE/Y/PID_D",      //14
   "FIRST/POSHOLD/RATE/Y/PID_IMAX",   //15

   "FIRST/POSHOLD/POS/Z/PID_P",    //16
   "FIRST/POSHOLD/POS/Z/PID_I",    //17
   "FIRST/POSHOLD/POS/Z/PID_D",    //18
   "FIRST/POSHOLD/POS/Z/PID_IMAX",    //19

   "FIRST/POSHOLD/RATE/Z/PID_P",      //20
   "FIRST/POSHOLD/RATE/Z/PID_I",      //21
   "FIRST/POSHOLD/RATE/Z/PID_D",      //22
   "FIRST/POSHOLD/RATE/Z/PID_IMAX",    //23


   "FIRST/NAV/POS/X/PID_P",
   "FIRST/NAV/POS/X/PID_I",
   "FIRST/NAV/POS/X/PID_D",
   "FIRST/NAV/POS/X/PID_IMAX",

   "FIRST/NAV/RATE/X/PID_P",  
   "FIRST/NAV/RATE/X/PID_I",  
   "FIRST/NAV/RATE/X/PID_D",  
   "FIRST/NAV/RATE/X/PID_IMAX",

   "FIRST/NAV/POS/Y/PID_P",
   "FIRST/NAV/POS/Y/PID_I",
   "FIRST/NAV/POS/Y/PID_D",
   "FIRST/NAV/POS/Y/PID_IMAX",

   "FIRST/NAV/RATE/Y/PID_P",  
   "FIRST/NAV/RATE/Y/PID_I",  
   "FIRST/NAV/RATE/Y/PID_D",  
   "FIRST/NAV/RATE/Y/PID_IMAX",

   "FIRST/NAV/POS/Z/PID_P",
   "FIRST/NAV/POS/Z/PID_I",
   "FIRST/NAV/POS/Z/PID_D",
   "FIRST/NAV/POS/Z/PID_IMAX",

   "FIRST/NAV/RATE/Z/PID_P",  
   "FIRST/NAV/RATE/Z/PID_I",  
   "FIRST/NAV/RATE/Z/PID_D",  
   "FIRST/NAV/RATE/Z/PID_IMAX"
};

//POSHOLD PARAMETER
pid_parameter_t pid_poshold_pos_param_X = {0, };
pid_parameter_t pid_poshold_rate_param_X = {0, };

pid_parameter_t pid_poshold_pos_param_Y = {0, };
pid_parameter_t pid_poshold_rate_param_Y = {0, };

pid_parameter_t pid_poshold_pos_param_Z = {0, };
pid_parameter_t pid_poshold_rate_param_Z = {0, };



//NAVIGATION PARAMETER
pid_parameter_t pid_nav_pos_param_X = {0, };
pid_parameter_t pid_nav_rate_param_X = {0, };

pid_parameter_t pid_nav_pos_param_Y = {0, };
pid_parameter_t pid_nav_rate_param_Y = {0, };

pid_parameter_t pid_nav_pos_param_Z = {0, };
pid_parameter_t pid_nav_rate_param_Z = {0, };





//DEFAULT POSHOLD PARAMETER
pid_parameter_t default_param_pos_X = {
   1.080l,
   0.255l,
   0.0l,
   200.0l
};
pid_parameter_t default_param_pos_Y = {
   1.205l,
   0.255l,
   0.0l,
   200.0l
};
pid_parameter_t default_param_pos_Z = {
   1.210l,
   0.255l,
   0.0l,
   200.0l
};


pid_parameter_t default_param_rate_X = {
   0.125l,
   0.020l,
   0.019l,
   300.0l
};
pid_parameter_t default_param_rate_Y = {
   0.122l,
   0.020l,
   0.017l,
   300.0l
};
pid_parameter_t default_param_rate_Z = {
   0.135l,
   0.015l,
   0.019l,
   500.0l
};








//DEFAULT NAVIGATION PARAMETER
pid_parameter_t default_param_nav_pos_X = {
   1.980l,
   0.0l,
   0.0l,
   200.0l
};
pid_parameter_t default_param_nav_pos_Y = {
   2.465l,
   0.0l,
   0.0l,
   200.0l
};
pid_parameter_t default_param_nav_pos_Z = {
   1.902l,
   0.0l,
   0.0l,
   200.0l
};


pid_parameter_t default_param_nav_rate_X = {
   0.065l,
   0.048l,
   0.050l,
   300.0l
};
pid_parameter_t default_param_nav_rate_Y = {
   0.071l,
   0.043l,
   0.056l,
   300.0l
};
pid_parameter_t default_param_nav_rate_Z = {
   0.094l,
   0.031l,
   0.010l,
   500.0l
};




pos_pid_parameter_t pid_default_param = {
   &default_param_pos_X,
   &default_param_rate_X,
   &default_param_pos_Y,
   &default_param_rate_Y,
   &default_param_pos_Z,
   &default_param_rate_Z,

   &default_param_nav_pos_X,
   &default_param_nav_rate_X,
   &default_param_nav_pos_Y,
   &default_param_nav_rate_Y,
   &default_param_nav_pos_Z,
   &default_param_nav_rate_Z
};


pos_pid_parameter_t pid_param = {
   //POSHOLD PARAM
   &pid_poshold_pos_param_X,
   &pid_poshold_rate_param_X,
   &pid_poshold_pos_param_Y,
   &pid_poshold_rate_param_Y,
   &pid_poshold_pos_param_Z,
   &pid_poshold_rate_param_Z,

   //NAV  PARAM
   &pid_nav_pos_param_X,
   &pid_nav_rate_param_X,
   &pid_nav_pos_param_Y,
   &pid_nav_rate_param_Y,
   &pid_nav_pos_param_Z,
   &pid_nav_rate_param_Z
};



double *get_param_n(int n) {
   return &((&(((&(pid_param.pos_pid_X))[n / 4])->pid_P))[n % 4]);
}
double *get_param_n(int pr, int xyz, int pidi) {
   int n = get_param_num(pr, xyz, pidi);
   return &((&(((&(pid_param.pos_pid_X))[n / 4])->pid_P))[n % 4]);
}

double *get_param_n(int nav, int pr, int xyz, int pidi) {
   int n = get_param_num(nav, pr, xyz, pidi);
   return &((&(((&(pid_param.pos_pid_X))[n / 4])->pid_P))[n % 4]);
}



double *get_default_param_n(int n) {
   return &((&(((&(pid_default_param.pos_pid_X))[n / 4])->pid_P))[n % 4]);
}
double *get_default_param_n(int pr, int xyz, int pidi) {
   int n = get_param_num(pr, xyz, pidi);
   return &((&(((&(pid_default_param.pos_pid_X))[n / 4])->pid_P))[n % 4]);
}
double *get_default_param_n(int nav, int pr, int xyz, int pidi) {
   int n = get_param_num(pr, xyz, pidi);
   return &((&(((&(pid_default_param.pos_pid_X))[n / 4])->pid_P))[n % 4]);
}





int get_param_num(int pr, int xyz, int pidi) {
   return pr * 4 + xyz * 8 + pidi;
}

int get_param_num(int nav, int pr, int xyz, int pidi) {
   return nav * 24 + pr * 4 + xyz * 8 + pidi;
}


void set_param_n(int n, double data) {
   ros::param::set(param_list[n], data);
}
void set_param_n(int pr, int xyz, int pidi, double data) {
   int n = get_param_num(pr, xyz, pidi);
   ros::param::set(param_list[n], data);
}
void set_param_n(int nav, int pr, int xyz, int pidi, double data) {
   int n = get_param_num(nav, pr, xyz, pidi);
   ros::param::set(param_list[n], data);
}



void update_param(int n) {
   *get_param_n(n) = ros::param::param((param_list[n]), *get_param_n(n));
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
      // std::cout << *default_ptr << std::endl;
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