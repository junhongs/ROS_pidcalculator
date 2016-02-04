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

pos_pid_parameter_t pid_param = {
   &pid_pos_param_X,
   &pid_rate_param_X,
   &pid_pos_param_Y,
   &pid_rate_param_Y,
   &pid_pos_param_Z,
   &pid_rate_param_Z
};

pid_parameter_t default_param_pos = {
   1,
   0,
   0,
   200
};

pid_parameter_t default_param_rate = {
//Z value!
   0.4,
   0.002,
   0.005,
   200

   // 0.2,
   // 0.008,
   // 0.0045,
   // 200


};

pid_parameter_t reset_param_pos_X = default_param_pos;
pid_parameter_t reset_param_rate_X = default_param_rate;
pid_parameter_t reset_param_pos_Y = default_param_pos;
pid_parameter_t reset_param_rate_Y = default_param_rate;
pid_parameter_t reset_param_pos_Z = default_param_pos;
pid_parameter_t reset_param_rate_Z = default_param_rate;


double *get_param_n(int n) {
   return &((&((&(pid_param.pos_pid_X)[n / 4])->pid_P))[n % 4]);
}
double *get_param_n(int pr, int xyz, int pidi) {
   int n = get_param_num(pr,xyz,pidi);
   return &((&((&(pid_param.pos_pid_X)[n / 4])->pid_P))[n % 4]);
}

void update_param(int n){
   *get_param_n(n) = ros::param::param( (param_list[n]),*get_param_n(n));
}

int get_param_num(int pr, int xyz, int pidi) {
   return pr * 4 + xyz * 8 + pidi;
}
void set_param_n(int n, double data){
   ros::param::set(param_list[n], data);
}
void set_param_n(int pr, int xyz, int pidi, double data){
   int n = get_param_num(pr,xyz,pidi);
   ros::param::set(param_list[n], data);
}


void update_param(  pos_pid_parameter_t *pid_param) {
   pid_parameter_t *pid_param_tmp;
   pid_param_tmp = pid_param->pos_pid_X;
   ros::param::param("FIRST/POS/X/PID_P", pid_param_tmp->pid_P);
   ros::param::param("FIRST/POS/X/PID_I", pid_param_tmp->pid_I);
   ros::param::param("FIRST/POS/X/PID_D", pid_param_tmp->pid_D);
   ros::param::param("FIRST/POS/X/PID_IMAX", pid_param_tmp->pid_Imax);

   pid_param_tmp = pid_param->rate_pid_X;
   ros::param::param("FIRST/RATE/X/PID_P", pid_param_tmp->pid_P);
   ros::param::param("FIRST/RATE/X/PID_I", pid_param_tmp->pid_I);
   ros::param::param("FIRST/RATE/X/PID_D", pid_param_tmp->pid_D);
   ros::param::param("FIRST/RATE/X/PID_IMAX", pid_param_tmp->pid_Imax);


   pid_param_tmp = pid_param->pos_pid_Y;
   ros::param::param("FIRST/POS/Y/PID_P", pid_param_tmp->pid_P);
   ros::param::param("FIRST/POS/Y/PID_I", pid_param_tmp->pid_I);
   ros::param::param("FIRST/POS/Y/PID_D", pid_param_tmp->pid_D);
   ros::param::param("FIRST/POS/Y/PID_IMAX", pid_param_tmp->pid_Imax);

   pid_param_tmp = pid_param->rate_pid_Y;
   ros::param::param("FIRST/RATE/Y/PID_P", pid_param_tmp->pid_P);
   ros::param::param("FIRST/RATE/Y/PID_I", pid_param_tmp->pid_I);
   ros::param::param("FIRST/RATE/Y/PID_D", pid_param_tmp->pid_D);
   ros::param::param("FIRST/RATE/Y/PID_IMAX", pid_param_tmp->pid_Imax);


   pid_param_tmp = pid_param->pos_pid_Z;
   ros::param::param("FIRST/POS/Z/PID_P", pid_param_tmp->pid_P);
   ros::param::param("FIRST/POS/Z/PID_I", pid_param_tmp->pid_I);
   ros::param::param("FIRST/POS/Z/PID_D", pid_param_tmp->pid_D);
   ros::param::param("FIRST/POS/Z/PID_IMAX", pid_param_tmp->pid_Imax);

   pid_param_tmp = pid_param->rate_pid_Z;
   ros::param::param("FIRST/RATE/Z/PID_P", pid_param_tmp->pid_P);
   ros::param::param("FIRST/RATE/Z/PID_I", pid_param_tmp->pid_I);
   ros::param::param("FIRST/RATE/Z/PID_D", pid_param_tmp->pid_D);
   ros::param::param("FIRST/RATE/Z/PID_IMAX", pid_param_tmp->pid_Imax);
}

void set_param(pos_pid_parameter_t *pid_param) {
   pid_parameter_t *pid_param_tmp;

   pid_param_tmp = pid_param->pos_pid_X;
   if (pid_param_tmp->pid_P != -1)
      ros::param::set("FIRST/POS/X/PID_P", reset_param_pos_X.pid_P = pid_param_tmp->pid_P);
   if (pid_param_tmp->pid_I != -1)
      ros::param::set("FIRST/POS/X/PID_I", reset_param_pos_X.pid_I = pid_param_tmp->pid_I);
   if (pid_param_tmp->pid_D != -1)
      ros::param::set("FIRST/POS/X/PID_D", reset_param_pos_X.pid_D = pid_param_tmp->pid_D);
   if (pid_param_tmp->pid_Imax != -1)
      ros::param::set("FIRST/POS/X/PID_IMAX", reset_param_pos_X.pid_Imax = pid_param_tmp->pid_Imax);

   pid_param_tmp = pid_param->rate_pid_X;
   if (pid_param_tmp->pid_P != -1)
      ros::param::set("FIRST/RATE/X/PID_P", reset_param_rate_X.pid_P = pid_param_tmp->pid_P);
   if (pid_param_tmp->pid_I != -1)
      ros::param::set("FIRST/RATE/X/PID_I", reset_param_rate_X.pid_I = pid_param_tmp->pid_I);
   if (pid_param_tmp->pid_D != -1)
      ros::param::set("FIRST/RATE/X/PID_D", reset_param_rate_X.pid_D = pid_param_tmp->pid_D);
   if (pid_param_tmp->pid_Imax != -1)
      ros::param::set("FIRST/RATE/X/PID_IMAX", reset_param_rate_X.pid_Imax = pid_param_tmp->pid_Imax);





   pid_param_tmp = pid_param->pos_pid_Y;
   if (pid_param_tmp->pid_P != -1)
      ros::param::set("FIRST/POS/Y/PID_P", reset_param_pos_Y.pid_P = pid_param_tmp->pid_P);
   if (pid_param_tmp->pid_I != -1)
      ros::param::set("FIRST/POS/Y/PID_I", reset_param_pos_Y.pid_I = pid_param_tmp->pid_I);
   if (pid_param_tmp->pid_D != -1)
      ros::param::set("FIRST/POS/Y/PID_D", reset_param_pos_Y.pid_D = pid_param_tmp->pid_D);
   if (pid_param_tmp->pid_Imax != -1)
      ros::param::set("FIRST/POS/Y/PID_IMAX", reset_param_pos_Y.pid_Imax = pid_param_tmp->pid_Imax);

   pid_param_tmp = pid_param->rate_pid_Y;
   if (pid_param_tmp->pid_P != -1)
      ros::param::set("FIRST/RATE/Y/PID_P", reset_param_rate_Y.pid_P = pid_param_tmp->pid_P);
   if (pid_param_tmp->pid_I != -1)
      ros::param::set("FIRST/RATE/Y/PID_I", reset_param_rate_Y.pid_I = pid_param_tmp->pid_I);
   if (pid_param_tmp->pid_D != -1)
      ros::param::set("FIRST/RATE/Y/PID_D", reset_param_rate_Y.pid_D = pid_param_tmp->pid_D);
   if (pid_param_tmp->pid_Imax != -1)
      ros::param::set("FIRST/RATE/Y/PID_IMAX", reset_param_rate_Y.pid_Imax = pid_param_tmp->pid_Imax);






   pid_param_tmp = pid_param->pos_pid_Z;
   if (pid_param_tmp->pid_P != -1)
      ros::param::set("FIRST/POS/Z/PID_P", reset_param_pos_Z.pid_P = pid_param_tmp->pid_P);
   if (pid_param_tmp->pid_I != -1)
      ros::param::set("FIRST/POS/Z/PID_I", reset_param_pos_Z.pid_I = pid_param_tmp->pid_I);
   if (pid_param_tmp->pid_D != -1)
      ros::param::set("FIRST/POS/Z/PID_D", reset_param_pos_Z.pid_D = pid_param_tmp->pid_D);
   if (pid_param_tmp->pid_Imax != -1)
      ros::param::set("FIRST/POS/Z/PID_IMAX", reset_param_pos_Z.pid_Imax = pid_param_tmp->pid_Imax);

   pid_param_tmp = pid_param->rate_pid_Z;
   if (pid_param_tmp->pid_P != -1)
      ros::param::set("FIRST/RATE/Z/PID_P", reset_param_rate_Z.pid_P = pid_param_tmp->pid_P);
   if (pid_param_tmp->pid_I != -1)
      ros::param::set("FIRST/RATE/Z/PID_I", reset_param_rate_Z.pid_I = pid_param_tmp->pid_I);
   if (pid_param_tmp->pid_D != -1)
      ros::param::set("FIRST/RATE/Z/PID_D", reset_param_rate_Z.pid_D = pid_param_tmp->pid_D);
   if (pid_param_tmp->pid_Imax != -1)
      ros::param::set("FIRST/RATE/Z/PID_IMAX", reset_param_rate_Z.pid_Imax = pid_param_tmp->pid_Imax);
}


void reset_param( pos_pid_parameter_t *pid_param ) {
   pid_parameter_t *pid_param_tmp;
   pid_param_tmp = &reset_param_pos_X;
   ros::param::set("FIRST/POS/X/PID_P", pid_param_tmp->pid_P);
   ros::param::set("FIRST/POS/X/PID_I", pid_param_tmp->pid_I);
   ros::param::set("FIRST/POS/X/PID_D", pid_param_tmp->pid_D);
   ros::param::set("FIRST/POS/X/PID_IMAX", pid_param_tmp->pid_Imax);

   pid_param_tmp = &reset_param_rate_X;
   ros::param::set("FIRST/RATE/X/PID_P", pid_param_tmp->pid_P);
   ros::param::set("FIRST/RATE/X/PID_I", pid_param_tmp->pid_I);
   ros::param::set("FIRST/RATE/X/PID_D", pid_param_tmp->pid_D);
   ros::param::set("FIRST/RATE/X/PID_IMAX", pid_param_tmp->pid_Imax);
}



void init_param(  pos_pid_parameter_t *pid_param) {
   pid_parameter_t *pid_param_tmp;
   pid_parameter_t *default_param_tmp;
   /*
      if params are not exist, make and set the parameters.
      if exist, get the parameters.
   */
   pid_param_tmp = pid_param->pos_pid_X;
   default_param_tmp = &default_param_pos;
   if ( !(pid_param_tmp->pid_P = ros::param::param("FIRST/POS/X/PID_P", pid_param_tmp->pid_P))  )
      ros::param::set("FIRST/POS/X/PID_P", pid_param_tmp->pid_P = default_param_tmp->pid_P);
   if ( !(pid_param_tmp->pid_I = ros::param::param("FIRST/POS/X/PID_I", pid_param_tmp->pid_I)) )
      ros::param::set("FIRST/POS/X/PID_I", pid_param_tmp->pid_I = default_param_tmp->pid_I);
   if ( !(pid_param_tmp->pid_D = ros::param::param("FIRST/POS/X/PID_D", pid_param_tmp->pid_D)) )
      ros::param::set("FIRST/POS/X/PID_D", pid_param_tmp->pid_D = default_param_tmp->pid_D);
   if ( !(pid_param_tmp->pid_Imax = ros::param::param("FIRST/POS/X/PID_IMAX", pid_param_tmp->pid_Imax)) )
      ros::param::set("FIRST/POS/X/PID_IMAX", pid_param_tmp->pid_Imax = default_param_tmp->pid_Imax);

   reset_param_pos_X.pid_P = pid_param_tmp->pid_P;
   reset_param_pos_X.pid_I = pid_param_tmp->pid_I;
   reset_param_pos_X.pid_D = pid_param_tmp->pid_D;
   reset_param_pos_X.pid_Imax = pid_param_tmp->pid_Imax;



   pid_param_tmp = pid_param->rate_pid_X;
   default_param_tmp = &default_param_rate;
   if ( !(pid_param_tmp->pid_P = ros::param::param("FIRST/RATE/X/PID_P", pid_param_tmp->pid_P))  )
      ros::param::set("FIRST/RATE/X/PID_P", pid_param_tmp->pid_P = default_param_tmp->pid_P);
   if ( !(pid_param_tmp->pid_I = ros::param::param("FIRST/RATE/X/PID_I", pid_param_tmp->pid_I)) )
      ros::param::set("FIRST/RATE/X/PID_I", pid_param_tmp->pid_I = default_param_tmp->pid_I);
   if ( !(pid_param_tmp->pid_D = ros::param::param("FIRST/RATE/X/PID_D", pid_param_tmp->pid_D)) )
      ros::param::set("FIRST/RATE/X/PID_D", pid_param_tmp->pid_D = default_param_tmp->pid_D);
   if ( !(pid_param_tmp->pid_Imax = ros::param::param("FIRST/RATE/X/PID_IMAX", pid_param_tmp->pid_Imax)) )
      ros::param::set("FIRST/RATE/X/PID_IMAX", pid_param_tmp->pid_Imax = default_param_tmp->pid_Imax);

   reset_param_rate_X.pid_P = pid_param_tmp->pid_P;
   reset_param_rate_X.pid_I = pid_param_tmp->pid_I;
   reset_param_rate_X.pid_D = pid_param_tmp->pid_D;
   reset_param_rate_X.pid_Imax = pid_param_tmp->pid_Imax;









   pid_param_tmp = pid_param->pos_pid_Y;
   default_param_tmp = &default_param_pos;
   if ( !(pid_param_tmp->pid_P = ros::param::param("FIRST/POS/Y/PID_P", pid_param_tmp->pid_P))  )
      ros::param::set("FIRST/POS/Y/PID_P", pid_param_tmp->pid_P = default_param_tmp->pid_P);
   if ( !(pid_param_tmp->pid_I = ros::param::param("FIRST/POS/Y/PID_I", pid_param_tmp->pid_I)) )
      ros::param::set("FIRST/POS/Y/PID_I", pid_param_tmp->pid_I = default_param_tmp->pid_I);
   if ( !(pid_param_tmp->pid_D = ros::param::param("FIRST/POS/Y/PID_D", pid_param_tmp->pid_D)) )
      ros::param::set("FIRST/POS/Y/PID_D", pid_param_tmp->pid_D = default_param_tmp->pid_D);
   if ( !(pid_param_tmp->pid_Imax = ros::param::param("FIRST/POS/Y/PID_IMAX", pid_param_tmp->pid_Imax)) )
      ros::param::set("FIRST/POS/Y/PID_IMAX", pid_param_tmp->pid_Imax = default_param_tmp->pid_Imax);

   reset_param_pos_Y.pid_P = pid_param_tmp->pid_P;
   reset_param_pos_Y.pid_I = pid_param_tmp->pid_I;
   reset_param_pos_Y.pid_D = pid_param_tmp->pid_D;
   reset_param_pos_Y.pid_Imax = pid_param_tmp->pid_Imax;



   pid_param_tmp = pid_param->rate_pid_Y;
   default_param_tmp = &default_param_rate;
   if ( !(pid_param_tmp->pid_P = ros::param::param("FIRST/RATE/Y/PID_P", pid_param_tmp->pid_P))  )
      ros::param::set("FIRST/RATE/Y/PID_P", pid_param_tmp->pid_P = default_param_tmp->pid_P);
   if ( !(pid_param_tmp->pid_I = ros::param::param("FIRST/RATE/Y/PID_I", pid_param_tmp->pid_I)) )
      ros::param::set("FIRST/RATE/Y/PID_I", pid_param_tmp->pid_I = default_param_tmp->pid_I);
   if ( !(pid_param_tmp->pid_D = ros::param::param("FIRST/RATE/Y/PID_D", pid_param_tmp->pid_D)) )
      ros::param::set("FIRST/RATE/Y/PID_D", pid_param_tmp->pid_D = default_param_tmp->pid_D);
   if ( !(pid_param_tmp->pid_Imax = ros::param::param("FIRST/RATE/Y/PID_IMAX", pid_param_tmp->pid_Imax)) )
      ros::param::set("FIRST/RATE/Y/PID_IMAX", pid_param_tmp->pid_Imax = default_param_tmp->pid_Imax);

   reset_param_rate_Y.pid_P = pid_param_tmp->pid_P;
   reset_param_rate_Y.pid_I = pid_param_tmp->pid_I;
   reset_param_rate_Y.pid_D = pid_param_tmp->pid_D;
   reset_param_rate_Y.pid_Imax = pid_param_tmp->pid_Imax;


















   pid_param_tmp = pid_param->pos_pid_Z;
   default_param_tmp = &default_param_pos;
   if ( !(pid_param_tmp->pid_P = ros::param::param("FIRST/POS/Z/PID_P", pid_param_tmp->pid_P))  )
      ros::param::set("FIRST/POS/Z/PID_P", pid_param_tmp->pid_P = default_param_tmp->pid_P);
   if ( !(pid_param_tmp->pid_I = ros::param::param("FIRST/POS/Z/PID_I", pid_param_tmp->pid_I)) )
      ros::param::set("FIRST/POS/Z/PID_I", pid_param_tmp->pid_I = default_param_tmp->pid_I);
   if ( !(pid_param_tmp->pid_D = ros::param::param("FIRST/POS/Z/PID_D", pid_param_tmp->pid_D)) )
      ros::param::set("FIRST/POS/Z/PID_D", pid_param_tmp->pid_D = default_param_tmp->pid_D);
   if ( !(pid_param_tmp->pid_Imax = ros::param::param("FIRST/POS/Z/PID_IMAX", pid_param_tmp->pid_Imax)) )
      ros::param::set("FIRST/POS/Z/PID_IMAX", pid_param_tmp->pid_Imax = default_param_tmp->pid_Imax);

   reset_param_pos_Z.pid_P = pid_param_tmp->pid_P;
   reset_param_pos_Z.pid_I = pid_param_tmp->pid_I;
   reset_param_pos_Z.pid_D = pid_param_tmp->pid_D;
   reset_param_pos_Z.pid_Imax = pid_param_tmp->pid_Imax;



   pid_param_tmp = pid_param->rate_pid_Z;
   default_param_tmp = &default_param_rate;
   if ( !(pid_param_tmp->pid_P = ros::param::param("FIRST/RATE/Z/PID_P", pid_param_tmp->pid_P))  )
      ros::param::set("FIRST/RATE/Z/PID_P", pid_param_tmp->pid_P = default_param_tmp->pid_P);
   if ( !(pid_param_tmp->pid_I = ros::param::param("FIRST/RATE/Z/PID_I", pid_param_tmp->pid_I)) )
      ros::param::set("FIRST/RATE/Z/PID_I", pid_param_tmp->pid_I = default_param_tmp->pid_I);
   if ( !(pid_param_tmp->pid_D = ros::param::param("FIRST/RATE/Z/PID_D", pid_param_tmp->pid_D)) )
      ros::param::set("FIRST/RATE/Z/PID_D", pid_param_tmp->pid_D = default_param_tmp->pid_D);
   if ( !(pid_param_tmp->pid_Imax = ros::param::param("FIRST/RATE/Z/PID_IMAX", pid_param_tmp->pid_Imax)) )
      ros::param::set("FIRST/RATE/Z/PID_IMAX", pid_param_tmp->pid_Imax = default_param_tmp->pid_Imax);

   reset_param_rate_Z.pid_P = pid_param_tmp->pid_P;
   reset_param_rate_Z.pid_I = pid_param_tmp->pid_I;
   reset_param_rate_Z.pid_D = pid_param_tmp->pid_D;
   reset_param_rate_Z.pid_Imax = pid_param_tmp->pid_Imax;
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

   ros::param::del("FIRST/POS/Y/PID_P");
   ros::param::del("FIRST/POS/Y/PID_I");
   ros::param::del("FIRST/POS/Y/PID_D");
   ros::param::del("FIRST/POS/Y/PID_IMAX");
   ros::param::del("FIRST/RATE/Y/PID_P");
   ros::param::del("FIRST/RATE/Y/PID_I");
   ros::param::del("FIRST/RATE/Y/PID_D");
   ros::param::del("FFIRST/RATE/Y/PID_IMAX");

   ros::param::del("FIRST/POS/Z/PID_P");
   ros::param::del("FIRST/POS/Z/PID_I");
   ros::param::del("FIRST/POS/Z/PID_D");
   ros::param::del("FIRST/POS/Z/PID_IMAX");
   ros::param::del("FIRST/RATE/Z/PID_P");
   ros::param::del("FIRST/RATE/Z/PID_I");
   ros::param::del("FIRST/RATE/Z/PID_D");
   ros::param::del("FFIRST/RATE/Z/PID_IMAX");
}