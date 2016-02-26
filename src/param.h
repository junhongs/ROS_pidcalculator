#ifndef _PARAM_H
#define _PARAM_H

#include "ros/ros.h"


typedef struct pid_parameter_t {
	double pid_P;
	double pid_I;
	double pid_D;
	double pid_Imax;
} pid_parameter_t;

typedef struct pos_pid_parameter_t {
	pid_parameter_t *pos_pid_X;
	pid_parameter_t *rate_pid_X;

	pid_parameter_t *pos_pid_Y;
	pid_parameter_t *rate_pid_Y;

	pid_parameter_t *pos_pid_Z;
	pid_parameter_t *rate_pid_Z;


	pid_parameter_t *pos_nav_pid_X;
	pid_parameter_t *rate_nav_pid_X;

	pid_parameter_t *pos_nav_pid_Y;
	pid_parameter_t *rate_nav_pid_Y;

	pid_parameter_t *pos_nav_pid_Z;
	pid_parameter_t *rate_nav_pid_Z;
} pos_pid_parameter_t;


void set_param(pos_pid_parameter_t *pid_param);


void reset_param(void);

void delete_param(void);


void update_param(int);
void update_param();

void init_param(std::string&, double *, double *);
void init_param();

int get_param_num(int , int, int);
int get_param_num(int , int, int, int);
double *get_param_n(int );
double *get_param_n(int , int , int);
double *get_param_n(int, int, int, int);

double *get_default_param_n(int );
double *get_default_param_n(int , int, int);
double *get_default_param_n(int , int, int, int);


void set_param_n(int , double);
void set_param_n(int , int , int , double);
void set_param_n(int, int, int, int, double);

extern char param_list[][50];
extern pos_pid_parameter_t pid_param;
extern pos_pid_parameter_t pid_default_param;

extern pid_parameter_t pid_poshold_pos_param_X;
extern pid_parameter_t pid_poshold_rate_param_X;
extern pid_parameter_t pid_poshold_pos_param_Y;
extern pid_parameter_t pid_poshold_rate_param_Y;
extern pid_parameter_t pid_poshold_pos_param_Z;
extern pid_parameter_t pid_poshold_rate_param_Z;


extern pid_parameter_t pid_nav_pos_param_X;
extern pid_parameter_t pid_nav_rate_param_X;
extern pid_parameter_t pid_nav_pos_param_Y;
extern pid_parameter_t pid_nav_rate_param_Y;
extern pid_parameter_t pid_nav_pos_param_Z;
extern pid_parameter_t pid_nav_rate_param_Z;


extern pid_parameter_t default_param_pos_X;
extern pid_parameter_t default_param_rate_X;
extern pid_parameter_t default_param_pos_Y;
extern pid_parameter_t default_param_rate_Y;
extern pid_parameter_t default_param_pos_Z;
extern pid_parameter_t default_param_rate_Z;

extern pid_parameter_t default_nav_param_pos_X;
extern pid_parameter_t default_nav_param_rate_X;
extern pid_parameter_t default_nav_param_pos_Y;
extern pid_parameter_t default_nav_param_rate_Y;
extern pid_parameter_t default_nav_param_pos_Z;
extern pid_parameter_t default_nav_param_rate_Z;

#endif