#ifndef _PARAM_H
#define _PARAM_H

#include "ros/ros.h"
#define LOAD_PARAMFILE 9999

struct pid_parameter_t {
public:
	pid_parameter_t() :
		pid_P(0.0l),
		pid_I(0.0l),
		pid_D(0.0l),
		pid_Imax(0.0l)
	{}
	pid_parameter_t(double p, double i, double d, double im) :
		pid_P(p),
		pid_I(i),
		pid_D(d),
		pid_Imax(im)
	{}
	pid_parameter_t(pid_parameter_t& param) :
		pid_P(param.pid_P),
		pid_I(param.pid_I),
		pid_D(param.pid_D),
		pid_Imax(param.pid_Imax)
	{}

	double pid_P;
	double pid_I;
	double pid_D;
	double pid_Imax;
};




struct pos_pid_parameter_t {
public:
	pos_pid_parameter_t(pid_parameter_t *a, pid_parameter_t *b, pid_parameter_t *c, pid_parameter_t *d, pid_parameter_t *e, pid_parameter_t *f,
	                    pid_parameter_t *g, pid_parameter_t *h, pid_parameter_t *i, pid_parameter_t *j, pid_parameter_t *k, pid_parameter_t *l
	                   ):
		pos_pid_X(a),
		rate_pid_X(b),
		pos_pid_Y(c),
		rate_pid_Y(d),
		pos_pid_Z(e),
		rate_pid_Z(f),
		pos_nav_pid_X(g),
		rate_nav_pid_X(h),
		pos_nav_pid_Y(i),
		rate_nav_pid_Y(j),
		pos_nav_pid_Z(k),
		rate_nav_pid_Z(l)
	{}
	pos_pid_parameter_t():
		pos_pid_X(NULL),
		rate_pid_X(NULL),
		pos_pid_Y(NULL),
		rate_pid_Y(NULL),
		pos_pid_Z(NULL),
		rate_pid_Z(NULL),
		pos_nav_pid_X(NULL),
		rate_nav_pid_X(NULL),
		pos_nav_pid_Y(NULL),
		rate_nav_pid_Y(NULL),
		pos_nav_pid_Z(NULL),
		rate_nav_pid_Z(NULL)
	{}
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
};


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
double *get_param_n(int, pos_pid_parameter_t *);

double *get_default_param_n(int );
double *get_default_param_n(int , int, int);
double *get_default_param_n(int , int, int, int);


void set_param_n(int , double);
void set_param_n(int , int , int , double);
void set_param_n(int, int, int, int, double);



void save_param(const char*);
void save_param(const char*, pos_pid_parameter_t *);
int load_param(const char*);
int load_param(const char*, pos_pid_parameter_t *);
void delete_file_param();


extern char param_list[][50];
extern pos_pid_parameter_t pid_param;
extern pos_pid_parameter_t pid_default_param;

extern pid_parameter_t __pid_poshold_pos_param_X;
extern pid_parameter_t __pid_poshold_rate_param_X;
extern pid_parameter_t __pid_poshold_pos_param_Y;
extern pid_parameter_t __pid_poshold_rate_param_Y;
extern pid_parameter_t __pid_poshold_pos_param_Z;
extern pid_parameter_t __pid_poshold_rate_param_Z;


extern pid_parameter_t __pid_nav_pos_param_X;
extern pid_parameter_t __pid_nav_rate_param_X;
extern pid_parameter_t __pid_nav_pos_param_Y;
extern pid_parameter_t __pid_nav_rate_param_Y;
extern pid_parameter_t __pid_nav_pos_param_Z;
extern pid_parameter_t __pid_nav_rate_param_Z;


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