#ifndef __PID_H
#define __PID_H

#include "mytype.h"
/*******************************************************************************************
  * @define
 *******************************************************************************************/
#define ABS(X) ((X>0)? (X): (-X))

enum{
	POSITION_PID = 1,
	DELTA_PID = 2
};

/*******************************************************************************************
  * @Parameter
 *******************************************************************************************/
typedef struct __Fliter_t{
	int 	i;
	float 	value[6];
	float 	sum;
}Fliter_t;

typedef struct __Get_t{
	float 	beforelast;
	float 	last;
	float 	now;
}Get_t;

typedef struct __Err_t{
	float 	beforelast;
	float 	last;
	float 	now;
}Err_t;

typedef struct __Output_t{
	float 	fout;           /*前馈输出(速度环主用)*/
	float 	pout;						/*P输出*/	
	float 	iout;						/*I输出*/
	float 	dout;						/*D输出*/	
	float 	pos_out;				/*本次位置式输出*/
	float 	last_pos_out;		/*上次位置式输出*/
	float 	delta_u;				/*本次增量值*/
	float 	delta_out;			/*本次增量式输出 = last_delta_out + delta_u*/
	float 	last_delta_out;	/*上次增量式输出*/
}Output_t;

typedef struct __Pid_t{
	void (*PidParaInit)(struct __Pid_t *pid);
	/*需要自定义*/
	uint32_t 	 pid_mode;
	float 	 max_output;				/*输出限幅*/
	float 	 integral_uplimit;			/*积分正向抗饱和*/
	float 	 integral_downlimit;		/*积分负向抗饱和*/
	float   f_out_limit;
	float 	p;
	float 	i;
	float 	d;
	float 	f;        		/*(速度环主用)前馈系数*/
	float 	max_err;
	float 	mid_err;
	float 	min_err;
	float 	deadband;     	/*err < deadband return*/
	
	void (*PidSet_all)(struct __Pid_t *pid, uint32_t mode, 
		              float max_out, float integral_uplimit, float integral_downlimit,
									float p, float i, float d, 
								  float f, float max_err, float mid_err, float min_err, float deadband, float f_out_limit);
	void (*PidSet_pidf)(struct __Pid_t *pid, float p, float i, float d, float f);
  void (*PidSet_errband)(struct __Pid_t *pid, float max_err, float mid_err, float min_err, float deadband);
	void (*PidSet_limit)(struct __Pid_t *pid, float max_out, float integral_uplimit, float integral_downlimit, float f_out_limit);
	float (*PidCalc)(struct __Pid_t *pid, float get, float set);
	float up_k;
	float down_k;
	/*无需自定义*/							
	float 	set;					/*设定值*/
	Get_t 	get;					/*测量值*/
	Err_t 	err;					/*误差值*/
	Output_t 	output;			/*输出值*/
	
}Pid_t;

//extern Pid_t TG2006_pid_spd;
//extern Pid_t TG2006_pid_ang;
//extern Pid_t CM3508_pid_spd[4];
/*******************************************************************************************
  * @Function 
 *******************************************************************************************/
extern void ABSLimit(float *a, float abs_max);
extern void PidParaInit(Pid_t *pid);
extern void PidChangeSet(Pid_t *pid, float set);
extern float PidCalc(Pid_t *pid, float get, float set);

#endif
