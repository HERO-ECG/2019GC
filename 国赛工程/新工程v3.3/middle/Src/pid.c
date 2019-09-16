/*******************************************************************************************
  * @Include @Headfile
 *******************************************************************************************/
#include "para.h"
#include "pid.h"
/*******************************************************************************************
  * @Parameter @Statement
 *******************************************************************************************/
 
/*******************************************************************************************
  * @Func		ABSLimit
  * @Brief 	范围限制
  * @Param	float *a, float abs_max
  * @Retval	None
  * @Date		2018.10.9
  *	@Author	LZK
 *******************************************************************************************/
void ABSLimit(float *a, float abs_max){
	if(*a > abs_max)
		*a = abs_max;
	if(*a < -abs_max)
		*a = -abs_max;
}

void PidSet_all(Pid_t *pid, uint32_t mode, 
								 float max_out, float integral_uplimit, float integral_downlimit, 
	               float p, float i, float d, 
								 float deadband, float f, float max_err, float mid_err, float min_err, float f_out_limit){
	pid->pid_mode = mode;
	pid->max_output = ABS(max_out);
	pid->integral_uplimit = ABS(integral_uplimit);
	pid->integral_downlimit = -ABS(integral_downlimit);									 
	pid->p = p;
	pid->i = i;
	pid->d = d;
	pid->deadband = ABS(deadband);
	pid->f = f;
  pid->max_err = ABS(max_err);
	pid->mid_err = ABS(mid_err);
	pid->min_err = ABS(min_err);
	pid->f_out_limit = ABS(f_out_limit);
}

void PidSet_pidf(Pid_t *pid, float p, float i, float d, float f){							 
	pid->p = p;
	pid->i = i;
	pid->d = d;
	pid->f = f;
}

void PidSet_errband(Pid_t *pid, float max_err, float mid_err, float min_err, float deadband){
  pid->max_err = ABS(max_err);
	pid->mid_err = ABS(mid_err);
	pid->min_err = ABS(min_err);
	pid->deadband = ABS(deadband);
}

void PidSet_limit(Pid_t *pid, float max_out, float integral_uplimit, float integral_downlimit, float f_out_limit){
	pid->max_output = ABS(max_out);
	pid->integral_uplimit = ABS(integral_uplimit);
	pid->integral_downlimit = -ABS(integral_downlimit);			
	pid->f_out_limit = ABS(f_out_limit);	
}

void PidChangeSet(Pid_t *pid, float set){
	pid->set = set;
}
/*******************************************************************************************
  * @Func		PIDCalc
  * @Brief 	PID参数计算
  * @Param	Pid_t* pid, float get, float set
  * @Retval	pid_output.pos_out/pid_output.delta_out
  * @Date		2018.10.10
  *	@Author	LZK
 *******************************************************************************************/
float PidCalc(Pid_t* pid, float get, float set){
	float i_max_err,i_mid_err,i_min_err,abs_err;
	i_max_err = ABS(pid->max_err);
	i_mid_err = ABS(pid->mid_err);
	i_min_err = ABS(pid->min_err);
	
	pid->set = set;
	pid->get.now = get;
	pid->err.now = set - get;				/*set - measure*/  
	abs_err = ABS(pid->err.now);
	if(abs_err < pid->deadband)		
		return pid->pid_mode==POSITION_PID ? pid->output.pos_out : pid->output.delta_out;	  //pid死区
	
	if(pid->pid_mode == POSITION_PID){ /*离散型位置式PID*/ 						
		/*前馈(速度环主用)*/
		if(ABS(set) > 10){
			pid->output.fout = pid->f * set;	
			ABSLimit(&(pid->output.fout),pid->f_out_limit);
		}
		else{ pid->output.fout /= 1.5f; }/*f的分离*/
		/*pout*/
		pid->output.pout = pid->p * pid->err.now;
		/*积分分离*/
		if(pid->max_err > pid->mid_err && pid->mid_err > pid->min_err){				
			if(abs_err > i_max_err)  {  pid->output.iout /= 2; }
			else if(abs_err > i_mid_err)  { pid->output.iout += (i_max_err-abs_err)/(i_max_err-i_mid_err) * pid->i * pid->err.now; }
			else if(abs_err > i_min_err)  { pid->output.iout += pid->i * pid->err.now; }
			//else  pid->output.iout *= 1; /*积分死区*/			
		}					
		else 	
			pid->output.iout += pid->i * pid->err.now;
		/*抗积分饱和*/
		if(pid->integral_uplimit>0 && pid->integral_downlimit<0){
			if(pid->output.iout > pid->integral_uplimit)
				pid->output.iout = pid->integral_uplimit;
			else if(pid->output.iout < pid->integral_downlimit)
				pid->output.iout = pid->integral_downlimit;
		}
		else pid->output.iout = 0;
		/*dout*/
		pid->output.dout = pid->d * (pid->err.now - pid->err.last); 
			
		pid->output.pos_out = pid->output.fout + pid->output.pout + pid->output.iout + pid->output.dout;
		ABSLimit(&(pid->output.pos_out), pid->max_output);	
		pid->output.last_pos_out = pid->output.pos_out;	
	}
	else if(pid->pid_mode == DELTA_PID){ /*增量式PID*/			
		pid->output.pout = pid->p * (pid->err.now - pid->err.last);
		pid->output.iout = pid->i * pid->err.now;
		pid->output.dout = pid->d * (pid->err.now - 2*pid->err.last + pid->err.beforelast);
		/*抗积分饱和*/
		if(pid->integral_uplimit > pid->integral_downlimit){
			if(pid->output.iout > pid->integral_uplimit)
				pid->output.iout = pid->integral_uplimit;
			else if(pid->output.iout < pid->integral_downlimit)
				pid->output.iout = pid->integral_downlimit;
		}
		else pid->output.iout = 0;
		pid->output.delta_u = pid->output.pout + pid->output.iout + pid->output.dout;
		pid->output.delta_out = pid->output.last_delta_out + pid->output.delta_u;
		ABSLimit(&(pid->output.delta_out), pid->max_output);
		pid->output.last_delta_out = pid->output.delta_out;
	}
	pid->err.beforelast = pid->err.last;
	pid->err.last = pid->err.now;
	pid->get.beforelast = pid->get.last;
	pid->get.last = pid->get.now;
	return pid->pid_mode==POSITION_PID ? pid->output.pos_out : pid->output.delta_out;	
}

void PidParaInit(Pid_t *pid){
	pid->PidParaInit = &PidParaInit;
	pid->PidSet_all = &PidSet_all;
	pid->PidSet_errband = &PidSet_errband;
	pid->PidSet_limit = &PidSet_limit;
	pid->PidSet_pidf = &PidSet_pidf;
	pid->PidCalc = &PidCalc; 
	pid->pid_mode = POSITION_PID;
}
