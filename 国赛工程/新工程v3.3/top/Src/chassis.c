#include "chassis.h"
#include "ammo.h"
#include "pid.h"
#include "gpio.h"
#include "iic.h"
#include "t_monitor.h"
#include "m_remote.h"
#include "m_imu.h"
#include <math.h>

/*底盘功能函数文件*/

Chassis_t Chassis;

uint8_t ammo_stop_flag = 0;

uint8_t w_key_status = 0, w_key_status_last = 0;

void ChasssisChangeDirection(float theta);
void ChasssisChangeSpeed(float speed);

void Chassis_Init(float direction, float speed_base)
{
	Chassis.distance_forward = 330;
	Chassis.direction = direction;
	/*direction为底盘正方向，顺时针*/
	Chassis.lockflag = 0;
	Chassis.speed.base = speed_base;
	Chassis.mode.now = MOVE_REMOTE;
	Chassis.mode.last = Chassis.mode.now;
	Chassis.PID_yaw.PidParaInit = &PidParaInit;
	Chassis.PID_yaw.PidParaInit(&Chassis.PID_yaw);
	Chassis.PID_yaw.PidSet_pidf(&Chassis.PID_yaw, 60, 0, 0, 0);
	Chassis.PID_yaw.PidSet_limit(&Chassis.PID_yaw, 7000, 2000, -2000, 0);
}

/*模式切换*/
void ChassisMotion(uint8_t mode, float direction, float speed);
/*底盘运动函数*/
void ChassisSetSpeed(void);
void ClearTotalRound(Moto_t *moto);
void ChassisSetAngle(float theta, float distance_forward, float distance_right, float distance_yaw, uint8_t state);
void ChassisAllSet(void);
/*底盘视野切换函数*/
void PulseSignal(uint8_t trigger);
void ChassisChangeVersion(void);
/*辅助功能*/
TUBE_State PhotoelectricTubeDetect(void);
void SemiautoAmmoCapture(RC_Ctl_t rc_data);

#ifdef CHASSIS_TEST
	void ChassisAngleDetect(void);
#endif

#ifdef MEA_DIS
	float GetMeanDistance(void);
	void OneCountForward(RC_Ctl_t rc_data);
	void DistanceMeasure(RC_Ctl_t rc_data);
#endif

void ChassisFun(void){
	ChassisMotion(Chassis.mode.now, Chassis.speed.base, Chassis.direction);
	ChassisChangeVersion();
	GenerallySetChassisSpeed(RC_CtrlData, &Chassis);
	#ifdef MEA_DIS
		DistanceMeasure(RC_CtrlData);
		OneCountForward();
		ChassisSetSpeed();
	#else
		ChassisAllSet();
		Chassis.tube_state = PhotoelectricTubeDetect();
	#endif
	#ifdef CHASSIS_TEST
		ChassisAngleDetect();
	#endif
	PulseSignal(0);
	SemiautoAmmoCapture(RC_CtrlData);
}

/*模式切换*/

//  MOVE_REMOTE = 0,
//  MOVE_SE_CAPTURE = 1,
//	MOVE_AU_CAPTURE = 2,
//	MOVE_TRAIL = 3,
//	MOVE_APPLY = 4,

void ChassisMotion(uint8_t mode, float speed, float direction){
	switch(mode){
		/*前部模式有高低速区别*/
		case MOVE_REMOTE:{
			Chassis.direction = FORENT;
		}break;
		/*后部模式有高低速区别*/
		case MOVE_TRAIL:{
			Chassis.direction = BACK;
		}break;
		case MOVE_APPLY:{
			Chassis.direction = FORENT;
		}break;
		case MOVE_SE_CAPTURE:{
			Chassis.speed.base = LOWSPEED;
			Chassis.direction = LEFT;
		}break;
		case MOVE_AU_CAPTURE:{
			Chassis.speed.base = LOWSPEED;
			Chassis.direction = LEFT;
		}break;	
	}
}

/********底盘运动函数开始********/
#ifdef CHASSIS_TEST
void ChassisAngleDetect(void){
	static float ChassisUpAngle;
	static uint8_t init_sta = 0, init_cnt = 0;
	if(init_sta == 0){
		init_cnt++;
		if(init_cnt == 150){
			init_sta = 1;
			ChassisUpAngle = imu_data.az;
		}
	}
	else{
		ChassisUpAngle = 0.5f * ChassisUpAngle + 0.5f * imu_data.az;
		Chassis.angle.inclination_angle_now = ChassisUpAngle;
	}
	if(ChassisUpAngle > ANGLE_CLIMB){
		Chassis.angle.state_cnt	=	Chassis.angle.state_cnt > 200 ? 200 : Chassis.angle.state_cnt + 1;
	}
	else{ 
		if(Chassis.angle.state_cnt) Chassis.angle.state_cnt--;
	}
	if(Chassis.angle.state_cnt > 100){
		Chassis.angle.state = MOVE_CLIMB;
	}
	else if(Chassis.angle.state_cnt < 40){
		Chassis.angle.state = MOVE_ACLINIC;
	}
}

#endif

/*引入陀螺仪使底盘运动保持直线*/
void ChassisFollow(void){
	Chassis.speed.spd_yaw_gyro = imu_data.gz/16.384f;
	if(fabs(Chassis.speed.spd_yaw_gyro+Chassis.speed.spd_yaw_dps)>1.5f)
		Chassis.speed.spd_yaw_correct = PidCalc(&Chassis.PID_yaw, -Chassis.speed.spd_yaw_gyro, Chassis.speed.spd_yaw_dps);
	else
		Chassis.speed.spd_yaw_correct=0;
}

//static float PositiveWheel_speed_delay(float speed_forward){
//	static float last_speed=0;
//	if(last_speed>0)
//		speed_forward = last_speed + (speed_forward-last_speed)/100.0f;
//	last_speed = speed_forward;
//	return speed_forward;
//}

//static float NegativeWheel_speed_delay(float speed_forward){
//	static float last_speed=0;
//	if(last_speed<0)
//		speed_forward = last_speed + (speed_forward-last_speed)/100.0f;
//	
//	last_speed = speed_forward;
//	return speed_forward;
//}

void ChassisSetSpeed(void) 
{
	float theta = Chassis.direction;
	static float spd_forward, spd_right;
  for(uint8_t i = 0; i<6; i++) Chassis_MOTO[i].send_current_last = Chassis_MOTO[i].send_current;
	
	if(monitor_remote.status == monitor_regular)
	{	
//		spd_forward = Chassis.speed.spd_forward * 0.7f + 0.3f * spd_forward; 
//		spd_right = Chassis.speed.spd_right * 0.5f + 0.5f * spd_right;
		spd_forward = Chassis.speed.spd_forward; 
		spd_right = Chassis.speed.spd_right;		
		float ksin=0,kcos=0,W_sin=0,W_cos=0,D_sin=0,D_cos=0,calc_set_spd[7]={0,0,0,0,0,0,1};
		ksin = (float)sin(theta*3.1416f/180.0f);
		kcos = (float)cos(theta*3.1416f/180.0f);
		W_sin = spd_forward*ksin;
		W_cos = spd_forward*kcos;
		D_sin = spd_right*ksin;
		D_cos = spd_right*kcos;
			
		//速度分配矩阵
		#ifdef CHASSIS_TEST
//			calc_set_spd[0] = ( (W_cos-D_sin) + (W_sin+D_cos) + Chassis.speed.spd_yaw + Chassis.speed.spd_yaw_correct);
//			calc_set_spd[1] = (-(W_cos-D_sin) + (W_sin+D_cos) + Chassis.speed.spd_yaw + Chassis.speed.spd_yaw_correct);
//			calc_set_spd[2] = ( (W_cos-D_sin) - (W_sin+D_cos) + Chassis.speed.spd_yaw + Chassis.speed.spd_yaw_correct);
//			calc_set_spd[3] = (-(W_cos-D_sin) - (W_sin+D_cos) + Chassis.speed.spd_yaw + Chassis.speed.spd_yaw_correct);
//			calc_set_spd[4] =	( (W_cos-D_sin) + 0.77f*(Chassis.speed.spd_yaw + Chassis.speed.spd_yaw_correct));
//			calc_set_spd[5] = (-(W_cos-D_sin) + 0.77f*(Chassis.speed.spd_yaw + Chassis.speed.spd_yaw_correct));
		#else
			calc_set_spd[0] = ( (W_cos-D_sin) + (W_sin+D_cos));
			calc_set_spd[1] = (-(W_cos-D_sin) + (W_sin+D_cos));
			calc_set_spd[2] = ( (W_cos-D_sin) - (W_sin+D_cos));
			calc_set_spd[3] = (-(W_cos-D_sin) - (W_sin+D_cos));
			calc_set_spd[4] =	( (W_cos-D_sin));
			calc_set_spd[5] = (-(W_cos-D_sin));
		#endif

		//底盘各电机期望速度设定
//		for(int i=0;i<6;i++){
//			calc_set_spd[6] = calc_set_spd[6] > calc_set_spd[i] ? calc_set_spd[6] : calc_set_spd[i];
//		}
//		float coefficient[6] = {1,1,1,1,1,1};
		#ifdef CHASSIS_TEST
//			if(Chassis.angle.state == MOVE_CLIMB){
//				if(Chassis.angle.inclination_angle_now < Chassis.angle.inclination_angle_last){
//					for(uint8_t i = 0; i<4; i++) coefficient[i] *= tan(Chassis.angle.inclination_angle_now);
//				}
//				for(uint8_t i = 0; i<3; i++){
//					float unsafe_spd_coefficient = 1;
//					uint8_t unsafe_spd_num = 0;
//					if((fabs(ABS(Chassis_MOTO[i].getpara.speed_rpm) - ABS(Chassis_MOTO[i+1].getpara.speed_rpm))) > 5000.0f){
//						unsafe_spd_coefficient = ABS(Chassis_MOTO[i].getpara.speed_rpm)/ABS(Chassis_MOTO[i+1].getpara.speed_rpm);
//						unsafe_spd_coefficient = unsafe_spd_coefficient < 1.0f ? unsafe_spd_coefficient : 1.0f/unsafe_spd_coefficient;
//						unsafe_spd_num = unsafe_spd_coefficient < 1 ? i+1 : i;
//						coefficient[unsafe_spd_num] *= unsafe_spd_coefficient;
//					}
//				}
//				Chassis.angle.inclination_angle_last = Chassis.angle.inclination_angle_now;
//			}
		#endif
		for(int i=0;i<4;i++){	
			Chassis_MOTO[i].set_speed = Chassis.speed.base * calc_set_spd[i] + Chassis.speed.spd_yaw_correct;
		}
		for(int i=4;i<6;i++){	
			Chassis_MOTO[i].set_speed = Chassis.speed.base * calc_set_spd[i] + 0.77*Chassis.speed.spd_yaw_correct;
		}		
		
//		for(int i=0;i<4;i++){	
//			Chassis_MOTO[i].set_speed = Chassis.speed.base * calc_set_spd[i] ;
//		}
//		for(int i=4;i<6;i++){	
//			Chassis_MOTO[i].set_speed = Chassis.speed.base * calc_set_spd[i] ;
//		}		
	}
	else
	{
		for(int i=0;i<6;i++){	
			Chassis_MOTO[i].set_speed = 0;
		}
		spd_forward = 0;
		spd_right = 0;
		Chassis.speed.spd_yaw_correct = 0;
	}		
	/*速度环_&_电机输出信号赋值*/
	for(int i=0; i<6; i++)
	{
		if(spd_forward == 0 && spd_right == 0){
			Chassis_MOTO[i].send_current = Chassis_MOTO[i].pid_speed.PidCalc(&Chassis_MOTO[i].pid_speed, Chassis_MOTO[i].getpara.speed_rpm, Chassis_MOTO[i].set_speed);	  
		}
		else{
			Chassis_MOTO[i].send_current = Chassis_MOTO[i].pid_speed.PidCalc(&Chassis_MOTO[i].pid_speed, Chassis_MOTO[i].getpara.speed_rpm, Chassis_MOTO[i].set_speed);	  
			/*限制电流变化大小*/
			if(fabs(Chassis_MOTO[i].send_current - Chassis_MOTO[i].send_current_last) > 1800.0f){
				if(Chassis_MOTO[i].send_current > Chassis_MOTO[i].send_current_last){
					Chassis_MOTO[i].send_current = Chassis_MOTO[i].send_current_last + 1800.0f;
				}
				else if(Chassis_MOTO[i].send_current < Chassis_MOTO[i].send_current_last){
					Chassis_MOTO[i].send_current = Chassis_MOTO[i].send_current_last - 1800.0f;
				}
			}
		}
	}	
}

void ClearTotalRound(Moto_t *moto){
	moto->getpara.offset_round = (float)(moto->getpara.angle)/8192.0f;
	moto->getpara.round_cnt = 0;
	moto->getpara.total_round = 0;
}

void ChassisSetAngle(float theta, float distance_forward, float distance_right, float distance_yaw, uint8_t state) 
{
	//theta>0为右转
	static float angle_forward = 0, angle_right = 0, angle_yaw = 0;
	
	float ksin=0, kcos=0;
	ksin = (float)sin(theta*3.1416f/180.0f);
	kcos = (float)cos(theta*3.1416f/180.0f);
	
	static uint8_t stage_start_last = 0;
	static float distance_forward_last = 0,distance_right_last = 0,distance_yaw_last = 0;
	
	//distance单位mm
	if(!stage_start_last && state){
		for(uint8_t i=0; i<6; i++)	ClearTotalRound(&Chassis_MOTO[i]);
		angle_forward = 0;
		if(distance_forward>10 && ABS(distance_forward)<1000){
			distance_forward_last = distance_forward/BaseFrontDistance;
		}else{
			distance_forward_last = 0;
		}
		angle_right = 0;
		if(distance_right>10 &&ABS(distance_right)<1000){
			distance_right_last = distance_right/BaseRightDistance;
		}else{
			distance_right_last = 0;
		}
		angle_yaw = 0;
		if(ABS(distance_yaw)>5 &&ABS(distance_yaw)<1000){
			distance_yaw_last = distance_yaw/BaseYawAngle;
		}else{
			distance_yaw_last = 0;
		}
	}

	float W_sin=0,W_cos=0,D_sin=0,D_cos=0,calc_set_angle[6]={0};
	angle_forward = angle_forward*0.9f + distance_forward_last*0.1f;
	angle_right		= angle_right*0.8f + distance_right_last*0.2f;
	angle_yaw			= angle_yaw*0.8f + distance_yaw_last*0.2f;
	
	W_sin = angle_forward*ksin;
	W_cos = angle_forward*kcos;
	D_sin = angle_right*ksin;
	D_cos = angle_right*kcos;
	//角度分配矩阵	
	calc_set_angle[0] = ( (W_cos-D_sin) + (W_sin+D_cos) + angle_yaw);
	calc_set_angle[1] = (-(W_cos-D_sin) + (W_sin+D_cos) + angle_yaw);
	calc_set_angle[2] = ( (W_cos-D_sin) - (W_sin+D_cos) + angle_yaw);
	calc_set_angle[3] = (-(W_cos-D_sin) - (W_sin+D_cos) + angle_yaw);
	calc_set_angle[4] =	( (W_cos-D_sin) + 0.77f*angle_yaw);
	calc_set_angle[5] = (-(W_cos-D_sin) + 0.77f*angle_yaw);
		
		/*位置环_&_底盘各电机期望速度设定*/
		Chassis_MOTO[0].set_angle = BaseAngle * calc_set_angle[0];
		Chassis_MOTO[0].set_speed = 8000.0f * PidCalc(&Chassis_MOTO[0].pid_angle, Chassis_MOTO[0].getpara.total_round*19.35f, Chassis_MOTO[0].set_angle);
		/*速度环_&_电机输出信号赋值*/
//		if(Chassis_MOTO[0].set_speed > 3000){
//			Chassis_MOTO[0].set_speed = 3000;
//		}

	for(uint8_t i = 0; i<3; i++){
		Chassis_MOTO[2*i].send_current = PidCalc(&Chassis_MOTO[2*i].pid_speed, Chassis_MOTO[2*i].getpara.speed_rpm, Chassis_MOTO[0].set_speed);	  
		Chassis_MOTO[2*i+1].send_current = PidCalc(&Chassis_MOTO[2*i+1].pid_speed, Chassis_MOTO[2*i+1].getpara.speed_rpm, -Chassis_MOTO[0].set_speed);	  
	}

//		Chassis_MOTO[0].send_current = PidCalc(&Chassis_MOTO[0].pid_speed, Chassis_MOTO[0].getpara.speed_rpm, Chassis_MOTO[0].set_speed);	  
//		Chassis_MOTO[2].send_current = Chassis_MOTO[0].send_current;
//		Chassis_MOTO[4].send_current = Chassis_MOTO[0].send_current;
//		for(uint8_t i = 0; i<3; i++){
//			Chassis_MOTO[2*i + 1].send_current = -Chassis_MOTO[0].send_current;
//		}
	stage_start_last = state;
}



void ChassisAllSet(void){
	static uint8_t switch_cnt = 150, ac_enable = 0;
	if(switch_cnt != 150){
		if(switch_cnt == 1){
			for(uint8_t i = 0; i<6; i++){
				Chassis_MOTO[i].pid_speed.PidSet_limit(&Chassis_MOTO[i].pid_speed, 6000, 0, 0, 0);
				Chassis_MOTO[i].pid_speed.PidSet_pidf(&Chassis_MOTO[i].pid_speed, 0.5, 0, 0, 0);
			}
		}
		ChassisSetAngle(0, Chassis.distance_forward, Chassis.distance_right, Chassis.distance_yaw, ac_enable);
		if(switch_cnt == 149){
			for(uint8_t i = 0; i < 4; i++){ 
				Chassis_MOTO[i].pid_speed.PidSet_all(&Chassis_MOTO[i].pid_speed, POSITION_PID, CHASSISMAXCURRENT, 0, 0, 1.8, 0, 0, 0, 0, 0, 0, 0, 500); 
			}
			Chassis_MOTO[4].pid_speed.PidSet_all(&Chassis_MOTO[4].pid_speed, POSITION_PID, CHASSISMAXCURRENT, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0);
			Chassis_MOTO[5].pid_speed.PidSet_all(&Chassis_MOTO[5].pid_speed, POSITION_PID, CHASSISMAXCURRENT, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0);	
		}
		switch_cnt++;
	}
	else{
		/*计时切换*/
		if(switch_cnt == 150){
			if(Ammo.auto_enable == 1){
				switch_cnt = 1;
				ac_enable = 1;
			}
			else{
				ChassisFollow();
				ChassisSetSpeed();	
				ac_enable = 0;
			}
		}
	}
}

/********底盘运动函数结束********/

/********底盘视角切换函数********/
void PulseSignal(uint8_t trigger){
	static uint32_t count = 0;
	static uint8_t v_key_last = 0, enable = 0;
//	static uint16_t press_timer = 0;
	if((RC_CtrlData.key.key_data.V == 1 || trigger)&& !v_key_last)
	{	enable = 1; }
//	if(RC_CtrlData.key.key_data.V == 1) press_timer++;
//	else press_timer = 0;
//	if(press_timer > 150){ Chassis.video.now = 0; }
	/*初始化图像为取弹视角*/
	if(enable == 1){
		if(count%2 == 0){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		}
		if(count%2 == 1){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		}
		count++;
		enable = 0;
	}
	v_key_last = (RC_CtrlData.key.key_data.V || trigger);
}

float set_compare = MID;

void VersionChangeSlowly(float version){
	static float version_last, version_now;
	version_now = version;
	/*限制舵机旋转速度*/
	if(fabs(version_now - version_last)>100)
	{
		if(version_now >version_last)
			version_now=version_now-100;
		else
			version_now=version_now+100;
	}
	else if(fabs(version_now - version_last) > 10){
		if(version_now > version_last)
			version_now = version_last + 10.0f;
		else
			version_now = version_last - 10.0f;
	}		
	else
	{
		version_now = version;
	}
		
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, version_now);
	version_last = version_now;
}

void ChassisChangeVersion(void){
	if(Chassis.mode.now == MOVE_REMOTE){
		Chassis.version = MID;
	}
	else if(Chassis.mode.now == MOVE_APPLY){
		Chassis.version = DOWN;
	}
	else if(Chassis.direction == BACK || Chassis.direction == LEFT){
		Chassis.version = DOWN;
		/*video 0 上 1 后*/
		if(Chassis.mode.now == 1 || Chassis.mode.now == 2) Chassis.video.now = 0;
		else Chassis.video.now = 1;
		if(Chassis.video.last != Chassis.video.now){
			PulseSignal(1);
		}
	}
	Chassis.video.last = Chassis.video.now;
//	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, Chassis.version);
	VersionChangeSlowly(Chassis.version);
}

/*补弹辅助函数*/
TUBE_State PhotoelectricTubeDetect(void)
{
	TUBE_State tubestatus;
	static uint8_t count = 0;
	if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)) != 0)
	{
		count++;
		if(count > 10){
			tubestatus = TUBE_ALIGNED;
			count = 10;
		}
	}
	else{ 
		tubestatus =  TUBE_UNALIGNED;
		count = 0;
	}
	return tubestatus;
}

void SemiautoAmmoCapture(RC_Ctl_t rc_data)
{
	static uint8_t z_key_status = 0, z_key_status_last = 0;
	static uint8_t move_status = 0, move_status_last = 0;
	static uint8_t motion_switch = 0;
	static uint8_t press_count = 0;
	static uint8_t t_enable = 0;
	z_key_status = rc_data.key.key_data.Z;
	if(!z_key_status_last && z_key_status){
		if(press_count == 0){
			Ammo.Raising.lockheight = 1;
			Chassis.mode.now = 1;
			Chassis.semi_enable = 1;
			press_count = 1;
		}
		else if(press_count == 1){
			Ammo.Raising.lockheight = 0;
			Chassis.mode.now = 0;
			Chassis.lockflag = 0;
			Chassis.semi_enable = 0;
			press_count = 0;
		}
	}
//	/*辅助对齐功能开启*/
//	if(Chassis.semi_enable == 1 && Chassis.mode.now == 1){
//		/*上升沿捕获*/
//		if(rc_data.rc.ch2 != 1024)
//			{ move_status = UNSTOP; }
//		else{ move_status = STOP; }
//		if(Chassis.tube_state == TUBE_ALIGNED){
//			/*再触发启动*/
//			if(move_status_last == STOP && move_status == UNSTOP)
//				{ Chassis.lockflag = 0; motion_switch = 1; }
//			/*强制停止*/
//			else if(motion_switch == 0)
//				{ Chassis.speed.spd_right = 0;  Chassis.speed.spd_right = 0; Chassis.lockflag = 1;}
//		}
//		else
//			{	motion_switch = 0;}
//	}
//	/*辅助对齐功能关闭*/
//	else { motion_switch = 0; }
	w_key_status_last = w_key_status;
	z_key_status_last = z_key_status;
	move_status_last = move_status;
}

#ifdef MEA_DIS
uint8_t lockflag[8] = {0};
float distance_mem[6];
float distance_buffer = 0;
float distance_result = 0;

float GetMeanDistance(void){
	float buffer = 0, result = 0;
	for(uint8_t i = 0; i<6; i++){
		buffer += ABS(Chassis_MOTO[i].getpara.total_round);
	}
	result = buffer/6.0f;
	return result;
}

void OneCountForward(RC_Ctl_t rc_data){
	static uint8_t r_key_status_last = 0;	
	static uint8_t test_enable = 0;
	static uint8_t count = 0;
	if(rc_data.key.key_data.R&&!r_key_status_last){
		test_enable = 1;
		Chassis.lockflag = 1;
	}
	if(test_enable == 1){
		if(count == 0) for(uint8_t i = 0; i<6; i++) ClearTotalRound(&Chassis_MOTO[i]);
		count++;
		if(count < 150){
			for(uint8_t i = 0; i<6; i++){		
			/*位置环_&_底盘各电机期望速度设定*/
			Chassis_MOTO[i].set_angle = BaseAngle;
			Chassis_MOTO[i].pid_angle.max_output = 1;
			Chassis_MOTO[i].set_speed = 0.3f*Chassis.speed.base * PidCalc(&Chassis_MOTO[i].pid_angle, Chassis_MOTO[i].getpara.total_round*19.35f, Chassis_MOTO[i].set_angle);
			/*速度环_&_电机输出信号赋值*/
			Chassis_MOTO[i].send_current = PidCalc(&Chassis_MOTO[i].pid_speed, Chassis_MOTO[i].getpara.speed_rpm, Chassis_MOTO[i].set_speed);	  
			}
 		}
		else{ count = 0; test_enable = 0; Chassis.lockflag = 0; }
	}
	r_key_status_last = rc_data.key.key_data.R;
}

void DistanceMeasure(RC_Ctl_t rc_data){
	static uint8_t e_key_status = 0, e_key_status_last = 0;	
	static uint8_t press_count = 0;
	static uint8_t mea_begin = 0, mea_count = 0;
	static uint8_t timer = 0;
	e_key_status = rc_data.key.key_data.E;
	if(e_key_status && !e_key_status_last){
		if(press_count == 0){
			Ammo.Raising.lockheight = 1;
			Chassis.lockflag = 1;
			mea_begin = 1;
			press_count = 1;
		}
		else{
			Ammo.Raising.lockheight = 0;
			Chassis.lockflag = 0;
			mea_begin = 0;
			press_count = 0;			
		}
	}
	if(mea_begin != 0){
		if(PhotoelectricTubeDetect()&&mea_count<8){
			mea_count++;
		}
		switch(mea_count){ 
			case 0:
				Chassis.speed.spd_forward = 0.1;
			break;
			case 1:
				if(lockflag[mea_count - 1] != 1){
					Chassis.speed.spd_forward = 0;
					if(timer == 100) { timer = 0; lockflag[mea_count - 1] = 1; for(uint8_t i=0; i<6; i++)	ClearTotalRound(&Chassis_MOTO[i]);}
					else timer++;
				}
				else{ Chassis.speed.spd_forward = 0.1; }
			break;
			case 7:
				Chassis.speed.spd_forward = 0;
				distance_mem[mea_count - 2] = GetMeanDistance();
				for(uint8_t i = 0; i<8; i++) lockflag[i] = 0;
				for(uint8_t k = 0; k<6; k++) {distance_buffer += distance_mem[k]; distance_mem[k] = 0;}
				distance_result = distance_buffer/6.0f;
				Ammo.Raising.lockheight = 0;
				Chassis.lockflag = 0;
				mea_begin = 0;
				mea_count = 0;
				press_count = 0;
			break;	
			default:
				if(lockflag[mea_count - 1] != 1){
					Chassis.speed.spd_forward = 0;
					distance_mem[mea_count - 2] = GetMeanDistance();
					if(timer == 100) { timer = 0; lockflag[mea_count - 1] = 1; for(uint8_t i=0; i<6; i++)	ClearTotalRound(&Chassis_MOTO[i]);}
					else timer++;
				}
				else{ 
					if(mea_count%2 == 0){
						Chassis.speed.spd_forward = -0.1f*(0.5f + 0.25f*mea_count);
					}
					else{
						Chassis.speed.spd_forward = 0.075f + 0.025f*mea_count;
					}
				}
			break;
		}
	}
	e_key_status_last = e_key_status;
}
#endif
