#include "chassis.h"
#include "ammo.h"
#include "robodata.h"
#include "cylinder.h"
#include "ks103.h"
#include <math.h>

Ammo_t Ammo;

float test_cap = 41.0f;
float test_top = 52.0f;

/*取弹功能函数文件，包含升降电机控制和抓取电机控制*/

void AmmoInit(void){
	Ammo.auto_enable = 0;
	Ammo.Raising.height_now = HEIGHT_BOTTOM;
	Ammo.Raising.height_last = Ammo.Raising.height_now;
	Ammo.Raising.manual_raising_now = 0;
	Ammo.Raising.manual_raising_last = 0;
	Ammo.Raising.restitution_coefficient = 1.05;
	Ammo.Raising.height_change_cnt = 9.5f;
	Ammo.Capturing.autocap_enable = 0;
	Ammo.Capturing.position_now = IN;
	Ammo.Capturing.position_last = Ammo.Capturing.position_now;
	Ammo.Capturing.in_up = 40;
	Ammo.Capturing.throw_up = 50;
	Ammo.Capturing.f_set = 0.04;
//	Ammo.Capturing.f_set = 0;
	Capturing_MOTO[0].set_angle = TURN_IN;
}

void RiseHeightSet(void);
void ManualRaise(void);
void ManualGetAmmo(void);
void AutoGetAmmo(uint8_t key);
void CaptureMotoCtrl(float set, float set_f, float set_in_up, float set_throw_up);
void launch_one(uint8_t enable);
void AutoAmmo3(RC_Ctl_t rc_data);

float f_test = 0.04, throw_up = 50, in_up = 20;
float angle_test = 140.0f;

void AmmoFun(void){
	RiseHeightSet();
	ManualRaise();
	ManualGetAmmo();
	AutoGetAmmo(Ammo.Capturing.autocap_enable);
	AutoAmmo3(RC_CtrlData);
	CaptureMotoCtrl(Capturing_MOTO[0].set_angle, Ammo.Capturing.f_set, Ammo.Capturing.throw_up, Ammo.Capturing.in_up);
}

void CaptureMotoCtrl(float set, float set_f, float set_in_up, float set_throw_up){
	static float set_angle, set_angle_last;
	static uint8_t enable = 0;
	set_angle = set;
	if(set_angle != set_angle_last){ enable = 1; }
	if(enable == 1){
		float angle_err, damping_f;
		angle_err = set - (Capturing_MOTO[0].getpara.total_round*360.0f);
		damping_f = set_f*ABS(angle_err)/360.0f/9.5f;
		if(set_angle == TURN_IN){
			if(set_angle_last == TURN_THROW){
				enable = 0; Ammo.Capturing.position_now = IN;
			}
			else{
				damping_f *= set_in_up;
				Capturing_MOTO[0].pid_speed.PidSet_pidf(&Capturing_MOTO[0].pid_speed, Capturing_MOTO[0].pid_speed.p, 0, 0, damping_f);
				if(ABS(angle_err) < 40.0f*22.8f) { enable = 0; Ammo.Capturing.position_now = IN;}}
		}
		else if(set_angle == TURN_OUT){
			Capturing_MOTO[0].pid_speed.PidSet_pidf(&Capturing_MOTO[0].pid_speed, Capturing_MOTO[0].pid_speed.p, 0, 0, damping_f);
			if(ABS(angle_err) < 20.0f*22.8f) { enable = 0; Ammo.Capturing.position_now = OUT;}
		}
		else if(set_angle == TURN_THROW){
			damping_f *= set_throw_up;
			Capturing_MOTO[0].pid_speed.PidSet_pidf(&Capturing_MOTO[0].pid_speed, Capturing_MOTO[0].pid_speed.p, 0, 0, damping_f);
			if(ABS(angle_err) < 12.0f*22.8f) { enable = 0; Ammo.Capturing.position_now = OUT;}
		}
		else{ enable = 0;}
	}
	else{//enable = 0
		if(Ammo.Capturing.position_now == IN){
			Capturing_MOTO[0].pid_speed.PidSet_all(&Capturing_MOTO[0].pid_speed, POSITION_PID, 4500, 1000, -1000, 0.6, 0, 0, 0, 0, 0, 0, 0, 3500);
			Ammo.Capturing.position_now = STEADY;
		}
		else if(Ammo.Capturing.position_now == OUT){
			Capturing_MOTO[0].pid_speed.PidSet_all(&Capturing_MOTO[0].pid_speed, POSITION_PID, 8000, 1000, -1000, 1.2, 0, 0, 0, 0, 0, 0, 0, 5000);
			Ammo.Capturing.position_now = STEADY;
		}
	}
	Capturing_MOTO[0].set_speed = Capturing_MOTO[0].pid_angle.PidCalc(&Capturing_MOTO[0].pid_angle, Capturing_MOTO[0].getpara.total_round*360.0f, set);
	Capturing_MOTO[0].send_current = Capturing_MOTO[0].pid_speed.PidCalc(&Capturing_MOTO[0].pid_speed, Capturing_MOTO[0].getpara.speed_rpm, 22.76f*Capturing_MOTO[0].set_speed);	
	set_angle_last = set_angle;
	Ammo.Capturing.position_last = Ammo.Capturing.position_now;
}

void RiseHeightSet(void){
	static uint8_t init_cnt, init_sta = 0;
	static int reinit_cnt;
	static int fault_cnt;
	static int buffer_cnt = 0;
	static uint8_t buffer_cnt_enable = 0;
	static float Moto0_bottom, Moto1_bottom;
	if(Ammo.Raising.manual_raising_now == 0){
		/*init_sta用于初始化上电初始化*/
		if(init_sta == 0){
			init_cnt++;
			if(init_cnt == 200){
				init_sta = 1;
				Moto0_bottom = Raising_MOTO[0].getpara.offset_round;
				Moto1_bottom = Raising_MOTO[1].getpara.offset_round;
				Ammo.Raising.MEM_HEIGHT_0 = Moto0_bottom + 41.0f;
				Ammo.Raising.MEM_HEIGHT_1 = Moto1_bottom - 41.0f;
			}
		}
		if(init_sta == 1){
			switch(Ammo.Raising.height_now)
			{	
				case HEIGHT_BOTTOM:
					
/*新的复位代码*/				
//					if(Ammo.Raising.height_last != HEIGHT_BOTTOM){
//						buffer_cnt = 0;
//						buffer_cnt_enable = 1;
//					}
//					if(buffer_cnt_enable == 1 && (buffer_cnt < 1500))
//					{
//						buffer_cnt++;
//						if(Ammo.Raising.down_limit == 1){
//							if((fabs(Raising_MOTO[0].pid_angle.err.now) > 150 || fabs(Raising_MOTO[1].pid_angle.err.now) > 150)){
//								Ammo.reinit_sta = 1;
//							}
////							if(Ammo.reinit_sta == 1){
////								reinit_cnt++;
////								if(reinit_cnt == 800){
////									Ammo.reinit_sta = 0;
////									reinit_cnt = 0;
////								}
////							}
//						}
//						{}

//					}
//						
//						if(fault_cnt == 1500){
//						
//						
//						}
		
/*旧的自动复位代码*/
					/*保证升降机构能降到最低以确保能准确到达其他高度位置*/
					if((fabs(Raising_MOTO[0].pid_angle.err.now) > 150 || fabs(Raising_MOTO[1].pid_angle.err.now) > 150)){
						reinit_cnt++;
						if(reinit_cnt == 2000){
							Ammo.reinit_sta = 1;
							if(Ammo.Raising.down_limit == 1){
								reinit_cnt++;
							}
							else{
								reinit_cnt = 1999;
								fault_cnt++;
								if(fault_cnt == 4000){
									reinit_cnt++;
								}
							}
						}
						else if(reinit_cnt == 2600){
							Moto0_bottom = Raising_MOTO[0].getpara.total_round;
							Moto1_bottom = Raising_MOTO[1].getpara.total_round;
							Ammo.Raising.MEM_HEIGHT_0 = Moto0_bottom + 41.0f;
							Ammo.Raising.MEM_HEIGHT_1 = Moto1_bottom - 41.0f;
							Ammo.reinit_sta = 0;
							Ammo.Raising.down_limit = 0;
							fault_cnt = 0;
						}
					}
					else 
					{reinit_cnt = 1;
						Ammo.reinit_sta = 0;
						}		
					
					Raising_MOTO[0].set_angle = Moto0_bottom*360.0f;
					Raising_MOTO[1].set_angle = Moto1_bottom*360.0f;
					Ammo.Raising.height_change_cnt = 9.5f;
					Capturing_MOTO[0].set_angle = TURN_IN;
					CylinderEnable(3, 0);
					CylinderEnable(4, 0);
				break;
				case HEIGHT_APPLY:
					Raising_MOTO[0].set_angle = (Ammo.Raising.MEM_HEIGHT_0 + Ammo.Raising.height_change_cnt)*360.0f;
					Raising_MOTO[1].set_angle = Ammo.Raising.restitution_coefficient * (Ammo.Raising.MEM_HEIGHT_1 - Ammo.Raising.height_change_cnt)*360.0f;
					if(Raising_MOTO[0].pid_angle.err.now < 70 && Raising_MOTO[1].pid_angle.err.now < 70 && Ammo.Raising.up_limit == 0){
						Ammo.Raising.height_change_cnt += 0.3f;
					}
				break;
				case HEIGHT_CAP:
					Ammo.Raising.height_change_cnt = 9.5f;
					Raising_MOTO[0].set_angle = Ammo.Raising.MEM_HEIGHT_0*360.0f;
					Raising_MOTO[1].set_angle = Ammo.Raising.restitution_coefficient * Ammo.Raising.MEM_HEIGHT_1*360.0f;
					Ammo.Raising.down_limit = 0;
				break;				
				default:
					Raising_MOTO[0].set_angle = Moto0_bottom*360.0f;
					Raising_MOTO[1].set_angle = Moto1_bottom*360.0f;
				break;			
			}
			if(Ammo.Raising.height_now > Ammo.Raising.height_last){
				Raising_MOTO[0].pid_angle.PidSet_all(&Raising_MOTO[0].pid_angle, POSITION_PID, 160, 0, 0, 2.4, 0, 0, 0, 0, 800, 400, 200, 500);
				Raising_MOTO[0].pid_speed.PidSet_all(&Raising_MOTO[0].pid_speed, POSITION_PID, 6000, 3000, -1000, 2, 0.1, 9, 0, 0, 0, 0, 0, 500);
				Raising_MOTO[1].pid_angle.PidSet_all(&Raising_MOTO[1].pid_angle, POSITION_PID, 190, 0, 0, 2.5, 0, 0, 0, 0, 800, 400, 200, 500);
				Raising_MOTO[1].pid_speed.PidSet_all(&Raising_MOTO[1].pid_speed, POSITION_PID, 6000, 1000, -2000, 2, 0.1, 5.5, 0, 0, 2200, 1600, 1000, 500);
			}
			else if(Ammo.Raising.height_now < Ammo.Raising.height_last){
				Raising_MOTO[0].pid_angle.PidSet_all(&Raising_MOTO[0].pid_angle, POSITION_PID, 100, 500, -500, 1.2, 0, 0, 0, 0, 0, 0, 0, 500);
				Raising_MOTO[0].pid_speed.PidSet_all(&Raising_MOTO[0].pid_speed, POSITION_PID, 3000, 0, 0, 2.3, 0, 0, 0, 0, 0, 0, 0, 500);
				Raising_MOTO[1].pid_angle.PidSet_all(&Raising_MOTO[1].pid_angle, POSITION_PID, 120, 500, -500, 1.2, 0, 0, 0, 0, 0, 0, 0, 500);
				Raising_MOTO[1].pid_speed.PidSet_all(&Raising_MOTO[1].pid_speed, POSITION_PID, 3000, 0, 0, 2.3, 0	, 0, 0, 0, 0, 0, 500, 500);
			}
			for(uint8_t k=0; k<2; k++)
			{
				Raising_MOTO[k].set_speed = Raising_MOTO[k].pid_angle.PidCalc(&Raising_MOTO[k].pid_angle, Raising_MOTO[k].getpara.total_round*360.0f,  Raising_MOTO[k].set_angle);
				Raising_MOTO[k].send_current = Raising_MOTO[k].pid_speed.PidCalc(&Raising_MOTO[k].pid_speed, Raising_MOTO[k].getpara.speed_rpm, 22.4f*Raising_MOTO[k].set_speed);
			}
			Ammo.Raising.height_last = Ammo.Raising.height_now;
		}
	}
}

void ManualRaise(void){
	if(Ammo.Raising.height_now == HEIGHT_CAP){
		if(RC_CtrlData.key.key_data.shift==1){
			if(RC_CtrlData.key.key_data.R==1){
				Raising_MOTO[0].set_speed = 40;
				Raising_MOTO[1].set_speed = -40;
				Ammo.Raising.manual_raising_now = 1;
				
				Raising_MOTO[0].pid_speed.PidSet_all(&Raising_MOTO[0].pid_speed, POSITION_PID, 6000, 1500, -500, 4, 0, 0, 0, 0, 0, 0, 0, 500);
				Raising_MOTO[1].pid_speed.PidSet_all(&Raising_MOTO[1].pid_speed, POSITION_PID, 6000, 500, -500, 8, 0, 0, 0, 0, 0, 0, 0, 500);
				for(uint8_t k=0; k<2; k++)
				{
					Raising_MOTO[k].send_current = Raising_MOTO[k].pid_speed.PidCalc(&Raising_MOTO[k].pid_speed, Raising_MOTO[k].getpara.speed_rpm, 22.4f*Raising_MOTO[k].set_speed);
				}
			}
		}
		else if(RC_CtrlData.key.key_data.ctrl==1){
			if(RC_CtrlData.key.key_data.R==1){
				Raising_MOTO[0].set_speed = -40;
				Raising_MOTO[1].set_speed = 40;
				Ammo.Raising.manual_raising_now = 1;
				
				Raising_MOTO[0].pid_speed.PidSet_all(&Raising_MOTO[0].pid_speed, POSITION_PID, 4500, 500, -500, 2.0, 0, 0, 0, 0, 0, 0, 0, 500);	
				Raising_MOTO[1].pid_speed.PidSet_all(&Raising_MOTO[1].pid_speed, POSITION_PID, 4500, 500, -500, 1.6, 0, 0, 0, 0, 0, 0, 0, 500);
				for(uint8_t k=0; k<2; k++)
				{
					Raising_MOTO[k].send_current = Raising_MOTO[k].pid_speed.PidCalc(&Raising_MOTO[k].pid_speed, Raising_MOTO[k].getpara.speed_rpm, 22.4f*Raising_MOTO[k].set_speed);
				}
			}
		}
		else{
			Ammo.Raising.manual_raising_now = 0;			
		}
		if(Ammo.Raising.manual_raising_last && !Ammo.Raising.manual_raising_now){
			Ammo.Raising.MEM_HEIGHT_0 = Raising_MOTO[0].getpara.total_round;
			Ammo.Raising.MEM_HEIGHT_1 = Raising_MOTO[1].getpara.total_round;
		}
	}
	else {Ammo.Raising.manual_raising_now = 0;}
	Ammo.Raising.manual_raising_last = Ammo.Raising.manual_raising_now;
}

void ManualGetAmmo(void){
	static uint16_t state = 0;
	static uint8_t count_enable = 0;
	static uint8_t x_key_status_last = 0;

	if(!x_key_status_last && RC_CtrlData.key.key_data.X){
		count_enable = 1;
		Ammo.Raising.lockheight = 1;
	}
/*旧气缸状态
	1 弹匣   1补给 0闭合
	2 水平   1推出 0收回
	3 弹射   1弹出 0收回
	4 夹取   1夹紧 0松开
	5 拖车   1放下 0抬起
*/

/*新气缸状态
	1 夹取   1夹紧 0松开
	2 弹射   1弹出 0收回
	3 伸出   1伸出 0收回
	4 滑轨   1在后 0在前
	5 弹仓   1补给 0闭合
	6 拖车   1放下 0抬起
*/
	if(count_enable == 1){
		state++;
		switch(state){
			case 1:{
				Capturing_MOTO[0].set_angle = TURN_OUT;
			}break;
			case 450:{
				CylinderEnable(1, 1);
			}break;
			case 650:{
				Capturing_MOTO[0].set_angle = TURN_IN;
			}break;
//			case 1050:{
//				CylinderEnable(3, 0);
//			}break;
			case 1550:{
				CylinderEnable(1, 0);
			}break;
			case 1650:{
				CylinderEnable(2, 1);
			}break;
			case 2050:{
				CylinderEnable(2, 0);
			}break;
			case 2051:{
				count_enable = 0;
				Ammo.Raising.lockheight = 0;
				Chassis.lockflag = 0;
				state = 0;
			}break;
		}
	}
	x_key_status_last = RC_CtrlData.key.key_data.X;
}












//void AutoGetAmmo(uint8_t key){
//	static uint8_t enable, enable_last;
//	static uint8_t count = 0;
//	static uint16_t state = 0;
//	enable = key;
////	if(enable == 0){ mark = 0; }
//	Ammo.auto_state_last = Ammo.auto_state;
//	if(!enable_last && enable){
//		count++;
//		Ammo.auto_state = 0;
//	}
////	else count = 0;
///*旧气缸状态
//	1 弹匣   1补给 0闭合
//	2 水平   1推出 0收回
//	3 弹射   1弹出 0收回
//	4 夹取   1夹紧 0松开
//	5 拖车   1放下 0抬起
//*/

///*新气缸状态
//	1 夹取   1夹紧 0松开
//	2 弹射   1弹出 0收回
//	3 伸出   1伸出 0收回
//	4 滑轨   1在后 0在前
//	5 弹仓   1补给 0闭合
//	6 拖车   1放下 0抬起
//*/
//	if(count != 0){
//		state++;
//		switch(state){
//			case 1:{
//				Ammo.auto_state = 1;
//				Capturing_MOTO[0].set_angle = TURN_OUT;
//			}break;
//			case 800:{
//				CylinderEnable(1, CYOPEN);
//			}break;
//			case 1000:{
//				Ammo.Raising.height_now = HEIGHT_APPLY;
//			}
//			case 1200:{
//				Capturing_MOTO[0].set_angle = TURN_IN;
//			}break;
//			case 2340:{
//				Ammo.auto_state = 2;
//				CylinderEnable(1, CYCLOSE);
//			}break;
//			case 2560:{
//				CylinderEnable(2, CYOPEN);
//				Ammo.Raising.height_now = HEIGHT_CAP;
//			}break;
//			case 2780:{
//				CylinderEnable(2, CYCLOSE);
//				Ammo.auto_state = 3;
//			}break;
//			case 2810:{
//				enable = 0;			
//				state = 0;
//				count--;
//			}break;
//		}
//	}
//	enable_last = enable;
//}

//void AutoAmmo3(RC_Ctl_t rc_data){
//	static uint8_t a_key_status_last = 0, press_count = 0;
//	static uint8_t motion_switch;
//	if(!a_key_status_last && rc_data.key.key_data.A){
//		if(press_count == 0){
//			Ammo.Raising.lockheight = 1;
//			Chassis.lockflag = 1;
//			Chassis.mode.now = 2;
//			Ammo.auto_count = 0;
//			Ammo.wait_counter = 0;
//			press_count = 1;
//		}
//		else if(press_count == 1){  
//			Ammo.Raising.lockheight = 0;
//			Chassis.lockflag = 0;
//			Chassis.mode.now = 1;
//			Ammo.auto_count = 0;
//			Ammo.wait_counter = 0;
//			press_count = 0;
//		}
//	}
//	if(Chassis.mode.now == 2){
//		switch(Ammo.wait_counter){
//			case 0:
//				Ammo.auto_enable = 0;
//				motion_switch = 0;
//			break;
//			case 10:
//				Ammo.Capturing.autocap_enable = 1;
////				AutoGetAmmo(1);	
//			break;
//			case 3000:
//				if(Ammo.auto_count == 1) {
//					Ammo.Capturing.autocap_enable = 1;
////					AutoGetAmmo(1); 
//				}
//				else Ammo.wait_counter = 3000 - 1;
//			break;
//			case 6800:
//				if(Ammo.auto_count == 2) { 
//					Ammo.Capturing.autocap_enable = 1;
////					AutoGetAmmo(1); 
//				}
//				else Ammo.wait_counter = 6800 - 1;
//			break;
//			default:
//				Ammo.Capturing.autocap_enable = 0;
//			break;
//		}
//		Ammo.wait_counter++;
//		/*旧气缸状态
//			1 弹匣   1补给 0闭合
//			2 水平   1推出 0收回
//			3 弹射   1弹出 0收回
//			4 夹取   1夹紧 0松开
//			5 拖车   1放下 0抬起
//		*/

//		/*新气缸状态
//			1 夹取   1夹紧 0松开
//			2 弹射   1弹出 0收回
//			3 伸出   1伸出 0收回
//			4 滑轨   1在后 0在前
//			5 弹仓   1补给 0闭合
//			6 拖车   1放下 0抬起
//		*/
//		if(Ammo.auto_state == 2 && Ammo.auto_state_last == 1){
//			if(Ammo.auto_count == 1){
//				for(uint8_t i = 0; i<6; i++) ClearTotalRound(&Chassis_MOTO[i]);
//				Ammo.auto_enable = 1;
//			}
//			else if(Ammo.auto_count == 0){
//				CylinderEnable(4, 0);
//			}
//		}
//		else if(Ammo.auto_state == 3 && Ammo.auto_state_last == 2){
//			Ammo.auto_count++;
//			Ammo.Capturing.autocap_enable = 0;
////			AutoGetAmmo(0);
//			if(Ammo.auto_count == 3) 
//			{ Chassis.mode.now = 1; Chassis.lockflag = 0; Ammo.auto_enable = 0; motion_switch = 1;}
//		}
//	}
//	else if(Chassis.mode.now == 1 && motion_switch == 1){
//		Chassis.lockflag = 0;
//		Ammo.auto_count = 0;
//		Ammo.wait_counter = 0;
//		press_count = 0;
//		motion_switch = 0;
//		
//		
//		Ammo.Capturing.autocap_enable = 0;
//		
//		
//		
//	}
//	a_key_status_last = rc_data.key.key_data.A;
//}






#ifdef AMMO_OLD






void AutoGetAmmo(uint8_t key){
	static uint8_t enable, enable_last;
	static uint8_t count = 0;
	static uint16_t state = 0;
	enable = key;
//	if(enable == 0){ mark = 0; }
	Ammo.auto_state_last = Ammo.auto_state;
	if(!enable_last && enable){
		count++;
		Ammo.auto_state = 0;
	}
//	else count = 0;
/*旧气缸状态
	1 弹匣   1补给 0闭合
	2 水平   1推出 0收回
	3 弹射   1弹出 0收回
	4 夹取   1夹紧 0松开
	5 拖车   1放下 0抬起
*/

/*新气缸状态
	1 夹取   1夹紧 0松开
	2 弹射   1弹出 0收回
	3 伸出   1伸出 0收回
	4 滑轨   1在后 0在前
	5 弹仓   1补给 0闭合
	6 拖车   1放下 0抬起
*/
	if(count != 0){
		state++;
		switch(state){
			case 10:{
				Ammo.auto_state = 1;
				Capturing_MOTO[0].set_angle = TURN_OUT;
			}break;
			case 350:{
				CylinderEnable(1, CYOPEN);
			}break;
			case 450:{
				Ammo.Raising.height_now = HEIGHT_APPLY;
			}
			case 700:{
				Capturing_MOTO[0].set_angle = TURN_IN;
			}break;
			case 1500:{
				Ammo.auto_state = 2;
				CylinderEnable(1, CYCLOSE);
			}break;
			case 1600:{
				CylinderEnable(2, CYOPEN);
				Ammo.Raising.height_now = HEIGHT_CAP;
				Ammo.auto_state = 3;
			}break;
			case 1850:{
				CylinderEnable(2, CYCLOSE);
//				Ammo.auto_state = 3;
			}break;
			case 2000:{
				enable = 0;			
				state = 0;
				count--;
			}break;
		}
	}
	enable_last = enable;
}

void AutoAmmo3(RC_Ctl_t rc_data){
	static uint8_t a_key_status_last = 0, press_count = 0;
	static uint8_t motion_switch;
	if(!a_key_status_last && rc_data.key.key_data.A){
		if(press_count == 0){
			Ammo.Raising.lockheight = 1;
			Chassis.lockflag = 1;
			Chassis.mode.now = 2;
			Ammo.auto_count = 0;
			Ammo.wait_counter = 0;
			press_count = 1;
		}
		else if(press_count == 1){  
			Ammo.Raising.lockheight = 0;
			Chassis.lockflag = 0;
			Chassis.mode.now = 1;
			Ammo.auto_count = 0;
			Ammo.wait_counter = 0;
			press_count = 0;
		}
	}
	if(Chassis.mode.now == 2){
		switch(Ammo.wait_counter){
			case 0:
				Ammo.auto_enable = 0;
				motion_switch = 0;
			break;
			case 10:
				Ammo.Capturing.autocap_enable = 1;
//				AutoGetAmmo(1);	
			break;
			case 2000:
				if(Ammo.auto_count == 1) {
					Ammo.Capturing.autocap_enable = 1;
//					AutoGetAmmo(1); 
				}
				else Ammo.wait_counter = 2000 - 1;
			break;
			case 6000:
				if(Ammo.auto_count == 2) { 
					Ammo.Capturing.autocap_enable = 1;
//					AutoGetAmmo(1); 
				}
				else Ammo.wait_counter = 5500 - 1;
			break;
			default:
				Ammo.Capturing.autocap_enable = 0;
			break;
		}
		Ammo.wait_counter++;
		/*旧气缸状态
			1 弹匣   1补给 0闭合
			2 水平   1推出 0收回
			3 弹射   1弹出 0收回
			4 夹取   1夹紧 0松开
			5 拖车   1放下 0抬起
		*/

		/*新气缸状态
			1 夹取   1夹紧 0松开
			2 弹射   1弹出 0收回
			3 伸出   1伸出 0收回
			4 滑轨   1在后 0在前
			5 弹仓   1补给 0闭合
			6 拖车   1放下 0抬起
		*/
		if(Ammo.auto_state == 2 && Ammo.auto_state_last == 1){
			if(Ammo.auto_count == 1){
				for(uint8_t i = 0; i<6; i++) ClearTotalRound(&Chassis_MOTO[i]);
				Ammo.auto_enable = 1;
			}
			else if(Ammo.auto_count == 0){
				CylinderEnable(4, 0);
			}
		}
		else if(Ammo.auto_state == 3 && Ammo.auto_state_last == 2){
			Ammo.auto_count++;
			Ammo.Capturing.autocap_enable = 0;
//			AutoGetAmmo(0);
			if(Ammo.auto_count == 3) 
			{ Chassis.mode.now = 1; Chassis.lockflag = 0; Ammo.auto_enable = 0; motion_switch = 1;}
		}
	}
	else if(Chassis.mode.now == 1 && motion_switch == 1){
		Chassis.lockflag = 0;
		Ammo.auto_count = 0;
		Ammo.wait_counter = 0;
		press_count = 0;
		motion_switch = 0;
		
		
		Ammo.Capturing.autocap_enable = 0;
		
		
		
	}
	a_key_status_last = rc_data.key.key_data.A;
}






#else



void AutoGetAmmo(uint8_t key){
	static uint8_t enable, enable_last;
	static uint8_t count = 0;
	static uint16_t state = 0;
	enable = key;
//	if(enable == 0){ mark = 0; }
	Ammo.auto_state_last = Ammo.auto_state;
	if(!enable_last && enable){
		count++;
		Ammo.auto_state = 0;
	}
//	else count = 0;
/*旧气缸状态
	1 弹匣   1补给 0闭合
	2 水平   1推出 0收回
	3 弹射   1弹出 0收回
	4 夹取   1夹紧 0松开
	5 拖车   1放下 0抬起
*/

/*新气缸状态
	1 夹取   1夹紧 0松开
	2 弹射   1弹出 0收回
	3 伸出   1伸出 0收回
	4 滑轨   1在后 0在前
	5 弹仓   1补给 0闭合
	6 拖车   1放下 0抬起
*/
	if(count != 0){
		state++;
		switch(state){
			case 1:{
				CylinderEnable(1, CYCLOSE);
				Ammo.auto_state = 1;
			}break;
			case 50:{
				CylinderEnable(2, CYOPEN);
			}break;
			case 100:{
				Capturing_MOTO[0].set_angle = TURN_OUT;
			}break;

			case 550:{
				CylinderEnable(1, CYOPEN);
				CylinderEnable(2, CYCLOSE);
			}break;
			
			case 700:{
				Ammo.Raising.height_now = HEIGHT_APPLY;
			}break;
			case 860:{
				Capturing_MOTO[0].set_angle = TURN_IN;
			}break;
			case 1150:{
				Ammo.auto_state = 2;
			}break;
			case 1500:{
				Ammo.auto_state = 3;
				Ammo.Raising.height_now = HEIGHT_CAP;								
			}break;
			case 1501:{
				enable = 0;			
				state = 0;
				count--;
			}
		}
	}
	enable_last = enable;
}

void launch_one(uint8_t enable){
	static uint8_t enable_last;
	static uint16_t cnt_enable, cnt = 0;
	if(enable && !enable_last){
		cnt_enable = 1;
	}
	if(cnt_enable == 1){
		cnt++;
		if(cnt == 1){
			CylinderEnable(2, 1);
		}	
		if(cnt == 500){
			CylinderEnable(2, 0);
			cnt = 0;
			cnt_enable = 0;
		}
	}
	enable_last = enable;
}

/*引入自定义的标志位作为反馈确定进度状态*/
void AutoAmmo3(RC_Ctl_t rc_data){
	static uint8_t a_key_status_last = 0, press_count = 0;
	static uint8_t motion_switch;
	static uint8_t launch_enable = 0;
	if(!a_key_status_last && rc_data.key.key_data.A){
		if(press_count == 0){
			Ammo.Raising.lockheight = 1;
			Chassis.lockflag = 1;
			Chassis.mode.now = 2;
			Ammo.auto_count = 0;
			Ammo.wait_counter = 0;
			press_count = 1;
		}
		else if(press_count == 1){  
			Ammo.Raising.lockheight = 0;
			Chassis.lockflag = 0;
			Chassis.mode.now = 1;
			Ammo.auto_count = 0;
			Ammo.wait_counter = 0;
			press_count = 0;
		}
	}
	if(Chassis.mode.now == 2){
		switch(Ammo.wait_counter){
			case 0:
				Ammo.auto_enable = 0;
				motion_switch = 0;
			break;
			case 10:
				Ammo.Capturing.autocap_enable = 1;
			break;
			case 1550:
				if(Ammo.auto_count == 1) {
					Ammo.Capturing.autocap_enable = 1;
				}
				/*等待动作完成*/
				else Ammo.wait_counter = 1550 - 1;
			break;
			case 4550:
				if(Ammo.auto_count == 2) { 
					Ammo.Capturing.autocap_enable = 1;
				}
				/*等待动作完成*/
				else Ammo.wait_counter = 4550 - 1;
			break;
			default:
				Ammo.Capturing.autocap_enable = 0;
			break;
		}
		Ammo.wait_counter++;
		/*旧气缸状态
			1 弹匣   1补给 0闭合
			2 水平   1推出 0收回
			3 弹射   1弹出 0收回
			4 夹取   1夹紧 0松开
			5 拖车   1放下 0抬起
		*/

		/*新气缸状态
			1 夹取   1夹紧 0松开
			2 弹射   1弹出 0收回
			3 伸出   1伸出 0收回
			4 滑轨   1在后 0在前
			5 弹仓   1补给 0闭合
			6 拖车   1放下 0抬起
		*/
		if(Ammo.auto_state == 2 && Ammo.auto_state_last == 1){
			if(Ammo.auto_count == 1){
				for(uint8_t i = 0; i<6; i++) ClearTotalRound(&Chassis_MOTO[i]);
				Ammo.auto_enable = 1;
			}
			else if(Ammo.auto_count == 0){
				CylinderEnable(4, 0);
			}
		}
		else if(Ammo.auto_state == 3 && Ammo.auto_state_last == 2){
			Ammo.auto_count++;
			Ammo.Capturing.autocap_enable = 0;
			if(Ammo.auto_count == 3) 
			{ 
				launch_enable = 1;
				Chassis.mode.now = 1; 
				Chassis.lockflag = 0; 
				Ammo.auto_enable = 0; 
				motion_switch = 1;
				CylinderEnable(1, 0);
				Ammo.Raising.lockheight = 0;
			}
		}
	}
	else if(Chassis.mode.now == 1 && motion_switch == 1){
		Chassis.lockflag = 0;
		Ammo.auto_count = 0;
		Ammo.wait_counter = 0;
		press_count = 0;
		motion_switch = 0;
		
		
		Ammo.Capturing.autocap_enable = 0;
		launch_enable = 0;
		
		
	}
	launch_one(launch_enable);
	a_key_status_last = rc_data.key.key_data.A;
}

#endif
