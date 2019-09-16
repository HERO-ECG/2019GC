#include "robodata.h"
#include "chassis.h"
#include "ammo.h"
#include "t_moto.h"
#include "t_monitor.h"
#include "pid.h"
#include "ks103.h"
#include "m_remote.h"
#include "gpio.h"
#include "dji_Protocol.h"
#include "cylinder.h"

//uint8_t Up_Limit_Flag = 0;
//uint8_t Down_Limit_Flag = 0;

RoboData_t RoboData={&InitRoboData};//机器人用户数据类
KeyControl_t KeyCtrl = {0};

//float forwardangle, rightangle, yawangle;
float test;

uint8_t Mode_Switch = 1;			//底盘PID切换标志位

/*function：设置控制模式*******************************
 *parameter：Remote_t类，RoboControlMode_t类
 */
void SetRoboControlMode(Remote_t rc, RoboControlMode_t *rcmode){
	//左
	switch(rc.s1){
		case 1:
			rcmode->left_mode = LEFT_UP;break;
		case 2:
			rcmode->left_mode = LEFT_DOWN;break;
		case 3:
			rcmode->left_mode = LEFT_MID;break;
		default: 
			rcmode->left_mode = LEFT_MID;break;
	}
	//右
	switch(rc.s2){
		case 1:
			rcmode->ctrl_source = RIGHT_UP;break;
		case 2:
			rcmode->ctrl_source = RIGHT_DOWN;break;
		case 3:
			rcmode->ctrl_source = RIGHT_MID;break;
		default: 
			rcmode->ctrl_source = RIGHT_MID;break;
	}
}
void GenerallyKeySetControl(RC_Ctl_t RC_CtrlData, KeyControl_t KeyCtrl)
{
	static uint8_t q_key_status_last = 0, press_count = 0;
	
	if(RC_CtrlData.key.key_data.Q && !q_key_status_last){
		if(press_count == 0){
			Chassis.mode.now = MOVE_TRAIL;
			press_count = 1;
		}
		else if(press_count == 1){
			Chassis.mode.now = MOVE_REMOTE;
			press_count = 0;
		}
	}
	q_key_status_last = RC_CtrlData.key.key_data.Q;
}

/*function：设置底盘运动控制*******************************
 *parameter：RC_Ctl_t类，RoboControlMode_t类，ChassisControl_t类
 */
void GenerallySetChassisSpeed(RC_Ctl_t rc_data, Chassis_t *chassis){
	if(chassis->lockflag != 1){
		if(Chassis.direction == BACK){
			if(rc_data.rc.s1 != 1 && chassis->mode.now != 1 && chassis->mode.now != 2){ chassis->speed.base = HIGHSPEED; }
				else { chassis->speed.base = LOWSPEED;  
					#ifdef CHASSIS_TEST
					Chassis.mode.now = 5;
					#endif
				}//左拨杆下且不处于取弹状态，若处于则慢速
			if(rc_data.rc.ch3 != 0) { chassis->speed.spd_forward = (rc_data.rc.ch3 - 1024.0)/660.0; }
				else { chassis->speed.spd_forward = 0; }
			if(rc_data.rc.ch2 != 0) { chassis->speed.spd_right = -1.2f * (rc_data.rc.ch2 - 1024.0)/660.0; }
				else { chassis->speed.spd_right = 0; }	
			if(rc_data.rc.ch0 != 0) { chassis->speed.spd_yaw_dps = -(rc_data.rc.ch0 - 1024.0)/660.0*180.0f; }
				else { chassis->speed.spd_yaw_dps = 0; }
		}
		else{
			if(rc_data.rc.s1 != 1 && chassis->mode.now != 1 && chassis->mode.now != 2){ chassis->speed.base = HIGHSPEED; }
				else { chassis->speed.base = LOWSPEED;
					#ifdef CHASSIS_TEST
					Chassis.mode.now = 5;
					#endif
				}//左拨杆下且不处于取弹状态，若处于则慢速
			if(rc_data.rc.ch3 != 0) { chassis->speed.spd_forward = (rc_data.rc.ch3 - 1024.0)/660.0; }
				else { chassis->speed.spd_forward = 0; }
			if(rc_data.rc.ch2 != 0) { chassis->speed.spd_right = 1.2f * (rc_data.rc.ch2 - 1024.0)/660.0; }
				else { chassis->speed.spd_right = 0; }	
			if(rc_data.rc.ch0 != 0) { chassis->speed.spd_yaw_dps = (rc_data.rc.ch0 - 1024.0)/660.0 * 240.0f; }
				else { chassis->speed.spd_yaw_dps = 0; }
		}
	}
}

void GenerallySetRaisingHeight(RoboControlMode_t robo_ctrlmode, Chassis_t *chassis){
	static uint8_t ready_cnt = 0;
	if(Ammo.Raising.lockheight != 1 && robo_ctrlmode.left_mode == LEFT_MID){
		if(robo_ctrlmode.ctrl_source == RIGHT_DOWN){
			Ammo.Raising.height_now = HEIGHT_BOTTOM;
		}
		else if(robo_ctrlmode.ctrl_source == RIGHT_MID){
			Ammo.Raising.height_now = HEIGHT_CAP;
		}
		else if(robo_ctrlmode.ctrl_source == RIGHT_UP){
			Ammo.Raising.height_now = HEIGHT_APPLY;
		}
	}
//	if(RC_CtrlData.key.key_data.S && RC_CtrlData.key.key_data.shift){
//		Ammo.Raising.height_now = HEIGHT_BOTTOM;
//		ready_cnt++;
//		if()
//	}
}

/*function：遥控器数据处理函数*******************************
 *parameter：RoboData_t类
 */
void GetRemoteControlData(RoboData_t *RoboData){
	RCReadKey(&RC_CtrlData);
	GenerallyKeySetControl(RC_CtrlData, KeyCtrl);
	SetRoboControlMode(RC_CtrlData.rc, &RoboData->robo_ctrlmode);//设置控制模式
}


/*function：初始化机器人数据类RoboData内的数据*******************************
 *parameter：RoboData_t类
 */
void InitRoboData(RoboData_t *RoboData){
	Chassis_MOTO[0].MotoParaInit = &MotoParaInit;
		Chassis_MOTO[0].MotoParaInit(&Chassis_MOTO[0]);
	Chassis_MOTO[1].MotoParaInit = &MotoParaInit;
		Chassis_MOTO[1].MotoParaInit(&Chassis_MOTO[1]);
	Chassis_MOTO[2].MotoParaInit = &MotoParaInit;
		Chassis_MOTO[2].MotoParaInit(&Chassis_MOTO[2]);
	Chassis_MOTO[3].MotoParaInit = &MotoParaInit;
		Chassis_MOTO[3].MotoParaInit(&Chassis_MOTO[3]);
	Chassis_MOTO[4].MotoParaInit = &MotoParaInit;
		Chassis_MOTO[4].MotoParaInit(&Chassis_MOTO[4]);
	Chassis_MOTO[5].MotoParaInit = &MotoParaInit;
		Chassis_MOTO[5].MotoParaInit(&Chassis_MOTO[5]);
	
	Raising_MOTO[0].MotoParaInit = &MotoParaInit;
		Raising_MOTO[0].MotoParaInit(&Raising_MOTO[0]);
	Raising_MOTO[1].MotoParaInit = &MotoParaInit;
		Raising_MOTO[1].MotoParaInit(&Raising_MOTO[1]);
	
	Capturing_MOTO[0].MotoParaInit = &MotoParaInit;
		Capturing_MOTO[0].MotoParaInit(&Capturing_MOTO[0]);
	Capturing_MOTO[1].MotoParaInit = &MotoParaInit;
		Capturing_MOTO[1].MotoParaInit(&Capturing_MOTO[1]);
	
	monitor_remote.MonitorParaInit = &MonitorParaInit;
		monitor_remote.MonitorParaInit(&monitor_remote);
	monitor_chassis_moto.MonitorParaInit = &MonitorParaInit;
		monitor_chassis_moto.MonitorParaInit(&monitor_chassis_moto);
	monitor_raising_moto.MonitorParaInit = &MonitorParaInit;
		monitor_raising_moto.MonitorParaInit(&monitor_raising_moto);
		
	RC_CtrlData.RCDataParaInit = &RCDataParaInit;
		RC_CtrlData.RCDataParaInit(&RC_CtrlData);

//	Ks103_Front.KS103_SensorInit = &KS103_SensorInit;
//		Ks103_Front.KS103_SensorInit(&Ks103_Front);
//	Ks103_Left.KS103_SensorInit = &KS103_SensorInit;
//		Ks103_Left.KS103_SensorInit(&Ks103_Left);
//	Ks103_Right.KS103_SensorInit = &KS103_SensorInit;
//		Ks103_Right.KS103_SensorInit(&Ks103_Right);
	
	/*遥控初始化设置*/
	RoboData->GetRemoteControlData = &GetRemoteControlData;
	RoboData->robo_ctrlmode.left_mode = LEFT_DOWN;				//左拨杆模式
	RoboData->robo_ctrlmode.ctrl_source = RIGHT_DOWN;	//控制来源模式默认为来自遥控器
	RoboData->chassis_ctrl.chassis_order = Order_stop;
}
