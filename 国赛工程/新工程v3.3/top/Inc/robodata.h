#ifndef __ROBODATA_H
#define __ROBODATA_H

#include "main.h"
#include "chassis.h"
#include "mytype.h"
#include "m_remote.h"
#include "m_moto.h"

typedef enum{
	RIGHT_UP = 1,					//上岛模式切换
	RIGHT_MID = 3,				//正常运动模式
	RIGHT_DOWN = 2				//弹仓开合
}ControlSource_e;

typedef enum{
	LEFT_UP = 1,				//取弹机构
	LEFT_DOWN = 2,			//保险位
	LEFT_MID = 3				//抬升机构
}LeftLeverMode_e;

typedef enum{
	OPEN = 1,
	CLOSE = 0,
}ApplyMode_e;

typedef enum{
	Order_stop = 0,
	Order_start1 = 1,
	Order_startN = 2,
	Order_busy = 3
}Order_e;//指令

typedef enum{
	WorkingStatus_ready = 0,
	WorkingStatus_busy = 1,
	WorkingStatus_error = 2
}WorkingStatus_e;//运行状态

typedef struct{
	uint8_t left_mode;
	uint8_t ctrl_source;		//这几个枚举名称之后可以全工程替换修改
}RoboControlMode_t;				//机器人控制模式，由遥控器左右拨杆设置

typedef struct{
	float spd_forward;
	float spd_right;
	float spd_yaw_w;				//顺时针时针为正向
	float angle_yaw_w;
	Order_e chassis_order;
	WorkingStatus_e chassis_working_status;
}ChassisControl_t;				//底盘运动控制

typedef struct{
	float	raising_speed;
	float raising_angle;
}RaisingControl_t;

typedef struct{
	uint8_t key_q;
	uint8_t key_r;
	uint8_t key_c;
	uint8_t key_e;
}KeyControl_t;

typedef struct _RoboData_t{
	void (*GetRemoteControlData)(struct _RoboData_t *RoboData);		//遥控器数据处理函数
	
	RoboControlMode_t	robo_ctrlmode;															//机器人控制模式
	ChassisControl_t chassis_ctrl;																//底盘运动控制
	RaisingControl_t raising_ctrl;																//取弹丝杠控制
//	ApplyControl_t apply_ctrl;																		//补弹控制
}RoboData_t;//机器人用户数据总结构体

extern RoboData_t RoboData;//机器人用户数据类
extern KeyControl_t KeyCtrl;

//extern RoboControlMode_t robo_ctrlmode;//机器人控制模式
//extern ChassisControl_t chassis_ctrl;//底盘运动控制

extern void GenerallySetChassisSpeed(RC_Ctl_t rc_data, Chassis_t *chassis);
extern void GenerallySetRaisingHeight(RoboControlMode_t robo_ctrlmode, Chassis_t *chassis);
extern void InitRoboData(RoboData_t *RoboData);
extern void GenerallyKeySetControl(RC_Ctl_t RC_CtrlData, KeyControl_t KeyCtrl);

#endif
