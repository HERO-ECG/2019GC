#ifndef __ROBODATA_H
#define __ROBODATA_H

#include "main.h"
#include "chassis.h"
#include "mytype.h"
#include "m_remote.h"
#include "m_moto.h"

typedef enum{
	RIGHT_UP = 1,					//�ϵ�ģʽ�л�
	RIGHT_MID = 3,				//�����˶�ģʽ
	RIGHT_DOWN = 2				//���ֿ���
}ControlSource_e;

typedef enum{
	LEFT_UP = 1,				//ȡ������
	LEFT_DOWN = 2,			//����λ
	LEFT_MID = 3				//̧������
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
}Order_e;//ָ��

typedef enum{
	WorkingStatus_ready = 0,
	WorkingStatus_busy = 1,
	WorkingStatus_error = 2
}WorkingStatus_e;//����״̬

typedef struct{
	uint8_t left_mode;
	uint8_t ctrl_source;		//�⼸��ö������֮�����ȫ�����滻�޸�
}RoboControlMode_t;				//�����˿���ģʽ����ң�������Ҳ�������

typedef struct{
	float spd_forward;
	float spd_right;
	float spd_yaw_w;				//˳ʱ��ʱ��Ϊ����
	float angle_yaw_w;
	Order_e chassis_order;
	WorkingStatus_e chassis_working_status;
}ChassisControl_t;				//�����˶�����

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
	void (*GetRemoteControlData)(struct _RoboData_t *RoboData);		//ң�������ݴ�����
	
	RoboControlMode_t	robo_ctrlmode;															//�����˿���ģʽ
	ChassisControl_t chassis_ctrl;																//�����˶�����
	RaisingControl_t raising_ctrl;																//ȡ��˿�ܿ���
//	ApplyControl_t apply_ctrl;																		//��������
}RoboData_t;//�������û������ܽṹ��

extern RoboData_t RoboData;//�������û�������
extern KeyControl_t KeyCtrl;

//extern RoboControlMode_t robo_ctrlmode;//�����˿���ģʽ
//extern ChassisControl_t chassis_ctrl;//�����˶�����

extern void GenerallySetChassisSpeed(RC_Ctl_t rc_data, Chassis_t *chassis);
extern void GenerallySetRaisingHeight(RoboControlMode_t robo_ctrlmode, Chassis_t *chassis);
extern void InitRoboData(RoboData_t *RoboData);
extern void GenerallyKeySetControl(RC_Ctl_t RC_CtrlData, KeyControl_t KeyCtrl);

#endif
