#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "stm32f4xx_hal.h"
#include "m_remote.h"
#include "m_moto.h"

#ifdef BAOMING_TEST
	#define LOWSPEED 2000.0F
	#define HIGHSPEED 4000.0F
#else
	#define LOWSPEED 4000.0F
	#define HIGHSPEED 8000.0F
#endif

#define FORENT 0.0F
#define BACK 180.0F
#define LEFT 270.0F

//#define UP 1350
//#define MID 1180
//#define DOWN 530

#define MID 2100
#define DOWN 1200

/* SemiCap */
#define UNSTOP 1
#define STOP 0

#define ANGLE_CLIMB 3000

typedef struct{	
	float spd_forward;
	float spd_right;	
	float spd_yaw;
	float spd_yaw_dps;
	float spd_yaw_gyro;
	float spd_yaw_correct;
	float base;
}ChassisSpeed_t;

/*为自动补弹位置环预留接口*/
typedef struct{
	uint8_t state;
	uint8_t state_cnt;
	float vtripod_angle;
	float chassis_angle;
	float delta_angle;
	float inclination_angle_now;
	float inclination_angle_last;
}ChassisAngle_t;

typedef struct{
	uint8_t last;
	uint8_t now;
}ChassisMode_t;

typedef struct{
	uint8_t last;
	uint8_t now;
}ChassisVideo_t;

typedef enum
{
  TUBE_UNALIGNED = 0,
  TUBE_ALIGNED = 1,
}TUBE_State;

typedef struct{
	float direction;
	float version;
	float distance_forward;
	float distance_right;
	float distance_yaw;
	float cnt1;
	float cnt2;
	uint8_t lockflag;
	uint8_t semi_enable;
	TUBE_State tube_state;
	Pid_t PID_yaw;
	ChassisMode_t mode;
	ChassisVideo_t video;	
	ChassisSpeed_t speed;
	ChassisAngle_t angle;	
}Chassis_t;

extern Chassis_t Chassis;

typedef enum
{
	MOVE_REMOTE = 0,//平地
	MOVE_SE_CAPTURE = 1,//辅助对齐
	MOVE_AU_CAPTURE = 2,//自动三箱
	MOVE_TRAIL = 3,//拖车
	MOVE_APPLY = 4,//补弹
	MOVE_CLIMB = 5,
	MOVE_ACLINIC,
}MOVE_State;

#define TUBE_GPIO_PORT 	GPIOA
#define TUBE_1ST 				GPIO_PIN_5
#define TUBE_2ND				GPIO_PIN_6
#define TUBE_3RD				GPIO_PIN_7
#define TUBE_4TH				GPIO_PIN_8
#define TUBE_5TH				GPIO_PIN_9

#define CaptureAssistantOn  0x01
#define CaptureAssistantOff 0x00

#define CHASSISMAXCURRENT 11000.0F

//需要实测
#define BaseAngle 360.0f //360°~465mm(front)or~445mm(right)
#define BaseFrontDistance (465.0f)		//mm
#define BaseRightDistance (445.0f)		//mm
#define BaseYawAngle (70.0f)	//°

void Chassis_Init(float direction, float speed_base);
void ChassisFun(void);
void ChassisSetSpeed(void);
void ClearTotalRound(Moto_t *moto);
void ChassisSetAngle(float theta, float distance_forward, float distance_right, float distance_yaw, uint8_t state);
TUBE_State PhotoelectricTubeDetect(void);
void SemiautoAmmoCapture(RC_Ctl_t rc_data);
void n3AutoAmmoCapture(RC_Ctl_t rc_data);

#endif
