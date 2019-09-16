#ifndef __MOTO_H
#define __MOTO_H

#include "pid.h"
#include "moto_encoder.h"
#include "mytype.h"
/*******************************************************************************************
  * @define
 *******************************************************************************************/

/*******************************************************************************************
  * @Parameter
 *******************************************************************************************/
//typedef struct __Set_t{
//	float 	beforelast;
//	float 	last;
//	float 	now;
//}Set_t;

typedef struct __Moto_t{
	void (*MotoParaInit)(struct __Moto_t *ptr);
	Encoder_t 	getpara;
	
	Pid_t 	pid_angle; 
	Pid_t 	pid_speed;  
	
	float   set_angle;
	float   set_angle_last;
  float   set_speed;
	float   set_speed_last;
	float   send_current;
	float   send_current_last;
}Moto_t;

extern Moto_t Chassis_MOTO[6];
extern Moto_t Raising_MOTO[2];
extern Moto_t Capturing_MOTO[2];

/*******************************************************************************************
  * @Function 
 *******************************************************************************************/
extern void SetMotoCurrent(CAN_HandleTypeDef* hcan, uint32_t stdid, s16 iq1, s16 iq2, s16 iq3, s16 iq4); 
extern void MotoParaInit(Moto_t *ptr);

#endif
