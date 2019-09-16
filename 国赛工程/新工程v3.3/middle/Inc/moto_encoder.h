#ifndef __ENCODER_H
#define __ENCODER_H

#include "mytype.h"
/*******************************************************************************************
  * @define
 *******************************************************************************************/
enum{	
	STDID_3508_LOW = 0x200,
	MOTO1_3508_ID = 0x201,
	MOTO2_3508_ID = 0x202,
	MOTO3_3508_ID = 0x203,
	MOTO4_3508_ID = 0x204,
	
	STDID_3508_HIGH = 0x1FF,
	MOTO5_3508_ID = 0x205,
	MOTO6_3508_ID = 0x206,
	MOTO7_3508_ID = 0x207,
	MOTO8_3508_ID = 0x208
};

/*******************************************************************************************
  * @Parameter
 *******************************************************************************************/
typedef struct __Encoder_t{
	void (*ParaInit)(struct __Encoder_t *ptr);
	void (*GetEncoderMeasure)(struct __Encoder_t *ptr, CAN_HandleTypeDef* hcan);
	float  	speed_rpm;
	float   real_current;
	float  	given_current;
	float   given_current_last;
	uint8_t  	hall;
	uint16_t 	angle;				/*abs angle range:[0,8191]*/
	uint16_t 	last_angle;				/*abs angle range:[0,8191]*/
	
	float   other_angle;
	float   other_speed;
	
	uint8_t init_sta;	
	uint8_t init_cnt;
	uint16_t	offset_angle;
	float		round_cnt;
	int32_t total_angle;
	float		total_round;
	float   offset_round;
}Encoder_t;

/*******************************************************************************************
  * @Function 
 *******************************************************************************************/
extern void EncoderParaInit(Encoder_t *ptr);

#endif
