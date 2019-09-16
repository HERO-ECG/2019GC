#ifndef __AMMO_H
#define __AMMO_H

#include "stdint.h"

typedef enum{
  HEIGHT_BOTTOM = 0,
  HEIGHT_APPLY = 2,
	HEIGHT_CAP = 1,
}HEIGHT_State;

typedef enum{
	IN = 0,
	OUT = 1,
	STEADY = 2,
}CAP_State;

/*position 0 IN 1 OUT*/

typedef struct{
	uint8_t height_now;
	uint8_t height_last;
	uint8_t lockheight;
	uint8_t manual_raising_now;
	uint8_t manual_raising_last;
	float height_change_cnt;
	float restitution_coefficient;
	
	
	float MEM_HEIGHT_0;
  float MEM_HEIGHT_1;
	
	
	uint8_t up_limit;
	uint8_t down_limit;
}Raising_t;

typedef struct{
	uint8_t autocap_enable;
	uint8_t position_now;
	uint8_t position_last;
	float throw_up;
	float in_up;
	float f_set;
}Capturing_t;

typedef struct{
	uint8_t auto_state;
	uint8_t auto_state_last;
	uint8_t auto_enable;
	uint8_t auto_count;
	uint32_t wait_counter;
	uint8_t reinit_sta;
	Raising_t Raising;
	Capturing_t Capturing;
}Ammo_t;

extern Ammo_t Ammo;

#define TURN_OUT (Capturing_MOTO[0].getpara.offset_round - 9.85f)*360.0f
#define TURN_MID -1200
#define TURN_THROW (Capturing_MOTO[0].getpara.offset_round - 5.5f)*360.0f
#define TURN_IN  Capturing_MOTO[0].getpara.offset_round*360.0f

void AmmoFun(void);
void AmmoInit(void);
void ManualGetAmmo(void);
void AutoGetAmmo(uint8_t key);
void CaptureMotoCtrl(float set, float set_f, float set_in_up, float set_throw_up);

//extern void GetAmmo(void);
//extern uint8_t ammo_task[4];
//extern uint8_t ammo_flag;

#endif
