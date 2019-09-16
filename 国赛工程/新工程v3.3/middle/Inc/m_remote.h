#ifndef  __M_REMOTE_H
#define __M_REMOTE_H

#include "mytype.h"

/*******************************************************************************************
  * @define
 *******************************************************************************************/
#define KEYBOARD_Pressed   1
#define KEYBOARD_Released  0

/*******************************************************************************************
  * @Parameter
 *******************************************************************************************/
typedef struct __Remote
{
	uint16_t ch0;
	uint16_t ch1;
	uint16_t ch2;
	uint16_t ch3;
	uint8_t s1;
	uint8_t s2;
}Remote_t;

typedef struct __Mouse
{
	int16_t x_speed;//向左负
	int16_t y_speed;//向下正
	int16_t z_speed;
	int8_t press_1;
	int8_t press_2;
	
	int32_t x_positon;
	int32_t y_positon;
	int32_t z_positon;
}Mouse_t;	

typedef	struct __Key_Data
{
	  u8 W;
    u8 S;
		u8 A;
		u8 D;
		u8 Q;
		u8 E;
		u8 shift;
		u8 ctrl;
	  u8 R;
    u8 F;
		u8 G;
		u8 Z;
		u8 X;
		u8 C;
		u8 V;
		u8 B;
}Key_Data_t;
typedef	struct __Key
{
	uint16_t v;
	Key_Data_t key_data;
}Key_t;

typedef struct __RC_Ctl_t
{
	void (*RCDataParaInit)(struct __RC_Ctl_t *RC_CtrlData);
	Remote_t rc;
	Mouse_t mouse;
	Key_t key;
}RC_Ctl_t;
extern RC_Ctl_t RC_CtrlData;

/*******************************************************************************************
  * @Function 
 *******************************************************************************************/
extern void RCDataParaInit(RC_Ctl_t *RC_CtrlData);
extern void RemoteDataProcess(uint8_t *pData, RC_Ctl_t *RC_CtrlData);
extern void RCReadKey(RC_Ctl_t *RC_CtrlData);

#endif
