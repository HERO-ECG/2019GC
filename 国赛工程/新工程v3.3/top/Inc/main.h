#ifndef __MAIN_H
#define __MAIN_H

//#include "stm32f4xx_HAL.h"
//#include "mytype.h"

#define IST_INT_Pin 				GPIO_PIN_3
#define IST_INT_GPIO_Port 	GPIOE
#define IST_RST_Pin 				GPIO_PIN_2
#define IST_RST_GPIO_Port 	GPIOE
#define LASER_Pin						GPIO_PIN_13
#define LASER_GPIO_Port			GPIOG
#define KEY_Pin 						GPIO_PIN_10
#define KEY_GPIO_Port     	GPIOD
#define LED_GREEN_Pin       GPIO_PIN_14
#define LED_GREEN_GPIO_Port GPIOF
#define LED_RED_Pin         GPIO_PIN_11
#define LED_RED_GPIO_Port   GPIOE
#define POWER_Pin_PH2       GPIO_PIN_2
#define POWER_Pin_PH3       GPIO_PIN_3
#define POWER_Pin_PH4       GPIO_PIN_4
#define POWER_Pin_PH5 			GPIO_PIN_5
#define POWER_Port          GPIOH
#define IIC_PIN_SDA					GPIO_PIN_4
#define IIC_PIN_SCL					GPIO_PIN_5

//extern float var[8];
//extern void vcan_sendware(u8 *wareaddr, u32 waresize); 

//extern uint32_t time_piece;

#define time_piece_start	0x0001
#define time_piece_0010hz 0x0010
#define time_piece_0100hz 0x0100
#define time_piece_1000hz	0x1000

//功能开关
//#define MEA_DIS
//#define CHASSIS_TEST
//#define BAOMING_TEST

//#define AMMO_OLD
#define RED

#ifdef RED
	#define ROBOT_TX_ID	0x0002 //红方
	#define ROBOT_RX_ID	0x0102

#else
	#define ROBOT_TX_ID	0x000C
	#define ROBOT_RX_ID	0x0112 //蓝方

#endif

#endif /* __MAIN_H */
