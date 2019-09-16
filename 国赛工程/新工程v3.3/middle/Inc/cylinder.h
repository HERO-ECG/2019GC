#ifndef __CYLINDER_H
#define __CYLINDER_H

#include "stdint.h"

#define CYOPEN	1
#define CYCLOSE 0	

#define GPIO_cylinder   GPIOE
#define SER_Pin         GPIO_PIN_1
#define SCK_Pin         GPIO_PIN_5
#define RCK_Pin         GPIO_PIN_6

#define SER(a)                                                               \
            if(a)                                                            \
						{                                                                \
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);       \
						}                                                                \
						else                                                             \
						{                                                                \
							HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);     \
						}
#define SCK(a)                                                               \
            if(a)                                                            \
						{                                                                \
							HAL_GPIO_WritePin(GPIO_cylinder, SCK_Pin, GPIO_PIN_SET);       \
						}                                                                \
						else                                                             \
						{                                                                \
							HAL_GPIO_WritePin(GPIO_cylinder, SCK_Pin, GPIO_PIN_RESET);     \
						}
#define RCK(a)                                                               \
            if(a)                                                            \
						{                                                                \
							HAL_GPIO_WritePin(GPIO_cylinder, RCK_Pin, GPIO_PIN_SET);       \
						}                                                                \
						else                                                             \
						{                                                                \
							HAL_GPIO_WritePin(GPIO_cylinder, RCK_Pin, GPIO_PIN_RESET);     \
						}
											
extern uint8_t cylinder_number[6];
						
//void Send_595(void);
void CylinderEnable(uint8_t cylinder_num, uint8_t cylinder_state);
void CylinderSetControl(void);

						
						
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
