#include "ammo.h"
#include "main.h"
#include "gpio.h"
#include "can.h"
#include "m_moto.h"
#include "control.h"
#include "robodata.h"

//extern uint8_t Up_Limit_Flag;
uint8_t Down_Limit_Flag;

uint8_t count_enable = 0;

/*加了限位开关来检测升降机构是否回到了最底下*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	static uint8_t toggle_cnt = 0;
	if(GPIO_Pin == GPIO_PIN_12){//PH12 上面的
		Ammo.Raising.up_limit = 1;
	}
	else if(GPIO_Pin == GPIO_PIN_0){//PI0 下面的
		Ammo.Raising.down_limit = 1;
		if(toggle_cnt == 0){
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_SET);
			toggle_cnt = 1;
		}
		else if(toggle_cnt == 1){
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_RESET);
			toggle_cnt = 0;		
		}
	}
}

//void monitor_raisingmoto(Moto_t *moto){
//	static uint8_t count = 0;
//	if(count_enable == 1 && moto->send_current == moto->pid_speed.max_output && !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)){
//		count++;		
//	}
//	if(count > 250){
//		Ammo.Raising.limit_stop = 1;
//		count_enable = 0;
//		count = 0;
//	}
//	if(count_enable == 1 && !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)){
//		Raising_MOTO[0].MotoParaInit = &MotoParaInit;
//			Raising_MOTO[0].MotoParaInit(&Raising_MOTO[0]);
//		Raising_MOTO[1].MotoParaInit = &MotoParaInit;
//			Raising_MOTO[1].MotoParaInit(&Raising_MOTO[1]);
//		Ammo.Raising.limit_stop = 0;
//	}
//}
