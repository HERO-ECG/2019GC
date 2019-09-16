#include "cylinder.h"
#include "Chassis.h"
#include "ammo.h"
#include "m_remote.h"
#include "iic.h"
#include "stm32f4xx_hal.h"

uint8_t cylinder_number[6] = {0};

/*气缸功能函数文件*/

/*595气缸驱动板协议*/
//void Send_595(void)
//{
//  int i,a;
//  for (i = 7; i >= 0; i--)
//	{
//		a = cylinder_number[i];
//		SER(a);
//    SCK(1);
//    SCK(0);
//  }
//	RCK(0);
//  RCK(1);
//}

/*L298N控制*/
void CylinderEnable(uint8_t cylinder_num, uint8_t cylinder_state)
{
	if(cylinder_num == 1){
		if(cylinder_state == CYOPEN){
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
			cylinder_number[0] = 1;
		}
		else{
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
			cylinder_number[0] = 0;
		}
	}
	else if(cylinder_num == 2){
		if(cylinder_state == CYOPEN){
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);
			cylinder_number[1] = 1;
		}
		else{
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
			cylinder_number[1] = 0;
		} 
	}
	else if(cylinder_num == 3){
		if(cylinder_state == CYOPEN){
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
			cylinder_number[2] = 1;
		}
		else{
			cylinder_number[2] = 0;
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
		}
	}
	else if(cylinder_num == 4){
		if(cylinder_state == CYOPEN){
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
			cylinder_number[3] = 1;
		}
		else{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
			cylinder_number[3] = 0;
		}
	}
	else if(cylinder_num == 5){
		if(cylinder_state == CYOPEN){
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
			cylinder_number[4] = 1;
		}
		else{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
			cylinder_number[4] = 0;
		}
	}
	else if(cylinder_num == 6){
		if(cylinder_state == CYOPEN){
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
			cylinder_number[5] = 1;
		}
		else{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
			cylinder_number[5] = 0;
		}
	}
	
	
//	cylinder_number[cylinder_num-1] = cylinder_state;
//	Send_595();
}

/*旧气缸状态
	1 弹匣   1补给 0闭合
	2 水平   1推出 0收回
	3 弹射   1弹出 0收回
	4 夹取   1夹紧 0松开
	5 拖车   1放下 0抬起
*/

/*新气缸状态
	1 夹取   1夹紧 0松开
	2 弹射   1弹出 0收回
	3 伸出   1伸出 0收回
	4 滑轨   1在后 0在前
	5 弹仓   1补给 0闭合
	6 拖车   1放下 0抬起
*/

void CylinderSetControl(void)
{
	static uint16_t tanshe_cnt;
	static uint8_t tanshe_sta;
	if(RC_CtrlData.key.key_data.C==1){
		CylinderEnable(1, CYCLOSE);	 //爪子
		CylinderEnable(3, CYCLOSE);  //伸出
		CylinderEnable(4, CYCLOSE);	 //滑轨
		CylinderEnable(5, CYCLOSE);	 //弹仓
		CylinderEnable(6, CYCLOSE);	 //拖车
		tanshe_sta = 1;
		tanshe_cnt = 0;
	}
	if(tanshe_sta == 1){
		tanshe_cnt++;
		if(tanshe_cnt == 1){
			CylinderEnable(2, 1);
		}
		
		
		if(tanshe_cnt == 80){
			CylinderEnable(2, 0);
		}
		if(tanshe_cnt == 100){
			tanshe_sta = 0;
		}
	}
	
	if(RC_CtrlData.key.key_data.G==1)	CylinderEnable(6, CYOPEN); //拖车
	
	if(RC_CtrlData.key.key_data.F==1){
			if(Ammo.Raising.height_now == HEIGHT_APPLY){
				CylinderEnable(5, CYOPEN);				 //弹仓
			}
			else if(Ammo.Raising.height_now == HEIGHT_CAP){
				Ammo.Raising.lockheight = 1;
				Ammo.Raising.height_now = HEIGHT_APPLY;
			}
		}
	
	if(RC_CtrlData.key.key_data.shift==1)
	{
		if(RC_CtrlData.key.key_data.W==1)  CylinderEnable(3, CYOPEN);	 //伸出
//	  if(RC_CtrlData.key.key_data.F==1){
//			if(Ammo.Raising.height_now == HEIGHT_APPLY){
//				CylinderEnable(5, CYOPEN);				 //弹仓
//			}
//			else if(Ammo.Raising.height_now == HEIGHT_CAP){
//				Ammo.Raising.lockheight = 1;
//				Ammo.Raising.height_now = HEIGHT_APPLY;
//			}
//		}
		
		else if(RC_CtrlData.key.key_data.E==1)	CylinderEnable(4, CYOPEN);	 //滑轨
//		else if(RC_CtrlData.key.key_data.G==1)	CylinderEnable(6, CYOPEN); //拖车失效
		else if(RC_CtrlData.key.key_data.ctrl==1){
			Chassis.lockflag = 0;
			Ammo.Raising.lockheight = 0;
		}
	}
	else if(RC_CtrlData.key.key_data.ctrl==1)
	{

//		if(RC_CtrlData.key.key_data.E==1)  CylinderEnable(1, CYCLOSE);
		if(RC_CtrlData.key.key_data.F==1){
			CylinderEnable(5, CYCLOSE);				 //弹仓
			Ammo.Raising.lockheight = 0;
		}
		else if(RC_CtrlData.key.key_data.W==1)  CylinderEnable(3, CYCLOSE);  //伸出
		else if(RC_CtrlData.key.key_data.E==1)	CylinderEnable(4, CYCLOSE);	 //滑轨
		else if(RC_CtrlData.key.key_data.G==1) 		CylinderEnable(6, CYCLOSE);  //拖车失效
		/*强制初始化*/
		else if(RC_CtrlData.key.key_data.shift==1){
			Chassis.lockflag = 0;
			Ammo.Raising.lockheight = 0;
		}
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
