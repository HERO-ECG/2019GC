#include "cylinder.h"
#include "Chassis.h"
#include "ammo.h"
#include "m_remote.h"
#include "iic.h"
#include "stm32f4xx_hal.h"

uint8_t cylinder_number[6] = {0};

/*���׹��ܺ����ļ�*/

/*595����������Э��*/
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

/*L298N����*/
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

/*������״̬
	1 ��ϻ   1���� 0�պ�
	2 ˮƽ   1�Ƴ� 0�ջ�
	3 ����   1���� 0�ջ�
	4 ��ȡ   1�н� 0�ɿ�
	5 �ϳ�   1���� 0̧��
*/

/*������״̬
	1 ��ȡ   1�н� 0�ɿ�
	2 ����   1���� 0�ջ�
	3 ���   1��� 0�ջ�
	4 ����   1�ں� 0��ǰ
	5 ����   1���� 0�պ�
	6 �ϳ�   1���� 0̧��
*/

void CylinderSetControl(void)
{
	static uint16_t tanshe_cnt;
	static uint8_t tanshe_sta;
	if(RC_CtrlData.key.key_data.C==1){
		CylinderEnable(1, CYCLOSE);	 //צ��
		CylinderEnable(3, CYCLOSE);  //���
		CylinderEnable(4, CYCLOSE);	 //����
		CylinderEnable(5, CYCLOSE);	 //����
		CylinderEnable(6, CYCLOSE);	 //�ϳ�
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
	
	if(RC_CtrlData.key.key_data.G==1)	CylinderEnable(6, CYOPEN); //�ϳ�
	
	if(RC_CtrlData.key.key_data.F==1){
			if(Ammo.Raising.height_now == HEIGHT_APPLY){
				CylinderEnable(5, CYOPEN);				 //����
			}
			else if(Ammo.Raising.height_now == HEIGHT_CAP){
				Ammo.Raising.lockheight = 1;
				Ammo.Raising.height_now = HEIGHT_APPLY;
			}
		}
	
	if(RC_CtrlData.key.key_data.shift==1)
	{
		if(RC_CtrlData.key.key_data.W==1)  CylinderEnable(3, CYOPEN);	 //���
//	  if(RC_CtrlData.key.key_data.F==1){
//			if(Ammo.Raising.height_now == HEIGHT_APPLY){
//				CylinderEnable(5, CYOPEN);				 //����
//			}
//			else if(Ammo.Raising.height_now == HEIGHT_CAP){
//				Ammo.Raising.lockheight = 1;
//				Ammo.Raising.height_now = HEIGHT_APPLY;
//			}
//		}
		
		else if(RC_CtrlData.key.key_data.E==1)	CylinderEnable(4, CYOPEN);	 //����
//		else if(RC_CtrlData.key.key_data.G==1)	CylinderEnable(6, CYOPEN); //�ϳ�ʧЧ
		else if(RC_CtrlData.key.key_data.ctrl==1){
			Chassis.lockflag = 0;
			Ammo.Raising.lockheight = 0;
		}
	}
	else if(RC_CtrlData.key.key_data.ctrl==1)
	{

//		if(RC_CtrlData.key.key_data.E==1)  CylinderEnable(1, CYCLOSE);
		if(RC_CtrlData.key.key_data.F==1){
			CylinderEnable(5, CYCLOSE);				 //����
			Ammo.Raising.lockheight = 0;
		}
		else if(RC_CtrlData.key.key_data.W==1)  CylinderEnable(3, CYCLOSE);  //���
		else if(RC_CtrlData.key.key_data.E==1)	CylinderEnable(4, CYCLOSE);	 //����
		else if(RC_CtrlData.key.key_data.G==1) 		CylinderEnable(6, CYCLOSE);  //�ϳ�ʧЧ
		/*ǿ�Ƴ�ʼ��*/
		else if(RC_CtrlData.key.key_data.shift==1){
			Chassis.lockflag = 0;
			Ammo.Raising.lockheight = 0;
		}
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
