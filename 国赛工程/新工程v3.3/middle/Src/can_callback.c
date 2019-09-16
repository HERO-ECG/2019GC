/*******************************************************************************************
  * @Include @Headfile
 *******************************************************************************************/
#include "main.h"
#include "can.h"
#include "m_moto.h"
#include "moto_encoder.h"
#include "m_remote.h"
#include "robodata.h"
#include "t_monitor.h"
#include "dji_Protocol.h"

/*******************************************************************************************
  * @Parameter @Statement
 *******************************************************************************************/

/*******************************************************************************************
  * @Func			HAL_CAN_RxCpltCallback
  * @Brief    CAN接收回调函数
  * @Param		CAN_HandleTypeDef* hcan
  * @Retval		None
  * @Date     2018.10.10
  *	@Author		SZC		LZK添加CAN入口判断
 *******************************************************************************************/
uint8_t hcan2_enable = 0;

uint8_t can_init_cnt = 0;
uint8_t can_init_sta = 0;

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
	if(hcan == &hcan1){	
		switch(hcan->pRxMsg->StdId){
			case MOTO1_3508_ID:
			case MOTO2_3508_ID:
			case MOTO3_3508_ID:
			case MOTO4_3508_ID:
			case MOTO5_3508_ID:
			case MOTO6_3508_ID:{
				uint32_t can_id = 0;
				can_id = hcan->pRxMsg->StdId - MOTO1_3508_ID;
				Chassis_MOTO[can_id].getpara.GetEncoderMeasure(&Chassis_MOTO[can_id].getpara, hcan);
			}break;
		}
		__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);//重新使能can中断标志
		hcan2_enable = 1;
	}
	else if(hcan == &hcan2 && hcan2_enable == 1){
		switch(hcan->pRxMsg->StdId){
			case MOTO3_3508_ID:
				Capturing_MOTO[0].getpara.GetEncoderMeasure(&Capturing_MOTO[0].getpara, hcan);
			break;
			case MOTO6_3508_ID:
				Capturing_MOTO[1].getpara.GetEncoderMeasure(&Capturing_MOTO[1].getpara, hcan);
			break;
			case MOTO1_3508_ID:
				Raising_MOTO[0].getpara.GetEncoderMeasure(&Raising_MOTO[0].getpara, hcan);
			break;
			case MOTO2_3508_ID:
				Raising_MOTO[1].getpara.GetEncoderMeasure(&Raising_MOTO[1].getpara, hcan);
			break;
		}
		__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_FMP0);//重新使能can中断标志
	}
}
