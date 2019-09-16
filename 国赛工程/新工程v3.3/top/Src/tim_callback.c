#include "main.h"
#include "chassis.h"
#include "ammo.h"
#include "robodata.h"
#include "dji_Protocol.h"
#include "t_monitor.h"
#include "t_moto.h"
#include "ks103.h"
#include "control.h"

uint32_t time_piece = 0x0000;
extern uint8_t hcan2_enable;

uint8_t tsafd = 8;
uint8_t hcan2_switch;

uint8_t tim_init_cnt = 0;

int cnt_tim5 = 0;
int cnt_tim3 = 0;

static uint16_t recnt = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t remote_init_cnt = 0;
	static uint8_t remote_init_sta = 0;
	if(htim == &htim3){         //100hz 
		time_piece |= time_piece_0100hz;
		IMU_Get_Data();
		if(imu_data.init_sta == 1){
			ChassisFun();
		}
		if(remote_init_sta == 0){
			remote_init_cnt++;
			if(remote_init_cnt == 50)
				remote_init_sta = 1;
		}
		else if(remote_init_sta == 1){
			RoboData.GetRemoteControlData(&RoboData);
		}
		/*底盘电机信号发送*/
		if(monitor_chassis_moto.status == monitor_regular){
			SetMotoCurrent(&hcan1, STDID_3508_LOW, Chassis_MOTO[0].send_current, Chassis_MOTO[1].send_current, Chassis_MOTO[2].send_current, Chassis_MOTO[3].send_current);
			SetMotoCurrent(&hcan1, STDID_3508_HIGH, Chassis_MOTO[4].send_current, Chassis_MOTO[5].send_current, 0, 0);
		}
		else if(monitor_chassis_moto.status == monitor_err){
			SetMotoCurrent(&hcan1, STDID_3508_LOW, 0, 0, 0, 0);
			SetMotoCurrent(&hcan1, STDID_3508_HIGH, 0, 0, 0, 0);
		}
	}
	else if(htim == &htim4){		//10hz
		time_piece |= time_piece_0010hz;
		monitor_chassis_moto.monitor_process(&monitor_chassis_moto);	
//		monitor_raising_moto.monitor_process(&monitor_raising_moto);	
		CustomSend();
//		monitor_hcan.monitor_process(&monitor_hcan);
	}
	else if(htim == &htim5){		//1000hz
		time_piece |= time_piece_1000hz;
		AmmoFun();
		if(hcan2_enable == 1){
			if(Ammo.reinit_sta == 0){
				SetMotoCurrent(&hcan2, STDID_3508_LOW, Raising_MOTO[0].send_current, Raising_MOTO[1].send_current, Capturing_MOTO[0].send_current, 0);
			}
			else{
				SetMotoCurrent(&hcan2, STDID_3508_LOW, 0, 0, 0, 0);
//				if(Ammo.Raising.down_limit == 1){
//					recnt++;
//					if(recnt == 1300){
//						Ammo.reinit_sta = 0;
//						recnt = 0;
//						Ammo.Raising.down_limit = 0;
//					}
//				}

			}
//			SetMotoCurrent(&hcan2, STDID_3508_LOW, 0, 0, Capturing_MOTO[0].send_current, 0);
			SetMotoCurrent(&hcan2, STDID_3508_HIGH, 0, -Capturing_MOTO[0].send_current, 0, 0);
		}
	}
}

//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
//void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
//void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim);
//void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim);
