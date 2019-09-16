#include "t_monitor.h"
#include "m_remote.h"
#include "can.h"

Monitor_t monitor_remote;
Monitor_t monitor_chassis_moto;
Monitor_t monitor_raising_moto;
Monitor_t monitor_hcan;

void monitor_remote_process(Monitor_t *Monitor){
	Monitor->circle_number++;
	if(Monitor->circle_number > 3)
	{
		Monitor->status = monitor_err;
		RC_CtrlData.RCDataParaInit(&RC_CtrlData);
	}
	else
		Monitor->status = monitor_regular;
}//100hz

/*hall��ʾ��������ĵ���¶ȣ�����⵽�������ʱ���������λ*/
void monitor_chassis_moto_process(Monitor_t *Monitor){
	static uint8_t ok_cnt1 = 0;
	for(uint8_t i = 0; i<6; i++){
		if(Chassis_MOTO[i].getpara.hall == 0)
			Monitor->circle_number++;
		else 
			ok_cnt1++;
	}
	if(ok_cnt1 == 6){
		Monitor->circle_number = 0;
	}
	if(Monitor->circle_number > 66){
		Monitor->status = monitor_err;
		Monitor->circle_number = 0;
		HAL_NVIC_SystemReset();
	}
	else 
		Monitor->status = monitor_regular;
	
	ok_cnt1 = 0;
}

void monitor_raising_moto_process(Monitor_t *Monitor){
	static uint8_t ok_cnt2 = 0;
	for(uint8_t i = 0; i<2; i++){
		if(Raising_MOTO[i].getpara.hall == 0)
			Monitor->circle_number++;
		else 
			ok_cnt2++;
	}
	if(ok_cnt2 == 2){
		Monitor->circle_number = 0;
	}
	if(Monitor->circle_number > 24){
		Monitor->status = monitor_err;
		Monitor->circle_number = 0;
		HAL_NVIC_SystemReset();
	}
	else 
		Monitor->status = monitor_regular;
	
	ok_cnt2 = 0;
}

void monitor_can_process(Monitor_t *Monitor){
	if(hcan1.State == HAL_TIM_STATE_ERROR || hcan2.State == HAL_TIM_STATE_ERROR){
		Monitor->circle_number++;
	}
	if(Monitor->circle_number > 10){
		Monitor->circle_number = 0;
		MX_CAN1_Init();//3508���canͨ�ų�ʼ�����ж���ռ���ȼ�Ϊ1
		MX_CAN2_Init();
		CAN_Filter_Init_Recv_All();//3508���canͨ�Ź�������ʼ��
	}
}

void MonitorParaInit(Monitor_t *Monitor){
	Monitor->MonitorParaInit = &MonitorParaInit;
	/*--�жϼ��Ӷ���--*/
	if(Monitor == &monitor_remote)
		Monitor->monitor_process = &monitor_remote_process;
	else if(Monitor == &monitor_chassis_moto)
		Monitor->monitor_process = &monitor_chassis_moto_process;
	else if(Monitor == &monitor_raising_moto)
		Monitor->monitor_process = &monitor_raising_moto_process;
	else if(Monitor == &monitor_hcan)
		Monitor->monitor_process = &monitor_can_process;
}
