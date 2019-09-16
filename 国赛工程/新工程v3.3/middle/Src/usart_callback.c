#include "main.h"
#include "usart.h"
#include "ks103.h"
#include "m_remote.h"
#include "dji_Protocol.h"
#include "t_monitor.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart1){		
			monitor_remote.circle_number = 0;
			RemoteDataProcess(uart1_rx_buff, &RC_CtrlData);
		HAL_UART_Receive_DMA(&huart1,uart1_rx_buff,len_uart1_rx_buff); 
	}
//	else if(huart == &huart2){
//		HAL_UART_Receive_DMA(&huart2,uart2_rx_buff,len_uart2_rx_buff);	
////		Ks103_Left.KS103_DataProcess(&Ks103_Left);
//	}
//	else if(huart == &huart3){
////		monitor_tx2.circle_number = 0;
//		HAL_UART_Receive_DMA(&huart3,uart3_rx_buff,len_uart3_rx_buff);
//	}
	else if(huart == &huart7){
		dji_DataProcess(uart7_rx_buff, &DJI_ReadData, &RoboData);
		HAL_UART_Receive_DMA(&huart7,uart7_rx_buff,len_uart7_rx_buff);
//		Ks103_Front.KS103_DataProcess(&Ks103_Front);
	}
//	else if(huart == &huart8){
//		HAL_UART_Receive_DMA(&huart8,uart8_rx_buff,len_uart8_rx_buff);	
////		Ks103_Right.KS103_DataProcess(&Ks103_Right);
//	}
}

