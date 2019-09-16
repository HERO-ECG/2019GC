#include "main.h"
#include "stm32f4xx_hal.h"
#include "mytype.h"
#include "control.h"
#include "dma.h"
#include "ks103.h"
#include "usb_device.h"
#include "chassis.h"
#include "ammo.h"
#include "t_moto.h"
#include "t_monitor.h"
#include "cylinder.h"

void SystemClock_Config(void);
void Error_Handler(void);

void all_bsp_init(void);
void all_pid_init(void);
void all_sensor_init(void);

float test_good;
extern uint32_t time_piece;
uint8_t user_count = 0;

float var[8];
void vcan_sendware(u8 *wareaddr, u32 waresize); 

void LightTest(void){
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 0){
		test_good = 1;
	}
	else{ test_good = 0; }
}

int main(void){
	all_bsp_init();
	all_sensor_init();
	time_piece |= time_piece_start;
	Chassis_Init(0, 8000);
	AmmoInit();
	all_pid_init();
	while (1){	
		//部分时间片
		if((time_piece&time_piece_0100hz) == time_piece_0100hz){
			time_piece &= ~time_piece_0100hz;
			monitor_remote.monitor_process(&monitor_remote);			//遥控器监视器
			/*气缸功能以及检查函数*/
			CylinderSetControl();
			GenerallySetRaisingHeight(RoboData.robo_ctrlmode, &Chassis);
//			user_count ++;
//			if(user_count == 10){
//				UserDataSend(13, 0xD180, 0x0112);
//				user_count=0;
//			}
		}
		else if((time_piece&time_piece_1000hz) == time_piece_1000hz){
			time_piece &= ~time_piece_1000hz;
			
//			var[0] = Raising_MOTO[0].getpara.speed_rpm;
//			var[1] = Raising_MOTO[1].getpara.speed_rpm;
//			var[3] = Raising_MOTO[0].send_current;
//			var[4] = Raising_MOTO[1].send_current;
//			var[5] = 0;
//			var[6] = 0;		
//			vcan_sendware((u8*) var, (u32)sizeof(var));
		}
	}
}	

void all_bsp_init(void){
	HAL_Init();//4位抢占优先级、0位响应优先级
	SystemClock_Config();

	MX_GPIO_Init();
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);			//亮
 	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);	//亮
//	HAL_GPIO_WritePin(POWER_Port, POWER_Pin_PH2, GPIO_PIN_SET);  						//POWER_24V
//	HAL_GPIO_WritePin(POWER_Port, POWER_Pin_PH3, GPIO_PIN_SET);  						//POWER_24V
	HAL_GPIO_WritePin(POWER_Port, POWER_Pin_PH4, GPIO_PIN_SET);  						//POWER_24V	
	HAL_GPIO_WritePin(POWER_Port, POWER_Pin_PH5, GPIO_PIN_SET);  						//POWER_24V
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);										//限位开关基准电平
	/*倒车雷达初始化低电平*/
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	/*气缸初始化*/
//	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);										//弹仓控制信号
//	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);											//机械臂控制信号
//	Send_595();
	
	/*------------初始化机构体&设置指针指向&机器人控制相关数据----------------*/
	InitRoboData(&RoboData);
	MX_SPI5_Init();
	MPU6500_Init();
	MX_CAN1_Init();//3508电机can通信初始化，中断抢占优先级为1
	MX_CAN2_Init();
	CAN_Filter_Init_Recv_All();//3508电机can通信过滤器初始化

	MX_USART1_UART_Init();//遥控器   波特率：100000
	MYDMA_Config(DMA2_Stream2,DMA_CHANNEL_4,&hdma_usart1_rx);//初始化USART1到DMA，USART1到DMA接收中断抢占优先级为0,中断服务函数在dma中
	while(HAL_UART_Receive_DMA(&huart1,uart1_rx_buff, len_uart1_rx_buff)!=HAL_OK);//DMA while(1)之前启动一下DMA接收
	
//	MX_USART2_UART_Init();//ks103   波特率：9600
//	MYDMA_Config(DMA1_Stream5,DMA_CHANNEL_4,&hdma_usart2_rx);//初始化DMA
//	while(HAL_UART_Receive_DMA(&huart2,uart2_rx_buff,6u)!=HAL_OK);//DMA方式
	
	MX_USART3_UART_Init();//数传电台//现也用作PID调参    波特率：115200
	MYDMA_Config(DMA1_Stream1,DMA_CHANNEL_4,&hdma_usart3_rx);	
	while(HAL_UART_Receive_DMA(&huart3,uart3_rx_buff, len_uart3_rx_buff)!=HAL_OK);

//	MX_USART6_UART_Init();//ks103   波特率：9600
//	MYDMA_Config(DMA2_Stream1,DMA_CHANNEL_5,&hdma_usart6_rx);
//	while(HAL_UART_Receive_DMA(&huart6,uart6_rx_buff,62u)!=HAL_OK);
	
	MX_USART7_UART_Init();//裁判系统
	MYDMA_Config(DMA1_Stream3,DMA_CHANNEL_5,&hdma_usart7_rx);
	{while(HAL_UART_Receive_DMA(&huart7,uart7_rx_buff,len_uart7_rx_buff)!=HAL_OK);}

	TIM2_Init(3000-1,84-1);//PWM舵机控制
	
	TIM3_Init(1000-1,840-1);//100hz		
	
	TIM4_Init(1000-1,8400-1);//10hz	
	
	TIM5_Init(100-1,840-1);//1000hz  

	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
	HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);//3508电机can通信接收中断配置
}
void all_pid_init(void){
	/*底盘电机初始化*/
	#ifdef BAOMING_TEST
	for(uint8_t i = 0; i < 2; i++){ 
		Chassis_MOTO[i].pid_angle.PidSet_all(&Chassis_MOTO[i].pid_angle, POSITION_PID, 1000.0f/Chassis.speed.base, 0, 0, 0.02, 0, 0, 0, 0, 0, 0, 0, 0);
		Chassis_MOTO[i].pid_speed.PidSet_all(&Chassis_MOTO[i].pid_speed, POSITION_PID, 5000, 2000, -2000, 4, 0.2, 0, 0, 0, 0, 0, 0, 500); 
	}
	for(uint8_t i = 2; i < 4; i++){ 
		Chassis_MOTO[i].pid_angle.PidSet_all(&Chassis_MOTO[i].pid_angle, POSITION_PID, 1000.0f/Chassis.speed.base, 0, 0, 0.02, 0, 0, 0, 0, 0, 0, 0, 0);
		Chassis_MOTO[i].pid_speed.PidSet_all(&Chassis_MOTO[i].pid_speed, POSITION_PID, 5000, 2000, -2000, 4, 0.2, 0, 0, 0, 0, 0, 0, 500); 
	}
	Chassis_MOTO[4].pid_angle.PidSet_all(&Chassis_MOTO[4].pid_angle, POSITION_PID, 1000.0f/Chassis.speed.base, 0, 0, 0.02, 0, 0, 0, 0, 0, 0, 0, 0);
	Chassis_MOTO[5].pid_angle.PidSet_all(&Chassis_MOTO[5].pid_angle, POSITION_PID, 1000.0f/Chassis.speed.base, 0, 0, 0.02, 0, 0, 0, 0, 0, 0, 0, 0);
	Chassis_MOTO[4].pid_speed.PidSet_all(&Chassis_MOTO[4].pid_speed, POSITION_PID, 5000, 2000, -2000, 4, 0.2, 0, 0, 0, 0, 0, 0, 0);
	Chassis_MOTO[5].pid_speed.PidSet_all(&Chassis_MOTO[5].pid_speed, POSITION_PID, 5000, 2000, -2000, 4, 0.2, 0, 0, 0, 0, 0, 0, 0);	
	#else
	for(uint8_t i = 0; i < 4; i++){ 
		Chassis_MOTO[i].pid_angle.PidSet_all(&Chassis_MOTO[i].pid_angle, POSITION_PID, 0.25, 0, 0, 0.08, 0, 0, 0, 0, 0, 0, 0, 0);
		Chassis_MOTO[i].pid_speed.PidSet_all(&Chassis_MOTO[i].pid_speed, POSITION_PID, CHASSISMAXCURRENT, 0, 0, 1.8, 0, 0, 0, 0, 0, 0, 0, 500); 
	}
	Chassis_MOTO[4].pid_angle.PidSet_all(&Chassis_MOTO[4].pid_angle, POSITION_PID, 0.25, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0, 0);
	Chassis_MOTO[5].pid_angle.PidSet_all(&Chassis_MOTO[5].pid_angle, POSITION_PID, 0.25, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0, 0);
	Chassis_MOTO[4].pid_speed.PidSet_all(&Chassis_MOTO[4].pid_speed, POSITION_PID, CHASSISMAXCURRENT, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0);
	Chassis_MOTO[5].pid_speed.PidSet_all(&Chassis_MOTO[5].pid_speed, POSITION_PID, CHASSISMAXCURRENT, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0);	
	#endif

	/*升降电机初始化*/
	Raising_MOTO[0].pid_angle.PidSet_all(&Raising_MOTO[0].pid_angle, POSITION_PID, 50, 500, -500, 2, 0, 0, 0, 0, 0, 0, 0, 500);
	Raising_MOTO[0].pid_speed.PidSet_all(&Raising_MOTO[0].pid_speed, POSITION_PID, 1200, 500, -500, 0.8, 0, 0, 0, 0, 0, 0, 0, 500);
	Raising_MOTO[1].pid_angle.PidSet_all(&Raising_MOTO[1].pid_angle, POSITION_PID, 50, 500, -500, 2, 0, 0, 0, 0, 0, 0, 0, 500);
	Raising_MOTO[1].pid_speed.PidSet_all(&Raising_MOTO[1].pid_speed, POSITION_PID, 1200, 500, -500, 0.8, 0, 0, 0, 0, 0, 0, 0, 500);
	
	Capturing_MOTO[0].pid_angle.PidSet_all(&Capturing_MOTO[0].pid_angle, POSITION_PID, 200, 200, -200, 2, 0, 0, 0, 0, 0, 0, 0, 0);
	Capturing_MOTO[0].pid_speed.PidSet_all(&Capturing_MOTO[0].pid_speed, POSITION_PID, 4000, 1000, -1000, 0.6, 0, 0, 0, 0, 0, 0, 0, 1500);
}//PID参数初始化

void all_sensor_init(void){
//	Ks103_Front.KS103_ParaInit(&Ks103_Front, 0xD8, &huart6, uart6_rx_buff);
//	Ks103_Left.KS103_ParaInit(&Ks103_Left, 0xE8, &huart2, uart2_rx_buff);
//	Ks103_Right.KS103_ParaInit(&Ks103_Right, 0xF8, &huart8, uart8_rx_buff);
	HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_SET);//灭
}

/** System Clock Configuration*/
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage */
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 12;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
	Error_Handler();
	}

	/**Initializes the CPU, AHB and APB busses clocks */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
	Error_Handler();
	}

	/**Configure the Systick interrupt time */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
		
  }
}

void vcan_sendware(u8 *wareaddr, u32 waresize)
{
#define CMD_WARE     3
    u8 cmdf[2] = {CMD_WARE, ~CMD_WARE};    //串口调试 使用的前命令
    u8 cmdr[2] = {~CMD_WARE, CMD_WARE};    //串口调试 使用的后命令

		HAL_UART_Transmit(&huart3,cmdf,sizeof(cmdf),1000);
		HAL_UART_Transmit(&huart3,wareaddr,waresize,1000);
		HAL_UART_Transmit(&huart3,cmdr,sizeof(cmdr),1000);
}

void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}

