#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include  "mytype.h"

#define len_uart1_rx_buff 36
#define len_uart2_rx_buff 6	
#define len_uart3_rx_buff 18	
#define len_uart6_rx_buff 44	
#define len_uart7_rx_buff 62		 
#define len_uart8_rx_buff 6

#define len_uart6_tx_buff 24

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart7;	 
extern UART_HandleTypeDef huart8;
	 
extern uint8_t uart1_rx_buff[len_uart1_rx_buff];
extern uint8_t uart2_rx_buff[len_uart2_rx_buff];	
extern uint8_t uart3_rx_buff[len_uart3_rx_buff];	
extern uint8_t uart6_rx_buff[len_uart6_rx_buff];	 
extern uint8_t uart7_rx_buff[len_uart7_rx_buff]; 
extern uint8_t uart8_rx_buff[len_uart8_rx_buff];

extern void Error_Handler(void);
extern void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
extern void uart_rx_idle_callback(UART_HandleTypeDef* huart);

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);
void MX_USART7_UART_Init(void);
void MX_USART8_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
