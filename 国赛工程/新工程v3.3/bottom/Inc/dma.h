
#ifndef __dma_H
#define __dma_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "mytype.h"
#include "usart.h"	

extern void Error_Handler(void);
extern DMA_HandleTypeDef hdma_usart1_rx;
	 
extern DMA_HandleTypeDef hdma_usart2_rx;
	 
extern DMA_HandleTypeDef hdma_usart3_rx;	
extern DMA_HandleTypeDef hdma_usart3_tx;	

extern DMA_HandleTypeDef hdma_usart6_rx; 
extern DMA_HandleTypeDef hdma_usart6_tx; 

extern DMA_HandleTypeDef hdma_usart7_rx;

extern DMA_HandleTypeDef hdma_usart8_rx;

extern void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,DMA_HandleTypeDef *hdma);
extern uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream);


void DMA1_Stream1_IRQHandler(void);
void DMA1_Stream3_IRQHandler(void);
void DMA1_Stream5_IRQHandler(void);
void DMA1_Stream6_IRQHandler(void);
void DMA2_Stream1_IRQHandler(void);
void DMA2_Stream2_IRQHandler(void);
#ifdef __cplusplus
}
#endif

#endif /* __dma_H */

