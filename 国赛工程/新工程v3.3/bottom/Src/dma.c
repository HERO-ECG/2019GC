#include "dma.h"

DMA_HandleTypeDef hdma_usart1_rx;

DMA_HandleTypeDef hdma_usart2_rx;

DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

DMA_HandleTypeDef hdma_usart7_rx;

DMA_HandleTypeDef hdma_usart8_rx;
//DMAx的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//从存储器->外设模式/8位数据宽度/存储器增量模式
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
//chx:DMA通道选择,@ref DMA_channel DMA_CHANNEL_0~DMA_CHANNEL_7
void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,DMA_HandleTypeDef *hdma)
{ 
	if((u32)DMA_Streamx>(u32)DMA2)//得到当前stream是属于DMA2还是DMA1
	{
        __HAL_RCC_DMA2_CLK_ENABLE();//DMA2时钟使能	
	}
	else 
	{
        __HAL_RCC_DMA1_CLK_ENABLE();//DMA1时钟使能 
	}
	 //Rx DMA配置   除优先级是低等优先级，他是中等优先级，其他已配置成和正点原子论坛上帖子相同的模式。
    hdma->Instance = DMA_Streamx;                            //数据流选择 DMA2_Stream2   串口2是DMA1_Stream5
    hdma->Init.Channel = chx;                                //通道选择 DMA_CHANNEL_4    串口2是DMA_CHANNEL_4
	  if((hdma == &hdma_usart6_tx) || (hdma == &hdma_usart3_tx))
		{
    hdma->Init.Direction = DMA_MEMORY_TO_PERIPH;             //存储器到外设			
		}
		else
		{
    hdma->Init.Direction = DMA_PERIPH_TO_MEMORY;             //外设到存储器		
		}
    hdma->Init.PeriphInc=DMA_PINC_DISABLE;                 //外设非增量模式
    hdma->Init.MemInc=DMA_MINC_ENABLE;                     //存储器增量模式
    hdma->Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //外设数据长度:8位
    hdma->Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //存储器数据长度:8位
    hdma->Init.Mode=DMA_CIRCULAR;                          //循环模式
  	hdma->Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
    hdma->Init.FIFOThreshold=DMA_FIFO_THRESHOLD_1QUARTERFULL;     
    hdma->Init.MemBurst=DMA_MBURST_SINGLE;                 //存储器突发单次传输
    hdma->Init.PeriphBurst=DMA_PBURST_SINGLE;              //外设突发单次传输	
	  if(hdma == &hdma_usart1_rx)
	  { 
		  __HAL_LINKDMA(&huart1,hdmarx,hdma_usart1_rx);     //将DMA与USART1联系起来(接收DMA)
		  hdma->Init.Priority=DMA_PRIORITY_VERY_HIGH;          //优先级
		 
		  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 1, 0);
      HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
	  }
	  else if(hdma == &hdma_usart3_rx)
	  {
		  __HAL_LINKDMA(&huart3,hdmarx,hdma_usart3_rx);    
	 	  hdma->Init.Priority=DMA_PRIORITY_VERY_HIGH;           //优先级
		 
	 		HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 2, 0);
      HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	  }	
	  else if(hdma == &hdma_usart6_rx)
	  {
		  __HAL_LINKDMA(&huart6,hdmarx,hdma_usart6_rx);    
	 	  hdma->Init.Priority=DMA_PRIORITY_VERY_HIGH;           //优先级
		 
	 		HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 1, 0);
      HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
	  }	
		else if(hdma == &hdma_usart7_rx)
	  {
		  __HAL_LINKDMA(&huart7,hdmarx,hdma_usart7_rx);    
	 	  hdma->Init.Priority=DMA_PRIORITY_MEDIUM;              //优先级
		 
	 		HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 1, 0);
      HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	  }	
		else if(hdma == &hdma_usart8_rx)
	  {
		  __HAL_LINKDMA(&huart8,hdmarx,hdma_usart8_rx);    
	 	  hdma->Init.Priority=DMA_PRIORITY_MEDIUM;              //优先级
		 
	 		HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 1, 0);
      HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	  }	
	  else if(hdma == &hdma_usart2_rx)
	  {
		  __HAL_LINKDMA(&huart2,hdmarx,hdma_usart2_rx);    
	 	  hdma->Init.Priority=DMA_PRIORITY_LOW;             		//优先级
		 
	 		HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 3, 0);
      HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	  }	
	  else if(hdma == &hdma_usart6_tx)
	  {
		  __HAL_LINKDMA(&huart6,hdmatx,hdma_usart6_tx);    
	 	  hdma->Init.Priority=DMA_PRIORITY_MEDIUM;              //优先级
	  }	
		else if(hdma == &hdma_usart3_tx)
	  {
		  __HAL_LINKDMA(&huart3,hdmatx,hdma_usart3_tx);    
	 	  hdma->Init.Priority=DMA_PRIORITY_MEDIUM;               //普通优先级
	  }	
	  
	  
	  if(HAL_DMA_Init(hdma) != HAL_OK)
    {
      Error_Handler();
    }	 
    HAL_DMA_DeInit(hdma);   
    HAL_DMA_Init(hdma);	
} 

//开启一次DMA传输
//huart:串口句柄
//pData：传输的数据指针
//Size:传输的数据量
void MYDMA_USART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    HAL_DMA_Start(huart->hdmatx, (u32)pData, (uint32_t)&huart->Instance->DR, Size);//开启DMA传输
    
    huart->Instance->CR3 |= USART_CR3_DMAT;//使能串口DMA发送
}	  

uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
  /* Return the number of remaining data units for DMAy Streamx */
  return ((uint16_t)(dma_stream->NDTR));
}

void DMA1_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart3_rx);	
} 

void DMA1_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart7_rx);	
} 

void DMA1_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart2_rx);	
} 

void DMA1_Stream6_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart8_rx);	
} 


void DMA2_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart6_rx);	
} 

void DMA2_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart1_rx);	
} 


