#include "usart.h"
#include "idle_interrupt.h"

uint8_t  dbus_buf[DBUS_BUFLEN];
static int UART_Receive_DMA_No_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);

__weak void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	/* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the uart_rx_idle_callback could be implemented in the user file
   */
}

static int UART_Receive_DMA_No_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
  uint32_t *tmp;
	
	/* Check that a Rx process is not already ongoing */
  if (huart->RxState == HAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0))
    {
        return HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(huart);

    huart->pRxBuffPtr = pData;
    huart->RxXferSize = Size;
    
    huart->ErrorCode  = HAL_UART_ERROR_NONE;
		huart->RxState = HAL_UART_STATE_BUSY_RX;

    /* Enable the DMA Stream */
    tmp = (uint32_t*)&pData;
    /* Enable the DMA Stream */
    HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, *(uint32_t*)tmp, Size);

    /* Enable the DMA transfer for the receiver request by setting the DMAR bit
    in the UART CR3 register */
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    /* Process Unlocked */
    __HAL_UNLOCK(huart);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

void dbus_uart_init(void)
{
  //open uart idle it
  __HAL_UART_CLEAR_IDLEFLAG(&huart1);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

  UART_Receive_DMA_No_IT(&huart1, dbus_buf, DBUS_MAX_LEN);
}


void uart_receive_handler(UART_HandleTypeDef *huart)
{  
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && 
      __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
  {
		/* clear idle it flag avoid idle interrupt all the time */
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		
		/* clear DMA transfer complete flag */
    __HAL_DMA_DISABLE(huart->hdmarx);
		
		/* handle received data in idle interrupt */
    uart_rx_idle_callback(huart);
		
		/* restart dma transmission */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);
    __HAL_DMA_ENABLE(huart->hdmarx);
  }
}
