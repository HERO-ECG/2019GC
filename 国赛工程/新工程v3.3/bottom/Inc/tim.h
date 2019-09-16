
#ifndef __tim_H
#define __tim_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include  "mytype.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8; 

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

extern void Error_Handler(void);
extern void TIM2_Init(u16 arr,u16 psc);
extern void TIM3_Init(u16 arr,u16 psc);
extern void TIM4_Init(u16 arr,u16 psc);
extern void TIM5_Init(u16 arr,u16 psc);
extern void TIM8_Init(u16 arr,u16 psc);
extern void TIM2_IRQHandler(void);
extern void TIM3_IRQHandler(void);
extern void TIM4_IRQHandler(void);
extern void TIM5_IRQHandler(void);
extern void TIM8_IRQHandler(void);
extern void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle);
extern void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle);

/* Callback in non blocking modes (Interrupt and DMA) *************************/
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
//void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
//void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
//void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim);
//void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ tim_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
