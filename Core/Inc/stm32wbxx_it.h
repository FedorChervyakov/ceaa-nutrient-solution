/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32wbxx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
 ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32WBxx_IT_H
#define __STM32WBxx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void PVD_PVM_IRQHandler(void);
void FLASH_IRQHandler(void);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI4_IRQHandler(void);
void DMA1_Channel1_IRQHandler(void);
void DMA1_Channel2_IRQHandler(void);
void DMA1_Channel3_IRQHandler(void);
void C2SEV_PWR_C2H_IRQHandler(void);
void TIM1_TRG_COM_TIM17_IRQHandler(void);
void I2C1_EV_IRQHandler(void);
void PWR_SOTF_BLEACT_802ACT_RFPHASE_IRQHandler(void);
void HSEM_IRQHandler(void);
/* USER CODE BEGIN EFP */
void RTC_WKUP_IRQHandler(void);
void IPCC_C1_TX_IRQHandler(void);
void IPCC_C1_RX_IRQHandler(void);
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32WBxx_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
