/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BAT_ADC_Pin GPIO_PIN_1
#define BAT_ADC_GPIO_Port GPIOA
#define Power_Monitor_Pin GPIO_PIN_2
#define Power_Monitor_GPIO_Port GPIOA
#define Bat_Discharge_EN_Pin GPIO_PIN_3
#define Bat_Discharge_EN_GPIO_Port GPIOA
#define LED_R_Pin GPIO_PIN_5
#define LED_R_GPIO_Port GPIOC
#define MC20_EN_Pin GPIO_PIN_0
#define MC20_EN_GPIO_Port GPIOB
#define LED_Y_Pin GPIO_PIN_1
#define LED_Y_GPIO_Port GPIOB
#define MC20_TX_Pin GPIO_PIN_10
#define MC20_TX_GPIO_Port GPIOB
#define MC20_RX_Pin GPIO_PIN_11
#define MC20_RX_GPIO_Port GPIOB
#define Bat_Charge_En_Pin GPIO_PIN_14
#define Bat_Charge_En_GPIO_Port GPIOB
#define DEBUG_TX_Pin GPIO_PIN_9
#define DEBUG_TX_GPIO_Port GPIOA
#define DEBUG_RX_Pin GPIO_PIN_10
#define DEBUG_RX_GPIO_Port GPIOA
#define DEBUG_EN_Pin GPIO_PIN_11
#define DEBUG_EN_GPIO_Port GPIOA
#define BAT_Charge_Status_Pin GPIO_PIN_3
#define BAT_Charge_Status_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
