/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

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
#define LED_CAN_Pin GPIO_PIN_0
#define LED_CAN_GPIO_Port GPIOC
#define LED_WARN_Pin GPIO_PIN_1
#define LED_WARN_GPIO_Port GPIOC
#define LED_IND_Pin GPIO_PIN_2
#define LED_IND_GPIO_Port GPIOC
#define LED_FAULT_Pin GPIO_PIN_3
#define LED_FAULT_GPIO_Port GPIOC
#define ADC_CH1_Pin GPIO_PIN_0
#define ADC_CH1_GPIO_Port GPIOA
#define ADC_CH2_Pin GPIO_PIN_1
#define ADC_CH2_GPIO_Port GPIOA
#define ADC_5VREF_Pin GPIO_PIN_2
#define ADC_5VREF_GPIO_Port GPIOA
#define AMS_FAULT2_Pin GPIO_PIN_3
#define AMS_FAULT2_GPIO_Port GPIOB
#define AMS_FAULT1_Pin GPIO_PIN_4
#define AMS_FAULT1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
