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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUTTON_NUCLEO_Pin GPIO_PIN_13
#define BUTTON_NUCLEO_GPIO_Port GPIOC
#define BUTTON_NUCLEO_EXTI_IRQn EXTI15_10_IRQn
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
#define FAN_TACH6_Pin GPIO_PIN_4
#define FAN_TACH6_GPIO_Port GPIOA
#define LED_NUCLEO_Pin GPIO_PIN_5
#define LED_NUCLEO_GPIO_Port GPIOA
#define FAN_TACH4_Pin GPIO_PIN_6
#define FAN_TACH4_GPIO_Port GPIOA
#define FAN_TACH3_Pin GPIO_PIN_7
#define FAN_TACH3_GPIO_Port GPIOA
#define FAN_TACH2_Pin GPIO_PIN_4
#define FAN_TACH2_GPIO_Port GPIOC
#define FAN_TACH1_Pin GPIO_PIN_5
#define FAN_TACH1_GPIO_Port GPIOC
#define FAN_PWM_Pin GPIO_PIN_0
#define FAN_PWM_GPIO_Port GPIOB
#define SPI_MSTR_Pin GPIO_PIN_6
#define SPI_MSTR_GPIO_Port GPIOC
#define SPI_SLOW_Pin GPIO_PIN_7
#define SPI_SLOW_GPIO_Port GPIOC
#define SPI_PHA_Pin GPIO_PIN_8
#define SPI_PHA_GPIO_Port GPIOC
#define SPI_POL_Pin GPIO_PIN_9
#define SPI_POL_GPIO_Port GPIOC
#define SPI_LED_EN_Pin GPIO_PIN_8
#define SPI_LED_EN_GPIO_Port GPIOA
#define SPI_NSS_Pin GPIO_PIN_15
#define SPI_NSS_GPIO_Port GPIOA
#define SPI_SCK_Pin GPIO_PIN_10
#define SPI_SCK_GPIO_Port GPIOC
#define SPI_MISO_Pin GPIO_PIN_11
#define SPI_MISO_GPIO_Port GPIOC
#define SPI_MOSI_Pin GPIO_PIN_12
#define SPI_MOSI_GPIO_Port GPIOC
#define AMS_FAULT_Pin GPIO_PIN_4
#define AMS_FAULT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
