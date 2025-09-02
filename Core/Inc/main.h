/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define USER_LED_Pin GPIO_PIN_13
#define USER_LED_GPIO_Port GPIOC
#define NOTE_1_Pin GPIO_PIN_0
#define NOTE_1_GPIO_Port GPIOA
#define NOTE_2_Pin GPIO_PIN_1
#define NOTE_2_GPIO_Port GPIOA
#define NOTE_3_Pin GPIO_PIN_2
#define NOTE_3_GPIO_Port GPIOA
#define NOTE_4_Pin GPIO_PIN_3
#define NOTE_4_GPIO_Port GPIOA
#define NOTE_5_Pin GPIO_PIN_4
#define NOTE_5_GPIO_Port GPIOA
#define NOTE_6_Pin GPIO_PIN_5
#define NOTE_6_GPIO_Port GPIOA
#define NOTE_7_Pin GPIO_PIN_6
#define NOTE_7_GPIO_Port GPIOA
#define NOTE_8_Pin GPIO_PIN_7
#define NOTE_8_GPIO_Port GPIOA
#define NOTE_11_Pin GPIO_PIN_0
#define NOTE_11_GPIO_Port GPIOB
#define NOTE_12_Pin GPIO_PIN_1
#define NOTE_12_GPIO_Port GPIOB
#define NOTE_13_Pin GPIO_PIN_2
#define NOTE_13_GPIO_Port GPIOB
#define NOTE_21_Pin GPIO_PIN_10
#define NOTE_21_GPIO_Port GPIOB
#define NOTE_22_Pin GPIO_PIN_12
#define NOTE_22_GPIO_Port GPIOB
#define NOTE_23_Pin GPIO_PIN_13
#define NOTE_23_GPIO_Port GPIOB
#define NOTE_24_Pin GPIO_PIN_14
#define NOTE_24_GPIO_Port GPIOB
#define NOTE_25_Pin GPIO_PIN_15
#define NOTE_25_GPIO_Port GPIOB
#define NOTE_9_Pin GPIO_PIN_8
#define NOTE_9_GPIO_Port GPIOA
#define NOTE_10_Pin GPIO_PIN_15
#define NOTE_10_GPIO_Port GPIOA
#define NOTE_14_Pin GPIO_PIN_3
#define NOTE_14_GPIO_Port GPIOB
#define NOTE_15_Pin GPIO_PIN_4
#define NOTE_15_GPIO_Port GPIOB
#define NOTE_16_Pin GPIO_PIN_5
#define NOTE_16_GPIO_Port GPIOB
#define NOTE_17_Pin GPIO_PIN_6
#define NOTE_17_GPIO_Port GPIOB
#define NOTE_18_Pin GPIO_PIN_7
#define NOTE_18_GPIO_Port GPIOB
#define NOTE_19_Pin GPIO_PIN_8
#define NOTE_19_GPIO_Port GPIOB
#define NOTE_20_Pin GPIO_PIN_9
#define NOTE_20_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
