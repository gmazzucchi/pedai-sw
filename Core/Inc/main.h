/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

extern I2S_HandleTypeDef hi2s1;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BOARD_LED_Pin         GPIO_PIN_13
#define BOARD_LED_GPIO_Port   GPIOC
#define LED_MIDI_Pin          GPIO_PIN_14
#define LED_MIDI_GPIO_Port    GPIOC
#define LED_SOUND_Pin         GPIO_PIN_15
#define LED_SOUND_GPIO_Port   GPIOC
#define USER_BUTTON_Pin       GPIO_PIN_0
#define USER_BUTTON_GPIO_Port GPIOA
#define R0_Pin                GPIO_PIN_1
#define R0_GPIO_Port          GPIOA
#define R1_Pin                GPIO_PIN_2
#define R1_GPIO_Port          GPIOA
#define R2_Pin                GPIO_PIN_3
#define R2_GPIO_Port          GPIOA
#define R3_Pin                GPIO_PIN_4
#define R3_GPIO_Port          GPIOA
#define MUX_C2_Pin            GPIO_PIN_5
#define MUX_C2_GPIO_Port      GPIOA
#define MUX_C1_Pin            GPIO_PIN_6
#define MUX_C1_GPIO_Port      GPIOA
#define MUX_C0_Pin            GPIO_PIN_7
#define MUX_C0_GPIO_Port      GPIOA
#define R4_Pin                GPIO_PIN_0
#define R4_GPIO_Port          GPIOB
#define R5_Pin                GPIO_PIN_1
#define R5_GPIO_Port          GPIOB
#define C0_Pin                GPIO_PIN_2
#define C0_GPIO_Port          GPIOB
#define C1_Pin                GPIO_PIN_10
#define C1_GPIO_Port          GPIOB
#define C2_Pin                GPIO_PIN_12
#define C2_GPIO_Port          GPIOB
#define C3_Pin                GPIO_PIN_13
#define C3_GPIO_Port          GPIOB
#define C4_Pin                GPIO_PIN_14
#define C4_GPIO_Port          GPIOB
#define C5_Pin                GPIO_PIN_15
#define C5_GPIO_Port          GPIOB
#define C6_Pin                GPIO_PIN_8
#define C6_GPIO_Port          GPIOA
#define C7_Pin                GPIO_PIN_9
#define C7_GPIO_Port          GPIOA
#define MUX3_DATA_Pin         GPIO_PIN_10
#define MUX3_DATA_GPIO_Port   GPIOA
#define R6_Pin                GPIO_PIN_4
#define R6_GPIO_Port          GPIOB
#define MUX1_DATA_Pin         GPIO_PIN_6
#define MUX1_DATA_GPIO_Port   GPIOB
#define MUX2_DATA_Pin         GPIO_PIN_7
#define MUX2_DATA_GPIO_Port   GPIOB
#define R7_Pin                GPIO_PIN_8
#define R7_GPIO_Port          GPIOB
#define MUX4_DATA_Pin         GPIO_PIN_9
#define MUX4_DATA_GPIO_Port   GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
