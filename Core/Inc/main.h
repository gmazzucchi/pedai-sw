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
#define BOARD_LED_Pin            GPIO_PIN_13
#define BOARD_LED_GPIO_Port      GPIOC
#define LED_MIDI_Pin             GPIO_PIN_14
#define LED_MIDI_GPIO_Port       GPIOC
#define LED_SOUND_Pin            GPIO_PIN_15
#define LED_SOUND_GPIO_Port      GPIOC
#define USER_BUTTON_Pin          GPIO_PIN_0
#define USER_BUTTON_GPIO_Port    GPIOA
#define SOUND_ON_Pin             GPIO_PIN_3
#define SOUND_ON_GPIO_Port       GPIOA
#define VOLUME_CONTROL_Pin       GPIO_PIN_4
#define VOLUME_CONTROL_GPIO_Port GPIOA
#define MUX_C2_Pin               GPIO_PIN_5
#define MUX_C2_GPIO_Port         GPIOA
#define MUX_C1_Pin               GPIO_PIN_6
#define MUX_C1_GPIO_Port         GPIOA
#define MUX_C0_Pin               GPIO_PIN_7
#define MUX_C0_GPIO_Port         GPIOA
#define LCD_RS_Pin               GPIO_PIN_0
#define LCD_RS_GPIO_Port         GPIOB
#define LCD_ENABLE_Pin           GPIO_PIN_1
#define LCD_ENABLE_GPIO_Port     GPIOB
#define LCD_D0_Pin               GPIO_PIN_2
#define LCD_D0_GPIO_Port         GPIOB
#define LCD_D1_Pin               GPIO_PIN_10
#define LCD_D1_GPIO_Port         GPIOB
#define LCD_D2_Pin               GPIO_PIN_12
#define LCD_D2_GPIO_Port         GPIOB
#define LCD_D3_Pin               GPIO_PIN_13
#define LCD_D3_GPIO_Port         GPIOB
#define LCD_D4_Pin               GPIO_PIN_14
#define LCD_D4_GPIO_Port         GPIOB
#define LCD_D5_Pin               GPIO_PIN_15
#define LCD_D5_GPIO_Port         GPIOB
#define LCD_D6_Pin               GPIO_PIN_8
#define LCD_D6_GPIO_Port         GPIOA
#define LCD_D7_Pin               GPIO_PIN_9
#define LCD_D7_GPIO_Port         GPIOA
#define MUX3_DATA_Pin            GPIO_PIN_10
#define MUX3_DATA_GPIO_Port      GPIOA
#define MUX6_DATA_Pin            GPIO_PIN_4
#define MUX6_DATA_GPIO_Port      GPIOB
#define MUX1_DATA_Pin            GPIO_PIN_6
#define MUX1_DATA_GPIO_Port      GPIOB
#define MUX2_DATA_Pin            GPIO_PIN_7
#define MUX2_DATA_GPIO_Port      GPIOB
#define MUX5_DATA_Pin            GPIO_PIN_8
#define MUX5_DATA_GPIO_Port      GPIOB
#define MUX4_DATA_Pin            GPIO_PIN_9
#define MUX4_DATA_GPIO_Port      GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
