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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin              GPIO_PIN_13
#define LED1_GPIO_Port        GPIOC
#define LED2_Pin              GPIO_PIN_14
#define LED2_GPIO_Port        GPIOC
#define USER_SWITCH_Pin       GPIO_PIN_15
#define USER_SWITCH_GPIO_Port GPIOC
#define LCD_RS_Pin            GPIO_PIN_0
#define LCD_RS_GPIO_Port      GPIOB
#define LCD_ENABLE_Pin        GPIO_PIN_1
#define LCD_ENABLE_GPIO_Port  GPIOB
#define LCD_D0_Pin            GPIO_PIN_2
#define LCD_D0_GPIO_Port      GPIOB
#define LCD_D1_Pin            GPIO_PIN_10
#define LCD_D1_GPIO_Port      GPIOB
#define LCD_D2_Pin            GPIO_PIN_12
#define LCD_D2_GPIO_Port      GPIOB
#define LCD_D3_Pin            GPIO_PIN_13
#define LCD_D3_GPIO_Port      GPIOB
#define LCD_D4_Pin            GPIO_PIN_14
#define LCD_D4_GPIO_Port      GPIOB
#define LCD_D5_Pin            GPIO_PIN_15
#define LCD_D5_GPIO_Port      GPIOB
#define LCD_D6_Pin            GPIO_PIN_8
#define LCD_D6_GPIO_Port      GPIOA
#define LCD_D7_Pin            GPIO_PIN_9
#define LCD_D7_GPIO_Port      GPIOA
#define MUX_C0_Pin            GPIO_PIN_10
#define MUX_C0_GPIO_Port      GPIOA
#define LCD_C1_Pin            GPIO_PIN_13
#define LCD_C1_GPIO_Port      GPIOA
#define LCD_C2_Pin            GPIO_PIN_14
#define LCD_C2_GPIO_Port      GPIOA
#define LCD_C3_Pin            GPIO_PIN_4
#define LCD_C3_GPIO_Port      GPIOB
#define MUX_D0_Pin            GPIO_PIN_6
#define MUX_D0_GPIO_Port      GPIOB
#define MUX_D1_Pin            GPIO_PIN_7
#define MUX_D1_GPIO_Port      GPIOB
#define MUX_D2_Pin            GPIO_PIN_8
#define MUX_D2_GPIO_Port      GPIOB
#define MUX_D3_Pin            GPIO_PIN_9
#define MUX_D3_GPIO_Port      GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
