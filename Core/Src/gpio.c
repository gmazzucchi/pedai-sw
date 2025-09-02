/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_LED_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USER_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NOTE_1_Pin NOTE_2_Pin NOTE_3_Pin NOTE_4_Pin
                           NOTE_5_Pin NOTE_6_Pin NOTE_7_Pin NOTE_8_Pin
                           NOTE_9_Pin NOTE_10_Pin */
  GPIO_InitStruct.Pin = NOTE_1_Pin|NOTE_2_Pin|NOTE_3_Pin|NOTE_4_Pin
                          |NOTE_5_Pin|NOTE_6_Pin|NOTE_7_Pin|NOTE_8_Pin
                          |NOTE_9_Pin|NOTE_10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : NOTE_11_Pin NOTE_12_Pin NOTE_13_Pin NOTE_21_Pin
                           NOTE_22_Pin NOTE_23_Pin NOTE_24_Pin NOTE_25_Pin
                           NOTE_14_Pin NOTE_15_Pin NOTE_16_Pin NOTE_17_Pin
                           NOTE_18_Pin NOTE_19_Pin NOTE_20_Pin */
  GPIO_InitStruct.Pin = NOTE_11_Pin|NOTE_12_Pin|NOTE_13_Pin|NOTE_21_Pin
                          |NOTE_22_Pin|NOTE_23_Pin|NOTE_24_Pin|NOTE_25_Pin
                          |NOTE_14_Pin|NOTE_15_Pin|NOTE_16_Pin|NOTE_17_Pin
                          |NOTE_18_Pin|NOTE_19_Pin|NOTE_20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
