/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2s.c
  * @brief   This file provides code for the configuration
  *          of the I2S instances.
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
#include "i2s.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

I2S_HandleTypeDef hi2s1;
DMA_HandleTypeDef hdma_spi1_tx;

/* I2S1 init function */
void MX_I2S1_Init(void) {
    /* USER CODE BEGIN I2S1_Init 0 */

    /* USER CODE END I2S1_Init 0 */

    /* USER CODE BEGIN I2S1_Init 1 */

    /* USER CODE END I2S1_Init 1 */
    hi2s1.Instance            = SPI1;
    hi2s1.Init.Mode           = I2S_MODE_MASTER_TX;
    hi2s1.Init.Standard       = I2S_STANDARD_PHILIPS;
    hi2s1.Init.DataFormat     = I2S_DATAFORMAT_16B;
    hi2s1.Init.MCLKOutput     = I2S_MCLKOUTPUT_DISABLE;
    hi2s1.Init.AudioFreq      = I2S_AUDIOFREQ_22K;
    hi2s1.Init.CPOL           = I2S_CPOL_LOW;
    hi2s1.Init.ClockSource    = I2S_CLOCK_PLL;
    hi2s1.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
    if (HAL_I2S_Init(&hi2s1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2S1_Init 2 */

    /* USER CODE END I2S1_Init 2 */
}

void HAL_I2S_MspInit(I2S_HandleTypeDef *i2sHandle) {
    GPIO_InitTypeDef GPIO_InitStruct             = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    if (i2sHandle->Instance == SPI1) {
        /* USER CODE BEGIN SPI1_MspInit 0 */

        /* USER CODE END SPI1_MspInit 0 */

        /** Initializes the peripherals clock
  */
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
        PeriphClkInitStruct.PLLI2S.PLLI2SN       = 192;
        PeriphClkInitStruct.PLLI2S.PLLI2SM       = 16;
        PeriphClkInitStruct.PLLI2S.PLLI2SR       = 2;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
            Error_Handler();
        }

        /* I2S1 clock enable */
        __HAL_RCC_SPI1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**I2S1 GPIO Configuration
    PA15     ------> I2S1_WS
    PB3     ------> I2S1_CK
    PB5     ------> I2S1_SD
    */
        GPIO_InitStruct.Pin       = GPIO_PIN_15;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin       = GPIO_PIN_3 | GPIO_PIN_5;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* I2S1 DMA Init */
        /* SPI1_TX Init */
        hdma_spi1_tx.Instance                 = DMA2_Stream2;
        hdma_spi1_tx.Init.Channel             = DMA_CHANNEL_2;
        hdma_spi1_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hdma_spi1_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_spi1_tx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_spi1_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        hdma_spi1_tx.Init.Mode                = DMA_CIRCULAR;
        hdma_spi1_tx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
        hdma_spi1_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(i2sHandle, hdmatx, hdma_spi1_tx);

        /* I2S1 interrupt Init */
        HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(SPI1_IRQn);
        /* USER CODE BEGIN SPI1_MspInit 1 */

        /* USER CODE END SPI1_MspInit 1 */
    }
}

void HAL_I2S_MspDeInit(I2S_HandleTypeDef *i2sHandle) {
    if (i2sHandle->Instance == SPI1) {
        /* USER CODE BEGIN SPI1_MspDeInit 0 */

        /* USER CODE END SPI1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_SPI1_CLK_DISABLE();

        /**I2S1 GPIO Configuration
    PA15     ------> I2S1_WS
    PB3     ------> I2S1_CK
    PB5     ------> I2S1_SD
    */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);

        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3 | GPIO_PIN_5);

        /* I2S1 DMA DeInit */
        HAL_DMA_DeInit(i2sHandle->hdmatx);

        /* I2S1 interrupt Deinit */
        HAL_NVIC_DisableIRQ(SPI1_IRQn);
        /* USER CODE BEGIN SPI1_MspDeInit 1 */

        /* USER CODE END SPI1_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
