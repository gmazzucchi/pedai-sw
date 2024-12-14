/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "key_reader.h"
#include "lcd1602a.h"
#include "player.h"
#include "tusb.h"

#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

I2S_HandleTypeDef hi2s1;
DMA_HandleTypeDef hdma_spi1_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S1_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_CRC_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#if PED_USB_DEVICE_CLASS == PED_USB_CDC_CLASS

uint32_t prevtime = 0;

void cdc_example_keep_alive_task(void) {
    char log_str[BUFSIZ];
    snprintf(log_str, BUFSIZ, "I am alive at time %lu, in Gigi's house\n\n", HAL_GetTick());
    if (tud_cdc_write_available() && (HAL_GetTick() - prevtime) > 6000) {
        prevtime = HAL_GetTick();
        tud_cdc_write_str(log_str);
        tud_cdc_write_flush();
    }
}

#elif PED_USB_DEVICE_CLASS == PED_USB_MIDI_CLASS

// void send_midi_note_on(uint8_t note, uint8_t velocity) {
//     uint8_t packet[4] = {
//         0x09,           // USB MIDI code index number (Note On)
//         0x90,           // MIDI message (Note On, channel 0)
//         note,           // Note number (e.g., middle C = 60)
//         velocity        // Velocity (0-127)
//     };
//     tud_midi_stream_write(0, packet, 4);
// }

// Variable that holds the current position in the sequence.
uint32_t note_pos = 0;

uint32_t board_millis() {
    return HAL_GetTick();
}

// Store example melody as an array of note values
const uint8_t note_sequence[] = {74, 78, 81, 86, 90, 93, 98, 102, 57, 61, 66, 69, 73, 78, 81, 85, 88, 92, 97, 100, 97, 92,
                                 88, 85, 81, 78, 74, 69, 66, 62,  57, 62, 66, 69, 74, 78, 81, 86, 90, 93, 97, 102, 97, 93,
                                 90, 85, 81, 78, 73, 68, 64, 61,  56, 61, 64, 68, 74, 78, 81, 86, 90, 93, 98, 102};

void midi_task(void) {
    static uint32_t start_ms = 0;

    uint8_t const cable_num = 0;  // MIDI jack associated with USB endpoint
    uint8_t const channel   = 0;  // 0 for channel 1

    // The MIDI interface always creates input and output port/jack descriptors
    // regardless of these being used or not. Therefore incoming traffic should be read
    // (possibly just discarded) to avoid the sender blocking in IO
    while (tud_midi_available()) {
        uint8_t packet[4];
        tud_midi_packet_read(packet);
    }

    // send note periodically
    if (board_millis() - start_ms < 286) {
        return;  // not enough time
    }
    start_ms += 286;

    // Previous positions in the note sequence.
    int previous = (int)(note_pos - 1);

    // If we currently are at position 0, set the
    // previous position to the last note in the sequence.
    if (previous < 0) {
        previous = sizeof(note_sequence) - 1;
    }

    // Send Note On for current position at full velocity (127) on channel 1.
    uint8_t note_on[3] = {0x90 | channel, note_sequence[note_pos], 127};
    tud_midi_stream_write(cable_num, note_on, 3);

    // Send Note Off for previous note.
    uint8_t note_off[3] = {0x80 | channel, note_sequence[previous], 0};
    tud_midi_stream_write(cable_num, note_off, 3);

    // Increment position
    note_pos++;

    // If we are at the end of the sequence, start over.
    if (note_pos >= sizeof(note_sequence)) {
        note_pos = 0;
    }
}

#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2S1_Init();
    MX_USB_OTG_FS_PCD_Init();
    MX_CRC_Init();
    MX_ADC1_Init();
    /* USER CODE BEGIN 2 */

    HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_SET);

#if PED_USB_DEVICE_CLASS == PED_USB_CDC_CLASS || PED_USB_DEVICE_CLASS == PED_USB_MIDI_CLASS
    tusb_rhport_init_t dev_init = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_AUTO};
    tusb_init(BOARD_TUD_RHPORT, &dev_init);
#endif

    lcd_1602a_init();
    lcd_1602a_write_text("INIT");

    uint32_t pstate = 0;
    uint32_t nstate = 0;

#if SOUND_PLAYER == PED_ENABLED
    player_init();
#endif

    const bitnotes_t melody[] = {
        (1 << bitnote_c1),
        (1 << bitnote_d1),
        (1 << bitnote_e1),
        (1 << bitnote_c1),
        (1 << bitnote_c1) | (1 << bitnote_e1),
        (1 << bitnote_d1) | (1 << bitnote_f1),
        (1 << bitnote_e1) | (1 << bitnote_g1),
        (1 << bitnote_c1) | (1 << bitnote_e1),
        (1 << bitnote_e1),
        (1 << bitnote_f1),
        (1 << bitnote_g1),
        (1 << bitnote_g1),
    };
    size_t melody_ptr             = 0;
    const size_t melody_len       = sizeof(melody) / sizeof(melody[0]);
    const size_t note_duration_ms = 1000;
    size_t last_changed_note      = HAL_GetTick();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

#if PED_USB_DEVICE_CLASS == PED_USB_CDC_CLASS
        tud_task();
        cdc_example_keep_alive_task();
#elif PED_USB_DEVICE_CLASS == PED_USB_MIDI_CLASS
        tud_task();
        // GPIO_PinState button = HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin);
        midi_task();
#endif

#if HARDWARE_KEYS_ENABLED == PED_ENABLED
        nstate = read_keys();
#else
        if (HAL_GetTick() - last_changed_note > note_duration_ms) {
            last_changed_note = HAL_GetTick();
            nstate            = melody[melody_ptr];
            melody_ptr++;
            melody_ptr %= melody_len;
        }
#endif

        // play only one note in blocking mode
        // HAL_I2S_Transmit(&hi2s1, (uint16_t*) sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L, 10000);

#if SOUND_PLAYER == PED_ENABLED
        player_routine(pstate, nstate);
#endif
        pstate = nstate;
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
  */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM            = 15;
    RCC_OscInitStruct.PLL.PLLN            = 144;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 5;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
  */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void) {
    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
    sConfig.Channel      = ADC_CHANNEL_1;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void) {
    /* USER CODE BEGIN CRC_Init 0 */

    /* USER CODE END CRC_Init 0 */

    /* USER CODE BEGIN CRC_Init 1 */

    /* USER CODE END CRC_Init 1 */
    hcrc.Instance = CRC;
    if (HAL_CRC_Init(&hcrc) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN CRC_Init 2 */

    /* USER CODE END CRC_Init 2 */
}

/**
  * @brief I2S1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S1_Init(void) {
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

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void) {
    /* USER CODE BEGIN USB_OTG_FS_Init 0 */

    /* USER CODE END USB_OTG_FS_Init 0 */

    /* USER CODE BEGIN USB_OTG_FS_Init 1 */

    /* USER CODE END USB_OTG_FS_Init 1 */
    hpcd_USB_OTG_FS.Instance                 = USB_OTG_FS;
    hpcd_USB_OTG_FS.Init.dev_endpoints       = 4;
    hpcd_USB_OTG_FS.Init.speed               = PCD_SPEED_FULL;
    hpcd_USB_OTG_FS.Init.dma_enable          = DISABLE;
    hpcd_USB_OTG_FS.Init.phy_itface          = PCD_PHY_EMBEDDED;
    hpcd_USB_OTG_FS.Init.Sof_enable          = DISABLE;
    hpcd_USB_OTG_FS.Init.low_power_enable    = DISABLE;
    hpcd_USB_OTG_FS.Init.lpm_enable          = DISABLE;
    hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
    hpcd_USB_OTG_FS.Init.use_dedicated_ep1   = DISABLE;
    if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USB_OTG_FS_Init 2 */

    /* USER CODE END USB_OTG_FS_Init 2 */
}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream0
  */
static void MX_DMA_Init(void) {
    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* Configure DMA request hdma_memtomem_dma2_stream0 on DMA2_Stream0 */
    hdma_memtomem_dma2_stream0.Instance                 = DMA2_Stream0;
    hdma_memtomem_dma2_stream0.Init.Channel             = DMA_CHANNEL_0;
    hdma_memtomem_dma2_stream0.Init.Direction           = DMA_MEMORY_TO_MEMORY;
    hdma_memtomem_dma2_stream0.Init.PeriphInc           = DMA_PINC_ENABLE;
    hdma_memtomem_dma2_stream0.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_memtomem_dma2_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_memtomem_dma2_stream0.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_memtomem_dma2_stream0.Init.Mode                = DMA_NORMAL;
    hdma_memtomem_dma2_stream0.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_memtomem_dma2_stream0.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    hdma_memtomem_dma2_stream0.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_memtomem_dma2_stream0.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_memtomem_dma2_stream0.Init.PeriphBurst         = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_memtomem_dma2_stream0) != HAL_OK) {
        Error_Handler();
    }

    /* DMA interrupt init */
    /* DMA2_Stream2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, BOARD_LED_Pin | LED_MIDI_Pin | LED_SOUND_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, MUX_C3_Pin | MUX_C2_Pin | MUX_C1_Pin | LCD_D6_Pin | LCD_D7_Pin | MUX_C0_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(
        GPIOB, LCD_RS_Pin | LCD_ENABLE_Pin | LCD_D0_Pin | LCD_D1_Pin | LCD_D2_Pin | LCD_D3_Pin | LCD_D4_Pin | LCD_D5_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : BOARD_LED_Pin LED_MIDI_Pin LED_SOUND_Pin */
    GPIO_InitStruct.Pin   = BOARD_LED_Pin | LED_MIDI_Pin | LED_SOUND_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : USER_BUTTON_Pin */
    GPIO_InitStruct.Pin  = USER_BUTTON_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : MUX_C3_Pin MUX_C2_Pin MUX_C1_Pin LCD_D6_Pin
                           LCD_D7_Pin MUX_C0_Pin */
    GPIO_InitStruct.Pin   = MUX_C3_Pin | MUX_C2_Pin | MUX_C1_Pin | LCD_D6_Pin | LCD_D7_Pin | MUX_C0_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : LCD_RS_Pin LCD_ENABLE_Pin LCD_D0_Pin LCD_D1_Pin
                           LCD_D2_Pin LCD_D3_Pin LCD_D4_Pin LCD_D5_Pin */
    GPIO_InitStruct.Pin   = LCD_RS_Pin | LCD_ENABLE_Pin | LCD_D0_Pin | LCD_D1_Pin | LCD_D2_Pin | LCD_D3_Pin | LCD_D4_Pin | LCD_D5_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : MUX_D0_Pin MUX_D1_Pin */
    GPIO_InitStruct.Pin  = MUX_D0_Pin | MUX_D1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
