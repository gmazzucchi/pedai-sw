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

#include "crc.h"
#include "dma.h"
#include "gpio.h"
#include "i2s.h"
#include "usb_otg.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "arm_math.h"
#include "dsp_tools.h"
#include "key_reader.h"
#include "lcd1602a.h"
#include "midi_player.h"
#include "ped_config.h"
#include "sound_player.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t get_current_time_ms() {
    return HAL_GetTick();
}

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

volatile bool tud_cdc_tx_is_completed = true;

void tud_cdc_tx_complete_cb(uint8_t itf) {
    tud_cdc_tx_is_completed = true;
}

void usb_write(const char *str_base) {
    char *str = str_base;
    while (strlen(str) > 64) {
        tud_cdc_n_write(0, str, 64 * sizeof(char));
        tud_cdc_write_flush();
        str += 64 * sizeof(char);
    }
    size_t len = strlen(str);
    tud_cdc_n_write(0, str, len * sizeof(char));
    tud_cdc_write_flush();
    char tmpbuf[2];
    tmpbuf[0] = '\r';
    tmpbuf[1] = '\n';
    tmpbuf[2] = '\0';
    tud_cdc_n_write(0, tmpbuf, 2 * sizeof(char));
    tud_cdc_write_flush();
}

#elif PED_USB_DEVICE_CLASS == PED_USB_MIDI_CLASS

/* 
    // MIDI EXAMPLE FOR SEND NOTE
    void send_midi_note_on(uint8_t note, uint8_t velocity) {
        uint8_t packet[4] = {
            0x09,           // USB MIDI code index number (Note On)
            0x90,           // MIDI message (Note On, channel 0)
            note,           // Note number (e.g., middle C = 60)
            velocity        // Velocity (0-127)
        };
        tud_midi_stream_write(0, packet, 4);
    } 
*/

/* 
    // MIDI EXAMPLE TASK 

    // Variable that holds the current position in the sequence.
    uint32_t note_pos = 0;

    // Store example melody as an array of note values
    const uint8_t note_sequence[] = {60,61,62,63,64,65,66,67,68,69,70,71};

    void midi_example_melody_task(void) {
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
        if (get_current_time_ms() - start_ms < 286) {
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
*/

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
    /* USER CODE BEGIN 2 */

    HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_SET);  // the board led has inverse logic

    /* 
        while(1) {
            HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_SET);
            HAL_Delay(1000);
            HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_RESET);
            HAL_Delay(100);
        } 
    */

#if PED_USB_DEVICE_CLASS == PED_USB_CDC_CLASS
    tusb_rhport_init_t dev_init = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_AUTO};
    tusb_init(BOARD_TUD_RHPORT, &dev_init);
#elif PED_USB_DEVICE_CLASS == PED_USB_MIDI_CLASS
    midi_player_init();
#endif

    lcd_1602a_init();
    lcd_1602a_write_text("INIT");

#if HW_KEY_ACQUISITION_MODE == HW_KEYS_MAT
    bool pstate[N_HW_KEYS] = {0};
    bool nstate[N_HW_KEYS] = {0};
#else
#error Not yet implemented
#endif

#if SOUND_PLAYER_I2S != PED_DISABLED
    sound_player_init();
#endif

#if HARDWARE_KEYS_ENABLED == PED_ENABLED

#if HW_KEY_ACQUISITION_MODE == HW_KEYS_MAT
    init_keys();
#else
#error Not yet implemented
#endif

#else
    size_t melody_ptr                  = 0;
#define melody_len 12
    const size_t note_duration_ms      = 1000;
    size_t last_changed_note           = HAL_GetTick();
    bool melody[melody_len][N_HW_KEYS] = {0};
    melody[0][32]                      = 1;
    melody[1][34]                      = 1;
    melody[2][36]                      = 1;
    melody[3][32]                      = 1;
    melody[4][32]                      = 1;
    melody[5][34]                      = 1;
    melody[6][36]                      = 1;
    melody[7][32]                      = 1;
    melody[8][36]                      = 1;
    melody[9][37]                      = 1;
    melody[10][39]                     = 1;
    melody[11][37]                     = 1;
#endif

    // As requestes by the MIDI example
    HAL_Delay(286);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

#if HARDWARE_KEYS_ENABLED == PED_ENABLED
        read_keys(nstate);
#else
        if (HAL_GetTick() - last_changed_note > note_duration_ms) {
            last_changed_note = HAL_GetTick();
            memcpy(nstate, melody[melody_ptr], N_HW_KEYS * sizeof(bool));
            melody_ptr++;
            melody_ptr %= melody_len;

            // char log_str[BUFSIZ];
            // ssize_t log_str_ptr = 0;
            // for (size_t isem = 1; isem <= N_HW_KEYS; isem++) {
            // if (nstate[isem]) {
            // ssize_t to_add = snprintf(log_str + log_str_ptr, BUFSIZ - log_str_ptr, "%s ", note_names[isem]);
            // log_str_ptr += to_add;
            // }
            // }
            // lcd_1602a_write_text(log_str);
        }
#endif

#if PED_USB_DEVICE_CLASS == PED_USB_CDC_CLASS
        tud_task();

        /* 
        *   static uint32_t last_sent_keys = 0;
        *   if (HAL_GetTick() - last_sent_keys > 200) {
        *       last_sent_keys      = HAL_GetTick();
        *       char buffer[BUFSIZ] = {0};
        *       size_t bptr         = 0;
        *       for (size_t i = 0; i < N_HW_KEYS; i++) {
        *           if (nstate[i]) {
        *               size_t to_add = snprintf(buffer + bptr, BUFSIZ, "X");
        *               bptr += to_add;
        *           } else {
        *               size_t to_add = snprintf(buffer + bptr, BUFSIZ, "0");
        *               bptr += to_add;
        *           }
        *       }
        *       buffer[bptr] = 0;
        *       PRINTLN(buffer, BUFSIZ);
        *   } 
        */

// Experiments with DSP library
#define N (32)

        // while (!tud_cdc_tx_is_completed);

        static uint32_t last_sent = 0;
        if (HAL_GetTick() - last_sent > 500) {
            last_sent = HAL_GetTick();

            tud_cdc_tx_is_completed = false;
            char log_str[BUFSIZ];
            int log_str_ptr = 0;

            q15_t input[N];
            for (size_t i = 0; i < N; i++) {
                input[i] = i;
            }
            q15_t output[N];
            if (!fft(output, input, N)) {
                snprintf(log_str, BUFSIZ, "FFT error");
            } else {
                for (size_t i = 0; i < N; i++) {
                    int to_add = snprintf(log_str + log_str_ptr, BUFSIZ - log_str_ptr, "%" PRIi16 " ", output[i]);
                    log_str_ptr += to_add;
                }
            }
            usb_write(log_str);
        }

        // cdc_example_keep_alive_task();
#elif PED_USB_DEVICE_CLASS == PED_USB_MIDI_CLASS
        tud_task();

        // This is the example task
        // midi_example_melody_task();

        midi_player_update(pstate, nstate);
#endif

        // play only one note in blocking mode
        // HAL_I2S_Transmit(&hi2s1, (uint16_t*) sample_D2_22kHz_corpo, SAMPLE_D2_22KHZ_CORPO_L, 10000);

#if SOUND_PLAYER_I2S != PED_DISABLED
        sound_player_routine(pstate, nstate);
#endif
        memcpy(pstate, nstate, N_HW_KEYS * sizeof(bool));  // pstate = nstate;
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

#if PED_USB_DEVICE_CLASS == PED_USB_CDC_CLASS
#warning[JUST A WARNING] Interrupts are not disabled in Error_Handler() to allow logging
    // __disable_irq();
    static uint32_t last_msg_sent = 0;
    while (1) {
        tud_task();
        if (HAL_GetTick() - last_msg_sent > 1000) {
            // last_msg_sent = HAL_GetTick();
            // char buffer[BUFSIZ];
            // snprintf(buffer, BUFSIZ, "PEDAL ERROR!!!");
            // PRINTLN(buffer, BUFSIZ);
        }
    }
#else
    __disable_irq();
    while (1)
        ;
#endif
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
