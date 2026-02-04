#include "sound_player.h"
#include "wavegen.h"

#include "arm_math.h"
#include "lcd1602a.h"
#include "main.h"
#include "samples/sample_22kHz_D2.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static bool ready_to_play_note                   = true;
static bool has_to_play_note                     = false;
static bool has_to_change_note                   = false;
static volatile bool sai_transfer_completed      = false;
static volatile bool sai_is_transmitting         = false;
static volatile bool sai_half_transfer_completed = false;

static int16_t tmp_adder[MAX_NOTE_LEN]         = {0};
static int16_t buffer0_sai[MAX_NOTE_LEN]       = {0};
static int16_t buffer1_sai[MAX_NOTE_LEN]       = {0};
static int16_t *sound_data_db[2]               = {buffer0_sai, buffer1_sai};
static size_t sound_data_db_len[2]             = {0, 0};
static bool active_b                           = 0;
static size_t note_buffer_position[n_bitnotes] = {0};

inline size_t min(size_t x, size_t y) {
    return (x > y) ? (y) : (x);
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
    sai_transfer_completed = true;
    sai_is_transmitting    = false;
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
    sai_half_transfer_completed = true;
}

void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s) {
    uint32_t error_code = HAL_I2S_GetError(hi2s);
    char log_buf[BUFSIZ];
    snprintf(log_buf, BUFSIZ, "%lu", error_code);
    lcd_1602a_write_text(log_buf);
}

#if SOUND_PLAYER_I2S != PED_DISABLED

void sound_player_init(void) {
    memset(tmp_adder, 0, sizeof(tmp_adder));
    memset(buffer0_sai, 0, sizeof(buffer0_sai));
    memset(buffer1_sai, 0, sizeof(buffer1_sai));
}

void sound_player_routine(bool *pstate_keys, bool *nstate_keys, bool *pstate_pedals, bool *nstate_pedals) {
    bool zero_buffer[N_HW_KEYS] = {0};

    if (!has_to_play_note && sai_is_transmitting) {
        // stop the sound
        HAL_I2S_DMAStop(&hi2s1);  // HAL_SAI_DMAStop(&hsai_BlockA1);
        sai_half_transfer_completed = false;
        sai_transfer_completed      = false;
        sai_is_transmitting         = false;

        ready_to_play_note = true;
        HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_SET);
    } else if (has_to_play_note && sai_transfer_completed) {
        sai_transfer_completed = false;
        sai_is_transmitting    = true;

        HAL_I2S_Transmit_DMA(&hi2s1, (uint16_t *)sound_data_db[active_b], (uint16_t)sound_data_db_len[active_b]);
        HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_RESET);

    } else if (has_to_play_note && ready_to_play_note) {
        ready_to_play_note  = false;
        sai_is_transmitting = true;

        HAL_I2S_Transmit_DMA(&hi2s1, (uint16_t *)sound_data_db[active_b], (uint16_t)sound_data_db_len[active_b]);
        HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_RESET);
    } else if (has_to_change_note) {
        has_to_change_note  = false;
        ready_to_play_note  = false;
        sai_is_transmitting = true;

        HAL_I2S_DMAStop(&hi2s1);  // HAL_SAI_DMAStop(&hsai_BlockA1);
        HAL_I2S_Transmit_DMA(&hi2s1, (uint16_t *)sound_data_db[active_b], (uint16_t)sound_data_db_len[active_b]);

        HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_RESET);
    }

#define ALL_KEYS_RELEASED (0)
    if (memcmp(pstate_keys, zero_buffer, N_HW_KEYS) == 0 && memcmp(nstate_keys, zero_buffer, N_HW_KEYS) == 0) {
        // np to np
        // ensure that no note is played
        lcd_1602a_clear_screen();
        has_to_play_note = false;
    }
    // else if (pstate_keys == 0b1111 && nstate_keys != 0b1111) { // Covered in the last case
    // }
    else if ((memcmp(pstate_keys, zero_buffer, N_HW_KEYS) != 0 && memcmp(nstate_keys, zero_buffer, N_HW_KEYS) == 0) &&
             (memcmp(pstate_pedals, zero_buffer, N_HW_PEDAL_KEYS) != 0 && memcmp(nstate_pedals, zero_buffer, N_HW_PEDAL_KEYS) == 0)) {
        // p to np
        // stop at the next iteration
        has_to_play_note = false;
    } else if ((memcmp(nstate_keys, pstate_keys, N_HW_KEYS) == 0) && memcmp(nstate_pedals, pstate_pedals, N_HW_PEDAL_KEYS)) {
        // p same note
        // continue with the same buffer
        has_to_play_note = true;
        // memcpy(doublebuffer_sai[!active_buffer_sai], doublebuffer_sai[active_buffer_sai], doublebuffer_sai_len[active_buffer_sai] * sizeof(int16_t));
    } else {
        // np to p
        // p different note
        // construct the note in the inactive buffer and then swap the buffer at the next iteration
        // has_to_play_note                         = true;
        has_to_change_note           = true;
        sound_data_db_len[!active_b] = compose_note(pstate_keys, nstate_keys, pstate_pedals, nstate_pedals, sound_data_db[!active_b], MAX_NOTE_LEN);
        active_b                     = !active_b;
    }
}

#endif
