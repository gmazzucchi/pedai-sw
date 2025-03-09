#ifndef PED_CONFIG_H
#define PED_CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define PED_ENABLED  (1U)
#define PED_DISABLED (0U)

#define MAX_NOTE_LEN (255U)

#define SOUND_PLAYER_DISABLED (PED_DISABLED)
#define SOUND_PLAYER_SAMPLED  (1)
#define SOUND_PLAYER_SYNTH    (2)
#define SOUND_PLAYER_I2S      (SOUND_PLAYER_DISABLED)

#define PED_USB_NONE_CLASS   (0U)
#define PED_USB_CDC_CLASS    (2U)
#define PED_USB_MIDI_CLASS   (3U)
#define PED_USB_DEVICE_CLASS (PED_USB_MIDI_CLASS)

#define AUDIO_FREQUENCY_HZ (22000.0)   // check "Real Audio Frequency" in CubeMX
#define BASE_FREQUENCY     (65.40639)  // frequency of the lowest note

#define LCD1602_ENABLED (PED_DISABLED)

#define PED_PHASE_VOCODER (PED_ENABLED)

#define SCALE_AMPLITUDE_AFTER_ADDING (PED_DISABLED)

#define HARDWARE_KEYS_ENABLED (PED_ENABLED)

// #if HARDWARE_KEYS_ENABLED == PED_ENABLED
/***
 * Different modes on how acquire the key signals
 */
#define HW_KEYS_MUX (0)
#define HW_KEYS_MAT (1)

#define HW_KEY_ACQUISITION_MODE HW_KEYS_MAT

#define N_HW_MAT_ROWS (8u)
#define N_HW_MAT_COLS (8u)
#define N_HW_KEYS     (N_HW_MAT_ROWS * N_HW_MAT_COLS)

// #endif

// prototypes

#define to_mV(raw_value) (float)raw_value * (3.3f / 4095.0f)

// TODO: if serial

/*
#define PRINTLN(b, s)      \
    tud_cdc_write_str(b);  \
    tud_cdc_write_flush(); \
    b[0] = '\r';           \
    b[1] = '\n';           \
    b[2] = 0;              \
    tud_cdc_write_str(b);  \
    tud_cdc_write_flush(); \
    memset(b, 0, s);
*/

/* 
    #define EMPTY_PRINTLN_CDC()     \
        log_str[0] = '\r';          \
        log_str[1] = '\n';          \
        log_str[2] = 0;             \
        tud_cdc_write_str(log_str); \
        tud_cdc_write_flush(); 
*/

uint32_t get_current_time_ms();

#define n_bitnotes    (63)
#define base_midinote (24)

typedef enum { adc1_r0 = 0, adc1_r1, adc1_mux, adc1_volume, n_adc_channels } adc1_channels_t;

typedef enum {
    KEY_STATE_RELEASED = 0,
    KEY_STATE_PRESSED,
} key_state_t;

extern const char note_names[n_bitnotes][4];

#endif  // PED_CONFIG_H
