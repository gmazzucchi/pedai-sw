#ifndef PED_CONFIG_H
#define PED_CONFIG_H

#include "ped_prototypes.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define PED_ENABLED  (1U)
#define PED_DISABLED (0U)

#define MAX_NOTE_LEN (2048U)

#define SOUND_PLAYER_I2S (PED_DISABLED)

#define PED_USB_NONE_CLASS   (0U)
#define PED_USB_CDC_CLASS    (2U)
#define PED_USB_MIDI_CLASS   (3U)
#define PED_USB_DEVICE_CLASS (PED_USB_CDC_CLASS)

#define LCD1602_ENABLED (PED_DISABLED)

#define PED_PHASE_VOCODER (PED_ENABLED)

#define SCALE_AMPLITUDE_AFTER_ADDING (PED_DISABLED)

#define HARDWARE_KEYS_ENABLED (PED_ENABLED)

#if HARDWARE_KEYS_ENABLED == PED_ENABLED
/***
 * Different modes on how acquire the key signals
 */
#define HW_KEYS_MUX (0)
#define HW_KEYS_MAT (1)

#define HW_KEY_ACQUISITION_MODE HW_KEYS_MAT

#define N_HW_MAT_ROWS (8u)
#define N_HW_MAT_COLS (8u)
#define N_HW_KEYS (N_HW_MAT_ROWS * N_HW_MAT_COLS)

#endif

#endif  // PED_CONFIG_H
