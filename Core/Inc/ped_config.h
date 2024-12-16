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
#define PED_USB_DEVICE_CLASS (PED_USB_MIDI_CLASS)

#define PED_PHASE_VOCODER (PED_ENABLED)

#define SCALE_AMPLITUDE_AFTER_ADDING (PED_DISABLED)

#define HARDWARE_KEYS_ENABLED (PED_DISABLED)

#endif  // PED_CONFIG_H
