#ifndef PED_CONFIG_H
#define PED_CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define PED_ENABLED  (1U)
#define PED_DISABLED (0U)

#define MAX_NOTE_LEN (2048U)

#define SOUND_PLAYER (PED_DISABLED)

#define PED_USB_NONE_CLASS   (0U)
#define PED_USB_CDC_CLASS    (2U)
#define PED_USB_MIDI_CLASS   (3U)
#define PED_USB_DEVICE_CLASS (PED_USB_MIDI_CLASS)

#define PED_PHASE_VOCODER (PED_ENABLED)

#define SCALE_AMPLITUDE_AFTER_ADDING (PED_DISABLED)

#define HARDWARE_KEYS_ENABLED (PED_DISABLED)

typedef enum {
    bitnote_c1 = 0,
    bitnote_c1_sharp,
    bitnote_d1,
    bitnote_e1_flat,
    bitnote_e1,
    bitnote_f1,
    bitnote_f1_sharp,
    bitnote_g1,
    bitnote_g1_sharp,
    bitnote_a1,
    bitnote_b1_flat,
    bitnote_b1,
    bitnote_c2,
    bitnote_c2_sharp,
    bitnote_d2,
    bitnote_e2_flat,
    bitnote_e2,
    bitnote_f2,
    bitnote_f2_sharp,
    bitnote_g2,
    bitnote_g2_sharp,
    bitnote_a2,
    bitnote_b2_flat,
    bitnote_b2,
    bitnote_ca3,
    bitnotes_n_ped_notes
} bitnotes_t;

#endif  // PED_CONFIG_H
