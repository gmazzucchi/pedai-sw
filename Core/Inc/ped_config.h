#ifndef PED_CONFIG_H
#define PED_CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define ENABLED  (1U)
#define DISABLED (0U)

#define MAX_NOTE_LEN (2048U)

#define PED_PHASE_VOCODER ENABLED

#define SCALE_AMPLITUDE_AFTER_ADDING DISABLED

#define HARDWARE_KEYS_ENABLED DISABLED

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
