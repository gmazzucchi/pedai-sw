#ifndef PED_TYPES_H
#define PED_TYPES_H

#define n_bitnotes    (63)
#define base_midinote (24)

typedef enum { adc1_r0 = 0, adc1_r1, adc1_mux, adc1_volume, n_adc_channels } adc1_channels_t;

typedef enum {
    KEY_STATE_RELEASED = 0,
    KEY_STATE_PRESSED,
} key_state_t;

extern const char note_names[n_bitnotes][4];

#endif  // PED_TYPES_H
