#ifndef PED_TYPES_H
#define PED_TYPES_H

#define n_bitnotes    (63)
#define base_midinote (24)

typedef enum {
    KEY_STATE_RELEASED = 0,
    KEY_STATE_PRESSED,
} key_state_t;

extern const char note_names[n_bitnotes][4];

#endif  // PED_TYPES_H
