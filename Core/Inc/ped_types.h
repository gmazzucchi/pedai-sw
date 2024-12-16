#ifndef PED_TYPES_H
#define PED_TYPES_H

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
    n_bitnotes
} bitnotes_t;

typedef enum {
    midinote_c1 = 36,
    midinote_c1_sharp,
    midinote_d1,
    midinote_e1_flat,
    midinote_e1,
    midinote_f1,
    midinote_f1_sharp,
    midinote_g1,
    midinote_g1_sharp,
    midinote_a1,
    midinote_b1_flat,
    midinote_b1,
    midinote_c2,
    midinote_c2_sharp,
    midinote_d2,
    midinote_e2_flat,
    midinote_e2,
    midinote_f2,
    midinote_f2_sharp,
    midinote_g2,
    midinote_g2_sharp,
    midinote_a2,
    midinote_b2_flat,
    midinote_b2,
    midinote_ca3,
    n_midinotes
} midi_note_t;

extern const char note_names[n_bitnotes][4];

#endif  // PED_TYPES_H
