#ifndef WAVEGEN_H
#define WAVEGEN_H

#include "ped_config.h"
#include "arm_math.h"

size_t compose_note(bool *pstate_keys, bool *nstate_keys, bool *pstate_pedals, bool *nstate_pedals, int16_t* current_note, size_t current_note_len);

#endif // WAVEGEN_H
