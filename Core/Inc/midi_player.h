#ifndef MIDI_PLAYER_H
#define MIDI_PLAYER_H

#include "ped_config.h"
#include "ped_types.h"

void midi_player_init();
void midi_player_update(uint32_t pstate, uint32_t nstate);

#endif  // MIDI_PLAYER_H
