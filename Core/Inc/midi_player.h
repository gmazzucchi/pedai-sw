#ifndef MIDI_PLAYER_H
#define MIDI_PLAYER_H

#include "ped_config.h"

void midi_player_init();
void midi_player_update(bool *pstate, bool *nstate);

#endif  // MIDI_PLAYER_H
