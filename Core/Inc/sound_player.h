#ifndef SOUND_PLAYER_H
#define SOUND_PLAYER_H

#include "ped_config.h"

void sound_player_init();
void sound_player_routine(bool *pstate_keys, bool *nstate_keys, bool *pstate_pedals, bool* nstate_pedals);

#endif  // SOUND_PLAYER_H
