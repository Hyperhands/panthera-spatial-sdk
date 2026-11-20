#pragma once
#include "peer_codec.h"
void panthera_peer_start(void);
bool panthera_peer_latest(struct panthera_peer_state *state,uint64_t *received);
