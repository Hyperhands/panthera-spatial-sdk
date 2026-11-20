#pragma once
#include "gello_radio_codec.h"
void gello_radio_start(void);
/* Snapshot with all transport and residence times already charged to ages. */
bool gello_radio_snapshot(struct gr_frame *out,uint64_t now);
