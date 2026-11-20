#pragma once
#include "gello/gello_web_codec.h"
void gello_web_submit(const struct gw_packet *p);
void gello_web_snapshot(struct gw_board *out,uint64_t now);

/* Called with robot_lock held by the homing owner. */
bool gello_web_home_inhibited(void);
