#pragma once
#include "gello_codec.h"
#include <stdbool.h>
#ifdef CONFIG_ROBOT_GELLO_RADIO_USB
#include "gello_radio_codec.h"
/* Dedicated USB worker only. The caller also validates total request age. */
bool gello_usb_exchange(const struct gr_frame *query, struct gr_frame *reply);
#endif
int gello_usb_start(void);
bool gello_usb_ready(void);
bool gello_usb_read(uint8_t id, struct gello_sample *sample, uint32_t *bad);

#ifdef CONFIG_ROBOT_GELLO_WEB
#include "gello_web_codec.h"
bool gello_usb_web_exchange(const struct gw_packet *query,struct gw_packet *reply);
#endif
