#pragma once
#include "gello_web_codec.h"
/* Pure mode planner: physical intent, telemetry and AP lifetime are separate. */
unsigned gw_mode_press(unsigned mode);
bool gw_mode_open_ap(unsigned mode,const struct gw_board board[2],const bool fresh[2]);
bool gw_mode_finished(unsigned mode,bool ap_on,const struct gw_board board[2],const bool fresh[2]);
unsigned gw_mode_action(unsigned mode,bool ap_on,const struct gw_board board[2],unsigned side);

/* Reconcile surviving pauses after receiver reboot, or lost NXP settings latches. */
bool gw_mode_needs_reentry(unsigned mode,const struct gw_board b[2],const bool fresh[2]);

/* Stop sending at the original deadline while allowing a delayed ACK to arrive. */
#define GW_USER_SEND_US 1000000LL
#define GW_USER_ACK_US 2500000LL
enum gw_delivery_result { GW_DELIVERY_WAIT, GW_DELIVERY_ACK, GW_DELIVERY_EXPIRED };
enum gw_delivery_result gw_delivery_check(const struct gw_packet *command,
    const struct gw_board *board,int64_t now,int64_t ack_until);
bool gw_delivery_can_send(int64_t now,int64_t send_until,bool fresh);
bool gw_profile_needs_sync(unsigned mode,bool valid,const struct gw_board *board,uint32_t crc);
