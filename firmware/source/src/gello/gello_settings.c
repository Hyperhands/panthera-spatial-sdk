#include "gello_settings.h"
unsigned gw_mode_press(unsigned mode) {
    return mode==GW_RUN_MODE?GW_ENTER_MODE:mode==GW_ENTER_MODE || mode==GW_SETTINGS_MODE?GW_EXIT_MODE:mode;
}
bool gw_mode_open_ap(unsigned mode,const struct gw_board b[2],const bool fresh[2]) {
    return mode==GW_ENTER_MODE && fresh[0] && fresh[1] &&
        (b[0].settings&GW_SETTINGS_ENTERED) && (b[1].settings&GW_SETTINGS_ENTERED) &&
        (b[0].flags&GW_PAUSED) && (b[1].flags&GW_PAUSED);
}
bool gw_mode_finished(unsigned mode,bool ap_on,const struct gw_board b[2],const bool fresh[2]) {
    return mode==GW_EXIT_MODE && !ap_on && fresh[0] && fresh[1] &&
        !(b[0].settings&GW_SETTINGS_ENTERED) && !(b[1].settings&GW_SETTINGS_ENTERED) &&
        !(b[0].flags&GW_PAUSED) && !(b[1].flags&GW_PAUSED);
}
unsigned gw_mode_action(unsigned mode,bool ap_on,const struct gw_board b[2],unsigned side) {
    if(side>1)return 0;
    if(mode==GW_ENTER_MODE)return b[side].settings&GW_SETTINGS_ENTERED?0:GW_ENTER;
    if(mode!=GW_EXIT_MODE || ap_on)return 0;
    /* Both leave settings while still paused, then release the pause intents. */
    if((b[0].settings|b[1].settings)&GW_SETTINGS_ENTERED)
        return b[side].settings&GW_SETTINGS_ENTERED?GW_LEAVE:0;
    return b[side].flags&GW_PAUSED?GW_RESUME:0;
}

bool gw_mode_needs_reentry(unsigned mode,const struct gw_board b[2],const bool fresh[2]) {
    /* After receiver reboot, expose any surviving NXP pause as settings again.
     * A reset must neither silently resume motion nor strand an invisible pause. */
    if(mode==GW_RUN_MODE)return
        (fresh[0] && ((b[0].settings&GW_SETTINGS_ENTERED) || (b[0].flags&GW_PAUSED))) ||
        (fresh[1] && ((b[1].settings&GW_SETTINGS_ENTERED) || (b[1].flags&GW_PAUSED)));
    return mode==GW_SETTINGS_MODE && ((fresh[0] && !(b[0].settings&GW_SETTINGS_ENTERED)) ||
                                    (fresh[1] && !(b[1].settings&GW_SETTINGS_ENTERED)));
}

enum gw_delivery_result gw_delivery_check(const struct gw_packet *command,
    const struct gw_board *board,int64_t now,int64_t ack_until) {
    if(board->boot==command->expected_boot && board->ack_session==command->session &&
       board->ack_sequence==command->sequence && board->ack_status)return GW_DELIVERY_ACK;
    return now>=ack_until?GW_DELIVERY_EXPIRED:GW_DELIVERY_WAIT;
}
bool gw_delivery_can_send(int64_t now,int64_t send_until,bool fresh) {
    return fresh && now<send_until;
}
bool gw_profile_needs_sync(unsigned mode,bool valid,const struct gw_board *board,uint32_t crc) {
    if(!valid || (mode!=GW_RUN_MODE && mode!=GW_SETTINGS_MODE))return false;
    /* An unacknowledged trial may already have changed the controller profile.
     * Do not restore an older selection over a live or expired manual trial. */
    return !(board->settings&GW_PROFILE_READY) ||
        (mode==GW_SETTINGS_MODE && !(board->flags&GW_MANUAL) && board->profile_crc!=crc);
}
