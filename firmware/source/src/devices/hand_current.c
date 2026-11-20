#include "hand_current.h"
#include <math.h>
#include <string.h>
bool hand_current_fresh(bool dynamic, bool active, uint32_t received, uint32_t mode,
                        uint64_t now, uint64_t updated) {
    return dynamic && active && received && (mode==2 || mode==3) && now>=updated && now-updated<HAND_CURRENT_FRESH_US;
}
bool hand_compliance_valid(const hand_compliance_command_value_t *v, uint32_t mode) {
    if ((mode != 2 && mode != 3) || v->duration_ms > 10000 || v->control_feedforward_ma > HAND_CURRENT_MAX_MA) return false;
    for (int i=0; i<6; ++i)
        if (!isfinite(v->target_q_rad[i]) || v->target_q_rad[i]<0 || v->target_q_rad[i]>65.535f
            || v->joint_current_ma[i]>HAND_CURRENT_MAX_MA
            || (mode==3 && v->joint_current_ma[i]!=v->control_feedforward_ma)) return false;
    return true;
}
bool hand_current_boosted(const uint32_t currents[6], uint32_t common,
                          const configure_request_value_t *baseline) {
    if (common > baseline->hand_control_feedforward_ma) return true;
    for (int i=0; i<6; ++i) if (currents[i] > baseline->hand_soft_current_ma[i]) return true;
    return false;
}
static void put16(uint8_t *p,uint16_t value){p[0]=value>>8;p[1]=value;}
size_t hand_control_payload(uint8_t payload[28], const hand_command_value_t *hand,
                            const configure_request_value_t *cfg) {
    if(cfg->hand_mode<1 || cfg->hand_mode>3) return 0;
    memset(payload,0,28);
    if(cfg->hand_mode==3){
        if(cfg->hand_control_feedforward_ma>HAND_CURRENT_MAX_MA) return 0;
        put16(payload,(uint16_t)cfg->hand_control_feedforward_ma);
    }else put16(payload,hand->duration_ms);
    for(int i=0;i<6;i++){
        float x=hand->target_q_rad[i]*1000;
        if(!isfinite(x)||x<0||x>65535) return 0;
        put16(payload+2+2*i,(uint16_t)lroundf(x));
        if(cfg->hand_mode!=3){
            uint32_t cur=cfg->hand_mode==1?cfg->hand_position_current_limit_ma[i]:cfg->hand_soft_current_ma[i];
            if(cur>65535) return 0;
            put16(payload+14+2*i,(uint16_t)cur);
        }
    }
    if(cfg->hand_mode==2){
        if(cfg->hand_control_feedforward_ma>65535) return 0;
        put16(payload+26,cfg->hand_control_feedforward_ma);
    }
    return cfg->hand_mode==3?14:cfg->hand_mode==2?28:26;
}
