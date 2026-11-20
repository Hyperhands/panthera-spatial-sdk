#include "gello_action.h"
#include "gello_radio_codec.h"
#include <math.h>
#include <string.h>
const uint16_t ga_limits[6]={7500,2000,9000,9000,9000,9000};
static float clamp(float x,float lo,float hi){return fminf(hi,fmaxf(lo,x));}
bool ga_valid(const struct ga_action *a) {
    if(a->curve>GA_FIVE_FINGER || !a->close_at || a->close_at>100 || a->current_ma<500 || a->current_ma>2000)return false;
    for(unsigned i=0;i<6;i++)if(a->start[i]>ga_limits[i] || a->end[i]>ga_limits[i])return false;
    return true;
}
void ga_encode(const struct ga_action *a,uint8_t out[GA_ACTION_SIZE]) {
    for(unsigned i=0;i<13;i++) {
        uint16_t v=i<6?a->start[i]:i<12?a->end[i-6]:a->current_ma;
        out[2*i]=v;out[2*i+1]=v>>8;
    }
    out[26]=a->curve;out[27]=a->close_at;
}
bool ga_decode(const uint8_t in[GA_ACTION_SIZE],struct ga_action *a) {
    memset(a,0,sizeof(*a));
    for(unsigned i=0;i<13;i++) {
        uint16_t v=in[2*i]|(uint16_t)in[2*i+1]<<8;
        if(i<6)a->start[i]=v;else if(i<12)a->end[i-6]=v;else a->current_ma=v;
    }
    a->curve=in[26];a->close_at=in[27];return ga_valid(a);
}
void ga_sample(const struct ga_action *a,float percent,float out_deg[6],float *current) {
    float p=clamp(percent,0,100),path=clamp(p*100/a->close_at,0,100);
    const float five_mid[6]={1,13.f/18,53.f/58,56.f/61,55.f/60,52.f/57};
    const float two_mid[6]={1,18.f/20,52.f/55,1,1,1};
    for(unsigned i=0;i<6;i++) {
        float u=path/100;
        if(a->curve!=GA_LINEAR) {
            const float *mid=a->curve==GA_TWO_FINGER?two_mid:five_mid;
            if(i==0)u=clamp(path/20,0,1);
            else if(path<=20)u=0;
            else if(path<=95)u=mid[i]*(path-20)/75;
            else u=mid[i]+(1-mid[i])*(path-95)/5;
        }
        out_deg[i]=(a->start[i]+(a->end[i]-a->start[i])*u)/100.f;
    }
    /* New actions use the whole trigger travel; legacy close_at=70 preserves
     * its old final pressure segment. The selected value is the maximum. */
    float force=a->close_at==100?p/100:clamp((p-a->close_at)/(100-a->close_at),0,1);
    *current=500+(a->current_ma-500)*force;
}
void ga_from_legacy(const uint16_t fractions[6],uint16_t current,struct ga_action *a) {
    memset(a,0,sizeof(*a));
    for(unsigned i=0;i<6;i++)a->end[i]=(ga_limits[i]*(uint32_t)fractions[i]+500)/1000;
    a->current_ma=current;a->curve=GA_FIVE_FINGER;a->close_at=70;
}
void ga_builtin(unsigned preset,struct ga_action *a) {
    memset(a,0,sizeof(*a));a->current_ma=500;a->close_at=100;
    const uint16_t old[6]={7500,2000,5500,6100,6000,5700};
    memcpy(a->end,old,sizeof(old));
    if(preset==1)a->curve=GA_FIVE_FINGER;
    else if(preset==2) {
        a->curve=GA_TWO_FINGER;
        for(unsigned i=3;i<6;i++)a->start[i]=a->end[i]=0;
    } else if(preset==3) {
        a->curve=GA_LINEAR;a->start[0]=7500;a->start[1]=2000;
    } else {
        a->curve=GA_FIVE_FINGER;memcpy(a->end,ga_limits,sizeof(a->end));
    }
}
uint32_t ga_crc(const struct ga_action *a) {uint8_t bytes[GA_ACTION_SIZE];ga_encode(a,bytes);return gr_crc(bytes,sizeof(bytes));}
