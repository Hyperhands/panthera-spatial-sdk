#include "gello_web_codec.h"
#include "gello_radio_codec.h"
#include <string.h>
static uint16_t r16(const uint8_t *p){return p[0]|(uint16_t)p[1]<<8;}
static uint32_t r32(const uint8_t *p){return r16(p)|(uint32_t)r16(p+2)<<16;}
static uint64_t r64(const uint8_t *p){return r32(p)|(uint64_t)r32(p+4)<<32;}
static void w16(uint8_t *p,uint16_t v){p[0]=v;p[1]=v>>8;}
static void w32(uint8_t *p,uint32_t v){w16(p,v);w16(p+2,v>>16);}
static void w64(uint8_t *p,uint64_t v){w32(p,v);w32(p+4,v>>32);}
bool gw_command_valid(const struct gw_packet *p) {
    if(p->mode>GW_EXIT_MODE)return false;
    if(!p->op)return true;
    if(p->node<11 || p->node>12 || p->op>GW_TOPOLOGY || !p->session || !p->sequence || !p->expected_boot)return false;
    return (p->op!=GW_HAND && p->op!=GW_PROFILE) || ga_valid(&p->action);
}
void gw_encode(const struct gw_packet *p,uint8_t out[GW_SIZE]) {
    memset(out,0,GW_SIZE);memcpy(out,"GW04",4);out[4]=p->kind;out[5]=p->node;out[6]=p->op;out[7]=p->mode;
    w64(out+8,p->session);w32(out+16,p->sequence);w64(out+20,p->expected_boot);w32(out+28,p->revision);
    bool action=p->op==GW_HAND || p->op==GW_PROFILE;
    if(action)ga_encode(&p->action,out+32);
    w32(out+208,p->exchange);
    for(unsigned b=0;b<2;b++) {
        if(action && b==0)continue;
        uint8_t *v=out+48+80*b;const struct gw_board *s=&p->board[b];
        w64(v,s->boot);w64(v+8,s->ack_session);w32(v+16,s->ack_sequence);w32(v+20,s->revision);w32(v+24,s->profile_crc);
        v[28]=s->ack_status;v[29]=s->phase;v[30]=s->error;v[31]=s->flags;
        for(unsigned i=0;i<6;i++){w32(v+32+4*i,(uint32_t)s->arm_mdeg[i]);w16(v+56+2*i,(uint16_t)s->hand_cdeg[i]);}
        w16(v+68,s->current_ma);v[70]=s->home_phase;v[71]=s->settings;w32(v+72,s->ready_flags);w32(v+76,s->age_ms);
    }
    w32(out+220,gr_crc(out,220));
}
bool gw_decode(const uint8_t *p,size_t n,struct gw_packet *out) {
    if(n!=GW_SIZE || memcmp(p,"GW04",4) || p[4]<GW_STATE || p[4]>GW_PEER_REPLY || p[7]>GW_EXIT_MODE || r32(p+212) || r32(p+216) || r32(p+220)!=gr_crc(p,220))return false;
    memset(out,0,sizeof(*out));out->kind=p[4];out->node=p[5];out->op=p[6];out->mode=p[7];out->session=r64(p+8);out->sequence=r32(p+16);
    out->expected_boot=r64(p+20);out->revision=r32(p+28);out->exchange=r32(p+208);
    bool action=out->op==GW_HAND || out->op==GW_PROFILE;
    if(action && !ga_decode(p+32,&out->action))return false;
    for(unsigned b=0;b<2;b++) {
        if(action && b==0)continue;
        const uint8_t *v=p+48+80*b;struct gw_board *s=&out->board[b];
        if(v[71]&~7u)return false;
        s->home_phase=v[70];s->settings=v[71];
        s->boot=r64(v);s->ack_session=r64(v+8);s->ack_sequence=r32(v+16);s->revision=r32(v+20);s->profile_crc=r32(v+24);
        s->ack_status=v[28];s->phase=v[29];s->error=v[30];s->flags=v[31];
        for(unsigned i=0;i<6;i++){s->arm_mdeg[i]=(int32_t)r32(v+32+4*i);s->hand_cdeg[i]=(int16_t)r16(v+56+2*i);}
        s->current_ma=r16(v+68);s->ready_flags=r32(v+72);s->age_ms=r32(v+76);
    }
    return gw_command_valid(out);
}
bool gw_parse(struct gw_parser *p,uint8_t byte,struct gw_packet *out) {
    if(p->used==GW_SIZE){memmove(p->bytes,p->bytes+1,--p->used);}
    p->bytes[p->used++]=byte;
    while(p->used>=4) {
        if(!memcmp(p->bytes,"GW04",4)) {
            if(p->used<GW_SIZE)return false;
            if(gw_decode(p->bytes,p->used,out)){p->used=0;return true;}
        }
        memmove(p->bytes,p->bytes+1,--p->used);
    }
    return false;
}

uint32_t gw_profile_crc(const uint16_t fingers[6],uint16_t current) {
    uint8_t data[14];for(unsigned i=0;i<6;i++)w16(data+2*i,fingers[i]);w16(data+12,current);
    return gr_crc(data,sizeof(data));
}

void gw_migrate_legacy_profile(uint16_t fingers[6]) {
    /* Preserve the previous 55/61/60/57 degree endpoints to 0.05 degree. */
    const unsigned old_degrees[4]={55,61,60,57};
    for(unsigned i=2;i<6;i++)fingers[i]=(fingers[i]*old_degrees[i-2]+45)/90;
}
