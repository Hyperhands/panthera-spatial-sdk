#include "gello_radio_codec.h"
#include <string.h>

static uint16_t r16(const uint8_t *p) { return p[0] | (uint16_t)p[1]<<8; }
static uint32_t r32(const uint8_t *p) { return r16(p) | (uint32_t)r16(p+2)<<16; }
static uint64_t r64(const uint8_t *p) { return r32(p) | (uint64_t)r32(p+4)<<32; }
static void w16(uint8_t *p,uint16_t v) { p[0]=v;p[1]=v>>8; }
static void w32(uint8_t *p,uint32_t v) { w16(p,v);w16(p+2,v>>16); }
static void w64(uint8_t *p,uint64_t v) { w32(p,v);w32(p+4,v>>32); }
uint32_t gr_crc(const uint8_t *p,size_t n) {
    uint32_t c=UINT32_MAX;
    for(size_t i=0;i<n;i++) { c^=p[i];for(int j=0;j<8;j++) c=(c>>1)^((0u-(c&1u))&0xedb88320u); }
    return ~c;
}
size_t gr_encode(const struct gr_frame *f,uint8_t out[GR_SAMPLE_SIZE]) {
    size_t n=f->kind==GR_QUERY?GR_QUERY_SIZE:GR_SAMPLE_SIZE;
    memset(out,0,n);memcpy(out,"GE01",4);out[4]=f->kind;
    w64(out+8,f->session);w32(out+16,f->sequence);
    if(f->kind==GR_SAMPLE) {
        w64(out+20,f->source_boot);w32(out+28,f->scan);
        uint16_t mask=0;
        for(unsigned i=0;i<GELLO_SERVO_COUNT;i++) {
            const struct gello_wire_sample *s=&f->samples[i];uint8_t *p=out+32+12*i;
            if(s->valid)mask|=1u<<i;
            w16(p,s->raw);w16(p+2,(uint16_t)s->velocity);w32(p+4,(uint32_t)s->ticks);w32(p+8,s->age_us);
        }
        w16(out+6,mask);
        for(unsigned i=0;i<5;i++)w32(out+200+4*i,f->counters[i]);
    }
    w32(out+n-4,gr_crc(out,n-4));return n;
}
bool gr_decode(const uint8_t *p,size_t n,struct gr_frame *f) {
    if(n<GR_QUERY_SIZE || memcmp(p,"GE01",4) || p[5] ||
       (p[4]!=GR_QUERY && p[4]!=GR_SAMPLE) ||
       n!=(p[4]==GR_QUERY?GR_QUERY_SIZE:GR_SAMPLE_SIZE) ||
       r32(p+n-4)!=gr_crc(p,n-4) || !r64(p+8) || (r16(p+6)&0xc000))return false;
    memset(f,0,sizeof(*f));f->kind=p[4];f->session=r64(p+8);f->sequence=r32(p+16);
    f->source_boot=r64(p+20);f->scan=r32(p+28);
    if(f->kind==GR_QUERY)return !r16(p+6) && !f->source_boot && !f->scan;
    if(!f->source_boot)return false;
    for(unsigned i=0;i<GELLO_SERVO_COUNT;i++) {
        struct gello_wire_sample *s=&f->samples[i];const uint8_t *r=p+32+12*i;
        s->valid=(r16(p+6)&(1u<<i))!=0;s->raw=r16(r);s->velocity=(int16_t)r16(r+2);
        s->ticks=(int32_t)r32(r+4);s->age_us=r32(r+8);
        if(s->raw>=4096 || (s->valid && (uint32_t)s->ticks%4096!=s->raw))return false;
    }
    for(unsigned i=0;i<5;i++)f->counters[i]=r32(p+200+4*i);
    return true;
}
static void discard(struct gr_parser *p,size_t n) { memmove(p->bytes,p->bytes+n,p->used-n);p->used-=n; }
bool gr_parse(struct gr_parser *p,uint8_t byte,struct gr_frame *f) {
    if(p->used==sizeof(p->bytes))discard(p,1);
    p->bytes[p->used++]=byte;
    while(p->used>=4) {
        if(memcmp(p->bytes,"GE01",4)) { discard(p,1);continue; }
        if(p->used<5)return false;
        unsigned kind=p->bytes[4];
        if(kind!=GR_QUERY && kind!=GR_SAMPLE) { discard(p,1);p->bad++;continue; }
        size_t n=kind==GR_QUERY?GR_QUERY_SIZE:GR_SAMPLE_SIZE;
        if(p->used<n)return false;
        if(gr_decode(p->bytes,n,f)) { discard(p,n);return true; }
        p->bad++;discard(p,1);
    }
    return false;
}
bool gr_matches(const struct gr_frame *f,uint64_t session,uint32_t seq,uint64_t sent,uint64_t now) {
    return f->kind==GR_SAMPLE && f->session==session && f->sequence==seq && now>=sent && now-sent<GR_MAX_RTT_US;
}
void gr_add_age(struct gr_frame *f,uint64_t elapsed) {
    for(unsigned i=0;i<GELLO_SERVO_COUNT;i++) {
        struct gello_wire_sample *s=&f->samples[i];
        s->age_us=elapsed>UINT32_MAX-s->age_us?UINT32_MAX:s->age_us+(uint32_t)elapsed;
        if(s->age_us>=GR_STALE_US)s->valid=false;
    }
}
bool gr_age_snapshot(struct gr_frame *f,uint64_t received_at,uint64_t caller_now,uint64_t copied_at) {
    uint64_t now=copied_at>caller_now?copied_at:caller_now;
    if(!received_at || now<received_at) { memset(f,0,sizeof(*f));return false; }
    gr_add_age(f,now-received_at);return true;
}
