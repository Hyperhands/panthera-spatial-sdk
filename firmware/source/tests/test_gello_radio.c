#include "gello/gello_radio_codec.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static bool feed(struct gr_parser *p,const uint8_t *bytes,size_t n,struct gr_frame *f) {
    bool got=false;for(size_t i=0;i<n;i++)got|=gr_parse(p,bytes[i],f);return got;
}
int main(void) {
    assert(gr_crc((const uint8_t *)"123456789",9)==0xcbf43926u);
    struct gr_frame f={.kind=GR_QUERY,.session=0xabcdef0123456789ull,.sequence=UINT32_MAX},out;
    uint8_t query[GR_SAMPLE_SIZE],sample[GR_SAMPLE_SIZE],corrupt[GR_SAMPLE_SIZE];
    assert(gr_encode(&f,query)==GR_QUERY_SIZE && gr_decode(query,GR_QUERY_SIZE,&out));
    assert(out.session==f.session && out.sequence==f.sequence);
    f.kind=GR_SAMPLE;f.source_boot=345;f.scan=123;
    for(unsigned i=0;i<14;i++)f.samples[i]=(struct gello_wire_sample){.raw=4090,.ticks=-6,.velocity=-123,.age_us=1000+i,.valid=true};
    f.counters[0]=456;
    assert(gr_encode(&f,sample)==GR_SAMPLE_SIZE && gr_decode(sample,sizeof(sample),&out));
    assert(out.samples[13].ticks==-6 && out.samples[0].velocity==-123 && out.counters[0]==456);
    for(unsigned i=0;i<sizeof(sample);i++) {
        memcpy(corrupt,sample,sizeof(sample));corrupt[i]^=1;assert(!gr_decode(corrupt,sizeof(corrupt),&out));
    }
    for(unsigned n=0;n<sizeof(sample);n++)assert(!gr_decode(sample,n,&out));
    struct gr_parser p={0};
    assert(!feed(&p,(uint8_t *)"xGGE00garbage",12,&out));
    assert(feed(&p,query,GR_QUERY_SIZE,&out) && out.kind==GR_QUERY);
    /* Every truncated prefix must resynchronize to the next intact packet. */
    for(unsigned n=1;n<GR_SAMPLE_SIZE;n++) {
        memset(&p,0,sizeof(p));assert(!feed(&p,sample,n,&out));
        assert(feed(&p,sample,sizeof(sample),&out) && out.kind==GR_SAMPLE);
    }
    assert(gr_decode(sample,sizeof(sample),&out));
    assert(gr_matches(&out,f.session,UINT32_MAX,100000,119999));
    assert(!gr_matches(&out,f.session,UINT32_MAX,100000,120000));
    assert(!gr_matches(&out,f.session,0,100000,101000)); /* replay/wrap */
    assert(!gr_matches(&out,f.session+1,UINT32_MAX,100000,101000)); /* reboot */
    assert(!gr_matches(&out,f.session,UINT32_MAX,100000,99999));
    gr_add_age(&out,2000);assert(out.samples[0].age_us==3000);
    gr_add_age(&out,46999);assert(out.samples[0].valid && !out.samples[1].valid);
    gr_add_age(&out,1);assert(!out.samples[0].valid);
    gr_add_age(&out,UINT64_MAX);assert(out.samples[0].age_us==UINT32_MAX);
    /* Reader timestamp predates a newly published frame after preemption.
     * It is a valid, newer frame, not a reset/empty snapshot. */
    assert(gr_decode(sample,sizeof(sample),&out));
    assert(gr_age_snapshot(&out,110000,100000,112000));
    assert(out.scan==123 && out.samples[0].valid && out.samples[0].age_us==3000);
    /* Time lost to preemption must still expire an actually old sample. */
    assert(gr_age_snapshot(&out,112000,112000,159000));
    assert(out.samples[0].age_us==50000 && !out.samples[0].valid);
    assert(gr_decode(sample,sizeof(sample),&out));
    assert(gr_age_snapshot(&out,110000,160000,112000));
    assert(out.samples[0].age_us==51000 && !out.samples[0].valid);
    assert(!gr_age_snapshot(&out,0,160000,160000) && out.scan==0);
    assert(!gr_age_snapshot(&out,200000,160000,160000));
    /* CRC-valid malformed sample still rejected. */
    f.samples[0].raw=17;gr_encode(&f,sample);assert(!gr_decode(sample,sizeof(sample),&out));
    puts("Radio: CRC, every bit/length, UART resync, request identity, RTT expiry and age saturation passed");
}
