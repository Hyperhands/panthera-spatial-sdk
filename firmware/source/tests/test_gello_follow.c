#include "standalone/gello_follow.h"
#include "standalone/gello_profile.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define DEG (3.14159265358979323846f/180.f)
static void ready(struct sg_follow *s,struct sg_input *in,const struct sg_config *cfg) {
    *s=(struct sg_follow){0};*in=(struct sg_input){.now=100000,.source_boot=123,.fresh=true};
    for(int i=0;i<7;i++)in->ticks[i]=4090;
    for(int k=0;k<102;k++) { in->now+=5000;in->scan++;sg_step(s,cfg,in); }
    assert(s->phase==SG_READY && s->zero_valid);
}
static void start(struct sg_follow *s,struct sg_input *in,const struct sg_config *cfg) {
    ready(s,in,cfg);in->can_start=true;
    for(int k=0;k<102;k++) { in->now+=5000;in->scan++;sg_step(s,cfg,in); }
    assert(s->phase==SG_FOLLOW);
}
static void check_initial_approach(const struct sg_config *cfg) {
    struct sg_follow s;struct sg_input in;ready(&s,&in,cfg);
    in.ticks[0]+=547;in.ticks[1]-=400;in.can_start=true;in.pause=true;
    for(unsigned k=0;k<200;k++){in.now+=4000;in.scan++;sg_step(&s,cfg,&in);assert(s.phase==SG_READY);}
    in.pause=false;
    for(unsigned k=0;k<125;k++){in.now+=4000;in.scan++;sg_step(&s,cfg,&in);assert(s.phase==SG_READY);}
    in.now+=4000;in.scan++;sg_step(&s,cfg,&in);
    assert(s.phase==SG_CATCHUP && !s.aligned);
    for(unsigned j=0;j<6;j++)assert(s.target[j]==in.actual[j]);
    float previous[6];memcpy(previous,s.target,sizeof(previous));
    for(unsigned k=0;k<3000 && s.phase!=SG_FOLLOW;k++) {
        in.now+=4000;in.scan++;memcpy(in.actual,s.target,sizeof(in.actual));sg_step(&s,cfg,&in);
        for(unsigned j=0;j<6;j++){assert(fabsf(s.target[j]-previous[j])<=30*DEG*.004f+1e-5f);previous[j]=s.target[j];}
    }
    assert(s.phase==SG_FOLLOW && s.error==SG_OK);
}
static void check_recovery(const struct sg_config *cfg) {
    struct sg_follow s;struct sg_input in;
    start(&s,&in,cfg);
    /* A long input outage is an unlatched HOLD, not an exit from control. */
    in.fresh=false;in.now+=4000;sg_step(&s,cfg,&in);
    uint64_t hold_at=in.now;float held[6];memcpy(held,s.target,sizeof(held));
    assert(s.phase==SG_HOLD);
    for(unsigned i=0;i<3000;i++) { /* 12 s of continuous loss */
        in.now+=4000;sg_step(&s,cfg,&in);
        assert(s.phase==SG_HOLD && s.error==SG_OK && s.loss_at==hold_at);
        for(unsigned j=0;j<6;j++)assert(s.target[j]==held[j]);
    }
    /* Recovery starts a bounded trajectory from the held pose to the master;
     * it must not jump and must not require the master to come back. The
     * 547-tick offset is the ~48 deg mismatch seen in the 2026-11-17 log. */
    in.ticks[0]+=547;in.fresh=true;in.now+=4000;in.scan++;sg_step(&s,cfg,&in);
    assert(s.phase==SG_CATCHUP && s.error==SG_OK);
    float previous[6];memcpy(previous,s.target,sizeof(previous));
    unsigned ticks=0;bool followed=false;
    for(;ticks<3000;ticks++) {
        in.now+=4000;in.scan++;in.actual[0]=s.target[0];sg_step(&s,cfg,&in);
        for(unsigned j=0;j<6;j++) {
            float delta=fabsf(s.target[j]-previous[j]);
            assert(delta<=30*DEG*.004f+1e-5f);
            previous[j]=s.target[j];
        }
        if(s.phase==SG_FOLLOW) { followed=true;break; }
    }
    assert(followed && ticks>10); /* it actually travelled instead of snapping */
    float master=547.f*(6.2831853071795864769f/4096.f)*cfg->factor[0];
    assert(fabsf(s.target[0]-master)<1e-5f);
    /* The hand target remains a real trajectory while the master keeps moving. */
    for(unsigned i=0;i<20;i++) {
        in.now+=4000;in.scan++;in.ticks[0]+=1;sg_step(&s,cfg,&in);
    }
    assert(s.phase==SG_FOLLOW);
    assert(fabsf(s.target[0]-(master+20.f*(6.2831853071795864769f/4096.f)*cfg->factor[0]))<1e-5f);

    /* Pause during catch-up freezes the trajectory; resume continues it. */
    in.ticks[0]+=100;in.pause=true;in.now+=4000;in.scan++;sg_step(&s,cfg,&in);
    assert(s.phase==SG_HOLD);
    memcpy(held,s.target,sizeof(held));
    for(unsigned i=0;i<100;i++) {
        in.now+=4000;in.scan++;sg_step(&s,cfg,&in);
        assert(s.phase==SG_HOLD);
        for(unsigned j=0;j<6;j++)assert(s.target[j]==held[j]);
    }
    in.pause=false;in.now+=4000;in.scan++;sg_step(&s,cfg,&in);
    assert(s.phase==SG_CATCHUP);
    for(unsigned i=0;i<3000 && s.phase!=SG_FOLLOW;i++) { in.now+=4000;in.scan++;sg_step(&s,cfg,&in); }
    assert(s.phase==SG_FOLLOW);

    /* A source reboot is also a hold-and-catch-up event, not a latched fault. */
    in.source_boot++;in.ticks[1]+=50;in.now+=4000;in.scan++;sg_step(&s,cfg,&in);
    assert(s.phase==SG_CATCHUP && s.error==SG_OK);
    for(unsigned i=0;i<3000 && s.phase!=SG_FOLLOW;i++) { in.now+=4000;in.scan++;sg_step(&s,cfg,&in); }
    assert(s.phase==SG_FOLLOW);

    /* Non-finite master data holds the last target; local feedback faults still stop. */
    in.home_zero[2]=NAN;in.now+=4000;in.scan++;sg_step(&s,cfg,&in);
    assert(s.phase==SG_HOLD && s.error==SG_OK);
    in.home_zero[2]=0;in.now+=4000;in.scan++;sg_step(&s,cfg,&in);
    assert(s.phase==SG_CATCHUP);
    in.fatal=SG_FEEDBACK;sg_step(&s,cfg,&in);
    assert(s.phase==SG_FAULT && s.error==SG_FEEDBACK);
}
int main(void) {
    for(unsigned side=0;side<2;side++) {
        const struct sg_config *cfg=&sg_configs[side];struct sg_follow s;struct sg_input in;
        check_recovery(cfg);
        check_initial_approach(cfg);
        ready(&s,&in,cfg);
        for(int k=0;k<150;k++) { in.now+=5000;sg_step(&s,cfg,&in); }
        assert(s.phase==SG_READY); /* No peer/mode/home readiness: no takeover. */
        start(&s,&in,cfg);
        for(int i=0;i<6;i++)in.ticks[i]+=10; /* 4095/0 unwrap crossing */
        in.now+=5000;sg_step(&s,cfg,&in);
        for(int i=0;i<6;i++)assert(fabsf(s.target[i]-10*(6.28318530718f/4096)*cfg->factor[i])<1e-6);
        in.fresh=false;in.actual[0]=.1f;in.now+=5000;sg_step(&s,cfg,&in);
        assert(s.phase==SG_HOLD && s.target[0]==.1f);
        in.actual[0]=.2f;in.now+=5000;sg_step(&s,cfg,&in);assert(s.target[0]==.1f);
        in.fresh=true;
        for(int k=0;k<20;k++) { in.now+=5000;sg_step(&s,cfg,&in); }
        assert(s.phase==SG_CATCHUP); /* mismatch is caught up, not rejected */
        for(int k=0;k<3000 && s.phase!=SG_FOLLOW;k++) { in.now+=5000;sg_step(&s,cfg,&in); }
        assert(s.phase==SG_FOLLOW && s.zero[0]==4090);
        for(int e=SG_FEEDBACK;e<=SG_SCHEDULER;e++) {
            start(&s,&in,cfg);in.fatal=(enum sg_error)e;sg_step(&s,cfg,&in);assert(s.phase==SG_FAULT);
        }
        start(&s,&in,cfg);in.actual[2]=NAN;sg_step(&s,cfg,&in);assert(s.phase==SG_FAULT);
        assert(fabsf(sg_grasp(cfg,(int32_t)cfg->grip_max))<1e-6);
        assert(fabsf(sg_grasp(cfg,(int32_t)cfg->grip_min)-100)<1e-6);
        assert(fabsf(sg_grasp(cfg,(int32_t)cfg->grip_min+4096)-100)<1e-6);
        float current=cfg->base_ma,q[6];sg_hand(cfg,100,.004f,&current,q);
        assert(fabsf(current-512)<1e-5);
        for(int k=0;k<200;k++)sg_hand(cfg,100,.004f,&current,q);
        assert(current==2000);
        const float max[6]={75,20,55,61,60,57};
        for(int i=0;i<6;i++)assert(fabsf(q[i]-max[i]*3.14159265359f/180)<1e-5);
        sg_hand(cfg,0,.004f,&current,q);assert(current==500);
        for(int i=0;i<6;i++)assert(q[i]==0);
    }
    /* Duplicate scans and moving masters cannot satisfy startup-zero capture. */
    struct sg_follow s={0};struct sg_input in={.now=1,.fresh=true,.source_boot=123,.scan=1};
    for(int k=0;k<200;k++) { in.now+=5000;sg_step(&s,&sg_configs[0],&in); }
    assert(!s.zero_valid);
    for(int k=0;k<200;k++) { in.now+=5000;in.scan++;in.ticks[2]+=9;sg_step(&s,&sg_configs[0],&in); }
    assert(!s.zero_valid);
    in.fresh=false;sg_step(&s,&sg_configs[0],&in);assert(s.phase==SG_WAIT && !s.count);
    puts("Standalone: startup gates, both mappings, wrap, unlatched stale hold, bounded catch-up, reboot, hand curve/current passed");
}
