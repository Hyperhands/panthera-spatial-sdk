#include "gello_follow.h"
#include <math.h>
#include <string.h>

#define RAD_PER_TICK (6.2831853071795864769f/4096.f)
#define DEG (3.14159265358979323846f/180.f)
/* Recovery is a trajectory, never a step: the held target approaches the
 * current master pose with a bounded speed and acceleration. Nominal follow
 * keeps the original direct position command once the target has caught up. */
#define CATCHUP_VEL (30.f*DEG)
#define CATCHUP_ACC (60.f*DEG)
#define MAX_STEP_S 0.05f
static float clamp(float x,float lo,float hi) { return fminf(hi,fmaxf(lo,x)); }
static void capture_start(struct sg_follow *s,const struct sg_input *in) {
    s->phase=SG_ZERO;s->stable_at=in->now;s->count=1;s->last_scan=in->scan;
    for(int i=0;i<6;i++)s->minimum[i]=s->maximum[i]=in->ticks[i],s->sum[i]=in->ticks[i];
}
static void enter_hold(struct sg_follow *s,const struct sg_input *in) {
    if(s->phase!=SG_HOLD) {
        s->hold_at=in->now;
        /* Freeze the measured pose. A target that was already leading during
         * catch-up must not keep moving while input is absent or paused. */
        memcpy(s->target,in->actual,sizeof(s->target));
    }
    s->phase=SG_HOLD;s->aligned_at=0;s->catch_at=0;s->step_at=0;
    memset(s->catch_velocity,0,sizeof(s->catch_velocity));
    s->loss_at=in->fresh?0:in->now;
}
static void start_catchup(struct sg_follow *s,uint64_t now) {
    s->phase=SG_CATCHUP;s->catch_at=now;s->step_at=now;s->aligned_at=0;
    memset(s->catch_velocity,0,sizeof(s->catch_velocity));
}
/* Returns true when every axis has reached the current master target. */
static bool catch_up(struct sg_follow *s,const float desired[6],uint64_t now) {
    float dt=0;
    if(s->step_at && now>s->step_at)dt=(now-s->step_at)*1e-6f;
    s->step_at=now;
    if(dt>MAX_STEP_S)dt=MAX_STEP_S;
    bool arrived=true;
    for(int i=0;i<6;i++) {
        float error=desired[i]-s->target[i];
        float reach=CATCHUP_VEL*dt;
        if(fabsf(error)<=reach) {
            s->target[i]=desired[i];s->catch_velocity[i]=0;continue;
        }
        arrived=false;
        float direction=error>0?1.f:-1.f;
        /* Brake on a sqrt profile so the target does not overshoot the master. */
        float limit=sqrtf(2.f*CATCHUP_ACC*fabsf(error));
        if(limit>CATCHUP_VEL)limit=CATCHUP_VEL;
        float delta=CATCHUP_ACC*dt;
        float velocity=s->catch_velocity[i]+clamp(direction*limit-s->catch_velocity[i],-delta,delta);
        s->catch_velocity[i]=velocity;s->target[i]+=velocity*dt;
        if((desired[i]-s->target[i])*direction<0) {
            s->target[i]=desired[i];s->catch_velocity[i]=0;
        }
    }
    return arrived;
}
void sg_step(struct sg_follow *s,const struct sg_config *cfg,const struct sg_input *in) {
    s->aligned=false;
    if(s->phase==SG_FAULT)return;
    if(in->fatal) { s->phase=SG_FAULT;s->error=in->fatal;return; }
    bool reboot=s->source_boot && in->source_boot && s->source_boot!=in->source_boot;
    if(in->source_boot)s->source_boot=in->source_boot;
    if(reboot) {
        s->aligned_at=0;
        if(!s->zero_valid) { s->phase=SG_WAIT;s->count=0; }
        else if(s->phase==SG_FOLLOW || s->phase==SG_CATCHUP || s->phase==SG_HOLD)enter_hold(s,in);
        /* READY stays READY: the servo ticks are absolute, so the established
         * zero remains usable and the next takeover still needs alignment. */
    }
    if(!s->zero_valid) {
        if(!in->fresh) { s->phase=SG_WAIT;s->count=0;return; }
        if(!s->count) { capture_start(s,in);return; }
        if(s->last_scan==in->scan)return;
        s->last_scan=in->scan;
        for(int i=0;i<6;i++) {
            if(in->ticks[i]<s->minimum[i])s->minimum[i]=in->ticks[i];
            if(in->ticks[i]>s->maximum[i])s->maximum[i]=in->ticks[i];
            if((int64_t)s->maximum[i]-s->minimum[i]>8) { capture_start(s,in);return; }
        }
        for(int i=0;i<6;i++)s->sum[i]+=in->ticks[i];
        s->count++;
        if(in->now-s->stable_at<500000 || s->count<10)return;
        for(int i=0;i<6;i++)s->zero[i]=(float)(s->sum[i]/s->count);
        s->zero_valid=true;s->phase=SG_READY;
    }
    float desired[6];bool aligned=true,master_ok=true;
    s->alignment_axis=1;s->alignment_error_rad=0;
    for(int i=0;i<6;i++) {
        desired[i]=in->home_zero[i]+((float)in->ticks[i]-s->zero[i])*RAD_PER_TICK*cfg->factor[i];
        if(!isfinite(in->actual[i]) || !isfinite(in->velocity[i])) {
            s->phase=SG_FAULT;s->error=SG_FEEDBACK;return;
        }
        if(!isfinite(desired[i])) { master_ok=false;continue; }
        float error=fabsf(desired[i]-in->actual[i]);
        if(error>s->alignment_error_rad)s->alignment_error_rad=error,s->alignment_axis=(unsigned)i+1;
        if(error>10*DEG || fabsf(in->velocity[i])>25*DEG)aligned=false;
    }
    if(!master_ok)aligned=false;
    s->aligned=aligned;
    if(!master_ok) {
        /* A malformed master sample must not end local control. Freeze the
         * last good target and wait for a finite pose. */
        if(s->phase==SG_FOLLOW || s->phase==SG_CATCHUP)enter_hold(s,in);
        s->aligned_at=0;return;
    }
    if(s->phase==SG_READY) {
        if(!in->fresh || !in->can_start || in->pause) { s->aligned_at=0;return; }
        if(!s->aligned_at)s->aligned_at=in->now;
        if(in->now-s->aligned_at<500000)return;
        /* First takeover uses the same bounded approach as reconnect. The
         * master need not be returned to the robot's current pose. */
        memcpy(s->target,in->actual,sizeof(s->target));
        start_catchup(s,in->now);
    }
    if(s->phase==SG_FOLLOW && (!in->fresh || in->pause))enter_hold(s,in);
    if(s->phase==SG_HOLD) {
        /* Loss is never a latched stop. Keep the last finite target under the
         * local MIT loop until fresh input returns, however long that takes. */
        if(!in->fresh) {
            if(!s->loss_at)s->loss_at=in->now;
            s->aligned_at=0;return;
        }
        s->loss_at=0;
        if(in->pause) { s->aligned_at=0;return; }
        start_catchup(s,in->now);
    }
    if(s->phase==SG_CATCHUP) {
        if(!in->fresh || in->pause) { enter_hold(s,in);return; }
        if(!catch_up(s,desired,in->now))return;
        s->phase=SG_FOLLOW;s->step_at=0;s->aligned_at=0;
    }
    if(s->phase==SG_FOLLOW)memcpy(s->target,desired,sizeof(s->target));
}
float sg_grasp(const struct sg_config *cfg,int32_t ticks) {
    float middle=(cfg->grip_min+cfg->grip_max)*.5f;
    float value=(float)ticks-4096.f*roundf(((float)ticks-middle)/4096.f);
    float opening=clamp((value-cfg->grip_min)/(cfg->grip_max-cfg->grip_min),0,1);
    if(cfg->grip_inverted)opening=1-opening;
    return 100*(1-opening);
}
void sg_hand(const struct sg_config *cfg,float percent,float elapsed,float *current,float target[6]) {
    float p=clamp(percent,0,100),path=clamp(p*100/cfg->close_at,0,100);
    float q[6]={0},bends[5]={13,53,56,55,52};
    q[0]=100*clamp(path/20,0,1);
    for(int i=0;i<5;i++)q[i+1]=path<=20?0:path<=95?bends[i]*(path-20)/75:bends[i]+path-95;
    for(int i=0;i<6;i++)target[i]=(q[i]*cfg->hand_scale[i]+cfg->hand_offset_deg[i])*DEG;
    float want=cfg->base_ma+(cfg->max_ma-cfg->base_ma)*clamp((p-cfg->close_at)/(100-cfg->close_at),0,1);
    *current=want<=*current?want:fminf(want,*current+cfg->slew_ma_s*fmaxf(0,elapsed));
}
