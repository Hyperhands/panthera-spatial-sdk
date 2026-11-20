#include "home_path.h"
#include <math.h>
#include <string.h>
#define PI 3.14159265358979323846f
/* User-taught intermediate pose, captured on left node 11 on 2026-11-17.
 * Both arms use the same joint-space pose; final encoder zero is unchanged.
 * Panthera v4 model: tool +X is about 15.2 degrees above horizontal. */
static const float waypoint[6]={-.030775491f,.434116572f,.752034128f,-.051771853f,-.036911413f,.132018223f};
/* Retain the existing whole-turn branch selection independently of the new
 * waypoint. Changing the waypoint must not change the final encoder zero. */
static const float branch_reference[6]={0,PI/2,2.886410263f,.255182391f,0,0};
bool home_path_start_ok(const float start[6]) {
    /* Starting pose is not a model-range gate. Motion faults remain separate. */
    for(int i=0;i<6;i++) if(!isfinite(start[i])) return false;
    return true;
}
bool home_path_reference(const float raw[6],float zero[6]) {
    if(!home_path_start_ok(raw)) return false;
    /* Freeze one whole-turn branch per joint for the entire home operation.
     * Use the established branch reference, then encoder zero on that branch.
     * A half-turn tie chooses the positive direction in both MCU and host. */
    for(int i=0;i<6;i++) zero[i]=2*PI*floorf((raw[i]-branch_reference[i])/(2*PI)+.5f);
    return home_path_start_ok(zero);
}
void home_path_begin(struct home_path *p,const float start[6],unsigned stage) {
    memcpy(p->start,start,sizeof(p->start));
    memcpy(p->goal,waypoint,sizeof(p->goal));
    if(stage==0) p->goal[0]=start[0];
    if(stage==2) memset(p->goal,0,sizeof(p->goal));
    p->distance=0;
    for(int i=0;i<6;i++) p->distance=fmaxf(p->distance,fabsf(p->goal[i]-start[i]));
    const float speed=30*PI/180,accel=30*PI/180;
    p->ramp=PI*speed/(2*accel);p->speed=speed;
    if(p->distance<speed*p->ramp) {
        p->ramp=fmaxf(.3f,sqrtf(PI*p->distance/(2*accel)));p->speed=p->distance/p->ramp;
    }
    p->duration=p->speed>0?p->distance/p->speed+p->ramp:.6f;
}
void home_path_sample(const struct home_path *p,float t,float q[6],float dq[6]) {
    if(p->distance<1e-9f) {memcpy(q,p->goal,sizeof(p->goal));memset(dq,0,6*sizeof(float));return;}
    float r=p->ramp,v=p->speed,flat=p->distance/v-r,x,speed;t=fmaxf(0,t);
    if(t<r) {x=v/2*(t-r/PI*sinf(PI*t/r));speed=v/2*(1-cosf(PI*t/r));}
    else if(t<r+flat) {x=v*(t-r/2);speed=v;}
    else if(t<p->duration) {float u=t-r-flat;x=v*(r/2+flat)+v/2*(u+r/PI*sinf(PI*u/r));speed=v/2*(1+cosf(PI*u/r));}
    else {x=p->distance;speed=0;}
    for(int i=0;i<6;i++) {float delta=(p->goal[i]-p->start[i])/p->distance;q[i]=p->start[i]+x*delta;dq[i]=speed*delta;}
}
