#include "gello_standalone.h"
#include "gello_profile.h"
#include "gello/gello_radio_link.h"
#include "homing/local_home.h"
#include "peer/peer_link.h"
#include "devices/hand_current.h"
#include "devices/hand_joints.h"
#include <zephyr/drivers/gpio.h>
#include <math.h>
#include <string.h>
#ifdef CONFIG_ROBOT_GELLO_WEB
#include "gello_web.h"
static void hold_hand(void);
#endif

/* PHD1 home_source values reserved by this candidate:
 * 4 = startup aligned/ready; 5 = following; 6 = input-loss/alignment/catch-up hold.
 * These are deliberately published only after all LOCAL readiness gates.
 * Existing values 1..3 retain boot/SW2/paired-home meaning. */
BUILD_ASSERT(CONFIG_ROBOT_NODE_ID==11 || CONFIG_ROBOT_NODE_ID==12);
BUILD_ASSERT(!IS_ENABLED(CONFIG_ROBOT_GELLO_RADIO_GATEWAY) || CONFIG_ROBOT_NODE_ID==12);
static const struct sg_config *const cfg=&sg_configs[CONFIG_ROBOT_NODE_ID-11];
static struct sg_follow follow;
static bool initialized,owned,mode_requested,hand_rearmed,was_following;
static uint64_t last_tick,mode_at,peer_boot;
static float hand_current;
/* HOLD is compliant, never a rigid pose lock: position target follows the
 * measured pose and position gain ramps to zero. Gravity feedforward and
 * damping stay active so the arm floats instead of falling. */
static float follow_stiffness=1;
/* Web-pushed master topology: 0=dual masters, 1=only the node 11 arm
 * follows (the web console's right-hand card), 2=only the node 12 arm
 * follows (the web console's left-hand card). The idle arm is held by its
 * own gates and never starts or faults; the active arm needs only its own
 * seven-node GELLO group to start and to follow. */
static uint8_t topology;
static uint32_t negative_ack;
static atomic_t color,phase_status,error_status,last_indicator_ms;
enum peer_issue {
    PI_LINK=1, PI_REVERSE=2, PI_DISABLED=4, PI_ARM=8,
    PI_HAND=16, PI_CAN=32, PI_BOOT=64, PI_STATE=128
};
enum ready_issue {
    RI_INPUT=1, RI_HOME=2, RI_ARM=4, RI_HAND=8, RI_CAN=16,
    RI_MODE=32, RI_LEASE=64, RI_ALIGNMENT=128, RI_PEER=256, RI_PROFILE=512,
    RI_TOPOLOGY=1024
};
static atomic_t ready_status,alignment_axis_status,alignment_mdeg_status;
/* First fault only; kept until reboot, also readable via SWD with the matching ELF. */
static struct {
    bool valid;
    uint64_t at,peer_age,boot,expected_boot;
    unsigned error,previous,peer_issues,peer_home,peer_source,peer_active;
    unsigned peer_reverse_age,peer_arm_age,peer_hand_age,peer_can;
} fault_snapshot;
static const struct gpio_dt_spec leds[3]={
    GPIO_DT_SPEC_GET(DT_ALIAS(led0),gpios),GPIO_DT_SPEC_GET(DT_ALIAS(led1),gpios),GPIO_DT_SPEC_GET(DT_ALIAS(led2),gpios)};
extern void robot_can_diagnostics(diagnostics_value_t *d);

#ifdef CONFIG_ROBOT_GELLO_WEB
#include "gello_web.inc"
#endif

static bool arm_ready(const struct home_input *s) {
    if(s->q_mask!=63 || s->dq_mask!=63 || s->fault_mask!=63)return false;
    for(unsigned i=0;i<6;i++)if(s->age[i]>=20000 || s->fault[i] || !isfinite(s->q[i]) || !isfinite(s->dq[i]))return false;
    return true;
}
static bool hand_ready(void) {
    if(atomic_get(&robot.hand_tx_disabled) || !robot.hand_feedback.online || !(robot.hand_feedback.valid_fields&2u)
       || robot.hand_feedback.field_age_us[1]>=100000)return false;
    for(unsigned i=0;i<6;i++)if(!isfinite(robot.hand_feedback.joint_q_rad[hand_active_feedback_index[i]]))return false;
    return true;
}
static unsigned peer_issues(const struct panthera_peer_state *p,bool connected,uint64_t age) {
    /* In single-arm topology the active side runs from its own seven-node
     * group alone; the held or physically absent peer can never take control
     * and must not gate the start or the web console's trial safety. */
    if(topology==CONFIG_ROBOT_NODE_ID-10)return 0;
    if(!connected)return PI_LINK;
    unsigned issues=0;
    if(p->peer_boot!=robot.boot || p->peer_age_us>=20000)issues|=PI_REVERSE;
    if(!p->actuators)issues|=PI_DISABLED;
    if(p->q_mask!=63 || p->dq_mask!=63 || p->fault_mask!=63)issues|=PI_ARM;
    for(unsigned i=0;i<6;i++)if((uint64_t)p->arm_age_us[i]+age>=20000 || p->fault[i] ||
        !isfinite(p->q[i]) || !isfinite(p->dq[i]))issues|=PI_ARM;
    if(!(p->hand_fields&2u) || (uint64_t)p->hand_age_us[1]+age>=100000)issues|=PI_HAND;
    if(p->can_state!=0)issues|=PI_CAN;
    if(owned && p->boot!=peer_boot)issues|=PI_BOOT;
    /* In single-arm topology the held peer can never take control, so its
     * prepared state is not a start gate for the active side. */
    if(topology!=CONFIG_ROBOT_NODE_ID-10 && !((p->home_state==HOME_HOLD && p->home_source==4) ||
         (p->home_state==HOME_IDLE && p->active && (p->home_source==5 || p->home_source==6))))issues|=PI_STATE;
    return issues;
}
void gello_standalone_init(void) {
    for(unsigned i=0;i<3;i++)if(gpio_is_ready_dt(&leds[i]))gpio_pin_configure_dt(&leds[i],GPIO_OUTPUT_INACTIVE);
    #ifdef CONFIG_ROBOT_GELLO_WEB
    web_reset();
#endif
    follow_stiffness=1;hand_current=cfg->base_ma;initialized=true;atomic_set(&color,4);
}
static void hold_hand(void) {
    /* Hold the last finite measured pose. A hand-feedback dropout must not
     * propagate NaN into the MIT target or end arm following. */
    for(unsigned i=0;i<6;i++)if(isfinite(robot.hand_feedback.joint_q_rad[hand_active_feedback_index[i]]))
        robot.hand.target_q_rad[i]=robot.hand_feedback.joint_q_rad[hand_active_feedback_index[i]];
    hand_current=cfg->base_ma;hand_rearmed=false;
}
void gello_standalone_tick(uint64_t now) {
    if(!initialized || (last_tick && now-last_tick<4000))return;
    uint64_t elapsed=last_tick?now-last_tick:4000;last_tick=now;
    struct gr_frame radio={0};bool have=gello_radio_snapshot(&radio,now);
    struct panthera_peer_state peer={0};uint64_t peer_at=0;
    bool connected=panthera_peer_latest(&peer,&peer_at) && now>=peer_at && now-peer_at<20000;
    diagnostics_value_t d;init_diagnostics(&d);robot_can_diagnostics(&d);
    struct sg_input input={.now=now,.source_boot=radio.source_boot,.scan=radio.scan,.fresh=have};
    for(unsigned i=0;i<7;i++) {
        const struct gello_wire_sample *s=&radio.samples[cfg->group*7+i];
        input.ticks[i]=s->ticks;
        if(!s->valid || s->age_us>=GR_STALE_US)input.fresh=false;
    }
    /* Before either arm starts, both seven-node groups must be present. */
    bool both_fresh=have;
    for(unsigned i=0;i<14;i++)if(!radio.samples[i].valid || radio.samples[i].age_us>=GR_STALE_US)both_fresh=false;
    /* Single-arm topology: the active side starts from its own group alone;
     * the idle side is forced into the compliant hold and never faults. */
    unsigned my_side=CONFIG_ROBOT_NODE_ID-11;
    bool allowed=topology==0 || topology==my_side+1;
    /* Teleop autonomy: each side activates on its own gates. In dual
     * topology a fresh own-group input is enough - if any leader sample is
     * stale, the stale group's own side cannot start either, so a start
     * never conflicts with the peer; the peer joins on its own gates later. */
    bool start_ok=topology==0?input.fresh:(allowed && input.fresh);
    k_spinlock_key_t key=k_spin_lock(&robot_lock);
    struct home_input actual;local_home_input(&actual);
    memcpy(input.actual,actual.q,sizeof(input.actual));memcpy(input.velocity,actual.dq,sizeof(input.velocity));
    memcpy(input.home_zero,local_home.zero,sizeof(input.home_zero));
    bool arm_ok=arm_ready(&actual),hand_ok=hand_ready(),can_ok=d.can_bus_state==0;
    unsigned peer_flags=peer_issues(&peer,connected,now>=peer_at?now-peer_at:UINT64_MAX);
    /* Ownership survives a peer reboot. Adopt its new identity only after
     * fresh bidirectional feedback, actuator health and prepared state agree.
     * Unhealthy/restarting peers still block web trials; no command is replayed. */
    if(owned && peer_flags==PI_BOOT) {
        peer_boot=peer.boot;
        peer_flags=0;
    }
    bool peer_ok=(peer_flags&~PI_STATE)==0;
    bool home_ok=local_home.phase==HOME_HOLD;
    bool peer_prepared=peer_flags==0;
    bool actuate=IS_ENABLED(CONFIG_ROBOT_GELLO_STANDALONE_ACTUATE);
    if(owned) {
        /* Master/peer/hand problems never release the arm owner. The follow
         * state machine holds and catches up; only a local control-path loss
         * (fresh arm feedback, scheduler, homing owner) can latch a stop. */
        if(elapsed>=50000)input.fatal=SG_SCHEDULER;
        else if(!arm_ok)input.fatal=SG_FEEDBACK;
        else if(local_home.phase!=HOME_IDLE || !robot.active)input.fatal=SG_HOME;
    } else if(local_home.phase==HOME_FAULT || local_home.phase==HOME_STOPPED || local_home.phase==HOME_SOFT_HOLD)input.fatal=SG_HOME;
    /* Use the existing CAN transaction/ack/readback guard for mode 3. */
    if(actuate && !owned && !mode_requested && home_ok && arm_ok && hand_ok && can_ok && start_ok && !robot.lease) {
        robot.hand_mode_request=3;robot.hand_guard_phase=0;robot.hand_guard_status=0;
        robot.config.hand_control_feedforward_ma=(uint32_t)cfg->base_ma;
        for(unsigned i=0;i<6;i++)robot.config.hand_soft_current_ma[i]=(uint32_t)cfg->base_ma;
        mode_requested=true;mode_at=now;
    }
    bool mode_ok=mode_requested && robot.hand_guard_phase==7 && robot.hand_guard_status==0 && robot.config.hand_mode==3;
    if(!owned && actuate && mode_requested && !mode_ok && (robot.hand_guard_phase==8 || now-mode_at>1000000))input.fatal=SG_HAND;
#ifdef CONFIG_ROBOT_GELLO_WEB
    bool web_safe=actuate && owned && !robot.lease && robot.active && arm_ok && hand_ok && can_ok && peer_prepared && input.fresh && !input.fatal;
    /* Releasing a pause only needs a viable local arm path. Peer/hand faults
     * keep the hand held but must not trap the arm in a user-requested pause. */
    bool web_resume=actuate && owned && !robot.lease && robot.active && arm_ok && can_ok && input.fresh && !input.fatal;
    web_apply(now,web_safe,web_resume || (!owned && !robot.lease && actuate && home_ok && arm_ok && hand_ok && can_ok && start_ok && !input.fatal));
#endif
    /* Recompute after web_apply: a topology command takes effect on the same
     * tick, so the held arm enters the compliant hold without a one-tick lag. */
    allowed=topology==0 || topology==my_side+1;
    start_ok=topology==0?input.fresh:(allowed && input.fresh);
#ifdef CONFIG_ROBOT_GELLO_WEB
    input.pause=web_paused || !allowed;
#endif
    input.can_start=actuate && start_ok && home_ok && arm_ok && hand_ok && can_ok && mode_ok && !robot.lease;
#ifdef CONFIG_ROBOT_GELLO_WEB
    input.can_start=input.can_start && web_profile_ready;
#endif
    if(!follow.zero_valid)input.fresh=start_ok;
    enum sg_phase previous=follow.phase;
    sg_step(&follow,cfg,&input);
    unsigned ready_flags=(!both_fresh?RI_INPUT:0) | (!home_ok?RI_HOME:0) | (!arm_ok?RI_ARM:0) |
        (!hand_ok?RI_HAND:0) | (!can_ok?RI_CAN:0) | (!mode_ok?RI_MODE:0) |
        (robot.lease?RI_LEASE:0) | (!follow.aligned?RI_ALIGNMENT:0) | (!peer_prepared?RI_PEER:0) |
        (!allowed?RI_TOPOLOGY:0);
#ifdef CONFIG_ROBOT_GELLO_WEB
    if(!web_profile_ready)ready_flags|=RI_PROFILE;
#endif
    if(follow.phase==SG_FAULT && previous!=SG_FAULT && actuate) {
        /* Never let a stale wireless snapshot outlive the existing CAN owner. */
        if(local_home.phase!=HOME_FAULT && local_home.phase!=HOME_STOPPED)local_home_soft_stop(HOME_PEER_LOST);
        robot.hand_dynamic_current=false;robot.config.hand_control_feedforward_ma=(uint32_t)cfg->base_ma;
        hand_rearmed=false;
    }
    if(actuate && follow.phase==SG_READY && home_ok) {
        /* Advertise local readiness independently of peer readiness and pose
         * mismatch. Initial takeover approaches the master with bounded speed. */
        local_home.source=(ready_flags&~(RI_PEER|RI_ALIGNMENT))==0 && !input.pause?4:1;
    }
    bool controlling=follow.phase==SG_FOLLOW || follow.phase==SG_HOLD || follow.phase==SG_CATCHUP;
    if(actuate && controlling) {
        float gravity[6];home_gravity(actual.q,gravity);
        bool finite=true;
        for(unsigned i=0;i<6;i++)if(!isfinite(gravity[i]))finite=false;
        if(!finite) {
            follow.phase=SG_FAULT;follow.error=SG_FEEDBACK;local_home_soft_stop(HOME_NONFINITE);
        } else {
            /* HOLD and teaching are compliant, never a rigid pose lock: track
             * the measured pose so the operator can move the arm, and ramp the
             * position gain to zero. Gravity feedforward and damping remain.
             * A held target must never stay behind while the arm is dragged,
             * otherwise resuming would command a jump back to the old pose. */
            bool compliant=follow.phase==SG_HOLD;
#ifdef CONFIG_ROBOT_GELLO_WEB
            if(web_teach)compliant=true;
#endif
            if(compliant)memcpy(follow.target,actual.q,sizeof(follow.target));
            float stiffness_goal=compliant?0.f:1.f;
            float stiffness_step=(float)elapsed/1000000.f;
            follow_stiffness+=fminf(stiffness_step,fmaxf(-stiffness_step,stiffness_goal-follow_stiffness));
#ifdef CONFIG_ROBOT_GELLO_WEB
            web_stiffness=follow_stiffness;
#endif
            for(unsigned i=0;i<6;i++) {
                robot.arm.mit[5*i]=follow.target[i];robot.arm.mit[5*i+1]=0;
                robot.arm.mit[5*i+2]=cfg->kp[i]*follow_stiffness;
                robot.arm.mit[5*i+3]=cfg->kd[i];robot.arm.mit[5*i+4]=gravity[i];
            }
            robot.arm.joint_mask=63;robot.arm_at=now;robot.arm_received++;robot.arm_seq++;
            if(!owned) {
                if(!local_home_activate()) {
                    follow.phase=SG_FAULT;follow.error=SG_HOME;local_home_soft_stop(HOME_TRACKING);
                } else {
                    owned=true;peer_boot=peer.boot;negative_ack=d.hand_control_negative_ack;
                    robot.hand_isolation=false;robot.zero_mit_load=false;robot.active=1;
                    robot.hand_angle_fast=true;robot.hand_poll_period_us=10000;hold_hand();
                }
            }
            if(owned && robot.active) {
                local_home.source=follow.phase==SG_FOLLOW?5:6;
                if(!hand_ok)hold_hand();
                else if(follow.phase!=SG_FOLLOW && was_following)hold_hand();
                float grasp=sg_grasp(cfg,input.ticks[6]);
                bool newly_armed=false;
#ifdef CONFIG_ROBOT_GELLO_WEB
                if(!web_manual) {
#endif
                if(follow.phase==SG_FOLLOW && hand_ok && input.fresh && !hand_rearmed && grasp<=5) { hand_rearmed=true;newly_armed=true; }
                if(follow.phase==SG_FOLLOW && hand_ok && hand_rearmed && !newly_armed)
#ifdef CONFIG_ROBOT_GELLO_WEB
                    web_gello_hand(grasp,(float)elapsed/1000000);
#else
                    sg_hand(cfg,grasp,(float)elapsed/1000000,&hand_current,robot.hand.target_q_rad);
#endif
                else hand_current=cfg->base_ma;
#ifdef CONFIG_ROBOT_GELLO_WEB
                }
                web_hand_tick(now,(float)elapsed/1000000,web_safe && follow.phase==SG_HOLD);
#endif
                robot.hand.duration_ms=0;robot.hand_seq++;robot.hand_at=now;robot.hand_received++;
                robot.hand_dynamic_current=true;
                robot.hand_compliance.control_feedforward_ma=(uint32_t)lroundf(hand_current);
                for(unsigned i=0;i<6;i++)robot.hand_compliance.joint_current_ma[i]=(uint32_t)cfg->base_ma;
            }
        }
    }
    if(follow.phase==SG_FAULT && !fault_snapshot.valid) {
        fault_snapshot.at=now;fault_snapshot.error=follow.error;fault_snapshot.previous=previous;
        fault_snapshot.peer_issues=peer_flags;fault_snapshot.peer_age=now>=peer_at?now-peer_at:UINT64_MAX;
        fault_snapshot.boot=peer.boot;fault_snapshot.expected_boot=peer_boot;
        fault_snapshot.peer_home=peer.home_state;fault_snapshot.peer_source=peer.home_source;
        fault_snapshot.peer_active=peer.active;fault_snapshot.peer_reverse_age=peer.peer_age_us;
        fault_snapshot.peer_hand_age=peer.hand_age_us[1];fault_snapshot.peer_can=peer.can_state;
        for(unsigned i=0;i<6;i++)if(peer.arm_age_us[i]>fault_snapshot.peer_arm_age)fault_snapshot.peer_arm_age=peer.arm_age_us[i];
        fault_snapshot.valid=true;
    }
#ifdef CONFIG_ROBOT_GELLO_WEB
    web_publish(now,&input,&actual,arm_ok,hand_ok,ready_flags);
#endif
    was_following=follow.phase==SG_FOLLOW;
    unsigned led=4;
    if(follow.phase==SG_FAULT)led=1;
    else if(!actuate && both_fresh)led=6; /* cyan: diagnostic, never success green */
    else if(follow.phase==SG_FOLLOW && peer_ok && peer.home_source==5)led=2;
    else if(follow.phase==SG_CATCHUP)led=(now/250000)%2?2:0;
    else if(follow.phase==SG_HOLD)led=(now/250000)%2?3:0;
    else if(follow.phase==SG_FOLLOW)led=2; /* green: this arm teleops, alone or with the peer */
    else if(follow.phase==SG_ZERO || follow.phase==SG_READY)led=3;
    else led=(now/500000)%2?4:0;
    atomic_set(&color,led);atomic_set(&phase_status,follow.phase);atomic_set(&error_status,follow.error);
    atomic_set(&ready_status,ready_flags);atomic_set(&alignment_axis_status,follow.alignment_axis);
    atomic_set(&alignment_mdeg_status,(int32_t)lroundf(fminf(follow.alignment_error_rad,1000.f)*57295.7795f));
    atomic_set(&last_indicator_ms,(uint32_t)(now/1000));
    k_spin_unlock(&robot_lock,key);
}
static void indicators(void *a,void *b,void *c) {
    ARG_UNUSED(a);ARG_UNUSED(b);ARG_UNUSED(c);uint32_t report=0;
    for(;;) {
        if(initialized) {
            unsigned value=(unsigned)atomic_get(&color);
            uint32_t published=(uint32_t)atomic_get(&last_indicator_ms);
            if(published && (uint32_t)(robot_us()/1000)-published>=50)value=1;
            for(unsigned i=0;i<3;i++)if(gpio_is_ready_dt(&leds[i]))gpio_pin_set_dt(&leds[i],(value>>i)&1);
            if(++report==50) {
                printk("GELLO standalone phase=%ld error=%ld color=%u actuate=%d\n",(long)atomic_get(&phase_status),
                    (long)atomic_get(&error_status),value,IS_ENABLED(CONFIG_ROBOT_GELLO_STANDALONE_ACTUATE));report=0;
                printk("GELLO ready_block=0x%lx align_axis=%ld align_mdeg=%ld\n",
                    (unsigned long)atomic_get(&ready_status),(long)atomic_get(&alignment_axis_status),
                    (long)atomic_get(&alignment_mdeg_status));
                k_spinlock_key_t key=k_spin_lock(&robot_lock);
                __typeof__(fault_snapshot) f=fault_snapshot;
                unsigned current_phase=follow.phase;
                uint64_t hold_at=follow.hold_at,loss_at=follow.loss_at,aligned_at=follow.aligned_at;
                uint64_t catch_at=follow.catch_at;
                k_spin_unlock(&robot_lock,key);
                if(current_phase==SG_HOLD || current_phase==SG_CATCHUP || current_phase==SG_FAULT)
                    printk("GELLO hold_since=%llu stale_since=%llu aligned_since=%llu catch_since=%llu\n",
                        (unsigned long long)hold_at,(unsigned long long)loss_at,(unsigned long long)aligned_at,
                        (unsigned long long)catch_at);
                if(f.valid)printk("GELLO first_fault us=%llu error=%u from=%u peer_flags=0x%x "
                    "peer_age=%llu reverse_age=%u arm_age=%u hand_age=%u can=%u home=%u source=%u active=%u boot=%llu expected=%llu\n",
                    (unsigned long long)f.at,f.error,f.previous,f.peer_issues,(unsigned long long)f.peer_age,
                    f.peer_reverse_age,f.peer_arm_age,f.peer_hand_age,f.peer_can,f.peer_home,f.peer_source,f.peer_active,
                    (unsigned long long)f.boot,(unsigned long long)f.expected_boot);
            }
        }
        k_sleep(K_MSEC(100));
    }
}
K_THREAD_DEFINE(gello_indicators,2048,indicators,NULL,NULL,NULL,10,0,0);
