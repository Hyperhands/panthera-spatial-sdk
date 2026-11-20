/* Integration test: production adapter + pure control logic, fake UART/peer/CAN.
 * No hardware I/O. Compile once for each node/actuation mode. */
#include <assert.h>
#include "../src/standalone/gello_standalone.c"
struct robot_state robot;
struct k_spinlock robot_lock;
struct home_context local_home;
uint64_t local_home_updated;
static uint64_t fake_now;
static struct gr_frame fake_input;
static struct panthera_peer_state fake_peer;
static diagnostics_value_t fake_diag;
static bool radio_available=true,peer_available=true;
static bool hand_follows_target;
static unsigned soft_stops,takeovers;
uint64_t robot_us(void) { return fake_now; }
void diagnostics_value_clear(diagnostics_value_t *d) { memset(d,0,sizeof(*d)); }
void robot_can_diagnostics(diagnostics_value_t *d) { *d=fake_diag; }
bool gello_radio_snapshot(struct gr_frame *f,uint64_t now) { (void)now;*f=fake_input;return radio_available; }
bool panthera_peer_latest(struct panthera_peer_state *p,uint64_t *at) { *p=fake_peer;*at=fake_now-1000;return peer_available; }
void local_home_input(struct home_input *in) {
    const arm_state_value_t *a=&robot.arm_feedback;
    memcpy(in->q,a->q_rad,sizeof(in->q));memcpy(in->dq,a->dq_rad_s,sizeof(in->dq));
    memcpy(in->age,a->feedback_age_us,sizeof(in->age));memcpy(in->fault,a->fault_code_raw,sizeof(in->fault));
    in->q_mask=a->valid_q_mask;in->dq_mask=a->valid_dq_mask;in->fault_mask=a->motor_fault_valid_mask;
}
bool local_home_activate(void) {
    struct home_input in;local_home_input(&in);
    if(!home_takeover_ok(&local_home,&in,robot.arm.mit,fake_now-robot.arm_at))return false;
    takeovers++;local_home.phase=HOME_IDLE;local_home.output=false;return true;
}
bool local_home_soft_stop(unsigned reason) {
    (void)reason;soft_stops++;robot.active=0;robot.arm_received=robot.hand_received=0;
    local_home.phase=HOME_SOFT_HOLD;return true;
}
static void tick(void) {
    if(hand_follows_target)for(unsigned j=0;j<6;j++)robot.hand_feedback.joint_q_rad[hand_active_feedback_index[j]]=robot.hand.target_q_rad[j];
    fake_now+=4000;fake_input.scan++;gello_standalone_tick(fake_now);
}
static void reset(void) {
    memset(&robot,0,sizeof(robot));memset(&follow,0,sizeof(follow));memset(&fake_input,0,sizeof(fake_input));
    memset(&fake_peer,0,sizeof(fake_peer));memset(&fake_diag,0,sizeof(fake_diag));
    memset(&fault_snapshot,0,sizeof(fault_snapshot));
    local_home=(struct home_context){.phase=HOME_HOLD,.output=true};
    initialized=owned=mode_requested=hand_rearmed=was_following=false;
    last_tick=mode_at=peer_boot=0;fake_now=100000;soft_stops=takeovers=negative_ack=0;
    radio_available=peer_available=true;robot.boot=10;
    hand_follows_target=false;
    robot.arm_feedback.valid_q_mask=robot.arm_feedback.valid_dq_mask=robot.arm_feedback.motor_fault_valid_mask=63;
    robot.hand_feedback.online=true;robot.hand_feedback.valid_fields=2;
    fake_input.source_boot=500;fake_input.scan=1;
    for(unsigned i=0;i<14;i++)fake_input.samples[i]=(struct gello_wire_sample){.raw=1000,.ticks=1000,.valid=true};
    fake_input.samples[cfg->group*7+6].ticks=(int32_t)cfg->grip_max;
    fake_peer.boot=20;fake_peer.peer_boot=10;fake_peer.actuators=1;
    fake_peer.q_mask=fake_peer.dq_mask=fake_peer.fault_mask=63;fake_peer.hand_fields=2;
    fake_peer.home_state=HOME_HOLD;fake_peer.home_source=1;
    gello_standalone_init();
#ifdef CONFIG_ROBOT_GELLO_WEB
    web_profile_ready=true; /* Existing scenarios assume startup profile delivery. */
#endif
}
static void run_start(void) {
    reset();for(unsigned i=0;i<150;i++)tick();
    assert(!robot.active && takeovers==0); /* no mode ACK and no prepared peer */
    if(!IS_ENABLED(CONFIG_ROBOT_GELLO_STANDALONE_ACTUATE))return;
    assert(robot.hand_mode_request==3);
    robot.hand_mode_request=0;robot.hand_guard_phase=7;robot.config.hand_mode=3;
    for(unsigned i=0;i<150;i++)tick();
    /* Autonomy: a locally ready side activates on its own gates alone; the
     * peer joins later on its own. */
    assert(robot.active && owned && takeovers==1 && follow.phase==SG_FOLLOW);
    assert(atomic_get(&color)==2); /* green while teleoping, peer state aside */
    fake_peer.home_source=4;tick(); /* healthy peer baseline for the web session */
    fake_peer.home_state=HOME_IDLE;fake_peer.home_source=5;fake_peer.active=1;tick();
    assert(atomic_get(&color)==2);
}
static void check_ready_advertisement(void) {
    reset();for(unsigned i=0;i<150;i++)tick();
    robot.hand_mode_request=0;robot.hand_guard_phase=7;robot.config.hand_mode=3;
    /* The live left J2 error after zero capture was 41 ticks = 3.60 deg, but
     * the fivefold alignment gate is 10 deg, so inject 205 ticks = 18.0 deg
     * to stay above it and keep advertising RI_ALIGNMENT on axis 2. */
    fake_input.samples[cfg->group*7+1].ticks+=205;
    for(unsigned i=0;i<400 && !(follow.phase==SG_FOLLOW && robot.active);i++)tick();
    assert(follow.phase==SG_FOLLOW && robot.active);
    /* The mismatch rides along as a diagnostic: RI_ALIGNMENT stays lit while
     * the arm follows (home sits in IDLE, so source 4 is a READY-state mark). */
    assert((atomic_get(&ready_status)&RI_ALIGNMENT) && atomic_get(&alignment_axis_status)==2);
    assert(atomic_get(&alignment_mdeg_status)>18000 && atomic_get(&alignment_mdeg_status)<18050);
    fake_input.samples[cfg->group*7+1].ticks-=205;tick();
    assert(follow.phase==SG_FOLLOW); /* mismatch clears, still diagnostic-only */
    robot.arm_feedback.dq_rad_s[2]=.1f;tick();
    assert(follow.phase==SG_FOLLOW); /* Velocity/alignment stay diagnostic. */
    robot.arm_feedback.dq_rad_s[2]=0;robot.lease=1;tick();
    assert(local_home.source!=4); /* A host lease blocks the local ready state. */
    robot.lease=0;tick();assert(follow.phase==SG_FOLLOW); /* release disturbs nothing */
}
static void check_peer_fault_details(void) {
    const unsigned reasons[]={PI_LINK,PI_REVERSE,PI_DISABLED,PI_ARM,PI_HAND,PI_CAN,PI_BOOT,PI_STATE};
    for(unsigned i=0;i<sizeof(reasons)/sizeof(reasons[0]);i++) {
        run_start();
        switch(reasons[i]) {
        case PI_LINK:peer_available=false;break;
        case PI_REVERSE:fake_peer.peer_age_us=20000;break;
        case PI_DISABLED:fake_peer.actuators=0;break;
        case PI_ARM:fake_peer.arm_age_us[2]=19000;break; /* plus packet age 1 ms */
        case PI_HAND:fake_peer.hand_age_us[1]=99000;break;
        case PI_CAN:fake_peer.can_state=1;break;
        case PI_BOOT:fake_peer.boot++;break;
        case PI_STATE:fake_peer.home_state=HOME_HOLD;fake_peer.home_source=1;fake_peer.active=0;break;
        }
        tick();
        /* Once this arm owns tracking, the other board cannot take it away. */
        assert(follow.phase==SG_FOLLOW && follow.error==SG_OK && robot.active && !soft_stops);
        assert(!fault_snapshot.valid);
    }
    /* The whole far side absent (board offline + its leader group stale):
     * the ready near side still starts, and the returning side joins without
     * disturbing the established follow. */
    reset();peer_available=false;
    unsigned other=(cfg->group^1)*7;
    for(unsigned i=other;i<other+7;i++)fake_input.samples[i].valid=false;
    for(unsigned i=0;i<150;i++)tick();
    assert(robot.hand_mode_request==3); /* the relaxed gate still arms the hand */
    robot.hand_mode_request=0;robot.hand_guard_phase=7;robot.config.hand_mode=3;
    for(unsigned i=0;i<150;i++)tick();
    assert(robot.active && owned && takeovers==1 && follow.phase==SG_FOLLOW);
    peer_available=true;fake_peer.home_source=4;
    for(unsigned i=other;i<other+7;i++)fake_input.samples[i].valid=true;
    for(unsigned i=0;i<150;i++)tick();
    assert(follow.phase==SG_FOLLOW && robot.active && !soft_stops);
    /* Actual startup-skew path: the peer is still READY when its input expires. */
    run_start();fake_peer.home_state=HOME_HOLD;fake_peer.home_source=4;fake_peer.active=0;tick();
    assert(follow.phase==SG_FOLLOW);
    fake_peer.home_source=1;tick();
    assert(follow.phase==SG_FOLLOW && follow.error==SG_OK && robot.active && !soft_stops);
}
static void check_alignment_hold(void) {
    run_start();radio_available=false;tick();
    assert(follow.phase==SG_HOLD && robot.active);
    float held[6];memcpy(held,follow.target,sizeof(held));
    for(unsigned i=0;i<3000;i++) { /* 12 s without input: hold, never latch */
        tick();assert(follow.phase==SG_HOLD && robot.active && !soft_stops);
        for(unsigned j=0;j<6;j++)assert(robot.arm.mit[j*5]==held[j]);
        assert(!hand_rearmed && robot.hand_compliance.control_feedforward_ma==500);
    }
    /* HOLD is compliant: zero position gain and the MIT target follows a drag. */
    assert(fabsf(robot.arm.mit[2])<1e-5f);
    robot.arm_feedback.q_rad[0]+=.05f;tick();
    assert(fabsf(robot.arm.mit[0]-robot.arm_feedback.q_rad[0])<1e-5f);
    /* Fresh input while the master is far away starts a bounded catch-up. */
    radio_available=true;fake_input.samples[cfg->group*7].ticks+=100;
    fake_peer.home_source=6; /* Peer HOLD is valid; a fault/stop still isn't. */
    float previous[6];memcpy(previous,follow.target,sizeof(previous));
    tick();
    assert(follow.phase==SG_CATCHUP && robot.active && !soft_stops);
    for(unsigned i=0;i<1000 && follow.phase!=SG_FOLLOW;i++) {
        tick();
        for(unsigned j=0;j<6;j++) {
            float delta=fabsf(follow.target[j]-previous[j]);
            assert(delta<=30*(3.14159265358979323846/180)*.004f+1e-5f);
            previous[j]=follow.target[j];
        }
        assert(robot.active && !soft_stops);
    }
    assert(follow.phase==SG_FOLLOW && robot.active && !soft_stops);
    /* A fresh calibration offset is followed directly again. */
    fake_input.samples[cfg->group*7].ticks-=100;
    for(unsigned i=0;i<1000 && follow.phase!=SG_FOLLOW;i++)tick();
    assert(follow.phase==SG_FOLLOW);
    /* Long-lived HOLD must still enforce real motor feedback failures. */
    radio_available=false;tick();radio_available=true;
    fake_input.samples[cfg->group*7].ticks+=100;tick();
    robot.arm_feedback.feedback_age_us[0]=20000;tick();
    assert(follow.phase==SG_FAULT && follow.error==SG_FEEDBACK && !robot.active && soft_stops==1);
}
static void check_hand_feedback_mapping(void) {
    reset();
    /* Distinct active and passive angles catch index shifts in all fingers. */
    const unsigned active[6]={0,1,3,5,7,9};
    for(unsigned j=0;j<11;j++)robot.hand_feedback.joint_q_rad[j]=.025f*(j+1);
    hold_hand();
    for(unsigned j=0;j<6;j++)assert(robot.hand.target_q_rad[j]==.025f*(active[j]+1));
    for(unsigned j=0;j<6;j++) {
        float saved=robot.hand_feedback.joint_q_rad[active[j]];
        robot.hand_feedback.joint_q_rad[active[j]]=NAN;
        assert(!hand_ready());hold_hand();assert(isfinite(robot.hand.target_q_rad[j]));
        robot.hand_feedback.joint_q_rad[active[j]]=saved;
    }
    const unsigned passive[5]={2,4,6,8,10};
    for(unsigned j=0;j<5;j++)robot.hand_feedback.joint_q_rad[passive[j]]=NAN;
    assert(hand_ready());hold_hand();
    for(unsigned j=0;j<6;j++)assert(robot.hand.target_q_rad[j]==.025f*(active[j]+1));
}
static void check_single_arm(void) {
    /* Topology 1 = left GELLO only. The active side starts from its own
     * seven-node group; the held side never starts and never faults. */
    if(CONFIG_ROBOT_NODE_ID==11) {
        reset();topology=1;
        for(unsigned i=7;i<14;i++)fake_input.samples[i].valid=false;
        for(unsigned i=0;i<150;i++)tick();
        assert(!robot.active && takeovers==0 && robot.hand_mode_request==3);
        robot.hand_mode_request=0;robot.hand_guard_phase=7;robot.config.hand_mode=3;
        /* The held peer's unprepared state must not block the active side:
         * it can never take control, so PI_STATE is skipped in this topology. */
        fake_peer.home_source=1;
        for(unsigned i=0;i<150;i++)tick();
        assert(robot.active && owned && takeovers==1 && follow.phase==SG_FOLLOW);
        /* Losing the active group holds compliantly; recovery resumes. */
        for(unsigned i=0;i<7;i++)fake_input.samples[i].valid=false;
        tick();
        assert(follow.phase==SG_HOLD && robot.active && !soft_stops);
        for(unsigned i=0;i<7;i++)fake_input.samples[i].valid=true;
        for(unsigned i=0;i<200 && follow.phase!=SG_FOLLOW;i++)tick();
        assert(follow.phase==SG_FOLLOW && robot.active);
        /* Switching the topology away holds without a soft stop; switching
         * back resumes the catch-up into follow. */
        topology=2;tick();
        assert(follow.phase==SG_HOLD && robot.active && !soft_stops);
        assert(atomic_get(&ready_status)&RI_TOPOLOGY);
        topology=1;
        for(unsigned i=0;i<200 && follow.phase!=SG_FOLLOW;i++)tick();
        assert(follow.phase==SG_FOLLOW && robot.active && takeovers==1);
        /* Console op path: GW_TOPOLOGY applies through the normal gw gate. */
        struct gw_packet command={.kind=GW_COMMAND,.node=CONFIG_ROBOT_NODE_ID,.op=GW_TOPOLOGY,
            .mode=GW_RUN_MODE,.session=77,.sequence=5,.expected_boot=robot.boot};
        command.board[1].profile_crc=2; /* right-only: holds this arm */
        gello_web_submit(&command);tick();
        assert(topology==2 && follow.phase==SG_HOLD && web_state.ack_status==GW_OK);
        command.sequence=6;command.revision=1;command.board[1].profile_crc=1;
        gello_web_submit(&command);
        for(unsigned i=0;i<200 && follow.phase!=SG_FOLLOW;i++)tick();
        assert(topology==1 && follow.phase==SG_FOLLOW);
        command.sequence=7;command.revision=2;command.board[1].profile_crc=3;
        gello_web_submit(&command);tick();
        assert(topology==1 && follow.phase==SG_FOLLOW && web_state.ack_status==GW_BAD);
        /* A physically absent peer (PI_LINK) must not gate the active side:
         * single-arm runs from its own group whether or not the held board
         * is powered. */
        reset();topology=1;peer_available=false;
        for(unsigned i=7;i<14;i++)fake_input.samples[i].valid=false;
        for(unsigned i=0;i<150;i++)tick();
        robot.hand_mode_request=0;robot.hand_guard_phase=7;robot.config.hand_mode=3;
        for(unsigned i=0;i<150;i++)tick();
        assert(robot.active && takeovers==1 && follow.phase==SG_FOLLOW);
        peer_available=true;
    } else {
        reset();topology=1;
        for(unsigned i=7;i<14;i++)fake_input.samples[i].valid=false;
        for(unsigned i=0;i<300;i++)tick();
        assert(!robot.active && !robot.hand_mode_request && takeovers==0);
        assert(atomic_get(&ready_status)&RI_TOPOLOGY);
        assert(follow.phase==SG_WAIT);
        /* The absent right master reappearing must not start this board while
         * the topology still holds it. */
        for(unsigned i=7;i<14;i++)fake_input.samples[i].valid=true;
        for(unsigned i=0;i<300;i++)tick();
        assert(!robot.active && !robot.hand_mode_request);
        /* Dual topology restores the original both-groups gates. */
        topology=0;
        for(unsigned i=0;i<300;i++)tick();
        assert(!robot.active && robot.hand_mode_request==3);
    }
    topology=0;
}
int main(void) {
    check_hand_feedback_mapping();
    run_start();
    if(!IS_ENABLED(CONFIG_ROBOT_GELLO_STANDALONE_ACTUATE)) {
        for(unsigned i=0;i<1000;i++)tick();
        assert(!robot.active && !robot.hand_mode_request && !robot.arm_received && !robot.hand_received);
        assert(atomic_get(&color)==6);puts("Standalone read-only: no mode/target/activation, cyan status passed");return 0;
    }
    check_ready_advertisement();check_peer_fault_details();check_alignment_hold();check_single_arm();run_start();
    fake_input.samples[cfg->group*7+6].ticks=(int32_t)cfg->grip_min;
    for(unsigned i=0;i<150;i++)tick();
    assert(robot.hand_compliance.control_feedforward_ma==2000);
    radio_available=false;tick();
    assert(follow.phase==SG_HOLD && robot.active && robot.hand_compliance.control_feedforward_ma==500);
    radio_available=true;
    for(unsigned i=0;i<150;i++)tick();
    assert(follow.phase==SG_FOLLOW && !hand_rearmed && robot.hand_compliance.control_feedforward_ma==500);
    fake_input.samples[cfg->group*7+6].ticks=(int32_t)cfg->grip_max;tick();
    assert(hand_rearmed);
    /* Losing the other board or the hand keeps this arm under local control. */
    peer_available=false;tick();assert(follow.phase==SG_FOLLOW && robot.active && !soft_stops);
    peer_available=true;for(unsigned i=0;i<500;i++)tick();assert(robot.active && takeovers==1);
    run_start();fake_input.source_boot++;tick();
    assert((follow.phase==SG_CATCHUP || follow.phase==SG_FOLLOW) && follow.error==SG_OK && robot.active);
    run_start();fake_peer.boot++;tick();assert(follow.phase==SG_FOLLOW && follow.error==SG_OK && robot.active);
    run_start();robot.hand_feedback.field_age_us[1]=100000;tick();
    assert(follow.phase==SG_FOLLOW && follow.error==SG_OK && robot.active &&
        robot.hand_compliance.control_feedforward_ma==500);
    run_start();fake_diag.hand_control_negative_ack++;tick();
    assert(follow.phase==SG_FOLLOW && follow.error==SG_OK && robot.active);
    reset();fake_diag.can_bus_state=1;for(unsigned i=0;i<200;i++)tick();
    assert(!robot.active && !robot.hand_mode_request && local_home.source!=4);
    run_start();local_home.phase=HOME_WAIT;robot.active=0;tick();assert(follow.error==SG_HOME && soft_stops==1);
    run_start();fake_now+=50000;tick();assert(follow.error==SG_SCHEDULER && !robot.active);
    reset();for(unsigned i=0;i<300;i++)tick();assert(follow.error==SG_HAND && !robot.active); /* mode timeout */
    puts("Standalone integration: mode ACK, two-board gates, green truth, hand rearm, unlatched hold/catch-up, local fault stop passed");
    return 0;
}
