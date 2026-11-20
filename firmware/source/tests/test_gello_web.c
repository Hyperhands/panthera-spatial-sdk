#define main original_integration_tests
#include "test_gello_standalone.c"
#undef main
static uint32_t test_seq;
static struct gw_packet cmd(unsigned op) {
    return (struct gw_packet){.kind=GW_COMMAND,.node=CONFIG_ROBOT_NODE_ID,.op=op,.mode=op==GW_ENTER?GW_ENTER_MODE:op==GW_LEAVE||op==GW_RESUME?GW_EXIT_MODE:GW_SETTINGS_MODE,.session=700,
        .sequence=++test_seq,.expected_boot=robot.boot,.revision=web_state.revision,
        .action={.end={7500,2000,9000,9000,9000,9000},.current_ma=1500,.curve=GA_FIVE_FINGER,.close_at=100}};
}
static void apply(unsigned op) {if(op==GW_HAND)hand_follows_target=true;struct gw_packet p=cmd(op);gello_web_submit(&p);tick();}
static void check_preview_sequence(struct ga_action action) {
    run_start();apply(GW_ENTER);
    for(unsigned j=0;j<6;j++)robot.hand_feedback.joint_q_rad[hand_active_feedback_index[j]]=action.start[j]*.0001745329252f;
    struct gw_packet p=cmd(GW_HAND);p.action=action;gello_web_submit(&p);tick();
    assert(web_hand_live && web_trial_prepared);
    for(unsigned k=0;k<800;k++) {
        if(k%100==0){struct gw_packet heartbeat=cmd(GW_KEEPALIVE);gello_web_submit(&heartbeat);}tick();
        for(unsigned j=1;j<6;j++)assert(fabsf(robot.hand.target_q_rad[j]-action.start[j]*.0001745329252f)<.00001f);
    }
    /* A commanded 75 degrees is not proof that the thumb reached it. */
    assert(fabsf(web_trial_percent-20)<.0001f);
    assert(web_state.settings&GW_HAND_POSITION_WAIT);
    hand_follows_target=true;
    float last[6];memcpy(last,robot.hand.target_q_rad,sizeof(last));
    for(unsigned k=0;k<800;k++) {
        if(k%100==0){struct gw_packet heartbeat=cmd(GW_KEEPALIVE);gello_web_submit(&heartbeat);}tick();
        for(unsigned j=0;j<6;j++) {
            assert(fabsf(robot.hand.target_q_rad[j]-last[j])<.002095f);
            last[j]=robot.hand.target_q_rad[j];
            if(action.start[j]==action.end[j])assert(fabsf(last[j]-action.start[j]*.0001745329252f)<.00001f);
        }
    }
    assert(web_trial_percent==100);
    for(unsigned j=0;j<6;j++)assert(fabsf(last[j]-action.end[j]*.0001745329252f)<.00001f);
    assert(!(web_state.settings&GW_HAND_POSITION_WAIT));
    p=cmd(GW_HAND);p.action=action;gello_web_submit(&p);tick();
    assert(!web_trial_prepared && web_trial_percent==0); /* reopen before replay */
    apply(GW_LEAVE);assert(!web_hand_live);
}
static void check_peer_restart(void) {
    const unsigned reasons[]={PI_LINK,PI_REVERSE,PI_DISABLED,PI_ARM,PI_HAND,PI_CAN,PI_STATE};
    for(unsigned i=0;i<sizeof(reasons)/sizeof(reasons[0]);i++) {
        run_start();apply(GW_ENTER);apply(GW_HAND);assert(web_hand_live);
        struct panthera_peer_state healthy=fake_peer;
        uint64_t old_boot=peer_boot;fake_peer.boot++;
        switch(reasons[i]) {
        case PI_LINK:peer_available=false;break;
        case PI_REVERSE:fake_peer.peer_boot++;break;
        case PI_DISABLED:fake_peer.actuators=0;break;
        case PI_ARM:fake_peer.arm_age_us[2]=19000;break;
        case PI_HAND:fake_peer.hand_age_us[1]=99000;break;
        case PI_CAN:fake_peer.can_state=1;break;
        case PI_STATE:fake_peer.home_state=HOME_WAIT;fake_peer.home_source=1;fake_peer.active=0;break;
        }
        tick();assert(peer_boot==old_boot && (web_state.ready_flags&RI_PEER));
        assert(!web_hand_live && hand_current==500);
        apply(GW_HAND);assert(web_state.ack_status==GW_BLOCKED && !web_hand_live);
        fake_peer=healthy;fake_peer.boot++;peer_available=true;tick();
        assert(peer_boot==fake_peer.boot && !(web_state.ready_flags&RI_PEER));
        assert(follow.phase==SG_HOLD && web_paused && !web_hand_live && !soft_stops);
        apply(GW_KEEPALIVE);assert(web_state.ack_status==GW_BLOCKED && !web_hand_live);
        apply(GW_HAND);assert(web_state.ack_status==GW_OK && web_hand_live);
    }
}
static void check_applied_profile(void) {
    run_start();apply(GW_ENTER);
    struct gw_packet p=cmd(GW_HAND);ga_builtin(2,&p.action);p.action.current_ma=900;
    for(unsigned j=3;j<6;j++)p.action.start[j]=p.action.end[j]=9000;
    hand_follows_target=true;gello_web_submit(&p);tick();
    assert(web_state.ack_status==GW_OK && ga_crc(&web_profile)==ga_crc(&p.action));
    struct gw_packet stale=cmd(GW_HAND);stale.revision++;gello_web_submit(&stale);tick();
    assert(web_state.ack_status==GW_STALE && ga_crc(&web_profile)==ga_crc(&p.action));
    apply(GW_LEAVE);apply(GW_RESUME);
    assert(!web_settings && !web_paused && !web_manual);
    web_gello_hand(100,1);
    for(unsigned j=0;j<6;j++)assert(fabsf(robot.hand.target_q_rad[j]-p.action.end[j]*.0001745329252f)<.00001f);
    assert(hand_current==900);
}
int main(void) {
    original_integration_tests();
    if(!IS_ENABLED(CONFIG_ROBOT_GELLO_STANDALONE_ACTUATE)) {
        reset();apply(GW_PAUSE);assert(web_state.ack_status==GW_BLOCKED && !web_paused);return 0;
    }
    check_peer_restart();
    check_applied_profile();
    struct ga_action preview;ga_builtin(2,&preview);check_preview_sequence(preview);
    for(unsigned j=3;j<6;j++)preview.start[j]=preview.end[j]=9000;
    check_preview_sequence(preview);
    ga_builtin(1,&preview);check_preview_sequence(preview);
    check_hand_feedback_mapping();tick();
    const unsigned active[6]={0,1,3,5,7,9};
    for(unsigned j=0;j<6;j++)assert(web_state.hand_cdeg[j]==lroundf(.025f*(active[j]+1)*5729.57795f));
    run_start();
    float legacy_target[6],legacy_current=cfg->base_ma;
    sg_hand(cfg,100,1,&legacy_current,legacy_target);web_gello_hand(100,1);
    for(unsigned j=0;j<6;j++)assert(fabsf(legacy_target[j]-robot.hand.target_q_rad[j])<.000873f);
    reset();web_profile_ready=false;
    for(unsigned i=0;i<150;i++)tick();
    assert(!owned && !robot.active && (web_state.ready_flags&512));
    struct gw_packet initial=cmd(GW_PROFILE);initial.mode=GW_RUN_MODE;gello_web_submit(&initial);tick();assert(web_profile_ready && web_state.ack_status==GW_OK);
    assert(web_state.profile_crc==ga_crc(&web_profile));
    local_home.phase=HOME_MOVE;apply(GW_ENTER);assert(web_state.ack_status==GW_BLOCKED && !web_settings);
    local_home.phase=HOME_HAND_WAIT;apply(GW_ENTER);assert(web_settings && gello_web_home_inhibited());
    apply(GW_LEAVE);assert(!web_settings && web_paused);apply(GW_RESUME);assert(!gello_web_home_inhibited());
    run_start();apply(GW_HAND);assert(web_state.ack_status==GW_BLOCKED && !web_manual);
    apply(GW_ENTER);assert(web_settings && web_paused && follow.phase==SG_HOLD && robot.active && !follow.loss_at);
    struct gw_board snap;gello_web_snapshot(&snap,fake_now-50000);assert(snap.age_ms==0);
    float held[6];memcpy(held,follow.target,sizeof(held));
    fake_input.samples[cfg->group*7].ticks+=100;
    for(unsigned i=0;i<800;i++){tick();}
    assert(follow.phase==SG_HOLD && !soft_stops && !follow.loss_at);
    for(unsigned i=0;i<6;i++)assert(follow.target[i]==held[i]);
    assert(web_stiffness<.00001f && robot.arm.mit[2]==0); /* settings HOLD is compliant too */
    struct gw_packet p=cmd(GW_HAND);hand_follows_target=true;gello_web_submit(&p);tick();
    assert(web_state.ack_status==GW_OK && web_hand_live && web_manual);
    uint32_t rev=web_state.revision;gello_web_submit(&p);tick();assert(web_state.revision==rev);
    float previous[6];memcpy(previous,robot.hand.target_q_rad,sizeof(previous));
    for(unsigned i=0;i<100;i++) {
        tick();
        for(unsigned j=0;j<6;j++) {
            assert(robot.hand.target_q_rad[j]>=previous[j]);
            assert(robot.hand.target_q_rad[j]-previous[j]<.002095f);
            previous[j]=robot.hand.target_q_rad[j];
        }
    }
    assert(hand_current==1500);
    /* Follow the live CAN configuration handoff through to CMD09 bytes. */
    configure_request_value_t wire_cfg=robot.config;uint8_t payload[28];
    memcpy(wire_cfg.hand_soft_current_ma,robot.hand_compliance.joint_current_ma,sizeof(wire_cfg.hand_soft_current_ma));
    wire_cfg.hand_control_feedforward_ma=robot.hand_compliance.control_feedforward_ma;
    assert(wire_cfg.hand_soft_current_ma[0]==500 && wire_cfg.hand_control_feedforward_ma==1500);
    assert(hand_control_payload(payload,&robot.hand,&wire_cfg)==14);
    assert(payload[0]==5 && payload[1]==220);
    apply(GW_KEEPALIVE);assert(web_state.ack_status==GW_OK && web_state.revision==rev);
    for(unsigned i=0;i<400;i++){tick();}
    assert(!web_hand_live && web_manual && web_paused && hand_current==500 && follow.phase==SG_HOLD);
    assert(web_state.flags&GW_LEASE_EXPIRED);
    apply(GW_KEEPALIVE);assert(web_state.ack_status==GW_BLOCKED && !web_hand_live);
    apply(GW_HAND);assert(web_hand_live);
    apply(GW_GELLO_HAND);assert(!web_manual && !hand_rearmed && web_paused);
    apply(GW_RESUME);assert(web_state.ack_status==GW_BLOCKED && web_paused);
    apply(GW_LEAVE);assert(!web_settings && web_paused);
    apply(GW_RESUME);assert(!web_paused && follow.phase==SG_CATCHUP);
    for(unsigned i=0;i<400;i++){tick();}
    assert(follow.phase==SG_FOLLOW && robot.active && !soft_stops); /* caught up automatically */
    /* A hand-only fault must not trap the arm in a user-requested pause. */
    run_start();apply(GW_PAUSE);assert(web_paused);
    robot.hand_feedback.field_age_us[1]=100000;tick();
    apply(GW_RESUME);assert(web_state.ack_status==GW_OK && !web_paused && robot.active);
    p=cmd(GW_PAUSE);p.expected_boot++;gello_web_submit(&p);tick();assert(!web_paused && web_state.ack_status==GW_STALE);
    p=cmd(GW_PAUSE);p.revision++;gello_web_submit(&p);tick();assert(!web_paused && web_state.ack_status==GW_STALE);
    p=cmd(GW_PAUSE);gello_web_submit(&p);web_received=fake_now-400000;tick();assert(!web_paused && web_state.ack_status==GW_STALE);
    apply(GW_PAUSE);radio_available=false;
    for(unsigned i=0;i<510;i++){tick();}
    assert(follow.phase==SG_HOLD && follow.error==SG_OK && robot.active && !soft_stops);
    apply(GW_RESUME);assert(web_state.ack_status==GW_BLOCKED && web_paused); /* no fresh input yet */
    radio_available=true;apply(GW_RESUME);assert(web_state.ack_status==GW_OK && !web_paused);
    for(unsigned i=0;i<400;i++){tick();}
    assert(follow.phase==SG_FOLLOW && robot.active && !soft_stops);
    run_start();apply(GW_ENTER);robot.lease=44;apply(GW_HAND);
    assert(web_state.ack_status==GW_BLOCKED && !web_manual);robot.lease=0;
    apply(GW_HAND);assert(web_hand_live);radio_available=false;tick();
    assert(!web_hand_live && hand_current==500 && follow.phase==SG_HOLD);
    radio_available=true;apply(GW_KEEPALIVE);assert(!web_hand_live && web_state.ack_status==GW_BLOCKED);
    /* Pausing before initial ownership must not create an unrecoverable pause. */
    reset();apply(GW_PAUSE);
    for(unsigned i=0;i<150;i++){tick();}robot.hand_mode_request=0;robot.hand_guard_phase=7;robot.config.hand_mode=3;
    fake_peer.home_source=4;
    for(unsigned i=0;i<150;i++){tick();}assert(!owned && follow.phase==SG_READY && web_paused);
    apply(GW_RESUME);assert(web_state.ack_status==GW_OK && !web_paused);
    for(unsigned i=0;i<150;i++){tick();}assert(owned && follow.phase==SG_FOLLOW);
    run_start();apply(GW_PROFILE);assert(web_state.ack_status==GW_BLOCKED);
    apply(GW_ENTER);p=cmd(GW_PROFILE);p.action.end[0]=1875;p.action.current_ma=750;gello_web_submit(&p);tick();
    assert(web_state.ack_status==GW_OK && web_profile.current_ma==750 && web_profile.end[0]==1875);
    float full[6],current=500;struct sg_config effective=*cfg;effective.max_ma=750;
    sg_hand(&effective,100,1,&current,full);web_gello_hand(100,1);
    float offset=cfg->hand_offset_deg[0]*.017453292519943295f;
    assert(fabsf(robot.hand.target_q_rad[0]-(offset+(full[0]-offset)*.25f))<1e-5f && hand_current<=750);
    for(unsigned j=2;j<6;j++)assert(fabsf(robot.hand.target_q_rad[j]-1.57079632679f)<1e-5f);
    /* Trial and saved GELLO profiles share the same complete four-finger range. */
    apply(GW_HAND);
    for(unsigned k=0;k<2500;k++){tick();if(k%100==0)apply(GW_KEEPALIVE);}
    for(unsigned j=2;j<6;j++)assert(fabsf(robot.hand.target_q_rad[j]-1.57079632679f)<1e-5f);
    p=cmd(GW_HAND);p.mode=GW_RUN_MODE;p.action.current_ma=700;gello_web_submit(&p);tick();assert(web_state.ack_status==GW_BLOCKED && web_profile.current_ma==1500);
    p=cmd(GW_PROFILE);p.action.current_ma=499;uint32_t seq=web_state.ack_sequence;gello_web_submit(&p);tick();assert(web_state.ack_sequence==seq && web_profile.current_ma==1500);
    run_start();apply(GW_TEACH);assert(web_state.ack_status==GW_BLOCKED && !web_teach);
    apply(GW_ENTER);apply(GW_TEACH);assert(web_teach && (web_state.settings&GW_TEACHING));
    for(unsigned k=0;k<260;k++){tick();}assert(web_stiffness<.00001f);
    robot.arm_feedback.q_rad[0]+=.1f;tick();assert(fabsf(follow.target[0]-robot.arm_feedback.q_rad[0])<.00001f);
    assert(robot.arm.mit[2]==0);apply(GW_HOLD);assert(!web_teach);
    /* "Hold" is compliant too: it follows a further drag and stays at zero gain. */
    robot.arm_feedback.q_rad[0]+=.1f;
    for(unsigned k=0;k<260;k++){tick();}
    assert(web_stiffness<.00001f && robot.arm.mit[2]==0);
    assert(fabsf(follow.target[0]-robot.arm_feedback.q_rad[0])<.00001f);
    apply(GW_TEACH);apply(GW_LEAVE);assert(!web_teach && web_paused);
    puts("Web: pause ownership, gated manual hand, speed/current limits, duplicate/stale commands, deadman, automatic catch-up and local fault latch passed");
}
