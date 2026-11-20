/* Included by the standalone owner. All state below is under robot_lock;
 * only the control tick mutates targets, never the USB or HTTP task. */
static struct gw_packet web_pending;
static bool web_pending_valid,web_paused,web_manual,web_hand_live;
static uint64_t web_received,web_lease_until,web_published;
static struct ga_action web_trial,web_profile;
static bool web_trial_prepared,web_trial_waiting;
static float web_trial_percent;
static bool web_settings,web_profile_ready,web_teach;
/* Read by SWD diagnostics; volatile keeps the mirror visible in the ELF. */
static volatile float web_stiffness=1;
bool gello_web_home_inhibited(void) { return web_paused; }
static struct gw_board web_state;
void gello_web_submit(const struct gw_packet *p) {
    if(!gw_command_valid(p) || !p->op || p->node!=CONFIG_ROBOT_NODE_ID)return;
    k_spinlock_key_t key=k_spin_lock(&robot_lock);
    if(!web_pending_valid && !(web_state.ack_session==p->session &&
        (int32_t)(p->sequence-web_state.ack_sequence)<=0)) {
        web_pending=*p;web_received=robot_us();web_pending_valid=true;
    }
    k_spin_unlock(&robot_lock,key);
}
void gello_web_snapshot(struct gw_board *out,uint64_t now) {
    k_spinlock_key_t key=k_spin_lock(&robot_lock);
    *out=web_state;uint64_t at=web_published;
    k_spin_unlock(&robot_lock,key);
    uint64_t copied=robot_us();if(copied>now)now=copied;
    uint64_t age=at && now>=at?(now-at)/1000:UINT32_MAX;
    out->age_ms=age>UINT32_MAX?UINT32_MAX:(uint32_t)age;
}
static void web_reset(void) {
    memset(&web_state,0,sizeof(web_state));web_pending_valid=web_paused=web_manual=web_hand_live=false;
    web_received=web_lease_until=web_published=0;
    web_settings=web_profile_ready=web_teach=false;web_stiffness=1;
    web_trial_prepared=web_trial_waiting=false;web_trial_percent=0;
    uint16_t legacy[6]={1000,1000,1000,1000,1000,1000};
    gw_migrate_legacy_profile(legacy);ga_from_legacy(legacy,(uint16_t)cfg->max_ma,&web_profile);
}
static void web_apply(uint64_t now,bool safe,bool resume_ok) {
    if(!web_pending_valid)return;
    const struct gw_packet *p=&web_pending;web_pending_valid=false;
    uint8_t result=GW_OK;
    if(p->expected_boot!=robot.boot || p->revision!=web_state.revision || now<web_received || now-web_received>300000)result=GW_STALE;
    else if(p->op==GW_ENTER) {
        bool held=(owned && local_home.phase==HOME_IDLE) || local_home.phase==HOME_HOLD || local_home.phase==HOME_HAND_WAIT ||
            local_home.phase==HOME_FAULT || local_home.phase==HOME_SOFT_HOLD || local_home.phase==HOME_STOPPED ||
            (local_home.phase==HOME_IDLE && !robot.active && !local_home.output);
        if(robot.lease || !held || p->mode!=GW_ENTER_MODE)result=GW_BLOCKED;
        else {web_settings=true;web_paused=true;}
    } else if(p->op==GW_LEAVE) {
        if(p->mode!=GW_EXIT_MODE || !web_settings)result=GW_BLOCKED;
        else {
            if(web_hand_live)hold_hand();
            web_teach=false;web_settings=false;web_manual=web_hand_live=false;web_paused=true;hand_rearmed=false;
        }
    } else if(p->op==GW_PROFILE) {
        if(robot.lease || !((web_settings && p->mode==GW_SETTINGS_MODE) ||
                           (!owned && !robot.active && !web_profile_ready && p->mode==GW_RUN_MODE)))result=GW_BLOCKED;
        else if(p->action.current_ma<cfg->base_ma || p->action.current_ma>cfg->max_ma)result=GW_BAD;
        else {
            web_profile=p->action;
            web_profile_ready=true;
        }
    } else if(p->op==GW_TOPOLOGY) {
        /* Topology value rides in board[1].profile_crc: a command field the
         * peer relay preserves untouched. 0=dual, 1=node 11 only, 2=node 12
         * only (the web console's right/left hand card respectively). */
        if(p->board[1].profile_crc>2)result=GW_BAD;
        else topology=p->board[1].profile_crc;
    } else if(p->op==GW_RESUME) {
        bool startup=!owned && !robot.active && (local_home.phase==HOME_HAND_WAIT || local_home.phase==HOME_HOLD);
        if(p->mode!=GW_EXIT_MODE || web_settings || !web_paused || robot.lease ||
           !(resume_ok || startup || follow.phase==SG_FAULT))result=GW_BLOCKED;
        else {web_paused=false;web_manual=web_hand_live=false;hand_rearmed=false;}
    } else if(!IS_ENABLED(CONFIG_ROBOT_GELLO_STANDALONE_ACTUATE) || follow.phase==SG_FAULT)result=GW_BLOCKED;
    else if(p->op==GW_PAUSE)web_paused=true;
    else if(p->op==GW_GELLO_HAND) {
        if(!web_settings || p->mode!=GW_SETTINGS_MODE || !web_paused || !safe)result=GW_BLOCKED;
        else {if(web_hand_live)hold_hand();web_manual=web_hand_live=false;hand_rearmed=false;}
    } else if(p->op==GW_TEACH || p->op==GW_HOLD) {
        if(!web_settings || p->mode!=GW_SETTINGS_MODE || !web_paused || !safe)result=GW_BLOCKED;
        else {web_teach=p->op==GW_TEACH;if(web_hand_live)hold_hand();web_manual=web_hand_live=false;hand_rearmed=false;}
    } else if(p->op==GW_HAND) {
        if(!web_settings || p->mode!=GW_SETTINGS_MODE || !web_paused || !safe)result=GW_BLOCKED;
        else if(p->action.current_ma<cfg->base_ma || p->action.current_ma>cfg->max_ma)result=GW_BAD;
        else {
            /* Apply also selects the action for the next GELLO takeover.
             * Named persistence remains a separate ESP32 bank operation. */
            web_profile=p->action;web_profile_ready=true;
            web_trial=p->action;
            hold_hand();web_trial_prepared=web_trial_waiting=false;web_trial_percent=0;
            web_manual=web_hand_live=true;web_lease_until=now+1500000;
        }
    } else if(p->op==GW_KEEPALIVE) {
        if(!web_settings || p->mode!=GW_SETTINGS_MODE || !web_manual || !web_hand_live ||
           !web_paused || !safe || now>=web_lease_until)result=GW_BLOCKED;
        else web_lease_until=now+1500000;
    } else result=GW_BAD;
    /* Heartbeats don't invalidate a user's in-flight setting revision. */
    if(result==GW_OK && p->op!=GW_KEEPALIVE)web_state.revision++;
    web_state.ack_session=p->session;web_state.ack_sequence=p->sequence;web_state.ack_status=result;
}
static void web_hand_tick(uint64_t now,float elapsed,bool safe) {
    if(!web_manual)return;
    if(!safe || !web_paused || now>=web_lease_until) {
        if(web_hand_live)hold_hand();
        web_hand_live=false;hand_current=cfg->base_ma;return;
    }
    if(!web_hand_live)return;

    const float rad=3.14159265358979323846f/180.f,step=30.f*elapsed;
    if(!web_trial_prepared) {
        /* Begin at this action's open pose, even when the previous trial was
         * closed. Preparing a fixed opposed thumb precedes linear finger bend. */
        bool reached=true;
        for(unsigned i=0;i<6;i++) {
            float desired=web_trial.start[i]/100.f*rad;
            float delta=desired-robot.hand.target_q_rad[i];
            robot.hand.target_q_rad[i]+=fminf(step*rad,fmaxf(-step*rad,delta));
            if(fabsf(desired-robot.hand.target_q_rad[i])>.0001f ||
               fabsf(desired-robot.hand_feedback.joint_q_rad[hand_active_feedback_index[i]])>2.f*rad)reached=false;
        }
        web_trial_prepared=reached;
        web_trial_waiting=!reached;
    } else {
        /* Reuse the GELLO curve, stopping at its segment boundaries. Advance
         * no faster than 30 deg/s on any channel, regardless of curve slopes. */
        float boundary=100,now_deg[6],next_deg[6],unused;
        if(web_trial.curve!=GA_LINEAR)boundary=web_trial_percent<20?20:web_trial_percent<95?95:100;
        ga_sample(&web_trial,web_trial_percent*web_trial.close_at/100.f,now_deg,&unused);
        ga_sample(&web_trial,boundary*web_trial.close_at/100.f,next_deg,&unused);
        bool thumb_ready=web_trial.curve==GA_LINEAR || web_trial_percent<20 ||
            fabsf(web_trial.end[0]/100.f*rad-robot.hand_feedback.joint_q_rad[hand_active_feedback_index[0]])<=2.f*rad;
        web_trial_waiting=!thumb_ready;
        if(thumb_ready) {
            float max_delta=0;
            for(unsigned i=0;i<6;i++)max_delta=fmaxf(max_delta,fabsf(next_deg[i]-now_deg[i]));
            web_trial_percent+= (boundary-web_trial_percent)*(max_delta<=step?1.f:step/max_delta);
        }
        ga_sample(&web_trial,web_trial_percent*web_trial.close_at/100.f,next_deg,&unused);
        for(unsigned i=0;i<6;i++)robot.hand.target_q_rad[i]=next_deg[i]*rad;
    }
    hand_current=web_trial.current_ma<=hand_current?web_trial.current_ma:fminf(web_trial.current_ma,hand_current+cfg->slew_ma_s*elapsed);
}
static void web_publish(uint64_t now,const struct sg_input *input,const struct home_input *actual,
                        bool arm_ok,bool hand_ok,unsigned ready_flags) {
    web_state.boot=robot.boot;web_state.phase=follow.phase;web_state.error=follow.error;
    web_state.flags=(web_paused?GW_PAUSED:0)|(web_manual?GW_MANUAL:0)|
        (IS_ENABLED(CONFIG_ROBOT_GELLO_STANDALONE_ACTUATE)?GW_ACTUATE:0)|(owned?GW_OWNED:0)|
        (arm_ok?GW_ARM_OK:0)|(hand_ok?GW_HAND_OK:0)|(input->fresh?GW_INPUT_OK:0)|
        (web_manual&&!web_hand_live?GW_LEASE_EXPIRED:0);
    web_state.ready_flags=ready_flags;
    web_state.profile_crc=ga_crc(&web_profile);
    web_state.home_phase=local_home.phase;
    web_state.settings=(web_settings?GW_SETTINGS_ENTERED:0)|(web_profile_ready?GW_PROFILE_READY:0)|(web_teach?GW_TEACHING:0)|
        (web_hand_live&&web_trial_waiting?GW_HAND_POSITION_WAIT:0);
    for(unsigned i=0;i<6;i++) {
        float a=actual->q[i]*57295.7795f,h=robot.hand_feedback.joint_q_rad[hand_active_feedback_index[i]]*5729.57795f;
        web_state.arm_mdeg[i]=isfinite(a)?(int32_t)lroundf(fmaxf(-2000000000.f,fminf(2000000000.f,a))):0;
        web_state.hand_cdeg[i]=isfinite(h)?(int16_t)lroundf(fmaxf(-32768,fminf(32767,h))):0;
    }
    web_state.current_ma=(uint16_t)lroundf(hand_current);web_published=now;
}

static void web_gello_hand(float grasp,float elapsed) {
    float degrees[6],current;
    ga_sample(&web_profile,grasp,degrees,&current);
    hand_current=current<=hand_current?current:fminf(current,hand_current+cfg->slew_ma_s*elapsed);
    for(unsigned i=0;i<6;i++)robot.hand.target_q_rad[i]=degrees[i]*(3.14159265358979323846f/180.f);
}
