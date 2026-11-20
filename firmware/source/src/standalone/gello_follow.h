#pragma once
#include <stdbool.h>
#include <stdint.h>

/* SG_CATCHUP is appended so existing phase numbers in logs stay stable. */
enum sg_phase { SG_WAIT, SG_ZERO, SG_READY, SG_FOLLOW, SG_HOLD, SG_FAULT, SG_CATCHUP=6 };
enum sg_error { SG_OK, SG_INPUT_LOST, SG_SOURCE_REBOOT, SG_FEEDBACK, SG_PEER, SG_HOME, SG_HAND, SG_SCHEDULER };
struct sg_config {
    unsigned group;
    float factor[6], kp[6], kd[6];
    float grip_min, grip_max;
    bool grip_inverted;
    float hand_scale[6], hand_offset_deg[6], close_at, base_ma, max_ma, slew_ma_s;
};
struct sg_input {
    uint64_t now, source_boot;
    uint32_t scan;
    bool fresh, can_start, pause;
    enum sg_error fatal;
    int32_t ticks[7];
    float actual[6], velocity[6], home_zero[6];
};
struct sg_follow {
    enum sg_phase phase;
    enum sg_error error;
    uint64_t source_boot, stable_at, loss_at, aligned_at;
    uint64_t catch_at, step_at;
    uint32_t last_scan, count;
    int32_t minimum[6], maximum[6];
    double sum[6];
    float zero[6], target[6];
    bool zero_valid;
    /* Pose mismatch is diagnostic; startup and reconnection use bounded catch-up. */
    bool aligned;
    unsigned alignment_axis; /* 1..6, largest absolute position error */
    float alignment_error_rad;
    uint64_t hold_at; /* Entry into this HOLD; loss_at instead tracks uninterrupted stale input. */
    /* Target trajectory used while catching up after HOLD. */
    float catch_velocity[6];
};
void sg_step(struct sg_follow *s,const struct sg_config *cfg,const struct sg_input *in);
float sg_grasp(const struct sg_config *cfg,int32_t ticks);
void sg_hand(const struct sg_config *cfg,float percent,float elapsed,float *current,float target[6]);
