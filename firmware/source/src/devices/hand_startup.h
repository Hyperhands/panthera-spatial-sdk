#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Boot-only sequencing. Never re-enter this gate after arm control starts. */
#define HAND_STARTUP_QUIET_US 1000000u
#define HAND_STARTUP_POLL_US 200000u
#define HAND_STARTUP_FRESH_US 250000u
#define HAND_STARTUP_STABLE_US 400000u
struct hand_startup {
    uint64_t started, first, last;
    unsigned samples;
    bool ready;
};
void hand_startup_init(struct hand_startup *s, uint64_t now);
bool hand_startup_quiet(uint64_t started, uint64_t now);
bool hand_startup_update(struct hand_startup *s, uint64_t now,
                         uint64_t angle_seen, bool valid);
