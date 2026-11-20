#include "hand_startup.h"
#include <string.h>

void hand_startup_init(struct hand_startup *s, uint64_t now) {
    memset(s, 0, sizeof(*s));
    s->started = now;
}
bool hand_startup_quiet(uint64_t started, uint64_t now) {
    return now < started || now - started < HAND_STARTUP_QUIET_US;
}
bool hand_startup_update(struct hand_startup *s, uint64_t now,
                         uint64_t seen, bool valid) {
    if (s->ready) return true;
    if (!valid || !seen || now < seen || hand_startup_quiet(s->started, seen)
            || now - seen >= HAND_STARTUP_FRESH_US) {
        s->first = s->last = 0;
        s->samples = 0;
        return false;
    }
    if (seen == s->last) return false; /* Cached telemetry is not a new reply. */
    if (!s->samples || seen < s->last || seen - s->last >= HAND_STARTUP_FRESH_US) {
        s->first = seen;
        s->samples = 0;
    }
    s->last = seen;
    s->samples++;
    s->ready = s->samples >= 3 && seen - s->first >= HAND_STARTUP_STABLE_US;
    return s->ready;
}
