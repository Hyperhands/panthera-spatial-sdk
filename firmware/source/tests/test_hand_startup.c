#include "devices/hand_startup.h"
#include "homing/homing.h"
#include <assert.h>
#include <stdio.h>

int main(void) {
    struct hand_startup s;
    hand_startup_init(&s,100);
    assert(hand_startup_quiet(100,99));
    assert(hand_startup_quiet(100,1000099));
    assert(!hand_startup_quiet(100,1000100));
    assert(!hand_startup_update(&s,1000100,0,false));
    assert(!hand_startup_update(&s,1000100,999999,true));
    assert(!hand_startup_update(&s,1000200,1000100,true));
    for(unsigned i=0;i<1000;i++)
        assert(!hand_startup_update(&s,1000200,1000100,true));
    assert(s.samples==1); /* Re-reading one packet never opens the gate. */
    assert(!hand_startup_update(&s,1200200,1200100,true));
    assert(hand_startup_update(&s,1400200,1400100,true));
    /* A completed boot gate never removes runtime arm output on hand loss. */
    assert(hand_startup_update(&s,9000000,0,false));
    hand_startup_init(&s,0);
    assert(!hand_startup_update(&s,1000000,1000000,true));
    assert(!hand_startup_update(&s,1250000,1000000,true));
    assert(s.samples==0); /* Stale feedback breaks continuity. */
    assert(!hand_startup_update(&s,1600000,1600000,true));
    assert(!hand_startup_update(&s,2000000,2000000,true));
    assert(s.samples==1); /* A missing 5 Hz reply starts a new window. */
    assert(!hand_startup_update(&s,2200000,2200000,true));
    assert(hand_startup_update(&s,2400000,2400000,true));
    hand_startup_init(&s,3000000);
    assert(!hand_startup_update(&s,4000000,4000001,true));
    assert(!hand_startup_update(&s,4000000,4000000,false));
    assert(!s.samples && !s.ready);
    /* Phase 17 cannot produce motion even with otherwise perfect feedback. */
    struct home_context h={.phase=HOME_HAND_WAIT};
    struct home_input in={.q_mask=63,.dq_mask=63,.fault_mask=63};
    home_tick(&h,&in,5000000);
    assert(h.phase==HOME_HAND_WAIT && !h.output);
    assert(!home_host_restartable(&h,&in));
    home_host_stop(&h);
    assert(home_stop_required(&h,5000000,5000000));
    home_abort(&h,HOME_MOTOR_FAULT);
    assert(home_stop_required(&h,5000000,5000000));
    puts("PASS boot quiet interval, distinct fresh replies, stale reset, one-shot gate and STOP priority");
}
