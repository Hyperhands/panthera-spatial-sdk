#include "devices/hand_transaction.h"
#include <assert.h>
#include <stdio.h>

int main(void) {
    struct hand_transaction s={0};
    assert(hand_transaction_ready(&s,0));
    hand_transaction_sent(&s,10,3,1000);
    assert(!hand_transaction_ready(&s,1000)); /* Control cannot overlap query. */
    assert(!hand_transaction_reply(&s,9,3,1100));
    assert(!hand_transaction_reply(&s,10,2,1100));
    assert(!hand_transaction_reply(&s,10,3,999));
    assert(hand_transaction_reply(&s,10,3,1200));
    assert(!hand_transaction_ready(&s,2199));
    assert(hand_transaction_ready(&s,2200));
    hand_transaction_sent(&s,9,3,2200);
    assert(!hand_transaction_ready(&s,2300)); /* Query cannot overlap control. */
    assert(!hand_transaction_reply(&s,10,3,2300));
    assert(!hand_transaction_expire(&s,2000));
    assert(!hand_transaction_expire(&s,7199));
    assert(hand_transaction_expire(&s,7200));
    assert(!hand_transaction_expire(&s,7201));
    assert(!hand_transaction_reply(&s,9,3,7300));
    assert(!hand_transaction_ready(&s,8199));
    assert(hand_transaction_ready(&s,8200));
    hand_transaction_sent(&s,3,7,8200);
    assert(!hand_transaction_reply(&s,9,3,8300));
    assert(hand_transaction_reply(&s,3,7,8400));
    hand_transaction_reset(&s,8500);
    assert(!s.pending && !hand_transaction_ready(&s,9499));
    assert(hand_transaction_ready(&s,9500));
    /* Sustained 50 Hz query + 50 Hz control with 100 us responses, no overlap. */
    unsigned queries=0,controls=0;
    for(uint64_t period=20000;period<10000000;period+=20000) {
        assert(hand_transaction_ready(&s,period));
        hand_transaction_sent(&s,10,3,period);queries++;
        assert(!hand_transaction_ready(&s,period+1));
        assert(hand_transaction_reply(&s,10,3,period+100));
        assert(hand_transaction_ready(&s,period+1100));
        hand_transaction_sent(&s,9,3,period+1100);controls++;
        assert(hand_transaction_reply(&s,9,3,period+1200));
    }
    assert(queries==499 && controls==499);
    /* Power-off/full-mailbox regression: the scheduler can call every 100 us,
     * but failed 5 Hz queries must not become hundreds of submissions/sec. */
    s=(struct hand_transaction){0};unsigned attempts=0;
    for(uint64_t now=0;now<2000000;now+=100) {
        if(!hand_transaction_ready(&s,now))continue;
        attempts++;hand_transaction_enqueue_failed(&s,now,200000);
        assert(!s.pending);
    }
    assert(attempts==10);
    assert(hand_transaction_ready(&s,2000000));
    hand_transaction_sent(&s,10,3,2000000);
    hand_transaction_enqueue_failed(&s,2000100,200000); /* Must not clear pending. */
    assert(s.pending);
    assert(hand_transaction_reply(&s,10,3,2001100));
    assert(hand_transaction_ready(&s,2002100));
    hand_transaction_enqueue_failed(&s,2002100,0);
    assert(!hand_transaction_ready(&s,2003099));
    assert(hand_transaction_ready(&s,2003100));
    hand_transaction_enqueue_failed(&s,2003100,200000);
    hand_transaction_enqueue_failed(&s,2003200,1000); /* Cannot shorten backoff. */
    assert(!hand_transaction_ready(&s,2203099));
    assert(hand_transaction_ready(&s,2203100));
    puts("PASS hand serialization, timeout, 50 Hz schedule and failed-enqueue backoff");
}
