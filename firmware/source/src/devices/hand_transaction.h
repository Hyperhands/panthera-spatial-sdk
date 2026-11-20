#pragma once
#include <stdbool.h>
#include <stdint.h>

/* One outstanding request across ALL hand commands, including mode-3 control.
 * Retain the existing 5 ms reply deadline; add 1 ms spacing after completion
 * or timeout. This does not alter arm scheduling or relax feedback timeouts. */
struct hand_transaction {
    bool pending;
    uint8_t command,index;
    uint64_t sent,ready_after;
};
bool hand_transaction_ready(const struct hand_transaction *s,uint64_t now);
void hand_transaction_sent(struct hand_transaction *s,uint8_t command,uint8_t index,uint64_t now);
bool hand_transaction_reply(struct hand_transaction *s,uint8_t command,uint8_t index,uint64_t now);
bool hand_transaction_expire(struct hand_transaction *s,uint64_t now);
void hand_transaction_reset(struct hand_transaction *s,uint64_t now);
/* Failed enqueue is not an outstanding transaction. Rate-limit another try;
 * this cannot cancel a frame already owned/retransmitted by the CAN hardware. */
void hand_transaction_enqueue_failed(struct hand_transaction *s,uint64_t now,uint32_t retry_us);
