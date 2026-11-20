#include "hand_transaction.h"
#include <string.h>

bool hand_transaction_ready(const struct hand_transaction *s,uint64_t now) {
    return !s->pending && now>=s->ready_after;
}
void hand_transaction_sent(struct hand_transaction *s,uint8_t command,uint8_t index,uint64_t now) {
    s->pending=true;s->command=command;s->index=index;s->sent=now;
}
bool hand_transaction_reply(struct hand_transaction *s,uint8_t command,uint8_t index,uint64_t now) {
    if(!s->pending || command!=s->command || index!=s->index || now<s->sent)return false;
    s->pending=false;s->ready_after=now+1000;return true;
}
bool hand_transaction_expire(struct hand_transaction *s,uint64_t now) {
    if(!s->pending || now<s->sent || now-s->sent<5000)return false;
    s->pending=false;s->ready_after=now+1000;return true;
}
void hand_transaction_reset(struct hand_transaction *s,uint64_t now) {
    memset(s,0,sizeof(*s));s->ready_after=now+1000;
}
void hand_transaction_enqueue_failed(struct hand_transaction *s,uint64_t now,uint32_t retry_us) {
    if(s->pending) return;
    if(retry_us<1000) retry_us=1000;
    uint64_t after=now+retry_us;
    if(after>s->ready_after) s->ready_after=after;
}
