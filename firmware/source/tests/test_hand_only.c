#include <assert.h>
#include <stdio.h>
#include "devices/hand_only.h"
int main(void) {
    for(unsigned id=0;id<0x20000;id++) {
        bool hand=(id>>8)==((ROBOT_HAND_HOST_ID<<8)|ROBOT_HAND_ID);
        assert(hand_only_tx_allowed(true,id)==hand);
        assert(hand_only_tx_allowed(false,id));
    }
    for(unsigned i=0;i<6;i++) assert(!hand_only_tx_allowed(true,0x8000|robot_motor_id(i)));
    puts("PASS hand-only final CAN egress allowlist: all 131072 IDs checked");
}
