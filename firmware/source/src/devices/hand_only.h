#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "can_codec.h"
/* Final CAN egress gate, independent of host commands or homing state. */
static inline bool hand_only_tx_allowed(bool hand_only, uint32_t id) {
    return !hand_only || (id >> 8) == ((ROBOT_HAND_HOST_ID << 8) | ROBOT_HAND_ID);
}
