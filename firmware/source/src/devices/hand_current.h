#pragma once
#include "robot_messages.h"
#include <stddef.h>
#define HAND_CURRENT_MAX_MA 2000u
#define HAND_CURRENT_FRESH_US 100000u
bool hand_compliance_valid(const hand_compliance_command_value_t *v, uint32_t mode);
bool hand_current_fresh(bool dynamic, bool active, uint32_t received, uint32_t mode,
                        uint64_t now, uint64_t updated);
bool hand_current_boosted(const uint32_t currents[6], uint32_t common,
                          const configure_request_value_t *baseline);
/* Actual hand CMD09 payload, shared by live CAN and offline packet checks. */
size_t hand_control_payload(uint8_t payload[28], const hand_command_value_t *hand,
                            const configure_request_value_t *cfg);
