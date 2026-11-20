#pragma once
#include <stdbool.h>
#include <stdint.h>
#define GA_LINEAR 0u
#define GA_TWO_FINGER 1u
#define GA_FIVE_FINGER 2u
#define GA_ACTION_SIZE 28u
struct ga_action {
    uint16_t start[6],end[6],current_ma;
    uint8_t curve,close_at;
};
extern const uint16_t ga_limits[6];
bool ga_valid(const struct ga_action *a);
void ga_encode(const struct ga_action *a,uint8_t out[GA_ACTION_SIZE]);
bool ga_decode(const uint8_t in[GA_ACTION_SIZE],struct ga_action *a);
void ga_sample(const struct ga_action *a,float percent,float out_deg[6],float *current);
void ga_from_legacy(const uint16_t fractions[6],uint16_t current,struct ga_action *a);
void ga_builtin(unsigned preset,struct ga_action *a);
uint32_t ga_crc(const struct ga_action *a);
