#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "gello_action.h"
#define GW_SIZE 224u
#define GW_STATE 1u
#define GW_COMMAND 2u
#define GW_PEER_QUERY 3u
#define GW_PEER_REPLY 4u
#define GW_PAUSE 1u
#define GW_RESUME 2u
#define GW_HAND 3u
#define GW_GELLO_HAND 4u
#define GW_KEEPALIVE 5u
#define GW_ENTER 6u
#define GW_LEAVE 7u
#define GW_PROFILE 8u
#define GW_TEACH 9u
#define GW_HOLD 10u
#define GW_TOPOLOGY 12u
#define GW_TEACHING 4u
#define GW_HAND_POSITION_WAIT 8u
#define GW_RUN_MODE 0u
#define GW_ENTER_MODE 1u
#define GW_SETTINGS_MODE 2u
#define GW_EXIT_MODE 3u
#define GW_SETTINGS_ENTERED 1u
#define GW_PROFILE_READY 2u
#define GW_OK 1u
#define GW_BAD 2u
#define GW_BLOCKED 3u
#define GW_STALE 4u
#define GW_PAUSED 1u
#define GW_MANUAL 2u
#define GW_ACTUATE 4u
#define GW_OWNED 8u
#define GW_ARM_OK 16u
#define GW_HAND_OK 32u
#define GW_INPUT_OK 64u
#define GW_LEASE_EXPIRED 128u
struct gw_board {
    uint64_t boot, ack_session;
    uint32_t ack_sequence, revision, profile_crc;
    uint8_t ack_status, phase, error, flags;
    int32_t arm_mdeg[6];
    int16_t hand_cdeg[6];
    uint16_t current_ma;
    uint8_t home_phase, settings;
    uint32_t ready_flags, age_ms;
};
struct gw_packet {
    uint8_t kind, node, op, mode;
    uint64_t session, expected_boot;
    uint32_t sequence, revision, exchange;
    struct ga_action action;
    struct gw_board board[2];
};
struct gw_parser { uint8_t bytes[GW_SIZE]; size_t used; };
void gw_encode(const struct gw_packet *p,uint8_t out[GW_SIZE]);
bool gw_decode(const uint8_t *p,size_t n,struct gw_packet *out);
bool gw_parse(struct gw_parser *p,uint8_t byte,struct gw_packet *out);
bool gw_command_valid(const struct gw_packet *p);

uint32_t gw_profile_crc(const uint16_t fingers[6],uint16_t current);

/* GW03 fractions use 90-degree endpoints for the four non-thumb channels. */
void gw_migrate_legacy_profile(uint16_t fingers[6]);
