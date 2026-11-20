#pragma once
#include "gello_codec.h"

/* GE01: explicit little endian, CRC32/IEEE, <=250 bytes over ESP-NOW.
 * A reply echoes the requesting NXP's boot + sequence. No clock sync required:
 * charge the ENTIRE request RTT to every sample's age. Never retransmit a
 * cached reply under a new nonce. UART uses the same self-framing packets. */
#define GR_QUERY_SIZE 36u
#define GR_SAMPLE_SIZE 224u
#define GR_QUERY 1u
#define GR_SAMPLE 2u
#define GR_MAX_RTT_US 20000u
#define GR_STALE_US 50000u
struct gr_frame {
    uint8_t kind;
    uint64_t session, source_boot;
    uint32_t sequence, scan;
    struct gello_wire_sample samples[GELLO_SERVO_COUNT];
    uint32_t counters[5];
};
struct gr_parser { uint8_t bytes[GR_SAMPLE_SIZE]; size_t used; uint32_t bad; };
uint32_t gr_crc(const uint8_t *data, size_t n);
size_t gr_encode(const struct gr_frame *f, uint8_t out[GR_SAMPLE_SIZE]);
bool gr_decode(const uint8_t *data, size_t n, struct gr_frame *f);
bool gr_parse(struct gr_parser *p, uint8_t byte, struct gr_frame *f);
bool gr_matches(const struct gr_frame *f, uint64_t session, uint32_t sequence,
                uint64_t sent, uint64_t now);
void gr_add_age(struct gr_frame *f, uint64_t elapsed);
/* A reader can be preempted after recording caller_now and before copying a
 * newer frame. Use the actual time after the copy; never reduce sample age. */
bool gr_age_snapshot(struct gr_frame *f, uint64_t received_at,
                     uint64_t caller_now, uint64_t copied_at);
