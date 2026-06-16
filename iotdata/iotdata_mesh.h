/* iotdata_mesh.h
 *
 * Mesh relay protocol definitions for iotdata.
 *
 * Variant 15 (0x0F) is reserved for mesh control packets. This header
 * defines the control types, packet structures, and helper functions
 * for packing/unpacking mesh headers.
 *
 * See: APPENDIX_MESH.md in the iotdata repository for the full protocol
 * specification including flows, state machines, and deployment guidance.
 *
 * Include this header in both gateway and hop node firmware. Sensors do
 * not need this header — they are mesh-unaware.
 */

#ifndef IOTDATA_MESH_H
#define IOTDATA_MESH_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* -------------------------------------------------------------------------
 * Constants
 * ----------------------------------------------------------------------- */

#define IOTDATA_MESH_VARIANT            0x0F

/* Control types (upper nibble of byte 4) */
#define IOTDATA_MESH_CTRL_BEACON        0x0
#define IOTDATA_MESH_CTRL_FORWARD       0x1
#define IOTDATA_MESH_CTRL_ACK           0x2
#define IOTDATA_MESH_CTRL_ROUTE_ERROR   0x3
#define IOTDATA_MESH_CTRL_NEIGHBOUR_RPT 0x4
#define IOTDATA_MESH_CTRL_PING          0x5 /* v2 */
#define IOTDATA_MESH_CTRL_PONG          0x6 /* v2 */

/* Route error reasons (lower nibble of byte 4) */
#define IOTDATA_MESH_REASON_PARENT_LOST 0x0
#define IOTDATA_MESH_REASON_OVERLOADED  0x1
#define IOTDATA_MESH_REASON_SHUTDOWN    0x2

/* Beacon flags */
#define IOTDATA_MESH_FLAG_ACCEPTING     0x01 /* gateway is accepting forwards */

/* Special values */
#define IOTDATA_MESH_PARENT_NONE        0xFFF /* orphaned — no parent */
#define IOTDATA_MESH_STATION_RESERVED   0x000 /* do not assign to nodes */

/* Protocol limits */
#define IOTDATA_MESH_TTL_DEFAULT        7
#define IOTDATA_MESH_TTL_MAX            255
#define IOTDATA_MESH_GENERATION_HALF    2048 /* for modular comparison */
#define IOTDATA_MESH_GENERATION_MOD     4096
#define IOTDATA_MESH_MAX_NEIGHBOURS     63

/* Packet sizes */
#define IOTDATA_MESH_BEACON_SIZE        9
#define IOTDATA_MESH_FORWARD_HDR_SIZE   6 /* + inner packet bytes */
#define IOTDATA_MESH_ACK_SIZE           8
#define IOTDATA_MESH_ROUTE_ERROR_SIZE   5
#define IOTDATA_MESH_NEIGHBOUR_HDR_SIZE 10 /* + 3 per entry */
#define IOTDATA_MESH_NEIGHBOUR_ENTRY_SZ 3
#define IOTDATA_MESH_PING_SIZE          8
#define IOTDATA_MESH_PONG_SIZE          8

/* Dedup ring default size */
#define IOTDATA_MESH_DEDUP_RING_SIZE    64

/* -------------------------------------------------------------------------
 * iotdata header peek — extract fields from the standard 4-byte header
 * ----------------------------------------------------------------------- */

static inline bool iotdata_mesh_peek_header(const uint8_t *buf, int len, uint8_t *variant, uint16_t *station_id, uint16_t *sequence) {
    if (len < 4)
        return false;
    *variant = (buf[0] >> 4) & 0x0F;
    *station_id = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];
    *sequence = ((uint16_t)buf[2] << 8) | buf[3];
    return true;
}

static inline uint8_t iotdata_mesh_peek_ctrl_type(const uint8_t *buf, int len) {
    if (len < 5)
        return 0xFF;
    return (buf[4] >> 4) & 0x0F;
}

/* -------------------------------------------------------------------------
 * 4+12 bit packing helper (used throughout the mesh protocol)
 * ----------------------------------------------------------------------- */

static inline void iotdata_mesh_pack_4_12(uint8_t *dst, uint8_t hi4, uint16_t lo12) {
    dst[0] = (uint8_t)((hi4 << 4) | ((lo12 >> 8) & 0x0F));
    dst[1] = (uint8_t)(lo12 & 0xFF);
}

static inline void iotdata_mesh_unpack_4_12(const uint8_t *src, uint8_t *hi4, uint16_t *lo12) {
    *hi4 = (src[0] >> 4) & 0x0F;
    *lo12 = ((uint16_t)(src[0] & 0x0F) << 8) | src[1];
}

/* -------------------------------------------------------------------------
 * Common mesh header (bytes 0–4): pack and unpack
 * ----------------------------------------------------------------------- */

static inline void iotdata_mesh_pack_header(uint8_t *buf, uint16_t sender_station, uint16_t sender_seq) {
    iotdata_mesh_pack_4_12(&buf[0], IOTDATA_MESH_VARIANT, sender_station);
    buf[2] = (uint8_t)(sender_seq >> 8);
    buf[3] = (uint8_t)(sender_seq & 0xFF);
    /* byte 4 left for caller to set ctrl_type + first payload nibble */
}

/* -------------------------------------------------------------------------
 * BEACON (ctrl_type 0x0) — 9 bytes
 *
 * byte 4-5: ctrl(4) | gateway_id(12)
 * byte 6:   cost(8)
 * byte 7:   flags(4) | generation[11:8](4)
 * byte 8:   generation[7:0](8)
 * ----------------------------------------------------------------------- */

typedef struct {
    uint16_t sender_station;
    uint16_t sender_seq;
    uint16_t gateway_id;
    uint8_t cost;
    uint8_t flags;
    uint16_t generation;
} iotdata_mesh_beacon_t;

static inline void iotdata_mesh_pack_beacon(uint8_t *buf, const iotdata_mesh_beacon_t *b) {
    iotdata_mesh_pack_header(buf, b->sender_station, b->sender_seq);
    iotdata_mesh_pack_4_12(&buf[4], IOTDATA_MESH_CTRL_BEACON, b->gateway_id);
    buf[6] = b->cost;
    buf[7] = (uint8_t)(((b->flags & 0x0F) << 4) | ((b->generation >> 8) & 0x0F));
    buf[8] = (uint8_t)(b->generation & 0xFF);
}

static inline bool iotdata_mesh_unpack_beacon(const uint8_t *buf, int len, iotdata_mesh_beacon_t *b) {
    if (len < IOTDATA_MESH_BEACON_SIZE)
        return false;
    uint8_t ctrl;
    iotdata_mesh_unpack_4_12(&buf[0], &ctrl, &b->sender_station);
    b->sender_seq = ((uint16_t)buf[2] << 8) | buf[3];
    iotdata_mesh_unpack_4_12(&buf[4], &ctrl, &b->gateway_id);
    b->cost = buf[6];
    b->flags = (buf[7] >> 4) & 0x0F;
    b->generation = ((uint16_t)(buf[7] & 0x0F) << 8) | buf[8];
    return true;
}

/* -------------------------------------------------------------------------
 * FORWARD (ctrl_type 0x1) — 6 + N bytes
 *
 * byte 4:   ctrl(4) | ttl[7:4](4)
 * byte 5:   ttl[3:0](4) | pad(4)
 * byte 6+:  inner_packet (byte-aligned)
 * ----------------------------------------------------------------------- */

typedef struct {
    uint16_t sender_station;
    uint16_t sender_seq;
    uint8_t ttl;
    const uint8_t *inner_packet; /* pointer into receive buffer, not owned */
    int inner_len;
    /* extracted from inner_packet header for convenience / dedup */
    uint16_t origin_station;
    uint16_t origin_sequence;
} iotdata_mesh_forward_t;

static inline void iotdata_mesh_pack_forward(uint8_t *buf, uint16_t sender_station, uint16_t sender_seq, uint8_t ttl, const uint8_t *inner, int inner_len) {
    iotdata_mesh_pack_header(buf, sender_station, sender_seq);
    buf[4] = (uint8_t)((IOTDATA_MESH_CTRL_FORWARD << 4) | ((ttl >> 4) & 0x0F));
    buf[5] = (uint8_t)((ttl & 0x0F) << 4);
    memcpy(&buf[6], inner, (size_t)inner_len);
}

static inline bool iotdata_mesh_unpack_forward(const uint8_t *buf, int len, iotdata_mesh_forward_t *f) {
    if (len < IOTDATA_MESH_FORWARD_HDR_SIZE + 4) /* need at least inner header */
        return false;
    uint8_t ctrl;
    iotdata_mesh_unpack_4_12(&buf[0], &ctrl, &f->sender_station);
    f->sender_seq = ((uint16_t)buf[2] << 8) | buf[3];
    f->ttl = (uint8_t)(((buf[4] & 0x0F) << 4) | ((buf[5] >> 4) & 0x0F));
    f->inner_packet = &buf[6];
    f->inner_len = len - IOTDATA_MESH_FORWARD_HDR_SIZE;
    /* extract origin from inner iotdata header */
    f->origin_station = ((uint16_t)(buf[6] & 0x0F) << 8) | buf[7];
    f->origin_sequence = ((uint16_t)buf[8] << 8) | buf[9];
    return true;
}

/* -------------------------------------------------------------------------
 * ACK (ctrl_type 0x2) — 8 bytes
 *
 * byte 4-5: ctrl(4) | fwd_station(12)
 * byte 6-7: fwd_seq(16)
 * ----------------------------------------------------------------------- */

typedef struct {
    uint16_t sender_station;
    uint16_t sender_seq;
    uint16_t fwd_station;
    uint16_t fwd_seq;
} iotdata_mesh_ack_t;

static inline void iotdata_mesh_pack_ack(uint8_t *buf, const iotdata_mesh_ack_t *a) {
    iotdata_mesh_pack_header(buf, a->sender_station, a->sender_seq);
    iotdata_mesh_pack_4_12(&buf[4], IOTDATA_MESH_CTRL_ACK, a->fwd_station);
    buf[6] = (uint8_t)(a->fwd_seq >> 8);
    buf[7] = (uint8_t)(a->fwd_seq & 0xFF);
}

static inline bool iotdata_mesh_unpack_ack(const uint8_t *buf, int len, iotdata_mesh_ack_t *a) {
    if (len < IOTDATA_MESH_ACK_SIZE)
        return false;
    uint8_t ctrl;
    iotdata_mesh_unpack_4_12(&buf[0], &ctrl, &a->sender_station);
    a->sender_seq = ((uint16_t)buf[2] << 8) | buf[3];
    iotdata_mesh_unpack_4_12(&buf[4], &ctrl, &a->fwd_station);
    a->fwd_seq = ((uint16_t)buf[6] << 8) | buf[7];
    return true;
}

/* -------------------------------------------------------------------------
 * ROUTE_ERROR (ctrl_type 0x3) — 5 bytes
 *
 * byte 4: ctrl(4) | reason(4)
 * ----------------------------------------------------------------------- */

typedef struct {
    uint16_t sender_station;
    uint16_t sender_seq;
    uint8_t reason;
} iotdata_mesh_route_error_t;

static inline bool iotdata_mesh_unpack_route_error(const uint8_t *buf, int len, iotdata_mesh_route_error_t *r) {
    if (len < IOTDATA_MESH_ROUTE_ERROR_SIZE)
        return false;
    uint8_t ctrl;
    iotdata_mesh_unpack_4_12(&buf[0], &ctrl, &r->sender_station);
    r->sender_seq = ((uint16_t)buf[2] << 8) | buf[3];
    r->reason = buf[4] & 0x0F;
    return true;
}

/* -------------------------------------------------------------------------
 * Duplicate suppression ring buffer
 * ----------------------------------------------------------------------- */

typedef struct {
    uint16_t station_id;
    uint16_t sequence;
} iotdata_mesh_dedup_entry_t;

typedef struct {
    iotdata_mesh_dedup_entry_t entries[IOTDATA_MESH_DEDUP_RING_SIZE];
    int head, size;
} iotdata_mesh_dedup_ring_t;

static inline void iotdata_mesh_dedup_init(iotdata_mesh_dedup_ring_t *ring) {
    memset(ring, 0, sizeof(*ring));
}

/* returns true if this is a NEW packet (not a duplicate) */
static inline bool iotdata_mesh_dedup_check_and_add(iotdata_mesh_dedup_ring_t *ring, uint16_t station_id, uint16_t sequence) {
    /* scan for duplicate */
    for (int i = 0; i < (ring->size < IOTDATA_MESH_DEDUP_RING_SIZE ? ring->size : IOTDATA_MESH_DEDUP_RING_SIZE); i++)
        if (ring->entries[i].station_id == station_id && ring->entries[i].sequence == sequence)
            return false; /* duplicate */
    /* new — add to ring */
    ring->entries[ring->head].station_id = station_id;
    ring->entries[ring->head].sequence = sequence;
    ring->head = (ring->head + 1) % IOTDATA_MESH_DEDUP_RING_SIZE;
    if (ring->size < IOTDATA_MESH_DEDUP_RING_SIZE)
        ring->size++;
    return true; /* new */
}

/* -------------------------------------------------------------------------
 * Generation comparison (modular, 12-bit)
 *
 * Returns true if gen_a is newer than gen_b.
 * ----------------------------------------------------------------------- */

static inline bool iotdata_mesh_generation_newer(uint16_t gen_a, uint16_t gen_b) {
    const uint16_t diff = (gen_a - gen_b) & (IOTDATA_MESH_GENERATION_MOD - 1);
    return diff > 0 && diff < IOTDATA_MESH_GENERATION_HALF;
}

/* -------------------------------------------------------------------------
 * RSSI quantisation (4-bit, 5 dBm steps from -120 dBm floor)
 * ----------------------------------------------------------------------- */

static inline uint8_t iotdata_mesh_rssi_encode(int rssi_dbm) {
    const int q = (rssi_dbm + 120) / 5;
    return (uint8_t)(q < 0 ? 0 : (q > 15 ? 15 : q));
}

static inline int iotdata_mesh_rssi_decode(uint8_t q) {
    return (int)q * 5 - 120;
}

/* -------------------------------------------------------------------------
 * Control type name (for logging)
 * ----------------------------------------------------------------------- */

static inline const char *iotdata_mesh_ctrl_name(uint8_t ctrl_type) {
    switch (ctrl_type) {
    case IOTDATA_MESH_CTRL_BEACON:
        return "BEACON";
    case IOTDATA_MESH_CTRL_FORWARD:
        return "FORWARD";
    case IOTDATA_MESH_CTRL_ACK:
        return "ACK";
    case IOTDATA_MESH_CTRL_ROUTE_ERROR:
        return "ROUTE_ERROR";
    case IOTDATA_MESH_CTRL_NEIGHBOUR_RPT:
        return "NEIGHBOUR_RPT";
    case IOTDATA_MESH_CTRL_PING:
        return "PING";
    case IOTDATA_MESH_CTRL_PONG:
        return "PONG";
    default:
        return "UNKNOWN";
    }
}

static inline const char *iotdata_mesh_reason_name(uint8_t reason) {
    switch (reason) {
    case IOTDATA_MESH_REASON_PARENT_LOST:
        return "parent_lost";
    case IOTDATA_MESH_REASON_OVERLOADED:
        return "overloaded";
    case IOTDATA_MESH_REASON_SHUTDOWN:
        return "shutdown";
    default:
        return "unknown";
    }
}

#endif /* IOTDATA_MESH_H */
