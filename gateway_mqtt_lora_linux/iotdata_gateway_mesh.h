
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

typedef bool (*mesh_packet_handler_t)(const uint8_t *packet, const int length);
typedef bool (*mesh_dedup_handler_t)(void *ctx, uint16_t station_id, uint16_t sequence);

typedef struct {
    uint32_t rx;    /* arrived as this type and unpacked */
    uint32_t err;   /* arrived as this type and would not unpack */
    uint64_t bytes; /* bytes seen of this type, accepted or not */
} mesh_ctrl_stat_t;

typedef struct {
    bool enabled;
    uint16_t station_id;                  /* this gateway's station_id for mesh packets */
    time_t beacon_interval;               /* seconds between beacon transmissions */
    uint16_t beacon_generation;           /* increments each beacon round */
    uint16_t mesh_seq;                    /* mesh packet sequence counter */
    time_t beacon_last;                   /* last beacon TX time */
    iotdata_mesh_dedup_ring_t dedup_ring; /* dedup ring */
    bool debug;
    /* handlers */
    mesh_packet_handler_t packet_handler;
    buffer_pool_t *pool; /* frames are built in pooled buffers, as everywhere else */
    mesh_dedup_handler_t dedup_handler;
    void *dedup_handler_ctx;
    /* statistics: see mesh_ctrl_stat_t above for the per-frame-type table */
/* iotdata_mesh_peek_ctrl_type() returns the 4-bit type (0x0..0xF), or 0xFF when the frame is too
   short to have one. 0x0..0x7 are assigned (iotdata_mesh.h); 0x8..0xF are not, and are counted
   rather than discarded so an unexpected type on air is visible instead of silent. */
#define MESH_CTRL_COUNT 16
    mesh_ctrl_stat_t ctrl[MESH_CTRL_COUNT];
    uint32_t stat_ctrl_runt;          /* shorter than a mesh header: no type to attribute it to */
    uint32_t stat_forwards_unwrapped; /* FORWARD-specific: inner packet extracted and processed */
    uint32_t stat_duplicates;         /* FORWARD-specific: origin+sequence already seen */
    /* tx side */
    uint32_t stat_beacons_tx;
    uint32_t stat_acks_tx;
    uint32_t stat_errors_tx;
    uint64_t stat_bytes_tx;
} mesh_state_t;

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static inline void mesh_stat_frame(mesh_state_t *const st, const uint8_t ctrl, const int len, const bool ok) {
    if (ctrl < (uint8_t)(sizeof(st->ctrl) / sizeof(st->ctrl[0]))) {
        mesh_ctrl_stat_t *const e = &st->ctrl[ctrl];
        if (ok)
            e->rx++;
        else
            e->err++;
        if (len > 0)
            e->bytes += (uint64_t)len;
    } else
        st->stat_ctrl_runt++; /* a runt: peek_ctrl_type could not read a type at all */
}

static inline uint32_t mesh_stat_total(const mesh_state_t *const s, const bool errors) {
    uint32_t n = 0;
    for (int i = 0; i < (int)(sizeof(s->ctrl) / sizeof(s->ctrl[0])); i++) {
        const mesh_ctrl_stat_t *const e = &s->ctrl[i];
        n += errors ? e->err : e->rx;
    }
    return n;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

bool mesh_transmit_beacon(mesh_state_t *st) {
    bool ok = false;
    const buffer_handle_t h = buffer_acquire(st->pool);
    if (h != BUFFER_NONE) {
        uint8_t *const buf = buffer_data(st->pool, h);
        const int len = iotdata_mesh_pack_beacon(buf, buffer_room(st->pool, h),
                                                 &(const iotdata_mesh_beacon_t){
                                                     .sender_station = st->station_id,
                                                     .sender_seq = st->mesh_seq++,
                                                     .gateway_id = st->station_id,
                                                     .cost = 0,
                                                     .flags = IOTDATA_MESH_FLAG_ACCEPTING,
                                                     .generation = st->beacon_generation,
                                                 });
        if (st->debug)
            PRINTF_INFO("mesh: tx BEACON generation=%" PRIu16 ", station=%04" PRIX16 "\n", st->beacon_generation, st->station_id);
        st->beacon_generation = (st->beacon_generation + 1) & (IOTDATA_MESH_GENERATION_MOD - 1);
        ok = len > 0 && st->packet_handler(buf, len);
        if (ok) {
            st->stat_beacons_tx++;
            st->stat_bytes_tx += (uint64_t)len;
        } else {
            st->stat_errors_tx++;
            PRINTF_ERROR("mesh: tx BEACON failed\n");
        }
        buffer_unref(st->pool, h); /* written straight to the radio: nothing else holds it */
    } else {
        st->stat_errors_tx++;
        PRINTF_ERROR("mesh: tx BEACON no frame buffer\n");
    }
    return ok;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

bool mesh_transmit_ack(mesh_state_t *st, uint16_t origin_station, uint16_t origin_sequence) {
    bool ok = false;
    const buffer_handle_t h = buffer_acquire(st->pool);
    if (h != BUFFER_NONE) {
        uint8_t *const buf = buffer_data(st->pool, h);
        const int len = iotdata_mesh_pack_ack(buf, buffer_room(st->pool, h),
                                              &(const iotdata_mesh_ack_t){
                                                  .sender_station = st->station_id,
                                                  .sender_seq = st->mesh_seq++,
                                                  .origin_station = origin_station,
                                                  .origin_sequence = origin_sequence,
                                              });
        if (st->debug)
            PRINTF_INFO("mesh: tx ACK for origin={station=%04" PRIX16 ", sequence=%" PRIu16 "}\n", origin_station, origin_sequence);
        ok = len > 0 && st->packet_handler(buf, len);
        if (ok) {
            st->stat_acks_tx++;
            st->stat_bytes_tx += (uint64_t)len;
        } else {
            st->stat_errors_tx++;
            PRINTF_ERROR("mesh: tx ACK failed\n");
        }
        buffer_unref(st->pool, h);
    } else {
        st->stat_errors_tx++;
        PRINTF_ERROR("mesh: tx ACK no frame buffer\n");
    }
    return ok;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

bool mesh_receive_forward_r(mesh_state_t *st, const uint8_t *buf, int len, iotdata_mesh_forward_t *const out) {
    if (iotdata_mesh_unpack_forward(buf, len, out)) {
        mesh_stat_frame(st, IOTDATA_MESH_CTRL_FORWARD, len, true);
        if (st->debug)
            PRINTF_INFO("mesh: rx FORWARD from station=%04" PRIX16 ", sequence=%" PRIu16 ", ttl=%" PRIu8 ", origin={station=%04" PRIX16 ", sequence=%" PRIu16 "}, inner-length=%d\n", out->sender_station, out->sender_seq, out->ttl,
                        out->origin_station, out->origin_sequence, out->inner_len);
        return true;
    } else {
        mesh_stat_frame(st, IOTDATA_MESH_CTRL_FORWARD, len, false);
        PRINTF_ERROR("mesh: rx FORWARD unpack failed (len=%d)\n", len);
        return false;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

bool mesh_receive_beacon_r(mesh_state_t *st, const uint8_t *buf, int len, iotdata_mesh_beacon_t *const b) {
    /* gateway receiving another gateway's beacon -- log for multi-gateway awareness */
    if (iotdata_mesh_unpack_beacon(buf, len, b)) {
        mesh_stat_frame(st, IOTDATA_MESH_CTRL_BEACON, len, true);
        if (st->debug)
            PRINTF_INFO("mesh: rx BEACON from gateway=%04" PRIX16 ", generation=%" PRIu16 ", cost=%" PRIu8 ", flags=0x%02" PRIX8 "\n", b->gateway_id, b->generation, b->cost, b->flags);
        return true;
    } else {
        mesh_stat_frame(st, IOTDATA_MESH_CTRL_BEACON, len, false);
        PRINTF_ERROR("mesh: rx BEACON unpack failed (len=%d)\n", len);
        return false;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* A gateway does not expect ACKs (it does not forward), but one on air is still a frame of a known
   type, so it is validated and counted like the rest rather than tallied blind by the caller. */
bool mesh_receive_ack(mesh_state_t *st, const uint8_t *buf, int len) {
    iotdata_mesh_ack_t a;
    if (iotdata_mesh_unpack_ack(buf, len, &a)) {
        mesh_stat_frame(st, IOTDATA_MESH_CTRL_ACK, len, true);
        if (st->debug)
            PRINTF_INFO("mesh: rx ACK from station=%04" PRIX16 " for origin={station=%04" PRIX16 ", sequence=%" PRIu16 "}\n", a.sender_station, a.origin_station, a.origin_sequence);
        return true;
    } else {
        mesh_stat_frame(st, IOTDATA_MESH_CTRL_ACK, len, false);
        PRINTF_ERROR("mesh: rx ACK unpack failed (len=%d)\n", len);
        return false;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

bool mesh_receive_route_error(mesh_state_t *st, const uint8_t *buf, int len) {
    iotdata_mesh_route_error_t err;
    if (iotdata_mesh_unpack_route_error(buf, len, &err)) {
        mesh_stat_frame(st, IOTDATA_MESH_CTRL_ROUTE_ERROR, len, true);
        PRINTF_INFO("mesh: rx ROUTE_ERROR from station=%04" PRIX16 ", reason=%s\n", err.sender_station, iotdata_mesh_reason_name(err.reason));
        return true;
    } else {
        mesh_stat_frame(st, IOTDATA_MESH_CTRL_ROUTE_ERROR, len, false);
        PRINTF_ERROR("mesh: rx ROUTE_ERROR unpack failed (len=%d)\n", len);
        return false;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

bool mesh_receive_neighbour_report_r(mesh_state_t *st, const uint8_t *buf, int len, iotdata_mesh_neighbour_report_t *const r) {
    if (iotdata_mesh_unpack_neighbour_report(buf, len, r)) {
        mesh_stat_frame(st, IOTDATA_MESH_CTRL_NEIGHBOUR_RPT, len, true);
        if (st->debug) {
            PRINTF_INFO("mesh: rx NEIGHBOUR_REPORT from %04" PRIX16 " parent=%04" PRIX16 " cost=%u gw=%04" PRIX16 ", %u neighbour(s):\n", r->sender_station, r->parent_id, (unsigned)r->my_cost, r->gateway_id, (unsigned)r->num_neighbours);
            for (int k = 0; k < (int)r->num_neighbours; k++) {
                iotdata_mesh_nbr_entry_t e;
                if (!iotdata_mesh_neighbour_report_entry(buf, len, k, &e))
                    break;
                PRINTF_INFO("      %04" PRIX16 " cost=%u rssi=%ddBm\n", e.station, (unsigned)e.cost, iotdata_mesh_rssi_from_q4(e.rssi_q4));
            }
        }
        return true;
    } else {
        mesh_stat_frame(st, IOTDATA_MESH_CTRL_NEIGHBOUR_RPT, len, false);
        PRINTF_ERROR("mesh: rx NEIGHBOUR_REPORT unpack failed (len=%d)\n", len);
        return false;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

bool mesh_receive_pong(mesh_state_t *st, const uint8_t *buf, int len) {
    uint8_t variant;
    uint16_t station_id, sequence;
    if (iotdata_mesh_peek_header(buf, len, &variant, &station_id, &sequence)) {
        mesh_stat_frame(st, IOTDATA_MESH_CTRL_PONG, len, true);
        if (st->debug)
            PRINTF_INFO("mesh: rx PONG from station=%04" PRIX16 " (%d bytes)\n", station_id, len);
        return true;
    } else {
        mesh_stat_frame(st, IOTDATA_MESH_CTRL_PONG, len, false);
        PRINTF_ERROR("mesh: rx PONG unpack failed (len=%d)\n", len);
        return false;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

bool mesh_begin(mesh_state_t *st, buffer_pool_t *pool, mesh_packet_handler_t packet_handler, mesh_dedup_handler_t dedup_handler, void *dedup_handler_ctx) {
    st->pool = pool; /* before the enabled check: a disabled mesh never transmits, but never half-built either */
    if (!st->enabled) {
        PRINTF_INFO("mesh: disabled, not starting\n");
        return true;
    }
    if (!packet_handler) {
        PRINTF_INFO("mesh: packet handler is required, not starting\n");
        return true;
    }
    st->packet_handler = packet_handler;
    st->dedup_handler = dedup_handler;
    st->dedup_handler_ctx = dedup_handler_ctx;
    iotdata_mesh_dedup_init(&st->dedup_ring);
    PRINTF_INFO("mesh: enabled, station=%04" PRIX16 ", beacon-interval=%" PRIu32 "s\n", st->station_id, (uint32_t)st->beacon_interval);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void mesh_end(__attribute__((unused)) mesh_state_t *st) {
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
