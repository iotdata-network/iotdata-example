
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

typedef bool (*mesh_packet_handler_t)(const uint8_t *packet, const int length);
typedef bool (*mesh_dedup_handler_t)(void *ctx, uint16_t station, uint16_t sequence);
typedef bool (*mesh_claimed_handler_t)(void *ctx, uint16_t station, uint16_t sequence);

/* DIRECT ACKS. A sensor frame heard here is acknowledged on the air, so that a relay which heard it
   too drops the forward it has queued -- it would only be carrying in what has already arrived. A
   relay matches an ACK on {origin, sequence} alone and from whoever sent it, so the ACK needs no
   destination: one answers every relay in earshot.
 *
 * Two gateways hear the same frame at the same instant, and two ACKs sent together collide -- at the
 * relay that matters, neither arrives. So each gateway has a SLOT: the first answers at once, each
 * later one a slot-width after. One waiting on its slot stands down when a lower-id peer has the
 * frame -- ddup's claim, over IP, ~30ms after the frame -- or when it hears another's ACK for it. The
 * claim is what does it: at 2.4k an ACK takes ~250ms from write to listener, and lands after the next
 * slot falls due. The first slot's reaches a relay ~370ms after the frame, inside its backoff (500ms
 * up); a later slot still beats some of them, and matters only when the first gateway did not hear
 * the frame at all. */
#define MESH_ACK_DIRECT_SLOT_MS 200u
#define MESH_ACK_DIRECT_SLOTS   4u /* the MESH_ACK_DIRECT_SLOT row's 1-4; from a station id, (station - 1) mod this */
#define MESH_ACK_DIRECT_PENDING 8  /* frames waiting on a slot at once */

typedef struct {
    bool used;
    uint16_t origin_station, origin_sequence;
    uint32_t due_ms;
} mesh_ack_pending_t;

typedef struct {
    uint32_t rx;    /* arrived as this type and unpacked */
    uint32_t err;   /* arrived as this type and would not unpack */
    uint64_t bytes; /* bytes seen of this type, accepted or not */
} mesh_ctrl_stat_t;

typedef struct {
    /* CACHED FROM THE CONFIG TABLE, and kept fresh by gateway_config_recache() rather than re-read.
       The receive path touches these per frame and the beacon interval every loop, which is the
       whole reason they are copies; the cost of a copy is that something has to refresh it, and
       that is what the notify on these rows is for. */
    bool enabled;
    bool debug;
    uint16_t beacon_interval_s; /* the root's cadence -- ours, and stated in every beacon we send */
    bool ack_direct;            /* acknowledge sensor frames heard directly (see MESH_ACK_DIRECT_*) */
    uint8_t ack_direct_slot;    /* as configured: 1-4, or 0 = from the station id, resolved when used */
    mesh_ack_pending_t ack_pending[MESH_ACK_DIRECT_PENDING];

    uint16_t beacon_generation;           /* increments each beacon round */
    uint32_t beacon_last_ms;              /* the beacon cadence, on the tick (0 = not yet primed) */
    iotdata_mesh_dedup_ring_t dedup_ring; /* dedup ring */
    /* handlers */
    mesh_packet_handler_t packet_handler;
    buffer_pool_t *pool; /* frames are built in pooled buffers, as everywhere else */
    iotdata_node_t *inode;
    mesh_dedup_handler_t dedup_handler;
    void *dedup_handler_ctx;
    mesh_claimed_handler_t claimed_handler; /* direct ACKs: does a lower-id gateway have this frame? (ddup, with dedup_handler_ctx) */
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
    uint32_t stat_acks_direct;         /* direct ACKs sent (a subset of stat_acks_tx) */
    uint32_t stat_acks_direct_yielded; /* held for our slot, then stood down: a lower-id peer had it, or its ACK came first */
    uint32_t stat_acks_direct_dropped; /* the pending table was full: not sent, the relays forward as before */
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
                                                     .sender_station = iotdata_node_station(st->inode),
                                                     .sender_seq = iotdata_node_sequence(st->inode),
                                                     .gateway_id = iotdata_node_station(st->inode),
                                                     .cost = 0,
                                                     .flags = IOTDATA_MESH_FLAG_ACCEPTING,
                                                     .generation = st->beacon_generation,
                                                     .interval_s = st->beacon_interval_s,
                                                 });
        if (st->debug)
            PRINTF_INFO("mesh: tx BEACON generation=%" PRIu16 ", station=%03" PRIX16 "\n", st->beacon_generation, iotdata_node_station(st->inode));
        st->beacon_generation = (st->beacon_generation + 1) & (IOTDATA_MESH_GENERATION_MOD - 1);
        if ((ok = len > 0 && st->packet_handler(buf, len))) {
            iotdata_node_sequence_used(st->inode);
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

static bool mesh_ack_pending_take(mesh_state_t *const st, const uint16_t origin_station, const uint16_t origin_sequence) {
    bool taken = false;
    for (int i = 0; i < MESH_ACK_DIRECT_PENDING; i++)
        if (st->ack_pending[i].used && st->ack_pending[i].origin_station == origin_station && st->ack_pending[i].origin_sequence == origin_sequence) {
            st->ack_pending[i].used = false;
            taken = true;
        }
    return taken;
}

bool mesh_transmit_ack(mesh_state_t *st, uint16_t origin_station, uint16_t origin_sequence) {
    bool ok = false;
    const buffer_handle_t h = buffer_acquire(st->pool);
    if (h != BUFFER_NONE) {
        uint8_t *const buf = buffer_data(st->pool, h);
        const int len = iotdata_mesh_pack_ack(buf, buffer_room(st->pool, h),
                                              &(const iotdata_mesh_ack_t){
                                                  .sender_station = iotdata_node_station(st->inode),
                                                  .sender_seq = iotdata_node_sequence(st->inode),
                                                  .origin_station = origin_station,
                                                  .origin_sequence = origin_sequence,
                                              });
        if (st->debug)
            PRINTF_INFO("mesh: tx ACK for origin={station=%03" PRIX16 ", sequence=%" PRIu16 "}\n", origin_station, origin_sequence);
        if ((ok = len > 0 && st->packet_handler(buf, len))) {
            iotdata_node_sequence_used(st->inode);
            (void)mesh_ack_pending_take(st, origin_station, origin_sequence); /* a forward's ACK answers a held direct one */
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
            PRINTF_INFO("mesh: rx FORWARD from station=%03" PRIX16 ", sequence=%" PRIu16 ", ttl=%" PRIu8 ", origin={station=%03" PRIX16 ", sequence=%" PRIu16 "}, inner-length=%d\n", out->sender_station, out->sender_seq, out->ttl,
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
            PRINTF_INFO("mesh: rx BEACON from gateway=%03" PRIX16 ", generation=%" PRIu16 ", cost=%" PRIu8 ", flags=0x%02" PRIX8 "\n", b->gateway_id, b->generation, b->cost, b->flags);
        return true;
    } else {
        mesh_stat_frame(st, IOTDATA_MESH_CTRL_BEACON, len, false);
        PRINTF_ERROR("mesh: rx BEACON unpack failed (len=%d)\n", len);
        return false;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static inline uint32_t mesh_ack_direct_delay_ms(const mesh_state_t *const st) {
    const uint8_t slot = (st->ack_direct_slot != 0u) ? (uint8_t)(st->ack_direct_slot - 1u) : (uint8_t)((uint16_t)(iotdata_node_station(st->inode) - 1u) % MESH_ACK_DIRECT_SLOTS);
    return (uint32_t)slot * MESH_ACK_DIRECT_SLOT_MS;
}

/* A sensor frame was heard directly: acknowledge it now, or hold it for this gateway's slot. */
void mesh_ack_direct(mesh_state_t *st, uint16_t origin_station, uint16_t origin_sequence, uint32_t now_ms) {
    if (!st->enabled || !st->ack_direct)
        return;
    const uint32_t delay_ms = mesh_ack_direct_delay_ms(st);
    if (delay_ms == 0u) {
        if (mesh_transmit_ack(st, origin_station, origin_sequence))
            st->stat_acks_direct++;
        return;
    }
    for (int i = 0; i < MESH_ACK_DIRECT_PENDING; i++)
        if (!st->ack_pending[i].used) {
            st->ack_pending[i] = (mesh_ack_pending_t){ .used = true, .origin_station = origin_station, .origin_sequence = origin_sequence, .due_ms = now_ms + delay_ms };
            return;
        }
    /* Full. Sending now would land in slot 0's ACK, so it is not sent at all: the relays forward,
       which is only what happened before there were direct ACKs. */
    st->stat_acks_direct_dropped++;
}

/* How long the receive loop may block before a held ACK falls due -- capped by what it would
   have waited anyway. The loop blocks in the radio read for seconds when the air is quiet, and a
   slot is a fraction of one. */
uint32_t mesh_ack_direct_wait_ms(const mesh_state_t *st, uint32_t now_ms, uint32_t wait_ms) {
    if (!st->enabled) /* mesh_tick would never drain them: don't spin on them either */
        return wait_ms;
    for (int i = 0; i < MESH_ACK_DIRECT_PENDING; i++)
        if (st->ack_pending[i].used) {
            const int32_t left = (int32_t)(st->ack_pending[i].due_ms - now_ms);
            const uint32_t w = (left <= 0) ? 1u : (uint32_t)left;
            if (w < wait_ms)
                wait_ms = w;
        }
    return wait_ms;
}

static void mesh_ack_direct_due(mesh_state_t *const st, const uint32_t now_ms) {
    for (int i = 0; i < MESH_ACK_DIRECT_PENDING; i++)
        if (st->ack_pending[i].used && (int32_t)(now_ms - st->ack_pending[i].due_ms) >= 0) {
            const mesh_ack_pending_t p = st->ack_pending[i];
            st->ack_pending[i].used = false;
            if (st->claimed_handler && st->claimed_handler(st->dedup_handler_ctx, p.origin_station, p.origin_sequence)) {
                st->stat_acks_direct_yielded++;
                if (st->debug)
                    PRINTF_INFO("mesh: direct ACK for origin={station=%03" PRIX16 ", sequence=%" PRIu16 "} not needed -- a lower-id gateway has it\n", p.origin_station, p.origin_sequence);
            } else if (mesh_transmit_ack(st, p.origin_station, p.origin_sequence))
                st->stat_acks_direct++;
        }
}

/* A gateway does not expect ACKs (it does not forward), but one on air is still a frame of a known
   type, so it is validated and counted like the rest rather than tallied blind by the caller. One
   is also how a gateway in a later slot learns that the frame it was holding an ACK for has been
   answered already -- and stands down. */
bool mesh_receive_ack(mesh_state_t *st, const uint8_t *buf, int len) {
    iotdata_mesh_ack_t a;
    if (iotdata_mesh_unpack_ack(buf, len, &a)) {
        mesh_stat_frame(st, IOTDATA_MESH_CTRL_ACK, len, true);
        if (mesh_ack_pending_take(st, a.origin_station, a.origin_sequence)) {
            st->stat_acks_direct_yielded++;
            if (st->debug)
                PRINTF_INFO("mesh: direct ACK for origin={station=%03" PRIX16 ", sequence=%" PRIu16 "} not needed -- station=%03" PRIX16 " answered first\n", a.origin_station, a.origin_sequence, a.sender_station);
        }
        if (st->debug)
            PRINTF_INFO("mesh: rx ACK from station=%03" PRIX16 " for origin={station=%03" PRIX16 ", sequence=%" PRIu16 "}\n", a.sender_station, a.origin_station, a.origin_sequence);
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
        PRINTF_INFO("mesh: rx ROUTE_ERROR from station=%03" PRIX16 ", reason=%s\n", err.sender_station, iotdata_mesh_reason_name(err.reason));
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
            PRINTF_INFO("mesh: rx NEIGHBOUR_REPORT from %03" PRIX16 " parent=%03" PRIX16 " cost=%" PRIu8 " gw=%03" PRIX16 ", %" PRIu8 " neighbour(s):\n", r->sender_station, r->parent_id, r->my_cost, r->gateway_id, r->num_neighbours);
            for (int k = 0; k < (int)r->num_neighbours; k++) {
                iotdata_mesh_nbr_entry_t e;
                char rbuf[STATIONS_RSSI_STR_MAX];
                if (!iotdata_mesh_neighbour_report_entry(buf, len, k, &e))
                    break;
                PRINTF_INFO("      %03" PRIX16 " cost=%" PRIu8 " rssi=%s\n", e.station, e.cost, stations_rssi_str(e.rssi_dbm, rbuf, sizeof(rbuf)));
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
// -----------------------------------------------------------------------------------------------------------------------------------------

void mesh_tick(mesh_state_t *st, const uint32_t now_ms) {
    if (!st->enabled)
        return;
    mesh_ack_direct_due(st, now_ms);
    if (intervalable_and_initial(st->beacon_interval_s, &st->beacon_last_ms, now_ms))
        mesh_transmit_beacon(st);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void mesh_begin(mesh_state_t *st, iotdata_node_t *inode, buffer_pool_t *pool, mesh_packet_handler_t packet_handler, mesh_dedup_handler_t dedup_handler, void *dedup_handler_ctx) {
    assert(packet_handler);
    st->pool = pool;
    st->inode = inode;
    st->packet_handler = packet_handler;
    st->dedup_handler = dedup_handler;
    st->dedup_handler_ctx = dedup_handler_ctx;
    iotdata_mesh_dedup_init(&st->dedup_ring);
    PRINTF_INFO("mesh: %s, station=%03" PRIX16 ", beacon-interval=%" PRIu16 "s, direct-ack=%s (after %" PRIu32 "ms)\n", st->enabled ? "enabled" : "disabled", iotdata_node_station(st->inode), st->beacon_interval_s,
                st->ack_direct ? "on" : "off", mesh_ack_direct_delay_ms(st));
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void mesh_end(__attribute__((unused)) mesh_state_t *st) {
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
