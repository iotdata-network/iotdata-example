
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

typedef bool (*mesh_packet_handler_t)(const uint8_t *packet, const int length);
typedef bool (*mesh_dedup_handler_t)(void *ctx, uint16_t station_id, uint16_t sequence);

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
    mesh_dedup_handler_t dedup_handler;
    void *dedup_handler_ctx;
    /* statistics */
    uint32_t stat_beacons_tx;
    uint32_t stat_beacons_rx;
    uint32_t stat_forwards_rx;
    uint32_t stat_forwards_unwrapped;
    uint32_t stat_forwards_unpack_err;
    uint32_t stat_duplicates;
    uint32_t stat_acks_tx;
    uint32_t stat_acks_rx;
    uint32_t stat_route_errors_rx;
    uint32_t stat_neighbour_reports_rx;
    uint32_t stat_pongs_rx;
    uint32_t stat_mesh_ctrl_rx;
    uint32_t stat_mesh_unknown;
    uint32_t stat_errors_tx;
    uint64_t stat_bytes_tx;
} mesh_state_t;

// -----------------------------------------------------------------------------------------------------------------------------------------

void mesh_beacon_send(mesh_state_t *state) {
    uint8_t buf[IOTDATA_MESH_BEACON_SIZE];
    const iotdata_mesh_beacon_t beacon = {
        .sender_station = state->station_id,
        .sender_seq = state->mesh_seq++,
        .gateway_id = state->station_id,
        .cost = 0,
        .flags = IOTDATA_MESH_FLAG_ACCEPTING,
        .generation = state->beacon_generation++,
    };
    state->beacon_generation &= (IOTDATA_MESH_GENERATION_MOD - 1);
    iotdata_mesh_pack_beacon(buf, &beacon);
    if (state->debug)
        printf("mesh: tx BEACON generation=%" PRIu16 ", station=0x%04" PRIX16 "\n", beacon.generation, beacon.sender_station);
    if (state->packet_handler(buf, IOTDATA_MESH_BEACON_SIZE)) {
        state->stat_beacons_tx++;
        state->stat_bytes_tx += (uint64_t)IOTDATA_MESH_BEACON_SIZE;
    } else {
        state->stat_errors_tx++;
        fprintf(stderr, "mesh: tx BEACON failed\n");
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void mesh_ack_send(mesh_state_t *state, uint16_t fwd_station, uint16_t fwd_seq) {
    uint8_t buf[IOTDATA_MESH_ACK_SIZE];
    const iotdata_mesh_ack_t ack = {
        .sender_station = state->station_id,
        .sender_seq = state->mesh_seq++,
        .fwd_station = fwd_station,
        .fwd_seq = fwd_seq,
    };
    iotdata_mesh_pack_ack(buf, &ack);
    if (state->debug)
        printf("mesh: tx ACK to station=0x%04" PRIX16 ", sequence=%" PRIu16 "\n", fwd_station, fwd_seq);
    if (state->packet_handler(buf, IOTDATA_MESH_ACK_SIZE)) {
        state->stat_acks_tx++;
        state->stat_bytes_tx += (uint64_t)IOTDATA_MESH_ACK_SIZE;
    } else {
        state->stat_errors_tx++;
        fprintf(stderr, "mesh: tx ACK failed\n");
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

bool mesh_handle_forward(mesh_state_t *state, const uint8_t *buf, int len, const uint8_t **inner, int *inner_len) {
    iotdata_mesh_forward_t fwd;
    if (!iotdata_mesh_unpack_forward(buf, len, &fwd)) {
        state->stat_forwards_unpack_err++;
        fprintf(stderr, "mesh: FORWARD unpack failed (len=%d)\n", len);
        return false;
    }
    state->stat_forwards_rx++;
    if (state->debug)
        printf("mesh: rx FORWARD from station=0x%04" PRIX16 ", sequence=%" PRIu16 ", ttl=%" PRIu8 ", origin={station=0x%04" PRIX16 ", sequence=%" PRIu16 "}, inner-length=%d\n", fwd.sender_station, fwd.sender_seq, fwd.ttl,
               fwd.origin_station, fwd.origin_sequence, fwd.inner_len);
    if (state->dedup_handler && !state->dedup_handler(state->dedup_handler_ctx, fwd.origin_station, fwd.origin_sequence)) {
        state->stat_duplicates++;
        if (state->debug)
            printf("mesh: rx FORWARD duplicate suppressed origin={station=0x%04" PRIX16 ", sequence=%" PRIu16 "}, inner-length=%d\n", fwd.origin_station, fwd.origin_sequence, fwd.inner_len);
        /* still ACK to prevent the forwarder from retrying */
        if (state->enabled)
            mesh_ack_send(state, fwd.sender_station, fwd.sender_seq);
        return false;
    }
    /* ACK the forwarder */
    if (state->enabled)
        mesh_ack_send(state, fwd.sender_station, fwd.sender_seq);
    state->stat_forwards_unwrapped++;
    *inner = fwd.inner_packet;
    *inner_len = fwd.inner_len;
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void mesh_handle_beacon(mesh_state_t *state, const uint8_t *buf, int len) {
    /* gateway receiving another gateway's beacon -- log for multi-gateway awareness */
    iotdata_mesh_beacon_t b;
    if (iotdata_mesh_unpack_beacon(buf, len, &b)) {
        state->stat_beacons_rx++;
        if (state->debug)
            printf("mesh: rx BEACON from gateway=0x%04" PRIX16 ", generation=%" PRIu16 ", cost=%" PRIu8 ", flags=0x%02" PRIX8 "\n", b.gateway_id, b.generation, b.cost, b.flags);
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void mesh_handle_route_error(mesh_state_t *state, const uint8_t *buf, int len) {
    iotdata_mesh_route_error_t err;
    if (iotdata_mesh_unpack_route_error(buf, len, &err)) {
        state->stat_route_errors_rx++;
        printf("mesh: rx ROUTE_ERROR from station=0x%04" PRIX16 ", reason=%s\n", err.sender_station, iotdata_mesh_reason_name(err.reason));
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void mesh_handle_neighbour_report(mesh_state_t *state, const uint8_t *buf, int len) {
    /* full topology aggregation is future work -- log receipt for now */
    uint8_t variant;
    uint16_t station_id, sequence;
    if (iotdata_mesh_peek_header(buf, len, &variant, &station_id, &sequence)) {
        state->stat_neighbour_reports_rx++;
        printf("mesh rx NEIGHBOUR_REPORT from station=0x%04" PRIX16 " (%d bytes)\n", station_id, len);
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void mesh_handle_pong(mesh_state_t *state, const uint8_t *buf, int len) {
    uint8_t variant;
    uint16_t station_id, sequence;
    if (iotdata_mesh_peek_header(buf, len, &variant, &station_id, &sequence)) {
        state->stat_pongs_rx++;
        printf("mesh: rx PONG from station=0x%04" PRIX16 " (%d bytes)\n", station_id, len);
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

bool mesh_begin(mesh_state_t *state, mesh_packet_handler_t packet_handler, mesh_dedup_handler_t dedup_handler, void *dedup_handler_ctx) {
    if (!state->enabled) {
        printf("mesh: disabled, not starting\n");
        return true;
    }
    if (!packet_handler) {
        printf("mesh: packet handler is required, not starting\n");
        return true;
    }
    state->packet_handler = packet_handler;
    state->dedup_handler = dedup_handler;
    state->dedup_handler_ctx = dedup_handler_ctx;
    iotdata_mesh_dedup_init(&state->dedup_ring);
    printf("mesh: enabled, station=0x%04" PRIX16 ", beacon-interval=%" PRIu32 "s\n", state->station_id, (uint32_t)state->beacon_interval);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void mesh_end(mesh_state_t *state) {
    (void)state;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
