
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#ifndef LORA_READ_TIMEOUT_MS
#define LORA_READ_TIMEOUT_MS 5000
#endif

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

bool ddup_insert_handler(void *ctx, uint16_t station, uint16_t sequence) {
    return ddup_check_and_add(((process_state_t *)ctx)->state_ddup, station, sequence);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* defined below; the admit point above it is what every caller actually uses */
void process_sensor_packet(process_state_t *st, const uint8_t *packet_buffer, int packet_length, uint8_t variant, uint16_t station, uint16_t sequence, const char *topic_prefix, const char *via, int packet_rssi_dbm);

/*
 * THE SINGLE ADMIT POINT for every sensor reading, direct or unwrapped from a mesh FORWARD. One
 * function, so the two paths cannot disagree about what has already been published.
 *
 * ddup_admit() returns one of three answers, and the third is the one that is new here:
 *   PUBLISH  nobody is contesting this -- ddup off, no reachable peers, or we are the lowest id and
 *            would win any election anyway. Straight out.
 *   REJECT   already decided. A retransmit, a second path through the mesh, a late forward.
 *   HOLD     somebody else might also have heard it, and they may outrank us. Keep it for hold_ms
 *            while the claims cross, then publish if we won.
 *
 * HOLD is what kills the duplicate the ring cannot: two gateways hearing ONE broadcast at the same
 * instant are both "first", so neither can suppress the other by arriving second. The ring handles
 * everything that arrives LATER; the election handles everything that arrives AT THE SAME TIME.
 *
 * A FULL HOLD QUEUE PUBLISHES RATHER THAN DROPS. A duplicate is a nuisance; a lost reading is data.
 *
 * ONE POINT FOR BOTH PATHS, and that is not tidiness. When the direct path and the forward path
 * each made their own dedup decision, a reading heard BOTH ways -- directly and through a relay --
 * went out twice to the same topic, "via direct" then "via mesh": one path recorded the reception
 * and the other looked it up, missed, and published again. They cannot disagree now, because they
 * ask the same function.
 */
static void process_admit_sensor(process_state_t *st, const uint8_t *buf, int len, uint8_t variant, uint16_t station, uint16_t sequence, const char *topic_prefix, const char *via, int packet_rssi_dbm) {
    switch (ddup_admit(st->state_ddup, station, sequence)) {
    case DDUP_PUBLISH:
        process_sensor_packet(st, buf, len, variant, station, sequence, topic_prefix, via, packet_rssi_dbm);
        break;
    case DDUP_HOLD:
        if (!ddup_hold_add(st->state_ddup, buf, len, variant, station, sequence, topic_prefix, via, packet_rssi_dbm))
            process_sensor_packet(st, buf, len, variant, station, sequence, topic_prefix, via, packet_rssi_dbm); /* queue full -> publish un-held */
        break;
    case DDUP_REJECT:
    default:
        break;
    }
}

/* A held reading won its election: out through the ordinary path, as though it had never waited. */
static void process_release_cb(void *ctx, const uint8_t *packet, int packet_len, uint8_t variant, uint16_t station, uint16_t sequence, const char *topic_prefix, const char *via, int rssi_dbm) {
    process_sensor_packet((process_state_t *)ctx, packet, packet_len, variant, station, sequence, topic_prefix, via, rssi_dbm);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

// packet_rssi_dbm is dBm as the driver reports it; LORA_RSSI_NONE means the radio did
// not supply one (RSSI capture off, or a mesh-relayed packet that this gateway never heard over
// the air). It is a property of THIS hop's reception, not of the sensor, hence 'rssi_packet'.
/*
 * Does this frame actually decode?
 *
 * A truncated or corrupt reception can still carry a plausible header -- the station and sequence
 * read fine -- and must NOT be allowed to claim that {station, sequence} in the dedup ring. If it
 * does, the good copy arriving a moment later via a relay is suppressed as a duplicate of a packet
 * we never actually received, and nothing is published at all. That defeats the point of having a
 * mesh: the relayed copy exists precisely to rescue a bad direct reception.
 *
 * Observed with a 56-byte node VERSION report split across two UART reads: the first chunk decoded
 * as far as the header, claimed {05BF, 5}, then failed on content -- and the relay's forward of the
 * same report was dropped as a duplicate.
 *
 * So: decode first, claim second. The cost is one throw-away TLV walk per direct packet, which on
 * this host is nothing next to being wrong.
 */
static bool process_packet_decodes(process_state_t *const st, const uint8_t *const buf, const int len) {
    /* Decoding is the whole test, and it must not also require a TLV -- nor a field.
       A packet may carry variant fields, TLVs, or both: the presence byte has a TLV flag so the
       two can coexist. A plain sensor sends fields and no TLV, a sleeping sensor sends fields plus
       a RECEIVE TLV, the TSA sends a proprietary TLV and no fields. Requiring tlv_count > 0 here
       rejected the first shape on the direct path, while the same bytes arriving inside a relay's
       FORWARD published normally, because that path never consults this gate.
       (node_on_packet does require a TLV, correctly: it is hunting for system TLVs to republish,
       which is a different question from whether the frame is a packet at all.) */
    return iotdata_decode(buf, (size_t)len, &st->_iotdata_dec) == IOTDATA_OK;
}

void process_sensor_packet(process_state_t *st, const uint8_t *packet_buffer, int packet_length, uint8_t variant, uint16_t station, uint16_t sequence, const char *topic_prefix, const char *via, int packet_rssi_dbm) {
    // Network-table tracking (per-path counts, dups, sequence gaps) is done by the caller at the
    // dedup point (netw_note_receive), so it also sees suppressed duplicates — which never reach here.
    const iotdata_variant_def_t *vdef;
    if (!(vdef = iotdata_get_variant(variant))) {
        PRINTF_ERROR("exec: unknown variant %" PRIu8 " (station=%03" PRIX16 ", size=%d)\n", variant, station, packet_length);
        stat_on_packet_decode_error(st->state_stat, station, variant);
        return;
    }
    char *json = NULL;
    iotdata_status_t rc;
    if ((rc = iotdata_decode_to_json(packet_buffer, (size_t)packet_length, &json, &st->_iotdata_scratch)) != IOTDATA_OK) {
        PRINTF_ERROR("exec: decode failed: %s (variant=%" PRIu8 ", station=%03" PRIX16 ", size=%d)\n", iotdata_strerror(rc), variant, station, packet_length);
        stat_on_packet_decode_error(st->state_stat, station, variant);
        return;
    }
    stat_on_packet_decoded(st->state_stat, station, sequence, variant, (uint16_t)packet_length, &st->_iotdata_scratch.dec);
    snprintf(st->_buffer_mqtt_topic, sizeof(st->_buffer_mqtt_topic), "%s" IOTDATA_NODE_MQTT_VARIANT_TOPIC "/%s/%03" PRIX16, topic_prefix, vdef->name, station);
    // Splice rssi_packet in as the first member rather than teaching the decoder about it: the
    // decoder emits the packet's own payload, and reception strength is not part of that payload.
    // Keeping it out of the codec means every variant gains the field for free and none of them
    // has to carry a field that only a gateway can know.
    const char *payload = json;
    if (packet_rssi_dbm != LORA_RSSI_NONE && json[0] == '{') {
        const int n = snprintf(st->_buffer_mqtt_message, sizeof(st->_buffer_mqtt_message), "{\"rssi_packet\":%d,%s", packet_rssi_dbm, json + 1);
        if (n > 0 && n < (int)sizeof(st->_buffer_mqtt_message))
            payload = st->_buffer_mqtt_message;
        else
            PRINTF_ERROR("exec: rssi splice would truncate (%d bytes), publishing without it\n", n);
    }
    if (!mqtt_send(st->_buffer_mqtt_topic, payload, (int)strlen(payload))) {
        PRINTF_ERROR("exec: mqtt send failed (topic=%s, size=%d)\n", st->_buffer_mqtt_topic, (int)strlen(payload));
        stat_on_packet_process_error(st->state_stat, station, variant);
    }
    if (st->debug)
        PRINTF_INFO("      -> %s (%d bytes%s%s)\n", st->_buffer_mqtt_topic, (int)strlen(payload), via ? " via " : "", via ? via : "");
    free(json);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

void process_mesh_packet(process_state_t *st, const uint8_t *packet_buffer, int packet_length, __attribute__((unused)) uint8_t variant, uint16_t station, uint16_t sequence, const char *topic_prefix, int packet_rssi_dbm,
                         const uint32_t now_ms) {
    const uint8_t ctrl_type = iotdata_mesh_peek_ctrl_type(packet_buffer, packet_length);
    if (st->state_mesh->debug)
        PRINTF_INFO("exec: mesh rx %s from station=%03" PRIX16 ", sequence=%" PRIu16 " (%d bytes)\n", iotdata_mesh_ctrl_name(ctrl_type), station, sequence, packet_length);
    netw_note_transmit(&st->network, station, sequence, now_ms);
    switch (ctrl_type) {
    case IOTDATA_MESH_CTRL_FORWARD: {
        iotdata_mesh_forward_t fw;
        if (mesh_receive_forward_r(st->state_mesh, packet_buffer, packet_length, &fw)) {
            const bool is_new = !st->state_mesh->dedup_handler || st->state_mesh->dedup_handler(st->state_mesh->dedup_handler_ctx, fw.origin_station, fw.origin_sequence);
            if (is_new)
                st->state_mesh->stat_forwards_unwrapped++;
            else {
                st->state_mesh->stat_duplicates++;
                if (st->state_mesh->debug)
                    PRINTF_INFO("exec: mesh FORWARD duplicate suppressed origin={station=%03" PRIX16 ", sequence=%" PRIu16 "}, inner-length=%d\n", fw.origin_station, fw.origin_sequence, fw.inner_len);
            }
            if (st->state_mesh->enabled)
                mesh_transmit_ack(st->state_mesh, fw.origin_station, fw.origin_sequence);
            netw_note_forward(&st->network, fw.sender_station, now_ms); /* the relay's forwarding load */
            uint8_t inner_variant = 0;
            uint16_t inner_station = 0, inner_sequence = 0;
            const bool inner_ok = (iotdata_peek(fw.inner_packet, (size_t)fw.inner_len, &inner_variant, &inner_station, &inner_sequence) == IOTDATA_OK);
            if (inner_ok) /* no rssi: that hop is relay->gateway, not sensor->gateway */
                netw_note_receive(&st->network, fw.origin_station, inner_variant, fw.origin_sequence, NETW_PATH_MESH, is_new, 0, fw.sender_station, now_ms);
            if (is_new) {
                /* A MESH FRAME INSIDE A FORWARD is handled exactly as one heard directly -- the
                   wrapper is transport, not meaning. Both the survey response and the neighbour
                   report travel this way now, precisely so they are acknowledged, retried, and
                   carried by relays between here and their origin. Recursing keeps one dispatch
                   rather than a second, partial copy of it; a FORWARD is excluded so a malformed
                   frame claiming to nest cannot recurse without bound. */
                if (inner_ok && inner_variant == IOTDATA_MESH_VARIANT) {
                    const uint8_t inner_ctrl = iotdata_mesh_peek_ctrl_type(fw.inner_packet, fw.inner_len);
                    if (inner_ctrl == IOTDATA_MESH_CTRL_FORWARD)
                        PRINTF_ERROR("exec: FORWARD nested inside a FORWARD from %03" PRIX16 " -- refusing\n", fw.origin_station);
                    else
                        process_mesh_packet(st, fw.inner_packet, fw.inner_len, inner_variant, fw.origin_station, fw.origin_sequence, topic_prefix, packet_rssi_dbm, now_ms);
                } else if (inner_ok) {
                    /* A forwarded packet carries system TLVs exactly as a direct one does -- a node
                       two hops out can only ever be heard this way, so without this its reports are
                       handed to the telemetry path alone and never reach the node topic. Addressed
                       by the ORIGIN, not the relay that carried it. */
                    process_admit_sensor(st, fw.inner_packet, fw.inner_len, inner_variant, inner_station, inner_sequence, topic_prefix, "mesh", packet_rssi_dbm);
                    (void)node_on_packet(st->state_node, fw.inner_packet, (size_t)fw.inner_len, fw.origin_station);
                } else {
                    PRINTF_ERROR("exec: mesh FORWARD inner packet peek failed (len=%d)\n", fw.inner_len);
                    stat_on_link_rx_drop(st->state_stat);
                }
            }
        } else
            stat_on_link_rx_drop(st->state_stat);
        break;
    }
    case IOTDATA_MESH_CTRL_BEACON: {
        iotdata_mesh_beacon_t b;
        if (mesh_receive_beacon_r(st->state_mesh, packet_buffer, packet_length, &b)) {
            stat_on_peer(st->state_stat, b.gateway_id, b.generation, b.cost, b.flags);
            netw_note_beacon(&st->network, station, b.flags, b.cost, b.generation, b.gateway_id, (packet_rssi_dbm != LORA_RSSI_NONE) ? packet_rssi_dbm : 0, now_ms);
        }
        break;
    }
    case IOTDATA_MESH_CTRL_ACK:
        (void)mesh_receive_ack(st->state_mesh, packet_buffer, packet_length);
        break;
    case IOTDATA_MESH_CTRL_ROUTE_ERROR:
        (void)mesh_receive_route_error(st->state_mesh, packet_buffer, packet_length);
        break;
    case IOTDATA_MESH_CTRL_NEIGHBOUR_RPT: {
        iotdata_mesh_neighbour_report_t r;
        if (mesh_receive_neighbour_report_r(st->state_mesh, packet_buffer, packet_length, &r))  /* mesh-layer log */
            netw_note_neighbour_report(&st->network, packet_buffer, packet_length, &r, now_ms); /* relay hears[] + both-ends RSSI */
        break;
    }
    case IOTDATA_MESH_CTRL_DIAG:
        mesh_stat_frame(st->state_mesh, ctrl_type, packet_length, true);
        if (!survey_on_frame(st->state_survey, packet_buffer, packet_length))
            (void)trace_on_frame(st->state_trace, packet_buffer, packet_length, packet_rssi_dbm);
        break;
    default:
        mesh_stat_frame(st->state_mesh, ctrl_type, packet_length, false);
        if (st->state_mesh->debug)
            PRINTF_INFO("exec: mesh rx unknown ctrl_type=0x%02" PRIX8 " from station=%03" PRIX16 "\n", ctrl_type, station);
        break;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

/*
 * How this gateway sees the mesh it is part of: the mesh group of STATUS.
 *
 * A gateway is a node like any other and answers the same question a relay does -- it simply has
 * different answers. It IS the root, so: state GATEWAY, cost 0, no parent and no parent RSSI, and
 * the counters a node only accumulates by having a parent (peer_new, reparent, failover, orphan)
 * stay 0 because a root does none of those things. Reporting them as zero is the honest answer,
 * not a gap: "never reparented" is true of a root by construction.
 */
static void exec_node_status_mesh(iotdata_node_status_mesh_t *const out) {
    const process_state_t *const st = g_exec;
    if (!st || !st->state_mesh->enabled)
        return; /* .present stays false: mesh off, so there is no group to report */
    out->present = true;
    out->state = IOTDATA_NODE_STATUS_MESH_STATE_GATEWAY;
    out->parent = 0;
    out->cost = 0;
    out->generation = st->state_mesh->beacon_generation;
    out->parent_rssi = 0;
    out->peers = (uint8_t)st->state_stat->peers_count;
    out->accepting = true;
    out->beacon_rx = st->state_mesh->ctrl[IOTDATA_MESH_CTRL_BEACON].rx;
    out->beacon_tx = st->state_mesh->stat_beacons_tx;
    out->rerr_rx = st->state_mesh->ctrl[IOTDATA_MESH_CTRL_ROUTE_ERROR].rx;
    out->rerr_tx = 0; /* the root never sends one: it has no route to lose */
    out->forwards = st->state_mesh->stat_forwards_unwrapped;
    out->duplicates = st->state_mesh->stat_duplicates;
}

static uint8_t exec_node_table_count(const uint8_t entry) {
    const process_state_t *const st = g_exec;
    if (!st)
        return 0;
    switch (entry) {
    case IOTDATA_NODE_STATUS_STATIONS_ENTRY:
        return (uint8_t)st->network.count;
    case IOTDATA_NODE_STATUS_MESH_PEERS_ENTRY:
        return (uint8_t)netw_peer_count(&st->network);
    case IOTDATA_NODE_STATUS_FILTERS_ENTRY:
        return (uint8_t)filter_count(&st->filter);
    default:
        return 0;
    }
}

static inline bool exec_table_any(__attribute__((unused)) const void *const e) {
    return true;
}
#define EXEC_TABLE_ANY exec_table_any

#define EXEC_TABLE_NTH_IF(arr, cnt, key, pred, idx, out) \
    do { \
        (out) = -1; \
        for (int _i = 0; _i < (cnt); _i++) { \
            if (!pred(&(arr)[_i])) \
                continue; \
            int _rank = 0; \
            for (int _j = 0; _j < (cnt); _j++) \
                if (pred(&(arr)[_j]) && ((arr)[_j].key < (arr)[_i].key || ((arr)[_j].key == (arr)[_i].key && _j < _i))) \
                    _rank++; \
            if (_rank == (int)(idx)) { \
                (out) = _i; \
                break; \
            } \
        } \
    } while (0)

#define EXEC_TABLE_NTH(arr, cnt, key, idx, out) EXEC_TABLE_NTH_IF(arr, cnt, key, EXEC_TABLE_ANY, idx, out)

static bool exec_node_table_row(const uint8_t entry, const uint8_t index, uint8_t *const row) {
    process_state_t *const st = g_exec;
    if (!st)
        return false;
    int slot = -1;
    switch (entry) {
    case IOTDATA_NODE_STATUS_STATIONS_ENTRY: {
        EXEC_TABLE_NTH_IF(st->network.s, st->network.count, station, netw_is_node, index, slot);
        if (slot < 0)
            return false;
        const netw_station_t *const e = &st->network.s[slot];
        iotdata_node_entry_put_u16(row, 0, e->station);
        iotdata_node_entry_put_u8(row, 2,
                                  (e->kind == NETW_KIND_GATEWAY)  ? IOTDATA_NODE_STATUS_STATIONS_KIND_GATEWAY
                                  : (e->kind == NETW_KIND_RELAY)  ? IOTDATA_NODE_STATUS_STATIONS_KIND_RELAY
                                  : (e->kind == NETW_KIND_SENSOR) ? IOTDATA_NODE_STATUS_STATIONS_KIND_SENSOR
                                                                  : IOTDATA_NODE_STATUS_STATIONS_KIND_UNKNOWN);
        iotdata_node_entry_put_u8(row, 3, e->variant);
        iotdata_node_entry_put_u8(row, 4, (uint8_t)(int8_t)e->rssi);
        iotdata_node_entry_put_age_s(row, 5, (uint32_t)(hw_time_ms() - e->last_seen_ms) / 1000u);
        iotdata_node_entry_put_u32(row, 8, e->rx_count);
        return true;
    }
    case IOTDATA_NODE_STATUS_MESH_PEERS_ENTRY: {
        EXEC_TABLE_NTH_IF(st->network.s, st->network.count, station, netw_is_peer, index, slot);
        if (slot < 0)
            return false;
        const netw_station_t *const e = &st->network.s[slot];
        iotdata_node_entry_put_u16(row, 0, e->station);
        iotdata_node_entry_put_u16(row, 2, e->gateway); /* the root IT advertises, which need not be us */
        iotdata_node_entry_put_u8(row, 4, e->cost);
        iotdata_node_entry_put_u16(row, 5, e->generation);
        iotdata_node_entry_put_u8(row, 7, (uint8_t)(int8_t)e->rssi);
        iotdata_node_entry_put_age_s(row, 8, (uint32_t)(hw_time_ms() - e->last_seen_ms) / 1000u);
        /* the root has no parent, so only ACCEPTING can be true of a peer here */
        iotdata_node_entry_put_u8(row, 11, (uint8_t)(e->accepting ? IOTDATA_NODE_STATUS_MESH_PEERS_FLAG_ACCEPTING : 0));
        return true;
    }
    case IOTDATA_NODE_STATUS_FILTERS_ENTRY: {
        EXEC_TABLE_NTH(st->filter.e, filter_count(&st->filter), station, index, slot);
        if (slot < 0)
            return false;
        const filter_entry_t *const e = &st->filter.e[slot];
        iotdata_node_entry_put_u16(row, 0, e->station);
        iotdata_node_entry_put_u8(row, 2, (e->action == FILTER_ALLOW) ? IOTDATA_NODE_CONTROL_NODE_FILTERS_ALLOW : IOTDATA_NODE_CONTROL_NODE_FILTERS_BLOCK);
        iotdata_node_entry_put_u8(row, 3, (e->source == FILTER_AUTO) ? IOTDATA_NODE_CONTROL_NODE_FILTERS_SCOPE_AUTO : IOTDATA_NODE_CONTROL_NODE_FILTERS_SCOPE_MANUAL);
        return true;
    }
    default:
        return false;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

void process_packet(process_state_t *st, const uint32_t now_ms) {
    if (st->buffer_rx == BUFFER_NONE)
        st->buffer_rx = buffer_acquire(st->buffer_pool);
    if (st->buffer_rx != BUFFER_NONE) {
        (void)buffer_reset(st->buffer_pool, st->buffer_rx);
        int packet_rssi_dbm = LORA_RSSI_NONE, packet_length;
        uint8_t *rxbuf = buffer_data(st->buffer_pool, st->buffer_rx);
        if (lora_read(rxbuf, buffer_room(st->buffer_pool, st->buffer_rx), &packet_length, &packet_rssi_dbm, LORA_READ_TIMEOUT_MS) == ESP_OK && packet_length > 0) {
            buffer_set_len(st->buffer_pool, st->buffer_rx, (uint16_t)packet_length);
            stat_on_link_rx_packet(st->state_stat, (uint16_t)packet_length);
            if (st->debug_data)
                debug_hexdump("data: ", rxbuf, (size_t)packet_length);
            if (packet_rssi_dbm != LORA_RSSI_NONE)
                stat_on_link_rssi_packet(st->state_stat, packet_rssi_dbm);
            else
                stat_on_link_rssi_packet_error(st->state_stat);
            uint8_t variant;
            uint16_t station, sequence;
            if (iotdata_peek(rxbuf, (size_t)packet_length, &variant, &station, &sequence) != IOTDATA_OK) {
                PRINTF_ERROR("exec: packet too short for iotdata header (size=%d)\n", packet_length);
                stat_on_link_rx_drop(st->state_stat);
            } else if (!filter_allows(&st->filter, station)) {
                st->filter.stat_blocked++;
                stat_on_link_rx_drop(st->state_stat);
                if (st->debug)
                    PRINTF_INFO("exec: FILTERED from=%03" PRIX16 " var=%" PRIu8 " len=%d (blocked)\n", station, variant, packet_length);
            } else if (variant == IOTDATA_MESH_VARIANT) {
                if (st->state_mesh->enabled)
                    process_mesh_packet(st, rxbuf, packet_length, variant, station, sequence, iotdata_node_config_string(MQTT_TOPIC_PREFIX), packet_rssi_dbm, now_ms);
                else {
                    stat_on_link_rx_mesh_unexpected(st->state_stat, station);
                    PRINTF_INFO("exec: mesh packet unexpected from station=%03" PRIX16 " while not enabled\n", station);
                }
            } else {
                // Dedup direct receptions against the SAME ring the forward path uses, so a sensor
                // heard both directly and via a relay publishes once — whichever path adds {station,
                // seq} first wins, the other is suppressed. is_new is true unless it was a duplicate
                // (always true when mesh is off). Track the reception either way for observability.
                /* The driver's value goes in UNTRANSLATED. It already distinguishes a reading
                   (< 0) from saturation (0) from none (LORA_RSSI_NONE), and the survey
                   encoding uses the same convention -- mapping "none" onto 0 here, as this
                   once did, published an absent reading as the strongest one possible. */
                (void)survey_on_direct(st->state_survey, station, sequence, packet_rssi_dbm);
                if (process_packet_decodes(st, rxbuf, packet_length)) {
                    /* ADMIT DECIDES, and the network table is told what it decided. A HOLD is
                       "new" as far as the table is concerned -- the reading was genuinely heard
                       here, whether or not this gateway ends up being the one to publish it. */
                    const ddup_verdict_t verdict = ddup_admit(st->state_ddup, station, sequence);
                    const bool is_new = (verdict != DDUP_REJECT);
                    netw_note_receive(&st->network, station, variant, sequence, NETW_PATH_DIRECT, is_new, (packet_rssi_dbm != LORA_RSSI_NONE) ? packet_rssi_dbm : 0, 0, now_ms);
                    if (verdict == DDUP_PUBLISH)
                        process_sensor_packet(st, rxbuf, packet_length, variant, station, sequence, iotdata_node_config_string(MQTT_TOPIC_PREFIX), "direct", packet_rssi_dbm);
                    else if (verdict == DDUP_HOLD && !ddup_hold_add(st->state_ddup, rxbuf, packet_length, variant, station, sequence, iotdata_node_config_string(MQTT_TOPIC_PREFIX), "direct", packet_rssi_dbm))
                        process_sensor_packet(st, rxbuf, packet_length, variant, station, sequence, iotdata_node_config_string(MQTT_TOPIC_PREFIX), "direct", packet_rssi_dbm); /* queue full -> publish un-held */
                } else {
                    stat_on_link_rx_drop(st->state_stat);
                    PRINTF_ERROR("exec: undecodable frame from station=%03" PRIX16 ", sequence=%" PRIu16 " (%d bytes): %s -- dropped before dedup, so a relayed copy can still be used\n", station, sequence, packet_length,
                                 iotdata_strerror(iotdata_decode(rxbuf, (size_t)packet_length, &st->_iotdata_dec)));
                }
                (void)node_on_packet(st->state_node, rxbuf, (size_t)packet_length, station);
            }
            if (st->buffer_rx != BUFFER_NONE && buffer_refs(st->buffer_pool, st->buffer_rx) > 1) {
                buffer_unref(st->buffer_pool, st->buffer_rx);
                st->buffer_rx = BUFFER_NONE;
            }
        }
    } else {
        stat_on_link_rx_drop(st->state_stat);
        PRINTF_ERROR("exec: no frame buffer (pool %u/%u): not reading this cycle\n", (unsigned)buffer_pool_used(st->buffer_pool), (unsigned)buffer_pool_total(st->buffer_pool));
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

bool process_run(process_state_t *st, iotdata_node_state_t *state_store, node_state_t *state_node, mesh_state_t *state_mesh, ddup_state_t *state_ddup, stat_state_t *state_stat, ctrl_state_t *state_ctrl, survey_state_t *state_survey,
                 trace_state_t *state_trace, volatile bool *running) {
    assert(st && state_node && state_mesh && state_ddup && state_stat && state_ctrl && state_survey && running);

    g_exec = st;
    filter_init(&st->filter);
    st->state_node = state_node;
    st->state_mesh = state_mesh;
    st->state_ddup = state_ddup;
    st->state_stat = state_stat;
    st->state_survey = state_survey;
    st->state_trace = state_trace;
    st->buffer_rx = BUFFER_NONE;

    char buf[16];
    PRINTF_INFO("exec: iotdata gateway (stats-display=%" PRIu16 "s, stats-publish=%" PRIu16 "s, topic-prefix=%s) :: mesh=%c%s\n", iotdata_node_config_u16(STAT_DISPLAY_INTERVAL_S), iotdata_node_config_u16(STAT_PUBLISH_INTERVAL_S),
                iotdata_node_config_string(MQTT_TOPIC_PREFIX), st->state_mesh->enabled ? 'y' : 'n', st->state_mesh->enabled ? snprintf_inline(buf, sizeof(buf), ", beacon=%" PRIu16 "s", st->state_mesh->beacon_interval_s) : "");

    for (int i = 0; i < IOTDATA_VARIANT_MAPS_COUNT; i++) {
        const iotdata_variant_def_t *vdef = iotdata_get_variant((uint8_t)i);
        PRINTF_INFO("exec: variant[%d] = \"%s\" (pres_bytes=%" PRIu8 ") -> %s" IOTDATA_NODE_MQTT_VARIANT_TOPIC "/%s/<station>\n", i, vdef->name, vdef->num_pres_bytes, iotdata_node_config_string(MQTT_TOPIC_PREFIX), vdef->name);
    }
    if (st->state_mesh->enabled)
        PRINTF_INFO("exec: variant[15] = mesh control (gateway station=%03" PRIX16 ")\n", iotdata_node_station(st->state_mesh->inode));

    while (*running) {

        const uint32_t now_ms = hw_time_ms();

        // node diagnostics
        if (*running)
            iotdata_node_diagnostics_tick(now_ms);
        // node content
        if (*running)
            gateway_content_tick(now_ms);

        // packet processing
        if (*running)
            process_packet(st, now_ms);

        // control processing
        if (*running)
            mesh_tick(state_mesh, now_ms); // mesh
        if (*running)
            survey_tick(state_survey, now_ms); // mesh survey
        if (*running)
            trace_tick(state_trace, now_ms); // mesh trace
        if (*running)
            node_tick(state_node, now_ms); // node
        if (*running)
            ctrl_tick(state_ctrl, now_ms); // node control
        if (*running)
            iotdata_node_console_tick(now_ms); // node console
        if (*running)
            ddup_hold_release(st->state_ddup, process_release_cb, st); // ddup
        if (*running && st->peers_ddup)
            ddup_peers_tick(st->peers_ddup, hw_clock()); // ddup peers

        // stats publish/display
        if (*running && iotdata_node_config_u16(STAT_PUBLISH_INTERVAL_S) > 0 && intervalable(iotdata_node_config_u16(STAT_PUBLISH_INTERVAL_S), &st->stat_publish_interval_last_ms, now_ms))
            stat_publish(st->state_stat, st->state_mesh, st->state_ddup);
        if (*running && iotdata_node_config_u16(STAT_DISPLAY_INTERVAL_S) > 0 && intervalable(iotdata_node_config_u16(STAT_DISPLAY_INTERVAL_S), &st->stat_display_interval_last_ms, now_ms))
            stat_display(st->state_stat, st->state_mesh, st->state_ddup);

        // network publish/display
        if (*running && iotdata_node_config_u16(STAT_DISPLAY_NETW_INTERVAL_S) > 0 && intervalable(iotdata_node_config_u16(STAT_DISPLAY_NETW_INTERVAL_S), &st->stat_netw_interval_last_ms, now_ms))
            netw_report(&st->network, iotdata_node_station(st->state_mesh->inode));
        if (*running && iotdata_node_config_u16(STAT_PUBLISH_NETW_INTERVAL_S) > 0 && intervalable(iotdata_node_config_u16(STAT_PUBLISH_NETW_INTERVAL_S), &st->stat_netw_publish_last_ms, now_ms))
            netw_publish(&st->network, iotdata_node_station(st->state_mesh->inode), iotdata_node_config_string(STAT_PUBLISH_MQTT_TOPIC_PREFIX), st->_buffer_netw_topic, sizeof(st->_buffer_netw_topic));

        // node state
        if (*running)
            iotdata_node_state_tick(state_store, now_ms);
    }

    if (st->buffer_rx != BUFFER_NONE) {
        buffer_unref(st->buffer_pool, st->buffer_rx);
        st->buffer_rx = BUFFER_NONE;
    }

    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
