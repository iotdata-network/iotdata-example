
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#ifndef LORA_READ_TIMEOUT_MS
#define LORA_READ_TIMEOUT_MS 5000
#endif

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

bool ddup_insert_handler(void *ctx, uint16_t station_id, uint16_t sequence) {
    return ddup_insert(((process_state_t *)ctx)->state_ddup, station_id, sequence);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/*
 * Gated on mesh only, NOT on ddup being enabled.
 *
 * ddup_insert() always does the local dedup-ring insert and consults `enabled` solely to decide
 * whether to also queue the entry for cross-gateway broadcast -- so calling it with ddup off is
 * both safe and necessary. Requiring state_ddup->enabled here meant that with the default
 * ddup-enable=false a DIRECT reception was never recorded, while the FORWARD path (which calls the
 * same dedup handler unconditionally) then looked the origin up, missed, and published the same
 * reading a second time. Observed: one sensor heard directly and via a relay, published twice to
 * the same topic, "via direct" then "via mesh".
 */
bool ddup_check_sensor_packet(process_state_t *st, uint16_t station_id, uint16_t sequence) {
    if (st->state_mesh->enabled)
        if (!ddup_insert(st->state_ddup, station_id, sequence)) {
            st->state_mesh->stat_duplicates++;
            if (st->debug || st->state_mesh->debug)
                PRINTF_INFO("exec: mesh direct packet duplicate suppressed (station=%04" PRIX16 ", sequence=%" PRIu16 ")\n", station_id, sequence);
            return false;
        }
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

// packet_rssi is the raw byte the radio appends to each received packet; 0 means the radio did
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

void process_sensor_packet(process_state_t *st, const uint8_t *packet_buffer, int packet_length, uint8_t variant_id, uint16_t station_id, uint16_t sequence, const char *topic_prefix, const char *via, uint8_t packet_rssi) {
    // Network-table tracking (per-path counts, dups, sequence gaps) is done by the caller at the
    // dedup point (netw_note_receive), so it also sees suppressed duplicates — which never reach here.
    const iotdata_variant_def_t *vdef;
    if ((vdef = iotdata_get_variant(variant_id)) == NULL) {
        PRINTF_ERROR("exec: unknown variant %" PRIu8 " (station=%04" PRIX16 ", size=%d)\n", variant_id, station_id, packet_length);
        stat_on_packet_decode_error(st->state_stat, station_id, variant_id);
        return;
    }
    char *json = NULL;
    iotdata_status_t rc;
    if ((rc = iotdata_decode_to_json(packet_buffer, (size_t)packet_length, &json, &st->_iotdata_scratch)) != IOTDATA_OK) {
        PRINTF_ERROR("exec: decode failed: %s (variant=%" PRIu8 ", station=%04" PRIX16 ", size=%d)\n", iotdata_strerror(rc), variant_id, station_id, packet_length);
        stat_on_packet_decode_error(st->state_stat, station_id, variant_id);
        return;
    }
    stat_on_packet_decoded(st->state_stat, station_id, sequence, variant_id, (uint16_t)packet_length, &st->_iotdata_scratch.dec);
    snprintf(st->_buffer_mqtt_topic, sizeof(st->_buffer_mqtt_topic), "%s/%s/%04" PRIX16, topic_prefix, vdef->name, station_id);
    // Splice rssi_packet in as the first member rather than teaching the decoder about it: the
    // decoder emits the packet's own payload, and reception strength is not part of that payload.
    // Keeping it out of the codec means every variant gains the field for free and none of them
    // has to carry a field that only a gateway can know.
    const char *payload = json;
    if (st->capture_rssi_packet && packet_rssi > 0 && json[0] == '{') {
        const int n = snprintf(st->_buffer_mqtt_message, sizeof(st->_buffer_mqtt_message), "{\"rssi_packet\":%d,%s", get_rssi_dbm(packet_rssi), json + 1);
        if (n > 0 && n < (int)sizeof(st->_buffer_mqtt_message))
            payload = st->_buffer_mqtt_message;
        else
            PRINTF_ERROR("exec: rssi splice would truncate (%d bytes), publishing without it\n", n);
    }
    if (!mqtt_send(st->_buffer_mqtt_topic, payload, (int)strlen(payload))) {
        PRINTF_ERROR("exec: mqtt send failed (topic=%s, size=%d)\n", st->_buffer_mqtt_topic, (int)strlen(payload));
        stat_on_packet_process_error(st->state_stat, station_id, variant_id);
    }
    if (st->debug)
        PRINTF_INFO("      -> %s (%d bytes%s%s)\n", st->_buffer_mqtt_topic, (int)strlen(payload), via ? " via " : "", via ? via : "");
    free(json);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

void process_mesh_packet(process_state_t *st, const uint8_t *packet_buffer, int packet_length, __attribute__((unused)) uint8_t variant_id, uint16_t station_id, uint16_t sequence, const char *topic_prefix, uint8_t packet_rssi) {
    const time_t now = time(NULL);
    const uint8_t ctrl_type = iotdata_mesh_peek_ctrl_type(packet_buffer, packet_length);
    if (st->state_mesh->debug)
        PRINTF_INFO("exec: mesh rx %s from station=%04" PRIX16 ", sequence=%" PRIu16 " (%d bytes)\n", iotdata_mesh_ctrl_name(ctrl_type), station_id, sequence, packet_length);
    netw_note_transmit(&st->network, station_id, sequence, now);
    switch (ctrl_type) {
    case IOTDATA_MESH_CTRL_FORWARD: {
        iotdata_mesh_forward_t fw;
        if (mesh_receive_forward_r(st->state_mesh, packet_buffer, packet_length, &fw)) {
            const bool is_new = (st->state_mesh->dedup_handler == NULL || st->state_mesh->dedup_handler(st->state_mesh->dedup_handler_ctx, fw.origin_station, fw.origin_sequence));
            if (is_new)
                st->state_mesh->stat_forwards_unwrapped++;
            else {
                st->state_mesh->stat_duplicates++;
                if (st->state_mesh->debug)
                    PRINTF_INFO("exec: mesh FORWARD duplicate suppressed origin={station=%04" PRIX16 ", sequence=%" PRIu16 "}, inner-length=%d\n", fw.origin_station, fw.origin_sequence, fw.inner_len);
            }
            if (st->state_mesh->enabled)
                mesh_transmit_ack(st->state_mesh, fw.origin_station, fw.origin_sequence);
            netw_note_forward(&st->network, fw.sender_station, now); /* the relay's forwarding load */
            uint8_t inner_variant = 0;
            uint16_t inner_station = 0, inner_sequence = 0;
            const bool inner_ok = (iotdata_peek(fw.inner_packet, (size_t)fw.inner_len, &inner_variant, &inner_station, &inner_sequence) == IOTDATA_OK);
            if (inner_ok) /* no rssi: that hop is relay->gateway, not sensor->gateway */
                netw_note_receive(&st->network, fw.origin_station, inner_variant, fw.origin_sequence, NETW_PATH_MESH, is_new, 0, fw.sender_station, now);
            if (is_new) {
                if (inner_ok) {
                    /* A forwarded packet carries system TLVs exactly as a direct one does -- a node
                       two hops out can only ever be heard this way, so without this its reports are
                       handed to the telemetry path alone and never reach the node topic. Addressed
                       by the ORIGIN, not the relay that carried it. */
                    (void)node_on_packet(st->state_node, fw.inner_packet, (size_t)fw.inner_len, fw.origin_station);
                    process_sensor_packet(st, fw.inner_packet, fw.inner_len, inner_variant, inner_station, inner_sequence, topic_prefix, "mesh", packet_rssi);
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
            netw_note_beacon(&st->network, station_id, b.flags, b.cost, b.generation, b.gateway_id, (st->capture_rssi_packet && packet_rssi > 0) ? get_rssi_dbm(packet_rssi) : 0, now);
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
        if (mesh_receive_neighbour_report_r(st->state_mesh, packet_buffer, packet_length, &r)) /* mesh-layer log */
            netw_note_neighbour_report(&st->network, packet_buffer, packet_length, &r, now);   /* relay hears[] + both-ends RSSI */
        break;
    }
    case IOTDATA_MESH_CTRL_PONG:
        (void)mesh_receive_pong(st->state_mesh, packet_buffer, packet_length);
        break;
    case IOTDATA_MESH_CTRL_PING:
        mesh_stat_frame(st->state_mesh, ctrl_type, packet_length, true);
        if (st->state_mesh->debug)
            PRINTF_INFO("exec: mesh rx %s from station=%04" PRIX16 " (not handled here)\n", iotdata_mesh_ctrl_name(ctrl_type), station_id);
        break;
    default:
        mesh_stat_frame(st->state_mesh, ctrl_type, packet_length, false);
        if (st->state_mesh->debug)
            PRINTF_INFO("exec: mesh rx unknown ctrl_type=0x%02" PRIX8 " from station=%04" PRIX16 "\n", ctrl_type, station_id);
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
    if (st == NULL || !st->state_mesh->enabled)
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

static uint8_t exec_node_table_count(const uint8_t type) {
    const process_state_t *const st = g_exec;
    if (st == NULL)
        return 0;
    switch (type) {
    case IOTDATA_NODE_TLV_MESH_STATIONS:
        return (uint8_t)st->network.count;
    case IOTDATA_NODE_TLV_MESH_PEERS:
        return (uint8_t)st->state_stat->peers_count;
    case IOTDATA_NODE_TLV_MESH_FILTERS:
        return (uint8_t)filter_count(&st->filter);
    default:
        return 0;
    }
}

/* Map a dense report index onto a sparse slot: every one of these tables leaves holes. */
#define EXEC_TABLE_NTH(arr, idx, out) \
    do { \
        int _c = 0; \
        (out) = -1; \
        for (int _i = 0; _i < (int)((sizeof(arr)) / sizeof((arr)[0])); _i++) \
            if ((arr)[_i].valid && _c++ == (int)(idx)) { \
                (out) = _i; \
                break; \
            } \
    } while (0)

static bool exec_node_table_row(const uint8_t type, const uint8_t index, uint8_t *const row) {
    process_state_t *const st = g_exec;
    if (st == NULL)
        return false;
    const time_t now = time(NULL);
    int slot = -1;
    switch (type) {
    case IOTDATA_NODE_TLV_MESH_STATIONS: {
        EXEC_TABLE_NTH(st->network.s, index, slot);
        if (slot < 0)
            return false;
        const netw_station_t *const e = &st->network.s[slot];
        iotdata_node_table_put_u16(row, 0, e->station);
        row[2] = (e->kind == NETW_KIND_GATEWAY)  ? IOTDATA_NODE_TABLE_KIND_GATEWAY
                 : (e->kind == NETW_KIND_RELAY)  ? IOTDATA_NODE_TABLE_KIND_RELAY
                 : (e->kind == NETW_KIND_SENSOR) ? IOTDATA_NODE_TABLE_KIND_SENSOR
                                                 : IOTDATA_NODE_TABLE_KIND_UNKNOWN;
        row[3] = e->variant;
        row[4] = (uint8_t)(int8_t)e->rssi;
        iotdata_node_table_put_u16(row, 5, (uint16_t)(now > e->last_seen ? (now - e->last_seen) : 0));
        iotdata_node_table_put_u32(row, 7, e->rx_count);
        return true;
    }
    case IOTDATA_NODE_TLV_MESH_PEERS: {
        EXEC_TABLE_NTH(st->state_stat->peers, index, slot);
        if (slot < 0)
            return false;
        const stat_peer_t *const e = &st->state_stat->peers[slot];
        iotdata_node_table_put_u16(row, 0, e->station_id);
        iotdata_node_table_put_u16(row, 2, st->state_mesh->station_id); /* we ARE the gateway */
        row[4] = e->cost;
        iotdata_node_table_put_u16(row, 5, e->generation);
        row[7] = 0; /* this peer table is built from beacons, which carry no RSSI of their own */
        iotdata_node_table_put_u16(row, 8, (uint16_t)(now > e->last_seen ? (now - e->last_seen) : 0));
        /* the root has no parent, so only ACCEPTING can be true of a peer here */
        row[10] = (uint8_t)((e->flags & IOTDATA_MESH_FLAG_ACCEPTING) ? IOTDATA_NODE_TABLE_PEER_ACCEPTING : 0);
        return true;
    }
    case IOTDATA_NODE_TLV_MESH_FILTERS: {
        EXEC_TABLE_NTH(st->filter.e, index, slot);
        if (slot < 0)
            return false;
        const filter_entry_t *const e = &st->filter.e[slot];
        iotdata_node_table_put_u16(row, 0, e->station);
        row[2] = (e->action == FILTER_ALLOW) ? IOTDATA_NODE_CONTROL_MESH_FILTERS_ALLOW : IOTDATA_NODE_CONTROL_MESH_FILTERS_BLOCK;
        row[3] = (e->source == FILTER_AUTO) ? IOTDATA_NODE_CONTROL_MESH_FILTERS_SCOPE_AUTO : IOTDATA_NODE_CONTROL_MESH_FILTERS_SCOPE_MANUAL;
        return true;
    }
    default:
        return false;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

bool process_run(process_state_t *st, node_state_t *state_node, mesh_state_t *state_mesh, ddup_state_t *state_ddup, stat_state_t *state_stat, ctrl_state_t *state_ctrl, volatile bool *running) {
    assert(st && state_node && state_mesh && state_ddup && state_stat && state_ctrl && running);

    g_exec = st;
    filter_init(&st->filter);
    st->state_node = state_node;
    st->state_mesh = state_mesh;
    st->state_ddup = state_ddup;
    st->state_stat = state_stat;
    st->rx_held = BUFFER_NONE;

    char buf[16];
    PRINTF_INFO("exec: iotdata gateway (stats-display=%" PRIu32 "s, stats-publish=%" PRIu32 "s, rssi=%" PRIu32 "s [packets=%c, channel=%c], topic-prefix=%s) :: mesh=%c%s\n", (uint32_t)st->stat_display_interval,
                (uint32_t)st->stat_publish_interval, (uint32_t)st->interval_rssi_channel, st->capture_rssi_packet ? 'y' : 'n', st->capture_rssi_channel ? 'y' : 'n', st->mqtt_topic_prefix, st->state_mesh->enabled ? 'y' : 'n',
                st->state_mesh->enabled ? snprintf_inline(buf, sizeof(buf), ", beacon=%" PRIu32 "s", (uint32_t)st->state_mesh->beacon_interval) : "");

    for (int i = 0; i < IOTDATA_VARIANT_MAPS_COUNT; i++) {
        const iotdata_variant_def_t *vdef = iotdata_get_variant((uint8_t)i);
        PRINTF_INFO("exec: variant[%d] = \"%s\" (pres_bytes=%" PRIu8 ") -> %s/%s/<station>\n", i, vdef->name, vdef->num_pres_bytes, st->mqtt_topic_prefix, vdef->name);
    }
    if (st->state_mesh->enabled)
        PRINTF_INFO("exec: variant[15] = mesh control (gateway station=%04" PRIX16 ")\n", st->state_mesh->station_id);

    while (*running) {

        // packet processing
        if (st->rx_held == BUFFER_NONE)
            st->rx_held = buffer_acquire(st->pool);
        if (st->rx_held != BUFFER_NONE) {
            (void)buffer_reset(st->pool, st->rx_held);
            int packet_rssi_dbm = 0, packet_length;
            uint8_t *rxbuf = buffer_data(st->pool, st->rx_held);
            if (lora_read(rxbuf, buffer_room(st->pool, st->rx_held), &packet_length, &packet_rssi_dbm, LORA_READ_TIMEOUT_MS) == ESP_OK && packet_length > 0) {
                buffer_set_len(st->pool, st->rx_held, (uint16_t)packet_length);
                const uint8_t packet_rssi = packet_rssi_dbm != 0 ? rssi_raw_from_dbm(packet_rssi_dbm) : 0;
                stat_on_link_rx_packet(st->state_stat, (uint16_t)packet_length);
                if (st->debug_data)
                    debug_hexdump("data: ", rxbuf, (size_t)packet_length);
                if (st->capture_rssi_packet) {
                    if (packet_rssi > 0)
                        stat_on_link_rssi_packet(st->state_stat, packet_rssi);
                    else
                        stat_on_link_rssi_packet_error(st->state_stat);
                }
                uint8_t variant_id;
                uint16_t station_id, sequence;
                if (iotdata_peek(rxbuf, (size_t)packet_length, &variant_id, &station_id, &sequence) != IOTDATA_OK) {
                    PRINTF_ERROR("exec: packet too short for iotdata header (size=%d)\n", packet_length);
                    stat_on_link_rx_drop(st->state_stat);
                } else if (!filter_allows(&st->filter, station_id)) {
                    st->filter.stat_blocked++;
                    stat_on_link_rx_drop(st->state_stat);
                    if (st->debug)
                        PRINTF_INFO("exec: FILTERED from=%04" PRIX16 " var=%u len=%d (blocked)\n", station_id, (unsigned)variant_id, packet_length);
                } else if (variant_id == IOTDATA_MESH_VARIANT) {
                    if (st->state_mesh->enabled)
                        process_mesh_packet(st, rxbuf, packet_length, variant_id, station_id, sequence, st->mqtt_topic_prefix, packet_rssi);
                    else {
                        stat_on_link_rx_mesh_unexpected(st->state_stat, station_id);
                        PRINTF_INFO("exec: mesh packet unexpected from station=%04" PRIX16 " while not enabled\n", station_id);
                    }
                } else {
                    // Dedup direct receptions against the SAME ring the forward path uses, so a sensor
                    // heard both directly and via a relay publishes once — whichever path adds {station,
                    // seq} first wins, the other is suppressed. is_new is true unless it was a duplicate
                    // (always true when mesh is off). Track the reception either way for observability.
                    /* a packet may carry system TLVs (node reports) as well as telemetry */
                    (void)node_on_packet(st->state_node, rxbuf, (size_t)packet_length, station_id);
                    if (process_packet_decodes(st, rxbuf, packet_length)) {
                        const bool is_new = ddup_check_sensor_packet(st, station_id, sequence);
                        netw_note_receive(&st->network, station_id, variant_id, sequence, NETW_PATH_DIRECT, is_new, (st->capture_rssi_packet && packet_rssi > 0) ? get_rssi_dbm(packet_rssi) : 0, 0, time(NULL));
                        if (is_new)
                            process_sensor_packet(st, rxbuf, packet_length, variant_id, station_id, sequence, st->mqtt_topic_prefix, "direct", packet_rssi);
                    } else {
                        stat_on_link_rx_drop(st->state_stat);
                        PRINTF_ERROR("exec: undecodable frame from station=%04" PRIX16 ", sequence=%" PRIu16 " (%d bytes): %s -- dropped before dedup, so a relayed copy can still be used\n", station_id, sequence, packet_length,
                                     iotdata_strerror(iotdata_decode(rxbuf, (size_t)packet_length, &st->_iotdata_dec)));
                    }
                }
                if (st->rx_held != BUFFER_NONE && buffer_refs(st->pool, st->rx_held) > 1) {
                    buffer_unref(st->pool, st->rx_held);
                    st->rx_held = BUFFER_NONE;
                }
            }
        } else {
            stat_on_link_rx_drop(st->state_stat);
            PRINTF_ERROR("exec: no frame buffer (pool %u/%u): not reading this cycle\n", (unsigned)buffer_pool_used(st->pool), (unsigned)buffer_pool_total(st->pool));
        }

        // rssi update
        if (*running && st->capture_rssi_channel && intervalable_and_initial(st->interval_rssi_channel, &st->interval_rssi_channel_last)) {
            int channel_rssi_dbm;
            if (lora_read_channel_rssi(&channel_rssi_dbm) == ESP_OK)
                stat_on_link_rssi_channel(st->state_stat, rssi_raw_from_dbm(channel_rssi_dbm));
            else
                stat_on_link_rssi_channel_error(st->state_stat);
        }

        // mesh beacons
        if (*running && st->state_mesh->enabled && intervalable_and_initial(st->state_mesh->beacon_interval, &st->state_mesh->beacon_last))
            mesh_transmit_beacon(st->state_mesh);

        // control
        if (*running)
            ctrl_tick(state_ctrl, state_node);
        if (*running)
            node_tick(state_node);

        // stats publish/display
        if (*running && st->stat_publish_interval > 0 && intervalable(st->stat_publish_interval, &st->stat_publish_interval_last) > 0)
            stat_publish(st->state_stat, st->state_mesh, st->state_ddup);
        if (*running && st->stat_display_interval > 0 && intervalable(st->stat_display_interval, &st->stat_display_interval_last) > 0)
            stat_display(st->state_stat, st->state_mesh, st->state_ddup);

        // network / stations table
        if (*running && st->stat_netw_interval > 0 && intervalable(st->stat_netw_interval, &st->stat_netw_interval_last) > 0)
            netw_report(&st->network, st->state_mesh->station_id);
    }

    if (st->rx_held != BUFFER_NONE) {
        buffer_unref(st->pool, st->rx_held);
        st->rx_held = BUFFER_NONE;
    }

    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
