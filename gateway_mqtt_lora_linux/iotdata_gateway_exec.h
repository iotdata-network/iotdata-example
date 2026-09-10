
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

typedef struct {
    const char *mqtt_topic_prefix;
    bool capture_rssi_packet;
    bool capture_rssi_channel;
    time_t interval_rssi_channel;
    time_t interval_rssi_channel_last;
    time_t stat_display_interval;
    time_t stat_display_interval_last;
    time_t stat_publish_interval;
    time_t stat_publish_interval_last;
    time_t stat_netw_interval;
    time_t stat_netw_interval_last;
    netw_t network; /* stations heard (mesh + sensor), printed every stat-network-interval */
    node_state_t *state_node;
    mesh_state_t *state_mesh;
    ddup_state_t *state_ddup;
    stat_state_t *state_stat;
    char _buffer_mqtt_topic[256], _buffer_mqtt_message[1024];
    iotdata_decode_to_json_scratch_t _iotdata_scratch;
    uint8_t _buffer_packet[LORA_PACKET_SIZE_MAX + 1]; /* +1 for RSSI byte */
    bool debug;
    bool debug_data;
} process_state_t;

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
                if (inner_ok)
                    process_sensor_packet(st, fw.inner_packet, fw.inner_len, inner_variant, inner_station, inner_sequence, topic_prefix, "mesh", packet_rssi);
                else {
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
    case IOTDATA_MESH_CTRL_MANAGE:
        /* assigned types this gateway does not act on -- MANAGE it originates rather than receives.
           Counted as themselves, so "seen but not handled" is distinguishable from "unrecognised". */
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

bool process_run(process_state_t *st, node_state_t *state_node, mesh_state_t *state_mesh, ddup_state_t *state_ddup, stat_state_t *state_stat, ctrl_state_t *state_ctrl, volatile bool *running) {
    assert(st && state_node && state_mesh && state_ddup && state_stat && state_ctrl && running);

    st->state_node = state_node;
    st->state_mesh = state_mesh;
    st->state_ddup = state_ddup;
    st->state_stat = state_stat;

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

/* First-byte wait in the receive poll; the driver then delimits the frame with a short inter-byte
   gap. Was the lora-read-timeout-packet config key. */
#ifndef LORA_READ_TIMEOUT_MS
#define LORA_READ_TIMEOUT_MS 5000
#endif

        // packet processing
        int packet_length;
        uint8_t packet_rssi = 0, channel_rssi = 0;
        /* lora_read strips the trailing RSSI byte and reports it in dBm; the stat layer stores the
           raw byte, so convert back. A 0 dBm reading means "none reported", as before. */
        int packet_rssi_dbm = 0;
        const bool got = lora_read(st->_buffer_packet, sizeof(st->_buffer_packet), &packet_length, &packet_rssi_dbm, LORA_READ_TIMEOUT_MS) == ESP_OK && packet_length > 0;
        packet_rssi = packet_rssi_dbm != 0 ? rssi_raw_from_dbm(packet_rssi_dbm) : 0;
        if (got && running) {
            stat_on_link_rx_packet(st->state_stat, (uint16_t)packet_length);
            if (st->debug_data)
                debug_hexdump("data: ", st->_buffer_packet, (size_t)packet_length);
            if (st->capture_rssi_packet) {
                if (packet_rssi > 0)
                    stat_on_link_rssi_packet(st->state_stat, packet_rssi);
                else
                    stat_on_link_rssi_packet_error(st->state_stat);
            }
            uint8_t variant_id;
            uint16_t station_id, sequence;
            if (iotdata_peek(st->_buffer_packet, (size_t)packet_length, &variant_id, &station_id, &sequence) != IOTDATA_OK) {
                PRINTF_ERROR("exec: packet too short for iotdata header (size=%d)\n", packet_length);
                stat_on_link_rx_drop(st->state_stat);
            } else if (variant_id == IOTDATA_MESH_VARIANT) {
                if (st->state_mesh->enabled)
                    process_mesh_packet(st, st->_buffer_packet, packet_length, variant_id, station_id, sequence, st->mqtt_topic_prefix, packet_rssi);
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
                (void)node_on_packet(st->state_node, st->_buffer_packet, (size_t)packet_length, station_id);
                const bool is_new = ddup_check_sensor_packet(st, station_id, sequence);
                netw_note_receive(&st->network, station_id, variant_id, sequence, NETW_PATH_DIRECT, is_new, (st->capture_rssi_packet && packet_rssi > 0) ? get_rssi_dbm(packet_rssi) : 0, 0, time(NULL));
                if (is_new)
                    process_sensor_packet(st, st->_buffer_packet, packet_length, variant_id, station_id, sequence, st->mqtt_topic_prefix, "direct", packet_rssi);
            }
        }

        // rssi update
        if (*running && st->capture_rssi_channel && intervalable_and_initial(st->interval_rssi_channel, &st->interval_rssi_channel_last)) {
            int channel_rssi_dbm = 0;
            if (lora_read_channel_rssi(&channel_rssi_dbm) == ESP_OK)
                stat_on_link_rssi_channel(st->state_stat, (channel_rssi = rssi_raw_from_dbm(channel_rssi_dbm)));
            else
                stat_on_link_rssi_channel_error(st->state_stat);
        }

        // mesh beacons
        if (*running && st->state_mesh->enabled && intervalable_and_initial(st->state_mesh->beacon_interval, &st->state_mesh->beacon_last))
            mesh_transmit_beacon(st->state_mesh);

        // control
        if (*running) {
            ctrl_tick(state_ctrl, state_node);
            node_tick(state_node);
        }

        // stats publish/display
        if (*running && st->stat_publish_interval > 0 && intervalable(st->stat_publish_interval, &st->stat_publish_interval_last) > 0)
            stat_publish(st->state_stat, st->state_mesh, st->state_ddup);
        if (*running && st->stat_display_interval > 0 && intervalable(st->stat_display_interval, &st->stat_display_interval_last) > 0)
            stat_display(st->state_stat, st->state_mesh, st->state_ddup);

        // network / stations table
        if (*running && st->stat_netw_interval > 0 && intervalable(st->stat_netw_interval, &st->stat_netw_interval_last) > 0)
            netw_report(&st->network, st->state_mesh->station_id);
    }

    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
