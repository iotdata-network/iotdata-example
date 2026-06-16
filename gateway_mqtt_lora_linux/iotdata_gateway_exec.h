
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
    mesh_state_t *mesh_state;
    ddup_state_t *ddup_state;
    stat_state_t *stat_state;
    bool debug;
    bool debug_data;
} process_state_t;

// -----------------------------------------------------------------------------------------------------------------------------------------

bool ddup_check_and_add_handler(void *ctx, uint16_t station_id, uint16_t sequence) {
    return ddup_check_and_add(((process_state_t *)ctx)->ddup_state, station_id, sequence);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

bool ddup_check_sensor_packet(process_state_t *state, uint16_t station_id, uint16_t sequence) {
    if (state->mesh_state->enabled)
        if (!ddup_check_and_add(state->ddup_state, station_id, sequence)) {
            state->mesh_state->stat_duplicates++;
            if (state->debug || state->mesh_state->debug)
                printf("process: mesh direct packet duplicate suppressed (station=0x%04" PRIX16 ", sequence=%" PRIu16 ")\n", station_id, sequence);
            return false;
        }
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void process_sensor_packet(process_state_t *state, const uint8_t *packet_buffer, int packet_length, uint8_t variant_id, uint16_t station_id, uint16_t sequence, const char *topic_prefix, const char *via) {
    const iotdata_variant_def_t *vdef;
    if ((vdef = iotdata_get_variant(variant_id)) == NULL) {
        fprintf(stderr, "process: unknown variant %" PRIu8 " (station=0x%04" PRIX16 ", size=%d)\n", variant_id, station_id, packet_length);
        stat_on_packet_decode_error(state->stat_state, station_id, variant_id);
        return;
    }
    char *json = NULL;
    iotdata_decode_to_json_scratch_t scratch;
    iotdata_status_t rc;
    if ((rc = iotdata_decode_to_json(packet_buffer, (size_t)packet_length, &json, &scratch)) != IOTDATA_OK) {
        fprintf(stderr, "process: decode failed: %s (variant=%" PRIu8 ", station=0x%04" PRIX16 ", size=%d)\n", iotdata_strerror(rc), variant_id, station_id, packet_length);
        stat_on_packet_decode_error(state->stat_state, station_id, variant_id);
        return;
    }
    stat_on_packet_decoded(state->stat_state, station_id, sequence, variant_id, (uint16_t)packet_length, &scratch.dec);
    char topic[255];
    snprintf(topic, sizeof(topic), "%s/%s/%04" PRIX16, topic_prefix, vdef->name, station_id);
    if (!mqtt_send(topic, json, (int)strlen(json))) {
        fprintf(stderr, "process: mqtt send failed (topic=%s, size=%d)\n", topic, (int)strlen(json));
        stat_on_packet_process_error(state->stat_state, station_id, variant_id);
    }
    if (state->debug)
        printf("  -> %s (%d bytes%s%s)\n", topic, (int)strlen(json), via ? " via " : "", via ? via : "");
    free(json);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void process_mesh_packet(process_state_t *state, const uint8_t *packet_buffer, int packet_length, uint8_t variant_id, uint16_t station_id, uint16_t sequence, const char *topic_prefix) {
    (void)variant_id;
    const uint8_t ctrl_type = iotdata_mesh_peek_ctrl_type(packet_buffer, packet_length);
    state->mesh_state->stat_mesh_ctrl_rx++;
    if (state->mesh_state->debug)
        printf("process: mesh rx %s from station=0x%04" PRIX16 ", sequence=%" PRIu16 " (%d bytes)\n", iotdata_mesh_ctrl_name(ctrl_type), station_id, sequence, packet_length);
    switch (ctrl_type) {
    case IOTDATA_MESH_CTRL_FORWARD: {
        const uint8_t *inner;
        int inner_len;
        if (mesh_handle_forward(state->mesh_state, packet_buffer, packet_length, &inner, &inner_len)) {
            uint8_t inner_variant;
            uint16_t inner_station, inner_sequence;
            if (iotdata_peek(inner, (size_t)inner_len, &inner_variant, &inner_station, &inner_sequence) != IOTDATA_OK) {
                fprintf(stderr, "process: mesh FORWARD inner packet peek failed (len=%d)\n", inner_len);
                stat_on_link_rx_drop(state->stat_state);
            } else if (ddup_check_sensor_packet(state, inner_station, inner_sequence))
                process_sensor_packet(state, inner, inner_len, inner_variant, inner_station, inner_sequence, topic_prefix, "mesh");
        }
        break;
    }
    case IOTDATA_MESH_CTRL_BEACON: {
        mesh_handle_beacon(state->mesh_state, packet_buffer, packet_length);
        iotdata_mesh_beacon_t b;
        if (iotdata_mesh_unpack_beacon(packet_buffer, packet_length, &b))
            stat_on_mesh_peer(state->stat_state, b.gateway_id, b.generation, b.cost, b.flags);
        break;
    }
    case IOTDATA_MESH_CTRL_ACK:
        state->mesh_state->stat_acks_rx++;
        if (state->mesh_state->debug)
            printf("process: mesh rx unexpected ACK from station=0x%04" PRIX16 "\n", station_id);
        break;
    case IOTDATA_MESH_CTRL_ROUTE_ERROR:
        mesh_handle_route_error(state->mesh_state, packet_buffer, packet_length);
        break;
    case IOTDATA_MESH_CTRL_NEIGHBOUR_RPT:
        mesh_handle_neighbour_report(state->mesh_state, packet_buffer, packet_length);
        break;
    case IOTDATA_MESH_CTRL_PONG:
        mesh_handle_pong(state->mesh_state, packet_buffer, packet_length);
        break;
    default:
        state->mesh_state->stat_mesh_unknown++;
        if (state->mesh_state->debug)
            printf("process: mesh rx unknown ctrl_type=0x%02" PRIX8 " from station=0x%04" PRIX16 "\n", ctrl_type, station_id);
        break;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

bool process_run(process_state_t *state, mesh_state_t *mesh_state, ddup_state_t *ddup_state, stat_state_t *stat_state, volatile bool *running) {
    uint8_t packet_buffer[E22900T22_PACKET_MAXSIZE + 1]; /* +1 for RSSI byte */
    int packet_length;

    state->mesh_state = mesh_state;
    state->ddup_state = ddup_state;
    state->stat_state = stat_state;

    printf("process: iotdata gateway (stats-display=%" PRIu32 "s, stats-publish=%" PRIu32 "s, rssi=%" PRIu32 "s [packets=%c, channel=%c], topic-prefix=%s", (uint32_t)state->stat_display_interval, (uint32_t)state->stat_publish_interval,
           (uint32_t)state->interval_rssi_channel, state->capture_rssi_packet ? 'y' : 'n', state->capture_rssi_channel ? 'y' : 'n', state->mqtt_topic_prefix);
    if (state->mesh_state->enabled)
        printf(", mesh=on, beacon=%" PRIu32 "s", (uint32_t)state->mesh_state->beacon_interval);
    printf(")\n");

    for (int i = 0; i < IOTDATA_VARIANT_MAPS_COUNT; i++) {
        const iotdata_variant_def_t *vdef = iotdata_get_variant((uint8_t)i);
        printf("process: variant[%d] = \"%s\" (pres_bytes=%" PRIu8 ") -> %s/%s/<station>\n", i, vdef->name, vdef->num_pres_bytes, state->mqtt_topic_prefix, vdef->name);
    }
    if (state->mesh_state->enabled)
        printf("process: variant[15] = mesh control (gateway station=0x%04" PRIX16 ")\n", state->mesh_state->station_id);

    while (*running) {

        // packet processing
        uint8_t packet_rssi = 0, channel_rssi = 0;
        if (device_packet_read(packet_buffer, sizeof(packet_buffer), &packet_length, &packet_rssi) && running) {
            stat_on_link_rx_packet(state->stat_state, (uint16_t)packet_length);
            if (state->debug_data)
                debug_hexdump("data: ", packet_buffer, (size_t)packet_length);
            if (state->capture_rssi_packet) {
                if (packet_rssi > 0)
                    stat_on_link_rssi_packet(state->stat_state, packet_rssi);
                else
                    stat_on_link_rssi_packet_error(state->stat_state);
            }
            uint8_t variant_id;
            uint16_t station_id, sequence;
            if (iotdata_peek(packet_buffer, (size_t)packet_length, &variant_id, &station_id, &sequence) != IOTDATA_OK) {
                fprintf(stderr, "process: packet too short for iotdata header (size=%d)\n", packet_length);
                stat_on_link_rx_drop(state->stat_state);
            } else if (variant_id == IOTDATA_MESH_VARIANT) {
                if (!state->mesh_state->enabled) {
                    stat_on_link_rx_mesh_unexpected(state->stat_state, station_id);
                    printf("process: mesh packet unexpected from station=0x%04" PRIX16 " while not enabled\n", station_id);
                } else
                    process_mesh_packet(state, packet_buffer, packet_length, variant_id, station_id, sequence, state->mqtt_topic_prefix);
            } else {
                process_sensor_packet(state, packet_buffer, packet_length, variant_id, station_id, sequence, state->mqtt_topic_prefix, "direct");
            }
        }

        // rssi update
        if (*running && state->capture_rssi_channel && intervalable(state->interval_rssi_channel, &state->interval_rssi_channel_last)) {
            if (device_channel_rssi_read(&channel_rssi))
                stat_on_link_rssi_channel(state->stat_state, channel_rssi);
            else
                stat_on_link_rssi_channel_error(state->stat_state);
        }

        // mesh beacons
        if (*running && state->mesh_state->enabled && intervalable(state->mesh_state->beacon_interval, &state->mesh_state->beacon_last))
            mesh_beacon_send(state->mesh_state);

        // stats publish/display
        if (*running && state->stat_publish_interval > 0 && intervalable(state->stat_publish_interval, &state->stat_publish_interval_last) > 0)
            stat_publish(state->stat_state, state->mesh_state, state->ddup_state);
        if (*running && state->stat_display_interval > 0 && intervalable(state->stat_display_interval, &state->stat_display_interval_last) > 0)
            stat_display(state->stat_state, state->mesh_state, state->ddup_state);
    }

    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
