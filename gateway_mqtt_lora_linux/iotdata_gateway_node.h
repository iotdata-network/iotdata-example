
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
//
// iotdata_gateway_node.h - the gateway's NODE personality: the system TLVs of iotdata_node.h.
//
// Every iotdata node (gateway, relay, sensor) answers the same system TLVs. This module is the
// gateway's implementation of that: it owns everything needed to BE a node, so the rest of the
// gateway does not have to know about it.
//
//   - builds the reports about ourselves: VERSION, VARIANT, STATUS, CONFIG, CONTROL, DIAGNOSTICS
//   - executes CONTROL commands addressed to us (or broadcast), in wire order, skipping any key
//     we do not implement
//   - owns the periodic/startup emission timers (the CONFIG period_* keys)
//
// TWO ENTRY POINTS, one body of logic:
//
//   gwnode_on_packet()  an iotdata packet arrived over the radio for us or broadcast
//   gwnode_on_mqtt()    an MQTT management request arrived for us or broadcast
//
// Both funnel into gwnode_process_tlvs(). A reply is published to MQTT as JSON (so a manager sees
// it regardless of which way it asked) and, when the request came off the radio, also transmitted
// back to the asker.
//
// Encoding is kvr throughout (see iotdata_node.h): binary values in a RAW TLV.
//
// -----------------------------------------------------------------------------------------------------------------------------------------

#ifndef IOTDATA_GATEWAY_NODE_H
#define IOTDATA_GATEWAY_NODE_H

#define GWNODE_KV_MAX      200 /* a kvr payload we build; a TLV caps at 255 anyway */
#define GWNODE_PACKET_MAX  240
#define GWNODE_TYPE_COUNT  8 /* types are 0x00..0x07; index by type for per-type state */
#define GWNODE_JSON_MAX    1024

typedef bool (*gwnode_tx_handler_t)(const uint8_t *packet, const int length);

typedef struct {
    uint16_t station_id; /* our own station: a request is "for us" if it matches, or is broadcast */
    uint16_t sequence;   /* our packet sequence for reports we originate */

    /* CONFIG: reporting cadence, indexed by TLV type. 0 = do not send periodically. */
    uint16_t period[GWNODE_TYPE_COUNT];
    time_t period_last[GWNODE_TYPE_COUNT];
    uint16_t startup; /* IOTDATA_NODE_STARTUP_* bitmask, emitted once when we come up */
    bool startup_done;

    /* what we report ON: borrowed, not owned */
    const char *version;
    const stat_state_t *stat;
    blackbox_handle_t *blackbox;

    /* how we answer */
    gwnode_tx_handler_t tx; /* radio; may be NULL */
    char topic_resp[128];   /* mqtt */

    /* down frames we are holding for nodes that were not listening when we sent them */
    iotdata_down_t down;

    /* stats */
    uint32_t stat_rx, stat_requests, stat_reports, stat_commands, stat_unknown;
} gwnode_state_t;

static gwnode_state_t *g_gwnode = NULL;

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static int gwnode_build_version(const gwnode_state_t *st, uint8_t *buf, const size_t size) {
    iotdata_kvr_t kv;
    iotdata_kvr_init(&kv, buf, size);
    iotdata_kvr_add_str(&kv, IOTDATA_NODE_VERSION_FIRMWARE, st->version ? st->version : "0");
    iotdata_kvr_add_str(&kv, IOTDATA_NODE_VERSION_APPLICATION, "iotdata_gateway");
    iotdata_kvr_add_str(&kv, IOTDATA_NODE_VERSION_PLATFORM, "linux");
    iotdata_kvr_add_str(&kv, IOTDATA_NODE_VERSION_BUILD, __DATE__);
    return kv.overflow ? -1 : (int)kv.len;
}

/* VARIANT is one key per variant produced, keyed by variant number. A gateway originates no
   telemetry, so it has none, and says so with an EMPTY TLV rather than by staying silent -- the
   mesh variant it speaks is not a telemetry variant and does not belong here. A node that DOES
   produce telemetry sends the manifest plus as many variants as fit, resuming from a cursor on the
   next call -- a whole suite is ~240 bytes and will not fit one frame:

       iotdata_kvr_add_u16(&kv, IOTDATA_NODE_VARIANT_MANIFEST, iotdata_node_variant_manifest());
       for (; *cursor <= IOTDATA_VARIANT_MAX; (*cursor)++) {
           const iotdata_variant_def_t *const d = iotdata_get_variant((uint8_t)*cursor);
           uint8_t val[GWNODE_KV_MAX];
           const size_t n = d ? iotdata_node_variant_encode(d, val, sizeof(val)) : 0;
           if (n == 0) continue;
           if (kv.len + 2u + n > size) break;   // leave it for the next packet
           iotdata_kvr_add(&kv, (uint8_t)*cursor, val, (uint8_t)n);
       }

   which is left until the field ids are globally assigned -- until then they mean nothing to a
   receiver built separately. */
static int gwnode_build_variant(__attribute__ ((unused)) const gwnode_state_t *st, uint8_t *buf, const size_t size) {
    iotdata_kvr_t kv;
    iotdata_kvr_init(&kv, buf, size);
    return kv.overflow ? -1 : (int)kv.len;
}

static int gwnode_build_status(const gwnode_state_t *st, uint8_t *buf, const size_t size) {
    iotdata_kvr_t kv;
    iotdata_kvr_init(&kv, buf, size);
    const time_t now = time(NULL);
    const uint32_t uptime = (st->stat != NULL && st->stat->start_time > 0) ? (uint32_t)(now - st->stat->start_time) : 0;
    iotdata_kvr_add_u32(&kv, IOTDATA_NODE_STATUS_UPTIME, uptime);
    iotdata_kvr_add_u8(&kv, IOTDATA_NODE_STATUS_REASON, IOTDATA_NODE_REASON_UNKNOWN);
    return kv.overflow ? -1 : (int)kv.len;
}

static int gwnode_build_config(const gwnode_state_t *st, uint8_t *buf, const size_t size) {
    iotdata_kvr_t kv;
    iotdata_kvr_init(&kv, buf, size);
    iotdata_kvr_add_u16(&kv, IOTDATA_NODE_CONFIG_PERIOD_VERSION, st->period[IOTDATA_NODE_TLV_VERSION]);
    iotdata_kvr_add_u16(&kv, IOTDATA_NODE_CONFIG_PERIOD_VARIANT, st->period[IOTDATA_NODE_TLV_VARIANT]);
    iotdata_kvr_add_u16(&kv, IOTDATA_NODE_CONFIG_PERIOD_CONTROL, st->period[IOTDATA_NODE_TLV_CONTROL]);
    iotdata_kvr_add_u16(&kv, IOTDATA_NODE_CONFIG_PERIOD_STATUS, st->period[IOTDATA_NODE_TLV_STATUS]);
    iotdata_kvr_add_u16(&kv, IOTDATA_NODE_CONFIG_PERIOD_CONFIG, st->period[IOTDATA_NODE_TLV_CONFIG]);
    iotdata_kvr_add_u16(&kv, IOTDATA_NODE_CONFIG_PERIOD_DIAGNOSTICS, st->period[IOTDATA_NODE_TLV_DIAGNOSTICS]);
    iotdata_kvr_add_u16(&kv, IOTDATA_NODE_CONFIG_STARTUP, st->startup);
    return kv.overflow ? -1 : (int)kv.len;
}

static int gwnode_build_control(__attribute__ ((unused)) const gwnode_state_t *st, uint8_t *buf, const size_t size) {
    iotdata_kvr_t kv;
    iotdata_kvr_init(&kv, buf, size);
    iotdata_kvr_add_flag(&kv, IOTDATA_NODE_CONTROL_REQUEST_VERSION);
    iotdata_kvr_add_flag(&kv, IOTDATA_NODE_CONTROL_REQUEST_VARIANT);
    iotdata_kvr_add_flag(&kv, IOTDATA_NODE_CONTROL_REQUEST_CONTROL);
    iotdata_kvr_add_flag(&kv, IOTDATA_NODE_CONTROL_REQUEST_STATUS);
    iotdata_kvr_add_flag(&kv, IOTDATA_NODE_CONTROL_REQUEST_CONFIG);
    iotdata_kvr_add_flag(&kv, IOTDATA_NODE_CONTROL_REQUEST_DIAGNOSTICS);
    return kv.overflow ? -1 : (int)kv.len;
}

static int gwnode_build_diagnostics(gwnode_state_t *st, uint8_t *buf, const size_t size, size_t *cursor) {
    iotdata_kvr_t kv;
    iotdata_kvr_init(&kv, buf, size);
    iotdata_kvr_add_u8(&kv, IOTDATA_NODE_DIAGNOSTICS_TYPE, IOTDATA_NODE_DIAG_BLACKBOX);
    if (st->blackbox != NULL) {
        char rec[BLACKBOX_LINE_MAX];
        while (blackbox_pull(st->blackbox, cursor, rec, sizeof(rec)) > 0) {
            const size_t n = strlen(rec);
            if (kv.len + 2u + n > size)
                break;
            iotdata_kvr_add(&kv, IOTDATA_NODE_DIAGNOSTICS_DATA, rec, (uint8_t)n);
        }
    }
    return kv.overflow ? -1 : (int)kv.len;
}

static int gwnode_build(gwnode_state_t *st, const uint8_t type, uint8_t *buf, const size_t size) {
    size_t cursor = 0;
    switch (type) {
    case IOTDATA_NODE_TLV_VERSION:
        return gwnode_build_version(st, buf, size);
    case IOTDATA_NODE_TLV_VARIANT:
        return gwnode_build_variant(st, buf, size);
    case IOTDATA_NODE_TLV_CONTROL:
        return gwnode_build_control(st, buf, size);
    case IOTDATA_NODE_TLV_STATUS:
        return gwnode_build_status(st, buf, size);
    case IOTDATA_NODE_TLV_CONFIG:
        return gwnode_build_config(st, buf, size);
    case IOTDATA_NODE_TLV_DIAGNOSTICS:
        return gwnode_build_diagnostics(st, buf, size, &cursor);
    default:
        return -1; /* nothing we can say about this type */
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static void gwnode_json_kvr(cJSON *obj, const uint8_t type, const uint8_t *kvbuf, const size_t kvlen) {
    size_t cur = 0;
    uint8_t key, vlen;
    const uint8_t *val;
    char namebuf[16], strbuf[BLACKBOX_LINE_MAX];
    while (iotdata_kvr_next(kvbuf, kvlen, &cur, &key, &val, &vlen)) {
        const char *name = iotdata_node_tlv_key_name(type, key);
        if (name == NULL) {
            snprintf(namebuf, sizeof(namebuf), "0x%02X", key);
            name = namebuf;
        }
        const uint8_t width = iotdata_node_tlv_key_width(type, key);
        if (width == 1 && vlen == 1)
            cJSON_AddNumberToObject(obj, name, (double)val[0]);
        else if (width == 2 && vlen == 2)
            cJSON_AddNumberToObject(obj, name, (double)iotdata_kvr_u16(val, vlen, 0));
        else if (width == 4 && vlen == 4)
            cJSON_AddNumberToObject(obj, name, (double)iotdata_kvr_u32(val, vlen, 0));
        else if (vlen == 0)
            cJSON_AddBoolToObject(obj, name, true); /* a flag: present, no value */
        else {
            (void)iotdata_kvr_str(val, vlen, strbuf, sizeof(strbuf));
            cJSON *existing = cJSON_GetObjectItem(obj, name);
            if (existing == NULL)
                cJSON_AddStringToObject(obj, name, strbuf);
            else { /* repeated key (e.g. several diagnostics records): collect into an array */
                if (!cJSON_IsArray(existing)) {
                    cJSON *arr = cJSON_CreateArray();
                    cJSON_AddItemToArray(arr, cJSON_CreateString(cJSON_GetStringValue(existing)));
                    cJSON_ReplaceItemInObject(obj, name, arr);
                    existing = arr;
                }
                cJSON_AddItemToArray(existing, cJSON_CreateString(strbuf));
            }
        }
    }
}

static void gwnode_publish_from(gwnode_state_t *st, const uint16_t station, const uint8_t type, const uint8_t *kvbuf, const size_t kvlen) {
    cJSON *root = cJSON_CreateObject();
    if (root) {
    char idbuf[8];
    snprintf(idbuf, sizeof(idbuf), "%04" PRIX16, station);
    cJSON_AddStringToObject(root, "station", idbuf);
    cJSON_AddStringToObject(root, "tlv", iotdata_node_tlv_name(type) ? iotdata_node_tlv_name(type) : "?");
    cJSON *data = cJSON_AddObjectToObject(root, "data");
    if (data != NULL)
        gwnode_json_kvr(data, type, kvbuf, kvlen);
    char *out = cJSON_PrintUnformatted(root);
    if (out != NULL) {
        (void)mqtt_send(st->topic_resp, out, (int)strlen(out));
        printf("node: report %s (%zu bytes kv)\n", iotdata_node_tlv_name(type) ? iotdata_node_tlv_name(type) : "?", kvlen);
        free(out);
    }
    cJSON_Delete(root);
	}
}

static bool gwnode_report(gwnode_state_t *st, const uint8_t type, const bool on_radio) {
    uint8_t kvbuf[GWNODE_KV_MAX];
    const int kvlen = gwnode_build(st, type, kvbuf, sizeof(kvbuf));
    if (kvlen < 0) /* not 0: an empty payload is a legitimate report */
        return false;
    st->stat_reports++;
    gwnode_publish_from(st, st->station_id, type, kvbuf, (uint8_t)kvlen);
    if (on_radio && st->tx != NULL) { /* also answer on the wire the request arrived on */
        iotdata_encoder_t enc;
        uint8_t packet[GWNODE_PACKET_MAX];
        if (iotdata_encode_begin(&enc, packet, sizeof(packet), 0, st->station_id, st->sequence) == IOTDATA_OK &&
            iotdata_encode_tlv(&enc, type, kvbuf, (uint8_t)kvlen) == IOTDATA_OK) {
            size_t len = 0;
            st->sequence = iotdata_sequence_next(st->sequence);
            if (iotdata_encode_end(&enc, &len) == IOTDATA_OK)
                (void)st->tx(packet, (int)len);
        }
    }
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static void gwnode_process_control(gwnode_state_t *st, const uint8_t *kvbuf, const size_t kvlen, const bool on_radio) {
    size_t cur = 0;
    uint8_t key, vlen;
    const uint8_t *val;
    while (iotdata_kvr_next(kvbuf, kvlen, &cur, &key, &val, &vlen)) {
        const uint8_t want = iotdata_node_tlv_control_type(key);
        if (want != IOTDATA_NODE_TLV_NONE) { /* a request: send the corresponding report */
            st->stat_requests++;
            if (!gwnode_report(st, want, on_radio))
                printf("node: request for %s -- nothing to report\n", iotdata_node_tlv_name(want) ? iotdata_node_tlv_name(want) : "?");
        } else {
        switch (key) {
        case IOTDATA_NODE_CONTROL_REBOOT:
            st->stat_commands++;
            printf("node: REBOOT requested (delay=%us) -- not implemented on the gateway\n", (unsigned)iotdata_kvr_u16(val, vlen, 0));
            break;
        default:
            /* unknown or not-implemented: skip it, do not fail the packet */
            st->stat_unknown++;
            printf("node: control key 0x%02X ignored (not implemented)\n", key);
            break;
        }
	}
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool gwnode_addressed_to_us(const gwnode_state_t *st, const uint16_t station) {
    return station == st->station_id || station == IOTDATA_STATION_BROADCAST;
}

static bool gwnode_send_down(gwnode_state_t *st, const uint8_t *kvbuf, const size_t kvlen, const uint16_t target) {
    if (st->tx == NULL)
        return false;
    iotdata_encoder_t enc;
    uint8_t packet[GWNODE_PACKET_MAX];
    size_t len = 0;
    if (iotdata_encode_begin(&enc, packet, sizeof(packet), 0, target, IOTDATA_SEQUENCE_DOWN) != IOTDATA_OK)
        return false;
    if (iotdata_encode_tlv(&enc, IOTDATA_NODE_TLV_CONTROL, kvbuf, (uint8_t)kvlen) != IOTDATA_OK)
        return false;
    if (iotdata_encode_end(&enc, &len) != IOTDATA_OK)
        return false;
    /* hold it as well as sending it: the target is probably asleep, and this is the only copy */
    const iotdata_down_ev_t ev = iotdata_down_offer(&st->down, target, packet, len);
    printf("node: down -> %04" PRIX16 " (%zu bytes, %s)\n", target, len, iotdata_down_ev_name(ev));
    return st->tx(packet, (int)len);
}

static bool gwnode_on_packet(const uint8_t *buf, const size_t len, const uint16_t station) {
    gwnode_state_t *const st = g_gwnode;
    iotdata_decoder_t dec;
    if (iotdata_decode(buf, len, &dec) != IOTDATA_OK || dec.tlv_count == 0)
        return false;

    /* This station has just spoken. If it says it is listening and we are holding a command for
       it, now is the only moment we can deliver -- it may be asleep again by the next tick. We are
       decoding anyway here, so unlike the relay there is nothing to check cheaply first. */
    iotdata_node_receive_t rx;
    if (iotdata_node_receive_find(&dec, &rx) && iotdata_down_holds(&st->down, station)) {
        size_t held_len = 0;
        const uint8_t *const held = iotdata_down_deliver(&st->down, station, &held_len);
        if (held != NULL && st->tx != NULL && iotdata_node_receive_accepts(&rx, IOTDATA_NODE_TLV_CONTROL)) {
            printf("node: %04" PRIX16 " is listening -> delivering %zu byte(s) held\n", station, held_len);
            (void)st->tx(held, (int)held_len);
        }
    }

    bool any = false;
    for (uint8_t i = 0; i < dec.tlv_count; i++) {
        const iotdata_decoder_tlv_t *const t = &dec.tlv[i];
        if (iotdata_tlv_type_is_system(t->type) && t->format == IOTDATA_TLV_FMT_RAW) {
        if (t->type != IOTDATA_NODE_TLV_CONTROL) {
        st->stat_rx++;
        gwnode_publish_from(st, station, t->type, t->raw, t->length);
        any = true;
	}
	}
    }
    return any;
}

static void gwnode_on_mqtt(const uint8_t *kvbuf, const size_t kvlen, const uint16_t target) {
    gwnode_state_t *const st = g_gwnode;
    if (gwnode_addressed_to_us(st, target)) {
        st->stat_rx++;
        gwnode_process_control(st, kvbuf, kvlen, false);
    }
    if (target != st->station_id)
        (void)gwnode_send_down(st, kvbuf, kvlen, target);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static void gwnode_begin(gwnode_state_t *st, const uint16_t station_id, const char *version, const stat_state_t *stat, blackbox_handle_t *blackbox, gwnode_tx_handler_t tx, const char *topic_prefix) {
    st->station_id = station_id;
    st->version = version;
    st->stat = stat;
    st->blackbox = blackbox;
    st->tx = tx;
    iotdata_down_init(&st->down);
    snprintf(st->topic_resp, sizeof(st->topic_resp), "%s/node/resp", topic_prefix ? topic_prefix : "iotdata");
    g_gwnode = st;
    printf("node: station=%04" PRIX16 ", responses on %s\n", station_id, st->topic_resp);
}

static void gwnode_tick(void) {
    gwnode_state_t *const st = g_gwnode;
    const time_t now = time(NULL);
    if (!st->startup_done) {
        st->startup_done = true;
        static const struct {
            uint16_t bit;
            uint8_t type;
        } once[] = {
            { IOTDATA_NODE_STARTUP_VERSION, IOTDATA_NODE_TLV_VERSION },
            { IOTDATA_NODE_STARTUP_VARIANT, IOTDATA_NODE_TLV_VARIANT },
            { IOTDATA_NODE_STARTUP_CONTROL, IOTDATA_NODE_TLV_CONTROL },
            { IOTDATA_NODE_STARTUP_STATUS, IOTDATA_NODE_TLV_STATUS },
            { IOTDATA_NODE_STARTUP_CONFIG, IOTDATA_NODE_TLV_CONFIG },
            { IOTDATA_NODE_STARTUP_DIAGNOSTICS, IOTDATA_NODE_TLV_DIAGNOSTICS },
        };
        for (size_t i = 0; i < sizeof(once) / sizeof(once[0]); i++)
            if (st->startup & once[i].bit)
                (void)gwnode_report(st, once[i].type, false);
    }
    for (uint8_t type = 1; type < GWNODE_TYPE_COUNT; type++) {
        if (st->period[type] == 0)
            continue;
        if (st->period_last[type] == 0)
            st->period_last[type] = now; /* prime: first report after one full interval */
        else if ((uint32_t)(now - st->period_last[type]) >= st->period[type]) {
            st->period_last[type] = now;
            (void)gwnode_report(st, type, false);
        }
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

#endif /* IOTDATA_GATEWAY_NODE_H */
