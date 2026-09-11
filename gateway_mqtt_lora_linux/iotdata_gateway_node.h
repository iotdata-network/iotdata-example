
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define NODE_KV_MAX 200 /* a kvr payload we build; a TLV caps at 255 anyway */

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

typedef bool (*node_tx_handler_t)(const uint8_t *packet, const int length);

/* A CONTROL key this layer does not know, offered to the application. Mesh management (peers,
   filters, the station table) belongs to the app, not the node protocol -- and a gateway takes
   part in the mesh exactly as a relay does, so it answers the same keys the same way. Returning
   false counts the key as unknown. */
typedef bool (*node_control_handler_t)(uint8_t key, const uint8_t *val, uint8_t vlen);

/* Filled on demand: how this station sees the mesh it is part of. */
typedef void (*node_status_mesh_handler_t)(iotdata_node_status_mesh_t *out);

/* The mesh tables the app keeps. Two calls because a report states the whole table's size before
   listing rows, so the count must be known before the first row is written. */
typedef uint8_t (*node_table_count_handler_t)(uint8_t type);
typedef bool (*node_table_row_handler_t)(uint8_t type, uint8_t index, uint8_t *out_row);

typedef struct {
    uint16_t station_id; /* our own station: a request is "for us" if it matches, or is broadcast */
    uint16_t sequence;   /* our packet sequence for reports we originate */

    /* CONFIG: reporting cadence, indexed by TLV type. 0 = do not send periodically. */
    uint16_t period[IOTDATA_NODE_TLV_SYSTEM_COUNT];
    time_t period_last[IOTDATA_NODE_TLV_SYSTEM_COUNT];
    uint16_t startup; /* IOTDATA_NODE_STARTUP_* bitmask, emitted once when we come up */
    bool startup_done;

    /* what we report ON: borrowed, not owned */
    const char *version;
    const stat_state_t *stat;
    bbox_state_t *bbox;

    /* how we answer */
    node_tx_handler_t tx; /* radio */
    node_control_handler_t control;
    node_status_mesh_handler_t status_mesh;
    node_table_count_handler_t table_count;
    node_table_row_handler_t table_row;
    /* the keys `control` handles, so the CONTROL report can advertise them */
    const uint8_t *control_keys;
    uint8_t control_keys_count;
    char topic_resp[128]; /* mqtt */

    /* down frames we are holding for nodes that were not listening when we sent them */
    iotdata_down_t down;

    /* stats */
    uint32_t stat_rx, stat_requests, stat_reports, stat_commands, stat_unknown;

    uint8_t _buffer_kv[NODE_KV_MAX];
    iotdata_encoder_t _iotdata_enc;
    iotdata_decoder_t _iotdata_dec;
    buffer_pool_t *pool;
    char _buffer_blackbox_rec[BLACKBOX_LINE_MAX];
} node_state_t;

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static int node_build_version(const node_state_t *st, uint8_t *buf, const size_t size) {
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
           uint8_t val[NODE_KV_MAX];
           const size_t n = d ? iotdata_node_variant_encode(d, val, sizeof(val)) : 0;
           if (n == 0) continue;
           if (kv.len + 2u + n > size) break;   // leave it for the next packet
           iotdata_kvr_add(&kv, (uint8_t)*cursor, val, (uint8_t)n);
       }

   which is left until the field ids are globally assigned -- until then they mean nothing to a
   receiver built separately. */
static int node_build_variant(__attribute__((unused)) const node_state_t *st, uint8_t *buf, const size_t size) {
    iotdata_kvr_t kv;
    iotdata_kvr_init(&kv, buf, size);
    return kv.overflow ? -1 : (int)kv.len;
}

/* `scope` is the STATUS_REQUEST value: which groups to report, 0 (absent) meaning all of them. */
static int node_build_status(const node_state_t *st, uint8_t *buf, const size_t size, const uint8_t scope) {
    iotdata_kvr_t kv;
    iotdata_kvr_init(&kv, buf, size);
    if (iotdata_node_status_scope_wants(scope, IOTDATA_NODE_STATUS_SCOPE_NODE)) {
        iotdata_kvr_add_u32(&kv, IOTDATA_NODE_STATUS_UPTIME, st->stat->start_time > 0 ? (uint32_t)(time(NULL) - st->stat->start_time) : 0);
        iotdata_kvr_add_u8(&kv, IOTDATA_NODE_STATUS_REASON, IOTDATA_NODE_REASON_UNKNOWN);
    }
    if (iotdata_node_status_scope_wants(scope, IOTDATA_NODE_STATUS_SCOPE_MESH) && st->status_mesh != NULL) {
        iotdata_node_status_mesh_t m;
        memset(&m, 0, sizeof(m));
        st->status_mesh(&m);
        iotdata_node_status_mesh_emit(&kv, &m); /* a no-op unless .present */
    }
    return kv.overflow ? -1 : (int)kv.len;
}

static int node_build_config(const node_state_t *st, uint8_t *buf, const size_t size) {
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

static int node_build_control(const node_state_t *st, uint8_t *buf, const size_t size) {
    iotdata_kvr_t kv;
    iotdata_kvr_init(&kv, buf, size);
    iotdata_kvr_add_flag(&kv, IOTDATA_NODE_CONTROL_VERSION_REQUEST);
    iotdata_kvr_add_flag(&kv, IOTDATA_NODE_CONTROL_VARIANT_REQUEST);
    iotdata_kvr_add_flag(&kv, IOTDATA_NODE_CONTROL_CONTROL_REQUEST);
    iotdata_kvr_add_flag(&kv, IOTDATA_NODE_CONTROL_STATUS_REQUEST);
    iotdata_kvr_add_flag(&kv, IOTDATA_NODE_CONTROL_CONFIG_REQUEST);
    iotdata_kvr_add_flag(&kv, IOTDATA_NODE_CONTROL_DIAGNOSTICS_REQUEST);
    /* the tables it keeps are requestable, one key each -- the capability is advertised whether
       or not a table happens to be empty right now */
    if (st->table_count != NULL)
        for (uint8_t type = IOTDATA_NODE_TLV_MESH_STATIONS; type <= IOTDATA_NODE_TLV_MESH_FILTERS; type++)
            iotdata_kvr_add_flag(&kv, iotdata_node_tlv_control_key(type));
    for (uint8_t i = 0; i < st->control_keys_count; i++)
        iotdata_kvr_add_flag(&kv, st->control_keys[i]);
    return kv.overflow ? -1 : (int)kv.len;
}

/* Records are pulled until the packet is full; the cursor lets the caller resume where this
   stopped. blackbox_pull() ADVANCES the cursor before we know whether the record fits, so a record
   that does not fit is put back by rewinding -- otherwise one record is silently lost at every
   packet boundary. A record too large for a KV value can never fit any packet, so it is skipped
   rather than rewound: rewinding would stall the stream on it forever. */
static int node_build_diagnostics(node_state_t *st, uint8_t *buf, const size_t size, size_t *cursor) {
    iotdata_kvr_t kv;
    iotdata_kvr_init(&kv, buf, size);
    iotdata_kvr_add_u8(&kv, IOTDATA_NODE_DIAGNOSTICS_TYPE, IOTDATA_NODE_DIAG_BLACKBOX);
    size_t prev = *cursor;
    while (blackbox_pull(&st->bbox->handle, cursor, st->_buffer_blackbox_rec, sizeof(st->_buffer_blackbox_rec)) > 0) {
        const size_t n = strlen(st->_buffer_blackbox_rec);
        if (n <= 255u) { /* a KV value length is one byte; larger is unreachable today, and skipped */
            if (kv.len + 2u + n > size) {
                *cursor = prev; /* does not fit THIS packet: put it back for the next one */
                break;
            }
            iotdata_kvr_add(&kv, IOTDATA_NODE_DIAGNOSTICS_DATA, st->_buffer_blackbox_rec, (uint8_t)n);
        }
        prev = *cursor;
    }
    return kv.overflow ? -1 : (int)kv.len;
}

/* As many rows as fit, after the whole table's count -- see iotdata_node.h. */
static int node_build_table(node_state_t *st, const uint8_t type, uint8_t *buf, const size_t size) {
    iotdata_kvr_t kv;
    iotdata_kvr_init(&kv, buf, size);
    const uint8_t row_size = iotdata_node_table_row_size(type);
    if (row_size == 0 || st->table_count == NULL || st->table_row == NULL)
        return -1;
    const uint8_t count = st->table_count(type);
    iotdata_kvr_add_u8(&kv, IOTDATA_NODE_TABLE_COUNT, count);
    for (uint8_t i = 0; i < count && (kv.len + 2u + (size_t)row_size > size); i++) {
        uint8_t row[IOTDATA_NODE_TABLE_ROW_MAX] = { 0x00 };
        if (st->table_row(type, i, row))
            iotdata_kvr_add(&kv, IOTDATA_NODE_TABLE_ROW, row, row_size);
    }
    return kv.overflow ? -1 : (int)kv.len;
}

static int node_build(node_state_t *st, const uint8_t type, uint8_t *buf, const size_t size, const uint8_t scope) {
    size_t cursor = 0;
    switch (type) {
    case IOTDATA_NODE_TLV_VERSION:
        return node_build_version(st, buf, size);
    case IOTDATA_NODE_TLV_VARIANT:
        return node_build_variant(st, buf, size);
    case IOTDATA_NODE_TLV_CONTROL:
        return node_build_control(st, buf, size);
    case IOTDATA_NODE_TLV_STATUS:
        return node_build_status(st, buf, size, scope);
    case IOTDATA_NODE_TLV_CONFIG:
        return node_build_config(st, buf, size);
    case IOTDATA_NODE_TLV_MESH_STATIONS:
    case IOTDATA_NODE_TLV_MESH_PEERS:
    case IOTDATA_NODE_TLV_MESH_FILTERS:
        return node_build_table(st, type, buf, size);
    case IOTDATA_NODE_TLV_DIAGNOSTICS:
        return node_build_diagnostics(st, buf, size, &cursor);
    default:
        return -1; /* nothing we can say about this type */
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

/*
 * A table report as JSON: the rows decoded into named fields, not left as hex.
 *
 * The generic renderer below would print each row as a hex string, which is true but unusable --
 * the whole point of a table over MQTT rather than a console dump is that something downstream can
 * read it. The row layouts live in iotdata_node.h and are read back with its accessors, so this
 * cannot drift from what a node writes.
 */
static const char *node_json_table_kind(const uint8_t k) {
    switch (k) {
    case IOTDATA_NODE_TABLE_KIND_GATEWAY:
        return "gateway";
    case IOTDATA_NODE_TABLE_KIND_RELAY:
        return "relay";
    case IOTDATA_NODE_TABLE_KIND_SENSOR:
        return "sensor";
    default:
        return "unknown";
    }
}

static void node_json_table_row(cJSON *const arr, const uint8_t type, const uint8_t *const val, const uint8_t vlen) {
    if (vlen != iotdata_node_table_row_size(type))
        return; /* not a row of this table: drop it rather than misread it */
    cJSON *const o = cJSON_CreateObject();
    if (o == NULL)
        return;
    char idbuf[4 + 1];
    cJSON_AddStringToObject(o, "station", snprintf_inline(idbuf, sizeof(idbuf), "%04" PRIX16, iotdata_node_table_get_u16(val, 0)));
    switch (type) {
    case IOTDATA_NODE_TLV_MESH_STATIONS:
        cJSON_AddStringToObject(o, "kind", node_json_table_kind(val[2]));
        cJSON_AddNumberToObject(o, "variant", val[3]);
        cJSON_AddNumberToObject(o, "rssi", (double)(int8_t)val[4]);
        cJSON_AddNumberToObject(o, "age", iotdata_node_table_get_u16(val, 5));
        cJSON_AddNumberToObject(o, "rx", iotdata_node_table_get_u32(val, 7));
        break;
    case IOTDATA_NODE_TLV_MESH_PEERS:
        cJSON_AddStringToObject(o, "gateway", snprintf_inline(idbuf, sizeof(idbuf), "%04" PRIX16, iotdata_node_table_get_u16(val, 2)));
        cJSON_AddNumberToObject(o, "cost", val[4]);
        cJSON_AddNumberToObject(o, "generation", iotdata_node_table_get_u16(val, 5));
        cJSON_AddNumberToObject(o, "rssi", (double)(int8_t)val[7]);
        cJSON_AddNumberToObject(o, "age", iotdata_node_table_get_u16(val, 8));
        cJSON_AddBoolToObject(o, "accepting", (val[10] & IOTDATA_NODE_TABLE_PEER_ACCEPTING) != 0);
        cJSON_AddBoolToObject(o, "parent", (val[10] & IOTDATA_NODE_TABLE_PEER_PARENT) != 0);
        break;
    case IOTDATA_NODE_TLV_MESH_FILTERS:
        cJSON_AddStringToObject(o, "action", val[2] == IOTDATA_NODE_CONTROL_MESH_FILTERS_ALLOW ? "allow" : val[2] == IOTDATA_NODE_CONTROL_MESH_FILTERS_BLOCK ? "block" : "none");
        cJSON_AddStringToObject(o, "source", val[3] == IOTDATA_NODE_CONTROL_MESH_FILTERS_SCOPE_AUTO ? "auto" : "manual");
        break;
    default:
        break;
    }
    cJSON_AddItemToArray(arr, o);
}

static void node_json_table(cJSON *const obj, const uint8_t type, const uint8_t *const kvbuf, const size_t kvlen) {
    cJSON *const rows = cJSON_AddArrayToObject(obj, "rows");
    size_t cur = 0;
    uint8_t key, vlen;
    const uint8_t *val;
    while (iotdata_kvr_next(kvbuf, kvlen, &cur, &key, &val, &vlen)) {
        if (key == IOTDATA_NODE_TABLE_COUNT)
            cJSON_AddNumberToObject(obj, "count", iotdata_kvr_u8(val, vlen, 0)); /* the TABLE's size */
        else if (key == IOTDATA_NODE_TABLE_ROW && rows != NULL)
            node_json_table_row(rows, type, val, vlen);
    }
}

/* Is this value printable text, or bytes? A NUL or a control byte means bytes -- and bytes must not
   go through a string conversion that stops at the first NUL. */
static bool node_json_value_is_text(const uint8_t *const val, const uint8_t vlen) {
    for (uint8_t i = 0; i < vlen; i++)
        if (val[i] < 0x20 || val[i] > 0x7E)
            return false;
    return true;
}

static void node_json_kvr(node_state_t *st, cJSON *obj, const uint8_t type, const uint8_t *kvbuf, const size_t kvlen) {
    size_t cur = 0;
    uint8_t key, vlen;
    const uint8_t *val;
    while (iotdata_kvr_next(kvbuf, kvlen, &cur, &key, &val, &vlen)) {
        char namebuf[4 + 1]; // XXX
        const char *name = iotdata_node_tlv_key_name(type, key);
        if (name == NULL)
            name = snprintf_inline(namebuf, sizeof(namebuf), "0x%02X", key);
        const uint8_t width = iotdata_node_tlv_key_width(type, key);
        if (width == 1 && vlen == 1)
            cJSON_AddNumberToObject(obj, name, (double)val[0]);
        else if (width == 2 && vlen == 2)
            cJSON_AddNumberToObject(obj, name, (double)iotdata_kvr_u16(val, vlen, 0));
        else if (width == 4 && vlen == 4)
            cJSON_AddNumberToObject(obj, name, (double)iotdata_kvr_u32(val, vlen, 0));
        else if (vlen == 0)
            cJSON_AddBoolToObject(obj, name, true); /* a flag: present, no value */
        else if (!node_json_value_is_text(val, vlen)) {
            /* Binary: hex, not a string. iotdata_kvr_str stops at the first NUL, so a value with
               one in it silently loses its tail -- which is how a mesh table row, rendered by this
               path before it had a renderer of its own, came out as two characters. Any key wider
               than 4 bytes that is not text lands here. */
            char hex[2 * IOTDATA_TLV_DATA_MAX + 1];
            size_t h = 0;
            for (uint8_t i = 0; i < vlen && h + 2 < sizeof(hex); i++)
                h += (size_t)snprintf(hex + h, sizeof(hex) - h, "%02X", val[i]);
            cJSON_AddStringToObject(obj, name, hex);
        } else {
            (void)iotdata_kvr_str(val, vlen, st->_buffer_blackbox_rec, sizeof(st->_buffer_blackbox_rec));
            cJSON *existing = cJSON_GetObjectItem(obj, name);
            if (existing == NULL)
                cJSON_AddStringToObject(obj, name, st->_buffer_blackbox_rec);
            else { /* repeated key (e.g. several diagnostics records): collect into an array */
                if (!cJSON_IsArray(existing)) {
                    cJSON *arr = cJSON_CreateArray();
                    cJSON_AddItemToArray(arr, cJSON_CreateString(cJSON_GetStringValue(existing)));
                    cJSON_ReplaceItemInObject(obj, name, arr);
                    existing = arr;
                }
                cJSON_AddItemToArray(existing, cJSON_CreateString(st->_buffer_blackbox_rec));
            }
        }
    }
}

static void node_publish_from(node_state_t *st, const uint16_t station, const uint8_t type, const uint8_t *kvbuf, const size_t kvlen) {
    cJSON *root = cJSON_CreateObject();
    if (root) {
        char buf[4 + 1];
        cJSON_AddStringToObject(root, "resp", "node"); /* shares a topic with the other responses */
        cJSON_AddStringToObject(root, "station", snprintf_inline(buf, sizeof(buf), "%04" PRIX16, station));
        cJSON_AddStringToObject(root, "tlv", iotdata_node_tlv_name(type) ? iotdata_node_tlv_name(type) : "?");
        cJSON *data = cJSON_AddObjectToObject(root, "data");
        if (data != NULL) {
            if (iotdata_node_tlv_is_table(type))
                node_json_table(data, type, kvbuf, kvlen);
            else
                node_json_kvr(st, data, type, kvbuf, kvlen);
        }
        char *out = cJSON_PrintUnformatted(root);
        if (out != NULL) {
            (void)mqtt_send(st->topic_resp, out, (int)strlen(out));
            PRINTF_INFO("node: report %s (%zu bytes kv)\n", iotdata_node_tlv_name(type) ? iotdata_node_tlv_name(type) : "?", kvlen);
            free(out);
        }
        cJSON_Delete(root);
    }
}

/* `scope` only means anything to STATUS; 0 is "every group", which is what every caller that is
   not answering an explicit request wants. */
static bool node_report_scoped(node_state_t *st, const uint8_t type, const uint8_t scope) {
    const int kvlen = node_build(st, type, st->_buffer_kv, sizeof(st->_buffer_kv), scope);
    if (kvlen < 0) /* not 0: an empty payload is a legitimate report */
        return false;
    st->stat_reports++;
    node_publish_from(st, st->station_id, type, st->_buffer_kv, (uint8_t)kvlen);
    return true;
}

static bool node_report(node_state_t *st, const uint8_t type) {
    return node_report_scoped(st, type, 0); /* 0 = every group */
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static void node_process_control(node_state_t *st, const uint8_t *kvbuf, const size_t kvlen) {
    size_t cur = 0;
    uint8_t key, vlen;
    const uint8_t *val;
    while (iotdata_kvr_next(kvbuf, kvlen, &cur, &key, &val, &vlen)) {
        const uint8_t want = iotdata_node_tlv_control_type(key);
        if (want != IOTDATA_NODE_TLV_NONE) { /* a request: send the corresponding report */
            st->stat_requests++;
            /* STATUS is the one request with a value: which groups to report. Anything else that
               carries a value is answered in full, as if it had carried none. */
            const uint8_t scope = (want == IOTDATA_NODE_TLV_STATUS && vlen >= 1) ? val[0] : 0u;
            if (!node_report_scoped(st, want, scope))
                PRINTF_INFO("node: request for %s -- nothing to report\n", iotdata_node_tlv_name(want) ? iotdata_node_tlv_name(want) : "?");
        } else {
            switch (key) {
            case IOTDATA_NODE_CONTROL_REBOOT:
                st->stat_commands++;
                PRINTF_INFO("node: REBOOT requested (delay=%us) -- not implemented on the gateway\n", (unsigned)iotdata_kvr_u16(val, vlen, 0));
                break;
            default:
                /* not ours: offer it to the app, which is where mesh management lives */
                if (st->control != NULL && st->control(key, val, vlen))
                    st->stat_commands++;
                else {
                    st->stat_unknown++; /* unknown or not implemented: skip it, do not fail the packet */
                    PRINTF_INFO("node: control key 0x%02X ignored (not implemented)\n", key);
                }
                break;
            }
        }
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool node_addressed_to_us(const node_state_t *st, const uint16_t station) {
    return station == st->station_id || station == IOTDATA_STATION_BROADCAST;
}

static bool node_send_down(node_state_t *st, const uint8_t *kvbuf, const size_t kvlen, const uint16_t target) {
    bool ok = false;
    const buffer_handle_t h = buffer_acquire(st->pool);
    if (h != BUFFER_NONE) {
        uint8_t *const buf = buffer_data(st->pool, h);
        size_t len = 0;
        ok = iotdata_encode_begin(&st->_iotdata_enc, buf, buffer_room(st->pool, h), 0, target, IOTDATA_SEQUENCE_DOWN) == IOTDATA_OK && iotdata_encode_tlv(&st->_iotdata_enc, IOTDATA_NODE_TLV_CONTROL, kvbuf, (uint8_t)kvlen) == IOTDATA_OK &&
             iotdata_encode_end(&st->_iotdata_enc, &len) == IOTDATA_OK;
        if (ok) {
            /* hold it as well as sending it: the target is probably asleep, so this may not be heard */
            const iotdata_down_ev_t ev = iotdata_down_offer(&st->down, target, buf, len);
            PRINTF_INFO("node: down -> %04" PRIX16 " (%zu bytes, %s)\n", target, len, iotdata_down_ev_name(ev));
            ok = st->tx(buf, (int)len);
        }
        buffer_unref(st->pool, h); /* written straight to the radio (and copied by the DOWN store) */
    } else
        PRINTF_ERROR("node: down -> %04" PRIX16 " failed: no frame buffer (pool %u/%u)\n", target, (unsigned)buffer_pool_used(st->pool), (unsigned)buffer_pool_total(st->pool));
    return ok;
}

static bool node_on_packet(node_state_t *const st, const uint8_t *buf, const size_t len, const uint16_t station) {
    bool ok = false;
    if (iotdata_decode(buf, len, &st->_iotdata_dec) == IOTDATA_OK && st->_iotdata_dec.tlv_count > 0) {

        /* This station has just spoken. If it says it is listening and we are holding a command for
        it, now is the only moment we can deliver -- it may be asleep again by the next tick. We are
        decoding anyway here, so unlike the relay there is nothing to check cheaply first. */
        iotdata_node_receive_t rx;
        if (iotdata_node_receive_find(&st->_iotdata_dec, &rx) && iotdata_down_holds(&st->down, station)) {
            size_t held_len = 0;
            const uint8_t *const held = iotdata_down_deliver(&st->down, station, &held_len);
            if (held != NULL && iotdata_node_receive_accepts(&rx, IOTDATA_NODE_TLV_CONTROL)) {
                PRINTF_INFO("node: %04" PRIX16 " is listening -> delivering %zu byte(s) held\n", station, held_len);
                (void)st->tx(held, (int)held_len);
            }
        }

        /*
         * A DOWN frame carries a COMMAND, not a report -- and we are the one who sent it. Relays
         * rebroadcast a downstream frame on behalf of a station they cannot reach directly, so our own
         * commands come back to us off the air, and republishing one as though a node had reported it
         * would be a lie about who said what.
         *
         * The sequence field is what distinguishes the two directions, so it is what this tests. It
         * used to be approximated by skipping CONTROL TLVs entirely -- which did keep echoes out, and
         * also silently dropped the legitimate CONTROL *report*, the one a node sends to say which
         * commands it accepts. So `node-control` could never work for any station.
         */
        if (!iotdata_node_is_down(st->_iotdata_dec.sequence))
            for (uint8_t i = 0; i < st->_iotdata_dec.tlv_count; i++) {
                const iotdata_decoder_tlv_t *const t = &st->_iotdata_dec.tlv[i];
                if (iotdata_tlv_type_is_system(t->type) && t->format == IOTDATA_TLV_FMT_RAW) {
                    st->stat_rx++;
                    node_publish_from(st, station, t->type, t->raw, t->length);
                    ok = true;
                }
            }
    }
    return ok;
}

static void node_on_mqtt(node_state_t *const st, const uint8_t *kvbuf, const size_t kvlen, const uint16_t target) {
    if (node_addressed_to_us(st, target)) {
        st->stat_rx++;
        node_process_control(st, kvbuf, kvlen);
    }
    if (target != st->station_id)
        (void)node_send_down(st, kvbuf, kvlen, target);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static void node_tick(node_state_t *const st) {
    const time_t now = time(NULL);
    if (!st->startup_done) {
        st->startup_done = true;
        static const struct {
            uint16_t bit;
            uint8_t type;
        } once[] = {
            { IOTDATA_NODE_STARTUP_VERSION, IOTDATA_NODE_TLV_VERSION }, { IOTDATA_NODE_STARTUP_VARIANT, IOTDATA_NODE_TLV_VARIANT }, { IOTDATA_NODE_STARTUP_CONTROL, IOTDATA_NODE_TLV_CONTROL },
            { IOTDATA_NODE_STARTUP_STATUS, IOTDATA_NODE_TLV_STATUS },   { IOTDATA_NODE_STARTUP_CONFIG, IOTDATA_NODE_TLV_CONFIG },   { IOTDATA_NODE_STARTUP_DIAGNOSTICS, IOTDATA_NODE_TLV_DIAGNOSTICS },
        };
        for (size_t i = 0; i < sizeof(once) / sizeof(once[0]); i++)
            if (st->startup & once[i].bit)
                (void)node_report(st, once[i].type);
    }
    for (uint8_t type = 1; type < (uint8_t)(sizeof(st->period) / sizeof(st->period[0])); type++)
        if (st->period[type] != 0) {
            if (st->period_last[type] == 0)
                st->period_last[type] = now; /* prime: first report after one full interval */
            else if ((uint32_t)(now - st->period_last[type]) >= st->period[type]) {
                st->period_last[type] = now;
                (void)node_report(st, type);
            }
        }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool node_begin(node_state_t *st, const uint16_t station_id, const char *version, const stat_state_t *stat, bbox_state_t *bbox, buffer_pool_t *pool, node_tx_handler_t tx, node_control_handler_t control,
                       node_status_mesh_handler_t status_mesh, node_table_count_handler_t table_count, node_table_row_handler_t table_row, const uint8_t *control_keys, const uint8_t control_keys_count, const char *topic_prefix) {
    assert(version && stat && bbox && pool && tx && topic_prefix);
    st->station_id = station_id;
    st->version = version;
    st->stat = stat;
    st->bbox = bbox;
    st->pool = pool;
    st->tx = tx;
    st->control = control;
    st->status_mesh = status_mesh;
    st->table_count = table_count;
    st->table_row = table_row;
    st->control_keys = control_keys;
    st->control_keys_count = control_keys_count;
    iotdata_down_init(&st->down);
    snprintf(st->topic_resp, sizeof(st->topic_resp), "%s" IOTDATA_GATEWAY_TOPIC_RESP, topic_prefix ? topic_prefix : "iotdata");
    PRINTF_INFO("node: station=%04" PRIX16 ", responses on %s\n", station_id, st->topic_resp);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void node_end(__attribute__((unused)) node_state_t *st) {
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
