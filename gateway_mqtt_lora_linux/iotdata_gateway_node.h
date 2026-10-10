
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

typedef bool (*node_tx_handler_t)(const uint8_t *packet, const int length);

/* A CONTROL key this layer does not know, offered to the application. Mesh management (peers,
   filters, the station table) belongs to the app, not the node protocol -- and a gateway takes
   part in the mesh exactly as a relay does, so it answers the same keys the same way. Returning
   false counts the key as unknown. */
/* An app command, already split into its subject and action by the dispatcher. */
typedef bool (*node_control_handler_t)(uint8_t subject, uint8_t action, const uint8_t *args, uint8_t arglen);

/* Filled on demand: how this station sees the mesh it is part of. */
typedef void (*node_status_mesh_handler_t)(iotdata_node_status_mesh_t *out);

/* The mesh tables the app keeps. Two calls because a report states the whole table's size before
   listing rows, so the count must be known before the first row is written. */
/* A table is named by its STATUS ENTRY key (STATIONS_ENTRY, FILTERS_ENTRY, MESH_PEERS_ENTRY),
   not by a TLV type -- there are no table types any more. */
typedef uint8_t (*node_table_count_handler_t)(uint8_t entry);
typedef bool (*node_table_row_handler_t)(uint8_t entry, uint8_t index, uint8_t *out_row);

typedef struct {
    iotdata_node_t *inode;

    /* CONFIG: reporting cadence, indexed by TLV type. 0 = do not send periodically. */
    uint16_t period[IOTDATA_NODE_TLV_SYSTEM_COUNT];
    time_t period_last[IOTDATA_NODE_TLV_SYSTEM_COUNT];
    uint16_t startup; /* bit N = emit type N once when we come up */
    bool startup_done;

    /* what we report ON: borrowed, not owned */
    const iotdata_node_version_caps_t *caps;
    bool reboot_requested; /* a REBOOT arrived: the loop stops and the supervisor restarts us */
    const stat_state_t *stat;
    bbox_state_t *bbox;

    /* how we answer */
    node_tx_handler_t tx; /* radio */
    node_control_handler_t control;
    node_status_mesh_handler_t status_mesh;
    node_table_count_handler_t table_count;
    node_table_row_handler_t table_row;
    /* the keys `control` handles, so the CONTROL report can advertise them */
    /* The protocol's own settings, and where they persist. Attached after node_begin() rather than
       passed through it: both or neither, since a block that cannot be flushed would answer a
       write with a promise rather than a fact. */
    /* The gateway's own CONFIG, reached through callbacks because the table is compiled into the
       APPLICATION and this header is included before the X-macro that expands it. Both NULL today:
       the gateway keeps its configuration in its own file and has not been migrated, so it answers
       a CONFIG request with an empty report -- which is an answer, and an honest one. */
    iotdata_node_config_build_fn config_build;
    iotdata_node_config_apply_fn config_apply;
    iotdata_node_settings_t *settings;
    iotdata_node_state_t *state;

    const uint8_t *control_actions;
    uint8_t control_actions_count;
    char topic_resp[128]; /* mqtt */

    /* down frames we are holding for nodes that were not listening when we sent them */
    iotdata_node_down_t down;

    /* stats */
    uint32_t stat_rx, stat_requests, stat_reports, stat_commands, stat_unknown;

    uint8_t _buffer_kv[IOTDATA_MAX_PACKET_SIZE];
    iotdata_encoder_t _iotdata_enc;
    iotdata_decoder_t _iotdata_dec;
    buffer_pool_t *pool;
    char _buffer_blackbox_rec[BLACKBOX_LINE_MAX];
} node_state_t;

/* WHO OWNS THE SCHEDULE -- settings when there are settings, the struct's own fields when there are
   not, and never a merge of the two. Settings are seeded dense, so a subject always has a record
   and there is nothing for the struct's copy to fill in; a record saying "never" means never.
   (The relay says exactly this in relay_node.h. The two nodes are separate modules by history, and
   the point of stating it identically is that the cadence cannot differ by accident.) */
static inline bool node_at_startup(const node_state_t *const st, const uint8_t type) {
    return (st->settings != NULL) ? iotdata_node_settings_at_startup(st->settings, type) : ((st->startup >> type) & 1u) != 0u;
}
static inline uint16_t node_period_s(const node_state_t *const st, const uint8_t type) {
    return (st->settings != NULL) ? iotdata_node_settings_period_s(st->settings, type) : st->period[type];
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static int node_build_version(const node_state_t *st, uint8_t *buf, const size_t size) {
    iotdata_kvr_t kv;
    iotdata_kvr_init(&kv, buf, size);
    return iotdata_node_version_pack(&kv, st->caps);
}

static int node_build_settings(const node_state_t *st, uint8_t *buf, const size_t size, const uint8_t *sel, const uint8_t sellen) {
    iotdata_kvr_t kv;
    iotdata_kvr_init(&kv, buf, size);
    if (st->settings == NULL)
        return kv.overflow ? -1 : (int)kv.len;
    return iotdata_node_settings_pack(&kv, st->settings, sel, sellen);
}

/* VARIANT is one key per variant this node PRODUCES, keyed by the variant number.

   A gateway produces none, and says so with an EMPTY TLV rather than by staying silent -- "I
   produce none" is an answer. Its compiled-in variant table is full, but that table is what it can
   DECODE from everybody else, which is a different question and must not be answered here; the
   mesh variant it speaks is not a telemetry variant either. A node that does produce telemetry
   calls iotdata_node_variant_pack() instead (iotdata_node_variant.h). */
static int node_build_variant(__attribute__((unused)) const node_state_t *st, uint8_t *buf, const size_t size) {
    return iotdata_node_variant_pack_none(buf, size);
}

static int node_build_control(const node_state_t *st, uint8_t *buf, const size_t size) {
    iotdata_kvr_t kv;
    iotdata_kvr_init(&kv, buf, size);
    return iotdata_node_control_pack(&kv, &(const iotdata_node_control_report_t){ .actions = st->control_actions, .actions_count = st->control_actions_count });
}

static int node_build_status(const node_state_t *st, uint8_t *buf, const size_t size, const uint8_t scope, iotdata_node_partial_t *const partial) {
    iotdata_kvr_t kv;
    iotdata_kvr_init(&kv, buf, size);
    const uint32_t cursor = (partial != NULL) ? partial->cursor : 0u;
    uint8_t ti = (uint8_t)(cursor >> 16), ri = (uint8_t)(cursor & 0xFFFFu);
    uint8_t packed = 0;
    bool more = false;
    if (cursor == 0) {
        iotdata_node_status_t s = { 0 };
        s.uptime_s = st->stat->start_time > 0 ? (uint32_t)(time(NULL) - st->stat->start_time) : 0;
        s.reason = IOTDATA_NODE_REASON_UNKNOWN;
        if (st->status_mesh != NULL)
            st->status_mesh(&s.mesh);
        if (st->table_count != NULL) {
            s.stations = st->table_count(IOTDATA_NODE_STATUS_STATIONS_ENTRY), s.has_stations = true;
            s.filters = st->table_count(IOTDATA_NODE_STATUS_FILTERS_ENTRY), s.has_filters = true;
            s.content = st->table_count(IOTDATA_NODE_STATUS_CONTENT_ENTRY), s.has_content = true;
            /* a table's size comes from the table, never from a second counter that could drift */
            s.mesh.peers = st->table_count(IOTDATA_NODE_STATUS_MESH_PEERS_ENTRY);
        }
        if (iotdata_node_status_pack(&kv, &s, scope) < 0)
            return -1;
    }
    if (st->table_count != NULL && st->table_row != NULL) {
        for (; ti < IOTDATA_NODE_STATUS_TABLE_COUNT && !more; ti++, ri = 0) {
            const iotdata_node_status_table_t *const t = &iotdata_node_status_tables[ti];
            if ((scope & t->scope) != 0) {
                const uint8_t count = st->table_count(t->entry), width = t->width;
                for (; ri < count; ri++) {
                    uint8_t row[IOTDATA_NODE_STATUS_ENTRY_MAX] = { 0x00 };
                    if (kv.len + 2u + (size_t)width > size) {
                        more = true;
                        break;
                    }
                    if (st->table_row(t->entry, ri, row)) {
                        iotdata_kvr_add(&kv, t->entry, row, width);
                        packed++;
                    }
                }
            }
        }
    }
    if (partial != NULL) {
        partial->chunk = packed;
        partial->more = more;
        partial->cursor = more ? (uint32_t)(((uint32_t)(ti - 1u) << 16) | ri) : 0u;
    }
    return kv.overflow ? -1 : (int)kv.len;
}

/* A node's own settings are PROPRIETARY keys, and this device has none to state -- so the report is
   an EMPTY kvr, which is an answer. The reporting cadence that used to live here is protocol
   configuration and is now the trigger record, carried by control(<subject>, set-trigger). */
/* The device's own configuration, as the [id|type][value] record stream the store and the wire
   share -- NOT kvr, because a config id is 12 bits and a kvr key is 8. Nothing is attached yet, so
   this reports EMPTY: the gateway's own settings still live in its .cfg file. An empty report is an
   answer, and the truthful one until that migration happens. */
static int node_build_config(const node_state_t *st, uint8_t *buf, const size_t size, iotdata_node_partial_t *partial) {
    return (st->config_build != NULL) ? st->config_build(buf, size, partial) : 0;
}

static int node_build_diagnostics(node_state_t *st, uint8_t *buf, const size_t size, iotdata_node_partial_t *partial) {
    iotdata_kvr_t kv;
    iotdata_kvr_init(&kv, buf, size);
    iotdata_kvr_add_u8(&kv, IOTDATA_NODE_DIAGNOSTICS_TYPE, IOTDATA_NODE_DIAG_BLACKBOX);
    size_t cursor = (partial != NULL) ? (size_t)partial->cursor : 0u;
    size_t prev = cursor;
    uint8_t packed = 0;
    bool more = false;
    while (blackbox_pull(&st->bbox->handle, &cursor, st->_buffer_blackbox_rec, sizeof(st->_buffer_blackbox_rec)) > 0) {
        const size_t n = strlen(st->_buffer_blackbox_rec);
        if (n <= 255u) { /* a KV value length is one byte; larger is unreachable today, and skipped */
            if (kv.len + 2u + n > size) {
                cursor = prev; /* does not fit THIS packet: put it back for the next one */
                more = true;
                break;
            }
            iotdata_kvr_add(&kv, IOTDATA_NODE_DIAGNOSTICS_DATA, st->_buffer_blackbox_rec, (uint8_t)n);
            packed++;
        }
        prev = cursor;
    }
    if (partial != NULL) {
        partial->cursor = more ? (uint32_t)cursor : 0u;
        partial->chunk = packed;
        partial->more = more;
        partial->total = (uint8_t)(partial->index + packed + (more ? 1u : 0u));
    }
    return kv.overflow ? -1 : (int)kv.len;
}

static int node_build(node_state_t *st, const uint8_t type, uint8_t *buf, const size_t size, const uint8_t scope, iotdata_node_partial_t *partial) {
    switch (type) {
    case IOTDATA_NODE_TLV_VERSION:
        return node_build_version(st, buf, size);
    case IOTDATA_NODE_TLV_VARIANT:
        return node_build_variant(st, buf, size);
    case IOTDATA_NODE_TLV_CONTROL:
        return node_build_control(st, buf, size);
    case IOTDATA_NODE_TLV_STATUS:
        return node_build_status(st, buf, size, scope, partial);
    case IOTDATA_NODE_TLV_CONFIG:
        return node_build_config(st, buf, size, partial);
    case IOTDATA_NODE_TLV_SETTINGS:
        return node_build_settings(st, buf, size, (scope != 0u) ? &scope : NULL, (scope != 0u) ? 1u : 0u);
    case IOTDATA_NODE_TLV_DIAGNOSTICS:
        return node_build_diagnostics(st, buf, size, partial);
    default:
        return -1; /* nothing we can say about this type */
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static const char *node_json_table_kind(const uint8_t k) {
    switch (k) {
    case IOTDATA_NODE_STATUS_STATIONS_KIND_GATEWAY:
        return "gateway";
    case IOTDATA_NODE_STATUS_STATIONS_KIND_RELAY:
        return "relay";
    case IOTDATA_NODE_STATUS_STATIONS_KIND_SENSOR:
        return "sensor";
    default:
        return "unknown";
    }
}

static const char *node_json_content_state(const uint8_t st) {
    switch (st) {
    case IOTDATA_NODE_STATUS_CONTENT_STATE_HOLDING:
        return "holding";
    case IOTDATA_NODE_STATUS_CONTENT_STATE_FETCHING:
        return "fetching";
    case IOTDATA_NODE_STATUS_CONTENT_STATE_CACHING:
        return "caching";
    case IOTDATA_NODE_STATUS_CONTENT_STATE_FAILED:
        return "failed";
    default:
        return "unknown";
    }
}

static void node_json_table_row(cJSON *const arr, const uint8_t entry, const uint8_t *const val, const uint8_t vlen) {
    /* A row may be LONGER than we know about: entries grow by appending, and an older reader must
       ignore the tail rather than discard every row. Shorter is still malformed. */
    if (vlen < iotdata_node_status_entry_size(entry))
        return;
    cJSON *const o = cJSON_CreateObject();
    if (o == NULL)
        return;
    char idbuf[4 + 1];
    cJSON_AddStringToObject(o, "station", snprintf_inline(idbuf, sizeof(idbuf), "%03" PRIX16, iotdata_node_entry_get_u16(val, 0)));
    switch (entry) {
    case IOTDATA_NODE_STATUS_STATIONS_ENTRY:
        cJSON_AddStringToObject(o, "kind", node_json_table_kind(val[2]));
        cJSON_AddNumberToObject(o, "variant", val[3]);
        cJSON_AddNumberToObject(o, "rssi", (double)(int8_t)val[4]);
        cJSON_AddNumberToObject(o, "age", iotdata_node_entry_get_u16(val, 5));
        cJSON_AddNumberToObject(o, "rx", iotdata_node_entry_get_u32(val, 7));
        break;
    case IOTDATA_NODE_STATUS_MESH_PEERS_ENTRY:
        cJSON_AddStringToObject(o, "gateway", snprintf_inline(idbuf, sizeof(idbuf), "%03" PRIX16, iotdata_node_entry_get_u16(val, 2)));
        cJSON_AddNumberToObject(o, "cost", val[4]);
        cJSON_AddNumberToObject(o, "generation", iotdata_node_entry_get_u16(val, 5));
        cJSON_AddNumberToObject(o, "rssi", (double)(int8_t)val[7]);
        cJSON_AddNumberToObject(o, "age", iotdata_node_entry_get_u16(val, 8));
        cJSON_AddBoolToObject(o, "accepting", (val[10] & IOTDATA_NODE_STATUS_MESH_PEERS_FLAG_ACCEPTING) != 0);
        cJSON_AddBoolToObject(o, "parent", (val[10] & IOTDATA_NODE_STATUS_MESH_PEERS_FLAG_PARENT) != 0);
        break;
    case IOTDATA_NODE_STATUS_CONTENT_ENTRY:
        cJSON_AddNumberToObject(o, "content_id", (double)(((uint32_t)val[0] << 16) | ((uint32_t)val[1] << 8) | val[2]));
        cJSON_AddStringToObject(o, "state", node_json_content_state(val[3]));
        cJSON_AddBoolToObject(o, "persistent", (val[4] & IOTDATA_NODE_STATUS_CONTENT_FLAG_PERSISTENT) != 0);
        cJSON_AddNumberToObject(o, "held", iotdata_node_entry_get_u16(val, 5));
        cJSON_AddNumberToObject(o, "total", iotdata_node_entry_get_u16(val, 7));
        cJSON_AddNumberToObject(o, "expiry", iotdata_node_entry_get_u16(val, 9));
        cJSON_AddNumberToObject(o, "idle", iotdata_node_entry_get_u16(val, 11));
        break;
    case IOTDATA_NODE_STATUS_FILTERS_ENTRY:
        cJSON_AddStringToObject(o, "action", val[2] == IOTDATA_NODE_CONTROL_NODE_FILTERS_ALLOW ? "allow" : val[2] == IOTDATA_NODE_CONTROL_NODE_FILTERS_BLOCK ? "block" : "none");
        cJSON_AddStringToObject(o, "source", val[3] == IOTDATA_NODE_CONTROL_NODE_FILTERS_SCOPE_AUTO ? "auto" : "manual");
        break;
    default:
        break;
    }
    cJSON_AddItemToArray(arr, o);
}

/* The array a table's rows go into, created on first use and named for the table, so one STATUS
   object can carry the scalars and any of the three tables side by side. */
static cJSON *node_json_table_array(cJSON *const obj, const uint8_t entry) {
    const char *const name = entry == IOTDATA_NODE_STATUS_STATIONS_ENTRY ? "stations" : entry == IOTDATA_NODE_STATUS_FILTERS_ENTRY ? "filters" : entry == IOTDATA_NODE_STATUS_CONTENT_ENTRY ? "content" : "peers";
    cJSON *arr = cJSON_GetObjectItem(obj, name);
    if (arr == NULL)
        arr = cJSON_AddArrayToObject(obj, name);
    return arr;
}

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
        if (type == IOTDATA_NODE_TLV_VERSION && iotdata_node_version_json_key(obj, name, key, val, vlen)) {
            // empty
        } else if (type == IOTDATA_NODE_TLV_STATUS && iotdata_node_status_json_key(obj, name, key, val, vlen)) {
            /* A variant pair carries its variant number inside the value, so render it under that
               number rather than under the key -- "variant"/"names" repeated would collide. The slots
               are rendered as slots: a hex blob says nothing about which field sits where, and an
               empty one is null, which is what the receiver is actually being told. */
        } else if (type == IOTDATA_NODE_TLV_VARIANT && (key == IOTDATA_NODE_VARIANT_ENTRY || key == IOTDATA_NODE_VARIANT_NAMES)) {
            const uint8_t variant = iotdata_node_variant_of(val, vlen);
            if (variant != IOTDATA_NODE_VARIANT_NONE) { /* malformed: dropped rather than misread */
                char vbuf[16];
                cJSON *ent = cJSON_GetObjectItem(obj, snprintf_inline(vbuf, sizeof(vbuf), "variant_%u", (unsigned)variant));
                if (ent == NULL)
                    ent = cJSON_AddObjectToObject(obj, vbuf);
                if (ent != NULL) {
                    if (key == IOTDATA_NODE_VARIANT_ENTRY) {
                        cJSON *const slots = cJSON_AddArrayToObject(ent, "fields");
                        const size_t n = iotdata_node_variant_decode(val, vlen);
                        for (size_t i = 0; i < n && slots != NULL; i++) {
                            const uint16_t id = iotdata_node_variant_field_at(val, vlen, i);
                            cJSON_AddItemToArray(slots, id == IOTDATA_NODE_VARIANT_FIELD_NONE ? cJSON_CreateNull() : cJSON_CreateNumber((double)id));
                        }
                    } else {
                        const char *const vn = iotdata_node_variant_name_at(val, vlen, 0);
                        if (vn != NULL)
                            cJSON_AddStringToObject(ent, "name", vn);
                        cJSON *const labels = cJSON_AddArrayToObject(ent, "labels");
                        for (size_t i = 1; labels != NULL; i++) {
                            const char *const f = iotdata_node_variant_name_at(val, vlen, i);
                            if (f == NULL)
                                break;
                            cJSON_AddItemToArray(labels, cJSON_CreateString(f));
                        }
                    }
                }
            }
        } else if (type == IOTDATA_NODE_TLV_STATUS && iotdata_node_status_entry_size(key) != 0) {
            node_json_table_row(node_json_table_array(obj, key), key, val, vlen);
        } else {
            if (width == 1 && vlen == 1)
                cJSON_AddNumberToObject(obj, name, (double)val[0]);
            else if (width == 2 && vlen == 2)
                cJSON_AddNumberToObject(obj, name, (double)iotdata_kvr_u16(val, vlen, 0));
            else if (width == 4 && vlen == 4)
                cJSON_AddNumberToObject(obj, name, (double)iotdata_kvr_u32(val, vlen, 0));
            else if (vlen == 0)
                cJSON_AddBoolToObject(obj, name, true); /* a flag: present, no value */
            else if (!node_json_value_is_text(val, vlen)) {
                char hex[2 * IOTDATA_TLV_LENGTH_MAX + 1]; // XXX
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
}

static void node_publish_from(node_state_t *st, const uint16_t station, const uint8_t type, const uint8_t *kvbuf, const size_t kvlen, const iotdata_node_partial_t *partial) {
    cJSON *root = cJSON_CreateObject();
    if (root) {
        char buf[4 + 1];
        cJSON_AddStringToObject(root, "resp", "node"); /* shares a topic with the other responses */
        cJSON_AddStringToObject(root, "station", snprintf_inline(buf, sizeof(buf), "%03" PRIX16, station));
        cJSON_AddStringToObject(root, "tlv", iotdata_node_tlv_name(type) ? iotdata_node_tlv_name(type) : "?");
        if (partial != NULL) {
            cJSON *const jp = cJSON_AddObjectToObject(root, "_partial");
            if (jp != NULL) {
                cJSON_AddNumberToObject(jp, "id", partial->id);
                cJSON_AddNumberToObject(jp, "total", partial->total);
                cJSON_AddNumberToObject(jp, "index", partial->index);
            }
        }
        cJSON *data = cJSON_AddObjectToObject(root, "data");
        if (data != NULL) {
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

static bool node_report_paged(node_state_t *st, const uint8_t type, const uint8_t scope, iotdata_node_partial_t *partial) {
    iotdata_node_partial_begin(partial, iotdata_node_sequence(st->inode));
    const int kvlen = node_build(st, type, st->_buffer_kv, sizeof(st->_buffer_kv), scope, partial);
    if (kvlen < 0) /* not 0: an empty payload is a legitimate report */
        return false;
    st->stat_reports++;
    node_publish_from(st, iotdata_node_station(st->inode), type, st->_buffer_kv, (size_t)kvlen, iotdata_node_partial_needed(partial) ? partial : NULL);
    iotdata_node_partial_sent(partial);
    return true;
}

static bool node_report_scoped(node_state_t *st, const uint8_t type, const uint8_t scope) {
    iotdata_node_partial_t partial = { 0 };
    return node_report_paged(st, type, scope, &partial);
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
        /* REQUEST: [subject][selection...] */
        if (key == IOTDATA_NODE_CONTROL_REQUEST && vlen >= 1) {
            const uint8_t subject = val[0];
            if (iotdata_node_tlv_is_reportable(subject)) {
                st->stat_requests++;
                const uint8_t scope = (subject == IOTDATA_NODE_TLV_STATUS && vlen >= 2) ? val[1] : 0u;
                if (!node_report_scoped(st, subject, scope))
                    PRINTF_INFO("node: request for %s -- nothing to report\n", iotdata_node_tlv_name(subject) ? iotdata_node_tlv_name(subject) : "?");
            } else {
                st->stat_unknown++;
                PRINTF_INFO("node: request for subject 0x%02X ignored (nothing to report)\n", subject);
            }
        }
        /* CONTROL: [subject][action][args...] */
        else if (key == IOTDATA_NODE_CONTROL_CONTROL && vlen >= 2) {
            const uint8_t subject = val[0], action = val[1];
            const uint8_t *const args = (vlen > 2) ? val + 2 : NULL;
            const uint8_t arglen = (uint8_t)(vlen - 2);
            if (subject == IOTDATA_NODE_SUBJECT_NODE && action == IOTDATA_NODE_ACTION_NODE_REBOOT) {
                st->stat_commands++;
                st->reboot_requested = true;
                PRINTF_INFO("node: REBOOT requested (delay=%" PRIu16 "s) -- stopping; the service unit restarts us\n", iotdata_kvr_u16(args, arglen, 0));
            } else {
                if (st->control != NULL && st->control(subject, action, args, arglen))
                    st->stat_commands++;
                else {
                    st->stat_unknown++; /* unknown or not implemented: skip it, do not fail the packet */
                    PRINTF_INFO("node: control %02X/%02X ignored (not implemented)\n", subject, action);
                }
            }
        } else {
            st->stat_unknown++; /* a proprietary key, or one from a newer manager */
        }
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool node_addressed_to_us(const node_state_t *st, const uint16_t station) {
    return station == iotdata_node_station(st->inode) || station == IOTDATA_STATION_BROADCAST;
}

/* `tlv` says what the payload IS. CONTROL for a command, CONFIG for a write -- a write is not a
   verb, so it travels as its own TLV rather than wrapped in a command, exactly as SETTINGS does. */
static bool node_send_down(node_state_t *st, const uint8_t *kvbuf, const size_t kvlen, const uint16_t target, const uint8_t tlv) {
    bool ok = false;
    const buffer_handle_t h = buffer_acquire(st->pool);
    if (h != BUFFER_NONE) {
        uint8_t *const buf = buffer_data(st->pool, h);
        size_t len = 0;
        ok = iotdata_encode_begin(&st->_iotdata_enc, buf, buffer_room(st->pool, h), 0, target, IOTDATA_SEQUENCE_DOWN) == IOTDATA_OK && iotdata_encode_tlv(&st->_iotdata_enc, tlv, kvbuf, (uint8_t)kvlen) == IOTDATA_OK &&
             iotdata_encode_end(&st->_iotdata_enc, &len) == IOTDATA_OK;
        if (ok) {
            const iotdata_node_down_ev_t ev = iotdata_node_down_offer(&st->down, target, buf, len, h, 0u /* the gateway does not model its own air rate: 0 means "assume it fits" */, hw_time_ms());
            PRINTF_INFO("node: down -> %03" PRIX16 " (%zu bytes, %s, %s)\n", target, len, iotdata_node_down_ev_name(ev), iotdata_node_tlv_name(tlv) ? iotdata_node_tlv_name(tlv) : "?");
            ok = st->tx(buf, (int)len);
        }
        buffer_unref(st->pool, h); /* written straight to the radio (and copied by the DOWN store) */
    } else
        PRINTF_ERROR("node: down -> %03" PRIX16 " failed: no frame buffer (pool %u/%u)\n", target, (unsigned)buffer_pool_used(st->pool), (unsigned)buffer_pool_total(st->pool));
    return ok;
}

static bool node_on_packet(node_state_t *const st, const uint8_t *buf, const size_t len, const uint16_t station) {
    bool ok = false;
    if (iotdata_decode(buf, len, &st->_iotdata_dec) == IOTDATA_OK && st->_iotdata_dec.tlv_count > 0) {
        /* This station has just spoken. If it says it is listening and we are holding a command for
        it, now is the only moment we can deliver -- it may be asleep again by the next tick. We are
        decoding anyway here, so unlike the relay there is nothing to check cheaply first. */
        iotdata_node_receive_t rx;
        if (iotdata_node_receive_find(&st->_iotdata_dec, &rx)) {
            /* Snooped unconditionally: the advertisement is how the table learns this station is
               deliverable at all, so it is recorded even when nothing is held for it yet. */
            size_t held_len = 0;
            const buffer_handle_t held = iotdata_node_down_window(&st->down, station, &rx, 0u /* the gateway does not model its own air rate: 0 means "assume it fits" */, hw_time_ms(), &held_len);
            if (held != BUFFER_NONE) {
                PRINTF_INFO("node: %03" PRIX16 " is listening -> delivering %zu byte(s) held\n", station, held_len);
                (void)st->tx(buffer_data(st->pool, held), (int)held_len);
                buffer_unref(st->pool, held); /* the table handed us a reference of our own */
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
        if (iotdata_node_is_down(st->_iotdata_dec.sequence)) {
            /*
             * A DOWN frame ADDRESSED TO US. A gateway is usually the top of the pile and only ever
             * sends these, but it is a station like any other: with two or three gateways, one can
             * address a command at another's station id, and the frame may cross several relay
             * hops to get there. Ignoring it would make a gateway the one node in the fleet that
             * cannot be driven over the air -- which is also why this must reach exactly the same
             * function the MQTT path does, rather than a second implementation that drifts.
             *
             * `station` is the TARGET here, not the sender: IOTDATA_SEQUENCE_DOWN inverts what the
             * field means. An exact match cannot be our own echo, because a gateway never sends a
             * DOWN frame to itself (node_on_mqtt executes those locally instead of airing them).
             *
             * A BROADCAST down is deliberately not executed here. It is addressed to us in the
             * sense that broadcast is addressed to everyone, but we cannot tell another gateway's
             * broadcast from the echo of our own coming back off a relay -- and acting on our own
             * echo would double-count and double-report every broadcast we send. Idempotence makes
             * that harmless rather than correct, so it waits for a way to tell them apart.
             */
            if (station == iotdata_node_station(st->inode))
                for (uint8_t i = 0; i < st->_iotdata_dec.tlv_count; i++) {
                    const iotdata_decoder_tlv_t *const t = &st->_iotdata_dec.tlv[i];
                    if (t->format == IOTDATA_TLV_FMT_RAW && t->type == IOTDATA_NODE_TLV_CONTROL) {
                        st->stat_rx++;
                        /* no sender to name: a DOWN frame's station field is the TARGET, and the
                        header carries nobody's return address */
                        PRINTF_INFO("node: CONTROL over the air, addressed to us (%u byte(s))\n", (unsigned)t->length);
                        node_process_control(st, t->raw, t->length);
                        ok = true;
                    } else if (t->format == IOTDATA_TLV_FMT_RAW && t->type == IOTDATA_NODE_TLV_SETTINGS) {
                        /* A SETTINGS TLV arriving IS the write: a write is not a verb, so nothing
                        carries it. Apply, persist NOW rather than on the tick, then answer with
                        what is true -- a value that did not take is simply not in the reply. */
                        st->stat_rx++;
                        st->stat_commands++;
                        PRINTF_INFO("node: SETTINGS write over the air (%u byte(s))\n", (unsigned)t->length);
                        if (st->settings != NULL && iotdata_node_settings_apply(st->settings, t->raw, t->length)) {
                            (void)iotdata_node_settings_commit(st->settings, st->state);
                            /* a written station takes effect NOW, so the reply goes out as who we
                            have become rather than as who we were */
                            (void)iotdata_node_adopt_station(st->inode, iotdata_node_settings_station(st->settings));
                        }
                        (void)node_report(st, IOTDATA_NODE_TLV_SETTINGS);
                        ok = true;
                    }
                }
        } else
            for (uint8_t i = 0; i < st->_iotdata_dec.tlv_count; i++) {
                const iotdata_decoder_tlv_t *const t = &st->_iotdata_dec.tlv[i];
                if (!iotdata_node_partial_is(t))
                    if (iotdata_tlv_type_is_system(t->type) && t->format == IOTDATA_TLV_FMT_RAW) {
                        st->stat_rx++;
                        iotdata_node_partial_t p;
                        const bool paged = iotdata_node_partial_of(&st->_iotdata_dec, i, &p);
                        node_publish_from(st, station, t->type, t->raw, t->length, paged ? &p : NULL);
                        ok = true;
                    }
            }
    }
    return ok;
}

/* A staged payload on its way out. `tlv` is what the queue tagged it as: a CONTROL command, which
   the gateway may also be the subject of, or a CONFIG write, which it relays. It does not yet apply
   one to ITSELF -- its own configuration is still the .cfg file, and answering as though it had
   would report something that did not happen. */
static void node_on_mqtt(node_state_t *const st, const uint8_t *kvbuf, const size_t kvlen, const uint16_t target, const uint8_t tlv) {
    if (node_addressed_to_us(st, target)) {
        st->stat_rx++;
        if (tlv == IOTDATA_NODE_TLV_CONTROL)
            node_process_control(st, kvbuf, kvlen);
        else if (tlv == IOTDATA_NODE_TLV_CONFIG) {
            st->stat_commands++;
            if (st->config_apply != NULL)
                (void)st->config_apply(kvbuf, kvlen, NULL);
            (void)node_report(st, IOTDATA_NODE_TLV_CONFIG);
        }
    }
    if (target != iotdata_node_station(st->inode))
        (void)node_send_down(st, kvbuf, kvlen, target, tlv);
}

/* Attach the gateway's own configuration. Separate for the reason the relay's is: the table does not
   exist yet when the node layer is set up. Unused while the gateway is unmigrated. */
__attribute__((unused)) static void node_config_attach(node_state_t *const st, const iotdata_node_config_build_fn build, const iotdata_node_config_apply_fn apply) {
    st->config_build = build;
    st->config_apply = apply;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static void node_tick(node_state_t *const st) {
    const time_t now = time(NULL);
    const int stale = iotdata_node_down_tick(&st->down, hw_time_ms());
    if (stale > 0)
        PRINTF_INFO("node: %d held command(s) expired unclaimed (ttl=%" PRIu32 "s)\n", stale, iotdata_node_down_ttl_ms(&st->down) / 1000u);
    if (!st->startup_done) {
        st->startup_done = true;
        /* bit N selects type N: the mapping table this replaces said the same thing in six entries
           that could drift from the numbering they mirrored. */
        for (uint8_t t = 0; t < IOTDATA_NODE_TLV_SYSTEM_COUNT; t++)
            if (iotdata_node_tlv_is_reportable(t) && node_at_startup(st, t))
                (void)node_report(st, t);
    }
    for (uint8_t type = 1; type < (uint8_t)(sizeof(st->period) / sizeof(st->period[0])); type++) {
        const uint16_t period_s = node_period_s(st, type);
        if (period_s != 0) {
            if (st->period_last[type] == 0)
                st->period_last[type] = now; /* prime: first report after one full interval */
            else if ((uint32_t)(now - st->period_last[type]) >= period_s) {
                st->period_last[type] = now;
                (void)node_report(st, type);
            }
        }
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

/* Attach the protocol settings and their store. Separate from node_begin() so its parameter list
   does not grow a pair that most callers pass as NULL. */
static void node_settings_attach(node_state_t *st, iotdata_node_settings_t *settings, iotdata_node_state_t *state) {
    if (st == NULL)
        return;
    st->settings = settings;
    st->state = state;
}

static bool node_begin(node_state_t *st, iotdata_node_t *const inode, const iotdata_node_version_caps_t *caps, const stat_state_t *stat, bbox_state_t *bbox, buffer_pool_t *pool, node_tx_handler_t tx, node_control_handler_t control,
                       node_status_mesh_handler_t status_mesh, node_table_count_handler_t table_count, node_table_row_handler_t table_row, const uint8_t *control_actions, const uint8_t control_actions_count, const char *topic_prefix) {
    assert(stat && bbox && pool && tx && topic_prefix);
    st->inode = inode;
    st->caps = caps;
    st->stat = stat;
    st->bbox = bbox;
    st->pool = pool;
    st->tx = tx;
    st->control = control;
    st->status_mesh = status_mesh;
    st->table_count = table_count;
    st->table_row = table_row;
    st->control_actions = control_actions;
    st->control_actions_count = control_actions_count;
    iotdata_node_down_init(&st->down, pool);
    /* The same defaults every other node comes up with. They used to be absent here, which left
       BOTH at zero -- no startup burst, no periodic STATUS -- so a gateway announced itself only
       when asked. Nobody decided that; it was the difference between a module that had the numbers
       and one that did not. SETTINGS still overrides either. */
    if (st->startup == 0)
        st->startup = IOTDATA_NODE_SETTINGS_DEFAULT_STARTUP;
    if (st->period[IOTDATA_NODE_TLV_STATUS] == 0)
        st->period[IOTDATA_NODE_TLV_STATUS] = (uint16_t)IOTDATA_NODE_SETTINGS_DEFAULT_PERIOD_STATUS_S;
    snprintf(st->topic_resp, sizeof(st->topic_resp), "%s" IOTDATA_NODE_MQTT_MANAGE_TOPIC_RESP, topic_prefix ? topic_prefix : "iotdata");
    PRINTF_INFO("node: station=%03" PRIX16 ", responses on %s\n", iotdata_node_station(st->inode), st->topic_resp);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void node_end(__attribute__((unused)) node_state_t *st) {
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
