
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include <cjson/cJSON.h>

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#ifndef MQTT_CLIENT_DEFAULT
#define MQTT_CLIENT_DEFAULT "iotdata_gateway"
#endif
#ifndef MQTT_SERVER_DEFAULT
#define MQTT_SERVER_DEFAULT "mqtt://localhost"
#endif
#ifndef MQTT_TLS_DEFAULT
#define MQTT_TLS_DEFAULT false
#endif
#ifndef MQTT_SYNCHRONOUS_DEFAULT
#define MQTT_SYNCHRONOUS_DEFAULT false
#endif
#ifndef MQTT_TOPIC_PREFIX_DEFAULT
#define MQTT_TOPIC_PREFIX_DEFAULT "iotdata"
#endif
#ifndef MQTT_RECONNECT_DELAY_DEFAULT
#define MQTT_RECONNECT_DELAY_DEFAULT 5
#endif
#ifndef MQTT_RECONNECT_DELAY_MAX_DEFAULT
#define MQTT_RECONNECT_DELAY_MAX_DEFAULT 60
#endif

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

/* Staged commands waiting for the main loop. Bounded on purpose: they come out of the same pool
   the receive path draws from, so an operator hammering the request topic must not be able to
   starve it. A full queue is an answer ("busy"), not a silent overwrite. */
#define CTRL_QUEUE_MAX    4
#define CTRL_QUEUE_TTL_MS 30000 /* a command nobody could send in 30s is stale, not queued */

#define CTRL_TAG_CONTROL  1

static const char *ctrl_tag_name(const uint8_t tag) {
    return tag == CTRL_TAG_CONTROL ? "CONTROL" : "NONE";
}

// -----------------------------------------------------------------------------------------------------------------------------------------

typedef struct {
    uint16_t station_id;
    char topic_req[128];
    buffer_pool_t *pool;
    /* Staging queue: produced by the mqtt-thread callback, drained by the main loop. Everything
       the callback wants done goes through here, because the mosquitto thread must not touch the
       radio or any of the node/mesh state the main loop owns.
       A QUEUE and not a single slot, because each request is a deliberate operator action and they
       are not interchangeable: three commands aimed at three stations must all arrive, and a slot
       delivers only the last. Ordering is by arrival -- `due_ms` is when the request landed, so
       they come back out in the order they were sent.
       This is the pool's cross-thread HANDOFF case: the callback acquires a buffer and builds the
       CONTROL payload in it, the main loop takes the handle and releases it. Only one thread ever
       holds a given buffer, so there is no moment where both sides touch the bytes. Neither the
       pool nor the queue needs a mutex HERE, because both carry their own (see the BUFFER_LOCK_*
       macros in iotdata_gateway.c) -- which is why this state has no lock of its own.
       The target station travels as the queue entry's key; the length lives in the buffer. */
    buffer_queue_t queue;
    buffer_queue_entry_t queue_slot[CTRL_QUEUE_MAX];
    bbox_state_t *bbox;
    char topic_resp[128];
    char _buffer_resp[244];
    char _buffer_blackbox_rec[BLACKBOX_LINE_MAX];
    time_t blackbox_tick_last; /* for the periodic (batched) flush tick */
    /* stats (the queue keeps its own: added, expired, rejected) */
    uint32_t stat_req_rx, stat_req_bad;
} ctrl_state_t;

/* The mosquitto message callback has no user-data argument, so the state is reached
   through this file-scope pointer, set in ctrl_begin. */
static ctrl_state_t *g_ctrl = NULL;

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

/* Management responses are JSON on the one response topic (IOTDATA_MQTT_MANAGE_TOPIC_RESP), sharing it
   with the node reports. Text that used to be published bare -- a blackbox status line, a record,
   an error -- is now a field, so a client parses one shape instead of sniffing at payloads.
 *
 * "resp" is the CLASS of response:
 *   node   a node TLV report          (+ "tlv", "data")   -- see node_publish_from
 *   diag   diagnostic output          (+ "source", and "text" or "index"/"record")
 *   error  the request was not usable (+ "text")
 * and "station" is always the node that answered, which for these is this gateway.
 *
 * A diag response names its SOURCE, because the blackbox recorder is one possible diagnostic and
 * not the definition of one: a different recorder, or a remote node's DIAGNOSTICS TLV, is the same
 * class of answer from a different place. The payload stays a string -- decoding a blackbox record
 * needs the @blackbox definitions header, which the client has and the gateway does not. */
#define CTRL_DIAG_SOURCE "blackbox" /* the recorder this gateway happens to run */

static void ctrl_respond(ctrl_state_t *const st, cJSON *const root) {
    if (root != NULL) {
        char idbuf[4 + 1];
        cJSON_AddStringToObject(root, "station", snprintf_inline(idbuf, sizeof(idbuf), "%04" PRIX16, st->station_id));
        char *const out = cJSON_PrintUnformatted(root);
        if (out != NULL) {
            (void)mqtt_send(st->topic_resp, out, (int)strlen(out));
            free(out);
        }
        cJSON_Delete(root);
    }
}

static cJSON *ctrl_respond_begin(const char *const resp, const char *const cmd) {
    cJSON *const root = cJSON_CreateObject();
    if (root != NULL) {
        cJSON_AddStringToObject(root, "resp", resp);
        cJSON_AddStringToObject(root, "cmd", cmd);
    }
    return root;
}

static void ctrl_respond_diag(ctrl_state_t *const st, const char *const cmd, const char *const text) {
    cJSON *const root = ctrl_respond_begin("diag", cmd);
    if (root != NULL) {
        cJSON_AddStringToObject(root, "source", CTRL_DIAG_SOURCE);
        cJSON_AddStringToObject(root, "text", text);
        ctrl_respond(st, root);
    }
}

static void ctrl_respond_diag_record(ctrl_state_t *const st, const char *const cmd, const int index, const char *const record) {
    cJSON *const root = ctrl_respond_begin("diag", cmd);
    if (root != NULL) {
        cJSON_AddStringToObject(root, "source", CTRL_DIAG_SOURCE);
        cJSON_AddNumberToObject(root, "index", index);
        cJSON_AddStringToObject(root, "record", record);
        ctrl_respond(st, root);
    }
}

static void ctrl_respond_error(ctrl_state_t *const st, const char *const cmd, const char *const text) {
    cJSON *const root = ctrl_respond_begin("error", cmd);
    if (root != NULL) {
        cJSON_AddStringToObject(root, "text", text);
        ctrl_respond(st, root);
    }
    PRINTF_ERROR("ctrl: %s: %s\n", cmd, text);
}

static void ctrl_diag_enable(const bool on) {
    ctrl_state_t *const st = g_ctrl;
    blackbox_enable(&st->bbox->handle, on);
    ctrl_respond_diag(st, on ? "diag-enable" : "diag-disable", on ? "enabled" : "disabled");
}

static void ctrl_diag_clear(void) {
    ctrl_state_t *const st = g_ctrl;
    blackbox_clear(&st->bbox->handle);
    ctrl_respond_diag(st, "diag-clear", "cleared");
}

static void ctrl_diag_dump(void) {
    ctrl_state_t *const st = g_ctrl;
    size_t cur = 0;
    int n = 0;
    while (blackbox_pull(&st->bbox->handle, &cur, st->_buffer_blackbox_rec, sizeof(st->_buffer_blackbox_rec)) > 0)
        ctrl_respond_diag_record(st, "diag-dump", n++, st->_buffer_blackbox_rec);
    ctrl_respond_diag(st, "diag-dump", snprintf_inline(st->_buffer_resp, sizeof(st->_buffer_resp), "dumped %d records", n));
}

static void ctrl_from_message(const char *topic __attribute__((unused)), const unsigned char *payload, const int len) {
    ctrl_state_t *const st = g_ctrl;
    st->stat_req_rx++;
    cJSON *const root = cJSON_ParseWithLength((const char *)payload, (size_t)len);
    if (root != NULL) {
        iotdata_control_args_t args;
        const iotdata_control_command_t *const c = iotdata_control_from_json(root, &args);
        const cJSON *const jc = cJSON_GetObjectItem(root, "cmd");
        const char *const cmd = (cJSON_IsString(jc) && jc->valuestring != NULL) ? jc->valuestring : "";
        if (c != NULL) {
            const buffer_handle_t h = buffer_acquire(st->pool);
            if (h != BUFFER_NONE) {
                iotdata_kvr_t kv;
                iotdata_kvr_init(&kv, buffer_data(st->pool, h), buffer_room(st->pool, h));
                (void)iotdata_control_build(&kv, c, &args);
                const size_t n = kv.overflow ? 0u : kv.len;
                if (n > 0) {
                    buffer_set_len(st->pool, h, (uint16_t)n);
                    const uint32_t now_ms = (uint32_t)__ticks_ms();
                    if (buffer_queue_add(&st->queue, h, now_ms, now_ms + CTRL_QUEUE_TTL_MS, CTRL_TAG_CONTROL, args.target))
                        PRINTF_INFO("ctrl: %s target=%04X station=%04X -> %zu byte control (staged %u)\n", cmd, (unsigned)args.target, (unsigned)args.station, n, (unsigned)buffer_queue_count(&st->queue));
                    else {
                        ctrl_respond_error(st, cmd, snprintf_inline(st->_buffer_resp, sizeof(st->_buffer_resp), "busy: %u commands already staged", (unsigned)buffer_queue_count(&st->queue)));
                        st->stat_req_bad++;
                    }
                } else {
                    ctrl_respond_error(st, cmd, snprintf_inline(st->_buffer_resp, sizeof(st->_buffer_resp), "payload would not fit (%zu bytes)", kv.len));
                    st->stat_req_bad++;
                }
                buffer_unref(st->pool, h);
            } else {
                ctrl_respond_error(st, cmd, "no frame buffer available");
                st->stat_req_bad++;
            }
        } else {
            ctrl_respond_error(st, cmd, "unknown command");
            st->stat_req_bad++;
        }
        cJSON_Delete(root);
    } else {
        PRINTF_ERROR("ctrl: bad JSON request (%d bytes)\n", len);
        st->stat_req_bad++;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static void ctrl_report_peers(const stat_state_t *const s) {
    PRINTF_INFO("exec: peers - %d\n", s->peers_count);
    for (int i = 0, c = 0; i < (int)(sizeof(s->peers) / sizeof(s->peers[0])) && c < s->peers_count; i++) {
        const stat_peer_t *const e = &s->peers[i];
        if (e->valid) {
            c++;
            PRINTF_INFO("exec:   %04" PRIX16 " cost=%u gen=%u flags=0x%02" PRIX8 " age=%lds\n", e->station_id, (unsigned)e->cost, (unsigned)e->generation, e->flags, (long)(time(NULL) - e->last_seen));
        }
    }
}

static void ctrl_report_filter(const filter_t *const f) {
    PRINTF_INFO("exec: filters - %d\n", filter_count(f));
    for (int i = 0, c = 0; i < (int)(sizeof(f->e) / sizeof(f->e[0])) && c < f->count; i++) {
        const filter_entry_t *const e = &f->e[i];
        if (e->valid) {
            c++;
            PRINTF_INFO("exec:   %04" PRIX16 " %s (%s)\n", e->station, e->action == FILTER_ALLOW ? "allow" : "block", e->source == FILTER_AUTO ? "auto" : "manual");
        }
    }
}

static const uint8_t ctrl_from_iotdata_keys[] = {
    IOTDATA_NODE_CONTROL_MESH_STATIONS_DUMP, IOTDATA_NODE_CONTROL_MESH_PEERS_UPDATE, IOTDATA_NODE_CONTROL_MESH_PEERS_CLEAR,   IOTDATA_NODE_CONTROL_MESH_PEERS_DUMP,   IOTDATA_NODE_CONTROL_MESH_FILTERS_UPDATE,
    IOTDATA_NODE_CONTROL_MESH_FILTERS_CLEAR, IOTDATA_NODE_CONTROL_MESH_FILTERS_DUMP, IOTDATA_NODE_CONTROL_DIAGNOSTICS_ENABLE, IOTDATA_NODE_CONTROL_DIAGNOSTICS_CLEAR, IOTDATA_NODE_CONTROL_DIAGNOSTICS_DUMP,
};

// -----------------------------------------------------------------------------------------------------------------------------------------

static bool ctrl_from_iotdata(const uint8_t key, const uint8_t *const val, const uint8_t vlen) {
    process_state_t *const st = g_exec;
    if (st == NULL)
        return false;
    switch (key) {

    case IOTDATA_NODE_CONTROL_MESH_STATIONS_DUMP:
        PRINTF_INFO("exec: CONTROL - MESH_STATIONS_DUMP\n");
        netw_report(&st->network, st->state_mesh->station_id);
        return true;

    case IOTDATA_NODE_CONTROL_MESH_PEERS_DUMP:
        PRINTF_INFO("exec: CONTROL - MESH_PEERS_DUMP\n");
        ctrl_report_peers(st->state_stat);
        return true;
    case IOTDATA_NODE_CONTROL_MESH_PEERS_UPDATE: {
        int n = 0;
        for (int i = 0; i + IOTDATA_NODE_CONTROL_MESH_UPDATE_ENTRY_SIZE <= (int)vlen; i += IOTDATA_NODE_CONTROL_MESH_UPDATE_ENTRY_SIZE) {
            const uint16_t station = (uint16_t)(((uint16_t)val[i] << 8) | val[i + 1]);
            if (val[i + 2] == IOTDATA_NODE_CONTROL_MESH_PEER_NONE) {
                PRINTF_INFO("exec: CONTROL - MESH_PEERS_UPDATE %04" PRIX16 " none -> %s\n", station, stat_mesh_peer_remove(st->state_stat, station) ? "removed" : "not present");
                n++;
            } else
                PRINTF_INFO("exec: CONTROL - MESH_PEERS_UPDATE %04" PRIX16 " action=0x%02" PRIX8 " (unknown, ignored)\n", station, val[i + 2]);
        }
        if (n > 0)
            ctrl_report_peers(st->state_stat);
        return true;
    }
    case IOTDATA_NODE_CONTROL_MESH_PEERS_CLEAR:
        PRINTF_INFO("exec: CONTROL - MESH_PEERS_CLEAR -> forgetting %d peer(s)\n", st->state_stat->peers_count);
        stat_mesh_peers_clear(st->state_stat);
        return true;

    case IOTDATA_NODE_CONTROL_MESH_FILTERS_DUMP:
        PRINTF_INFO("exec: CONTROL - MESH_FILTERS_DUMP\n");
        ctrl_report_filter(&st->filter);
        return true;
    case IOTDATA_NODE_CONTROL_MESH_FILTERS_UPDATE: {
        int n = 0;
        for (int i = 0; i + IOTDATA_NODE_CONTROL_MESH_UPDATE_ENTRY_SIZE <= (int)vlen; i += IOTDATA_NODE_CONTROL_MESH_UPDATE_ENTRY_SIZE) {
            const uint16_t station = (uint16_t)(((uint16_t)val[i] << 8) | val[i + 1]);
            switch (val[i + 2]) {
            case IOTDATA_NODE_CONTROL_MESH_FILTERS_NONE:
                PRINTF_INFO("exec: CONTROL - MESH_FILTER_UPDATE %04" PRIX16 " none -> %s\n", station, filter_remove(&st->filter, station) ? "removed" : "not present");
                n++;
                break;
            case IOTDATA_NODE_CONTROL_MESH_FILTERS_BLOCK:
            case IOTDATA_NODE_CONTROL_MESH_FILTERS_ALLOW: {
                /* the wire values are NONE/BLOCK/ALLOW = 0/1/2; the table's are BLOCK/ALLOW = 0/1 */
                const bool allow = (val[i + 2] == IOTDATA_NODE_CONTROL_MESH_FILTERS_ALLOW);
                const bool ok = filter_insert(&st->filter, station, allow ? FILTER_ALLOW : FILTER_BLOCK, FILTER_MANUAL);
                PRINTF_INFO("exec: CONTROL - MESH_FILTER_UPDATE %04" PRIX16 " %s -> %s\n", station, allow ? "allow" : "block", ok ? "ok" : "table full");
                if (ok && !allow) /* make the block take effect on what we already believe */
                    (void)stat_mesh_peer_remove(st->state_stat, station);
                n++;
                break;
            }
            default:
                PRINTF_INFO("exec: CONTROL - MESH_FILTER_UPDATE %04" PRIX16 " action=0x%02" PRIX8 " (unknown, ignored)\n", station, val[i + 2]);
                break;
            }
        }
        if (n > 0)
            ctrl_report_filter(&st->filter);
        return true;
    }
    case IOTDATA_NODE_CONTROL_MESH_FILTERS_CLEAR: {
        const filter_scope_t scope = (vlen >= 1) ? (filter_scope_t)val[0] : FILTER_SCOPE_ALL;
        PRINTF_INFO("exec: CONTROL - MESH_FILTER_CLEAR scope=%u -> %d cleared\n", (unsigned)scope, filter_clear(&st->filter, scope));
        ctrl_report_filter(&st->filter);
        return true;
    }

    case IOTDATA_NODE_CONTROL_DIAGNOSTICS_ENABLE: {
        const bool on = (vlen >= 1) ? (val[0] != 0u) : true;
        PRINTF_INFO("exec: CONTROL - DIAGNOSTICS_ENABLE -> %s\n", on ? "true" : "false");
        ctrl_diag_enable(on);
        return true;
    }
    case IOTDATA_NODE_CONTROL_DIAGNOSTICS_CLEAR:
        PRINTF_INFO("exec: CONTROL - DIAGNOSTICS_CLEAR\n");
        ctrl_diag_clear();
        return true;
    case IOTDATA_NODE_CONTROL_DIAGNOSTICS_DUMP:
        PRINTF_INFO("exec: CONTROL - DIAGNOSTICS_DUMP\n");
        ctrl_diag_dump();
        return true;

    default:
        return false; /* not ours: the node layer counts it unknown */
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void ctrl_tick(ctrl_state_t *const st, node_state_t *const ns) {
    const uint32_t now_ms = (uint32_t)__ticks_ms();
    const uint16_t stale = buffer_queue_expire(&st->queue, now_ms);
    if (stale > 0)
        PRINTF_WARN("ctrl: %u staged command(s) expired unsent\n", (unsigned)stale);
    uint32_t target = 0;
    const buffer_handle_t h = buffer_queue_take(&st->queue, now_ms, NULL, &target);
    if (h != BUFFER_NONE) {
        node_on_mqtt(ns, buffer_data(st->pool, h), buffer_len(st->pool, h), (uint16_t)target);
        buffer_unref(st->pool, h); /* the reference the queue handed over */
    }
    const time_t now = time(NULL);
    if (st->blackbox_tick_last != 0 && st->blackbox_tick_last != now)
        blackbox_tick(&st->bbox->handle, (uint32_t)(now - st->blackbox_tick_last) * 1000u);
    st->blackbox_tick_last = now;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

bool ctrl_begin(ctrl_state_t *st, const char *topic_prefix, uint16_t station_id, bbox_state_t *bbox, buffer_pool_t *pool) {
    assert(st && bbox && pool);
    memset(st, 0, sizeof(*st));
    st->station_id = station_id;
    st->bbox = bbox;
    st->pool = pool;
    buffer_queue_init(&st->queue, st->queue_slot, CTRL_QUEUE_MAX, pool);
    buffer_queue_set_tag_name(&st->queue, ctrl_tag_name);
    snprintf(st->topic_req, sizeof(st->topic_req), "%s" IOTDATA_MQTT_MANAGE_TOPIC_REQ, topic_prefix);
    snprintf(st->topic_resp, sizeof(st->topic_resp), "%s" IOTDATA_MQTT_MANAGE_TOPIC_RESP, topic_prefix);
    g_ctrl = st;
    if (!mqtt_subscribe(st->topic_req, MQTT_PUBLISH_QOS, ctrl_from_message)) {
        PRINTF_ERROR("ctrl: subscribe to '%s' failed\n", st->topic_req);
        return false;
    }
    PRINTF_INFO("ctrl: station=%04" PRIX16 ", request-topic='%s'\n", station_id, st->topic_req);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void ctrl_end(ctrl_state_t *st) {
    g_ctrl = NULL;
    buffer_queue_clear(&st->queue); /* requests staged but never drained still hold buffers */
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
