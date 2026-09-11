
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

static uint16_t ctrl_parse_target(const cJSON *jt) {
    if (jt != NULL) {
        if (cJSON_IsString(jt) && jt->valuestring != NULL) {
            if (strcmp(jt->valuestring, "all") == 0 || strcmp(jt->valuestring, "broadcast") == 0)
                return IOTDATA_STATION_BROADCAST;
            return (uint16_t)(strtol(jt->valuestring, NULL, 0) & 0x0FFF);
        }
        if (cJSON_IsNumber(jt))
            return (uint16_t)(jt->valueint & 0x0FFF);
    }
    return IOTDATA_STATION_BROADCAST;
}

static uint16_t ctrl_parse_station(const cJSON *jt) {
    if (jt != NULL) {
        if (cJSON_IsString(jt) && jt->valuestring != NULL)
            return (uint16_t)(strtol(jt->valuestring, NULL, 0) & 0x0FFF);
        if (cJSON_IsNumber(jt))
            return (uint16_t)(jt->valueint & 0x0FFF);
    }
    return 0;
}

static uint8_t ctrl_parse_scope_filter(const cJSON *js) {
    if (cJSON_IsString(js) && js->valuestring != NULL) {
        if (strcmp(js->valuestring, "manual") == 0)
            return IOTDATA_NODE_CONTROL_MESH_FILTERS_SCOPE_MANUAL;
        if (strcmp(js->valuestring, "auto") == 0)
            return IOTDATA_NODE_CONTROL_MESH_FILTERS_SCOPE_AUTO;
    }
    return IOTDATA_NODE_CONTROL_MESH_FILTERS_SCOPE_ALL;
}

static uint8_t ctrl_parse_action(const cJSON *js) {
    if (cJSON_IsString(js) && js->valuestring != NULL) {
        if (strcmp(js->valuestring, "block") == 0)
            return IOTDATA_NODE_CONTROL_MESH_FILTERS_BLOCK;
        if (strcmp(js->valuestring, "allow") == 0)
            return IOTDATA_NODE_CONTROL_MESH_FILTERS_ALLOW;
    }
    return IOTDATA_NODE_CONTROL_MESH_FILTERS_NONE; /* "none", "remove", absent: no entry */
}

static uint8_t ctrl_parse_scope_status(const cJSON *js) {
    if (cJSON_IsString(js) && js->valuestring != NULL) {
        uint8_t bits = 0;
        if (strstr(js->valuestring, "node") != NULL)
            bits |= IOTDATA_NODE_STATUS_SCOPE_NODE;
        if (strstr(js->valuestring, "mesh") != NULL)
            bits |= IOTDATA_NODE_STATUS_SCOPE_MESH;
        return bits; /* "node,mesh" names both; anything else (including "all") is 0 = every group */
    }
    return 0;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

/* Management responses are JSON on the one response topic (IOTDATA_GATEWAY_TOPIC_RESP), sharing it
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

/*
 * The recorder's actions, reachable from either direction.
 *
 * They are reached two ways and must behave identically: an operator's MQTT request addressed at
 * this gateway, and a node CONTROL key arriving in a DOWN frame (which, for a broadcast, is the
 * same command every other station just received). So they live here as plain functions and both
 * paths call them -- the alternative was the MQTT side short-circuiting its own diag before it
 * ever became a CONTROL key, which is how the two drifted apart in the first place.
 *
 * They reach the state through g_ctrl, like the mosquitto callback does, because a node CONTROL
 * hook has no context argument.
 *
 * Note there is no "diag status" among them: `diag` is DIAGNOSTICS_REQUEST, a request, which the
 * node layer answers with the DIAGNOSTICS TLV -- the same answer a relay gives. The recorder's
 * human-readable pool/canary summary is no longer published; it was a gateway-only shape.
 */
static void ctrl_diag_enable(const bool on) {
    ctrl_state_t *const st = g_ctrl;
    if (st == NULL)
        return;
    blackbox_enable(&st->bbox->handle, on);
    ctrl_respond_diag(st, on ? "diag-enable" : "diag-disable", on ? "enabled" : "disabled");
}

static void ctrl_diag_clear(void) {
    ctrl_state_t *const st = g_ctrl;
    if (st == NULL)
        return;
    blackbox_clear(&st->bbox->handle);
    ctrl_respond_diag(st, "diag-clear", "cleared");
}

static void ctrl_diag_dump(void) {
    ctrl_state_t *const st = g_ctrl;
    if (st == NULL)
        return;
    size_t cur = 0;
    int n = 0;
    while (blackbox_pull(&st->bbox->handle, &cur, st->_buffer_blackbox_rec, sizeof(st->_buffer_blackbox_rec)) > 0)
        ctrl_respond_diag_record(st, "diag-dump", n++, st->_buffer_blackbox_rec);
    snprintf(st->_buffer_resp, sizeof(st->_buffer_resp), "dumped %d records", n);
    ctrl_respond_diag(st, "diag-dump", st->_buffer_resp);
}

/* One entry of a mesh *_UPDATE value: { u16 station (big-endian), u8 action }. The wire format
   takes a list, so several stations can be changed in one command; this gateway's MQTT interface
   names one station per request, so it sends a list of one. */
static bool ctrl_kvr_mesh_update(iotdata_kvr_t *const kv, const uint8_t key, const uint16_t station, const uint8_t action) {
    const uint8_t e[IOTDATA_NODE_CONTROL_MESH_UPDATE_ENTRY_SIZE] = { (uint8_t)(station >> 8), (uint8_t)station, action };
    return iotdata_kvr_add(kv, key, e, (uint8_t)sizeof(e));
}

/*
 * The command vocabulary, as a table.
 *
 * NAMING. A subject is four letters, matching the device's own USB CLI (`vers`, `stat`, `logl`,
 * `boot` there; `vers vari ctrl stat conf diag cont` here), and verbs are full words appended with
 * a hyphen. The mesh subjects keep a `mesh-` prefix because that names a real group in the key
 * space (0x40 upward, as against `type << 3`); there is deliberately no `node-` prefix, because
 * since mesh management became node CONTROL such a prefix would be on every command and would
 * therefore distinguish nothing.
 *
 * The point of the four letters is that an operator drives the same node two ways -- over MQTT and
 * over a serial console -- and should not have to learn two sets of words for it.
 *
 * A table rather than a chain because the aliases are then free: the previous vocabulary is a
 * handful of extra rows, so recipes written against it keep working instead of rotting.
 */
typedef enum {
    CTRL_ARG_NONE,           /* a flag: the key present, no value */
    CTRL_ARG_FIXED,          /* a u8 the command itself fixes (enable = 1, disable = 0) */
    CTRL_ARG_SCOPE_STATUS,   /* a u8 from "scope", read as STATUS_SCOPE_* (which groups) */
    CTRL_ARG_SCOPE_FILTER,   /* a u8 from "scope", read as FILTER_SCOPE_* (which entries) */
    CTRL_ARG_STATION,        /* { u16 station, u8 action } -- the action fixed by the command */
    CTRL_ARG_STATION_ACTION, /* { u16 station, u8 action } -- the action named in the request */
    CTRL_ARG_REPORTS,        /* not one key: every report that can be asked for */
} ctrl_arg_t;

typedef struct {
    const char *name;
    uint8_t key;
    ctrl_arg_t arg;
    uint8_t fixed; /* FIXED: the value. STATION: the action. SCOPE: a scope override, or 0xFF. */
} ctrl_command_t;

#define CTRL_SCOPE_FROM_REQUEST 0xFF

static const ctrl_command_t ctrl_commands[] = {
    /* --- the system TLVs, one request each ------------------------------------------------- */
    { "vers", IOTDATA_NODE_CONTROL_VERSION_REQUEST, CTRL_ARG_NONE, 0 },
    { "vari", IOTDATA_NODE_CONTROL_VARIANT_REQUEST, CTRL_ARG_NONE, 0 },
    { "ctrl", IOTDATA_NODE_CONTROL_CONTROL_REQUEST, CTRL_ARG_NONE, 0 },
    { "stat", IOTDATA_NODE_CONTROL_STATUS_REQUEST, CTRL_ARG_SCOPE_STATUS, CTRL_SCOPE_FROM_REQUEST },
    { "conf", IOTDATA_NODE_CONTROL_CONFIG_REQUEST, CTRL_ARG_NONE, 0 },
    { "diag", IOTDATA_NODE_CONTROL_DIAGNOSTICS_REQUEST, CTRL_ARG_NONE, 0 },
    { "cont", IOTDATA_NODE_CONTROL_CONTENT_REQUEST, CTRL_ARG_NONE, 0 },
    { "reports", 0, CTRL_ARG_REPORTS, 0 }, /* every report a node can produce, in type order */

    /* --- generic system control -------------------------------------------------------------- */
    { "boot", IOTDATA_NODE_CONTROL_REBOOT, CTRL_ARG_NONE, 0 }, /* `boot` on the serial CLI too */

    /* --- the recorder ------------------------------------------------------------------------ */
    { "diag-enable", IOTDATA_NODE_CONTROL_DIAGNOSTICS_ENABLE, CTRL_ARG_FIXED, 1 },
    { "diag-disable", IOTDATA_NODE_CONTROL_DIAGNOSTICS_ENABLE, CTRL_ARG_FIXED, 0 },
    { "diag-clear", IOTDATA_NODE_CONTROL_DIAGNOSTICS_CLEAR, CTRL_ARG_NONE, 0 },
    { "diag-dump", IOTDATA_NODE_CONTROL_DIAGNOSTICS_DUMP, CTRL_ARG_NONE, 0 },

    /* --- mesh management ---------------------------------------------------------------------
       Each table has a REQUEST, answered by that table's report, and a DUMP, which prints where
       the node's console goes. The action commands are the wire keys spelled out: one UPDATE per
       table taking { station, action }, rather than a command per action -- so `mesh-filters-update
       <station> block` reads as the key it becomes, and adding an action never adds a command. */
    { "mesh-stations", IOTDATA_NODE_CONTROL_MESH_STATIONS_REQUEST, CTRL_ARG_NONE, 0 },
    { "mesh-stations-dump", IOTDATA_NODE_CONTROL_MESH_STATIONS_DUMP, CTRL_ARG_NONE, 0 },
    { "mesh-peers", IOTDATA_NODE_CONTROL_MESH_PEERS_REQUEST, CTRL_ARG_NONE, 0 },
    { "mesh-peers-update", IOTDATA_NODE_CONTROL_MESH_PEERS_UPDATE, CTRL_ARG_STATION_ACTION, 0 },
    { "mesh-peers-clear", IOTDATA_NODE_CONTROL_MESH_PEERS_CLEAR, CTRL_ARG_NONE, 0 },
    { "mesh-peers-dump", IOTDATA_NODE_CONTROL_MESH_PEERS_DUMP, CTRL_ARG_NONE, 0 },
    { "mesh-filters", IOTDATA_NODE_CONTROL_MESH_FILTERS_REQUEST, CTRL_ARG_NONE, 0 },
    { "mesh-filters-update", IOTDATA_NODE_CONTROL_MESH_FILTERS_UPDATE, CTRL_ARG_STATION_ACTION, 0 },
    { "mesh-filters-clear", IOTDATA_NODE_CONTROL_MESH_FILTERS_CLEAR, CTRL_ARG_SCOPE_FILTER, CTRL_SCOPE_FROM_REQUEST },
    { "mesh-filters-dump", IOTDATA_NODE_CONTROL_MESH_FILTERS_DUMP, CTRL_ARG_NONE, 0 },

    /* --- ALIASES: the previous vocabulary, so anything written against it still works --------- */
    { "node", 0, CTRL_ARG_REPORTS, 0 },
    { "node-version", IOTDATA_NODE_CONTROL_VERSION_REQUEST, CTRL_ARG_NONE, 0 },
    { "node-variant", IOTDATA_NODE_CONTROL_VARIANT_REQUEST, CTRL_ARG_NONE, 0 },
    { "node-control", IOTDATA_NODE_CONTROL_CONTROL_REQUEST, CTRL_ARG_NONE, 0 },
    { "node-status", IOTDATA_NODE_CONTROL_STATUS_REQUEST, CTRL_ARG_SCOPE_STATUS, CTRL_SCOPE_FROM_REQUEST },
    { "node-config", IOTDATA_NODE_CONTROL_CONFIG_REQUEST, CTRL_ARG_NONE, 0 },
    { "node-diagnostics", IOTDATA_NODE_CONTROL_DIAGNOSTICS_REQUEST, CTRL_ARG_NONE, 0 },
    { "node-content", IOTDATA_NODE_CONTROL_CONTENT_REQUEST, CTRL_ARG_NONE, 0 },
    /* `status` meant the MESH view, inherited from MANAGE. Kept exactly that, because changing
       what an existing name MEANS is worse than retiring it: `stat --scope mesh` is the new way. */
    { "status", IOTDATA_NODE_CONTROL_STATUS_REQUEST, CTRL_ARG_SCOPE_STATUS, IOTDATA_NODE_STATUS_SCOPE_MESH },
    { "stations", IOTDATA_NODE_CONTROL_MESH_STATIONS_DUMP, CTRL_ARG_NONE, 0 }, /* dumped, as it did */
    { "peers", IOTDATA_NODE_CONTROL_MESH_PEERS_DUMP, CTRL_ARG_NONE, 0 },
    { "filters", IOTDATA_NODE_CONTROL_MESH_FILTERS_DUMP, CTRL_ARG_NONE, 0 },
    { "peers-remove", IOTDATA_NODE_CONTROL_MESH_PEERS_UPDATE, CTRL_ARG_STATION, IOTDATA_NODE_CONTROL_MESH_PEER_NONE },
    { "peers-clear", IOTDATA_NODE_CONTROL_MESH_PEERS_CLEAR, CTRL_ARG_NONE, 0 },
    { "flush", IOTDATA_NODE_CONTROL_MESH_PEERS_CLEAR, CTRL_ARG_NONE, 0 },
    { "block", IOTDATA_NODE_CONTROL_MESH_FILTERS_UPDATE, CTRL_ARG_STATION, IOTDATA_NODE_CONTROL_MESH_FILTERS_BLOCK },
    { "allow", IOTDATA_NODE_CONTROL_MESH_FILTERS_UPDATE, CTRL_ARG_STATION, IOTDATA_NODE_CONTROL_MESH_FILTERS_ALLOW },
    { "unfilter", IOTDATA_NODE_CONTROL_MESH_FILTERS_UPDATE, CTRL_ARG_STATION, IOTDATA_NODE_CONTROL_MESH_FILTERS_NONE },
    { "filter-clear", IOTDATA_NODE_CONTROL_MESH_FILTERS_CLEAR, CTRL_ARG_SCOPE_FILTER, CTRL_SCOPE_FROM_REQUEST },
    /* the per-action names, superseded by *-update <station> <action> */
    { "mesh-filter", IOTDATA_NODE_CONTROL_MESH_FILTERS_DUMP, CTRL_ARG_NONE, 0 },
    { "mesh-filter-block", IOTDATA_NODE_CONTROL_MESH_FILTERS_UPDATE, CTRL_ARG_STATION, IOTDATA_NODE_CONTROL_MESH_FILTERS_BLOCK },
    { "mesh-filter-allow", IOTDATA_NODE_CONTROL_MESH_FILTERS_UPDATE, CTRL_ARG_STATION, IOTDATA_NODE_CONTROL_MESH_FILTERS_ALLOW },
    { "mesh-filter-none", IOTDATA_NODE_CONTROL_MESH_FILTERS_UPDATE, CTRL_ARG_STATION, IOTDATA_NODE_CONTROL_MESH_FILTERS_NONE },
    { "mesh-filter-clear", IOTDATA_NODE_CONTROL_MESH_FILTERS_CLEAR, CTRL_ARG_SCOPE_FILTER, CTRL_SCOPE_FROM_REQUEST },
    { "mesh-peers-remove", IOTDATA_NODE_CONTROL_MESH_PEERS_UPDATE, CTRL_ARG_STATION, IOTDATA_NODE_CONTROL_MESH_PEER_NONE },
};

static const ctrl_command_t *ctrl_command_find(const char *const name) {
    for (size_t i = 0; i < sizeof(ctrl_commands) / sizeof(ctrl_commands[0]); i++)
        if (strcmp(ctrl_commands[i].name, name) == 0)
            return &ctrl_commands[i];
    return NULL;
}

/* Build the CONTROL payload for one command. Returns false when nothing was added. */
static bool ctrl_command_build(iotdata_kvr_t *const kv, const ctrl_command_t *const c, const uint16_t station, const uint8_t scope_status, const uint8_t scope_filter, const uint8_t action) {
    switch (c->arg) {
    case CTRL_ARG_NONE:
        return iotdata_kvr_add_flag(kv, c->key);
    case CTRL_ARG_FIXED:
        return iotdata_kvr_add_u8(kv, c->key, c->fixed);
    case CTRL_ARG_SCOPE_STATUS:
    case CTRL_ARG_SCOPE_FILTER: {
        const uint8_t from_request = (c->arg == CTRL_ARG_SCOPE_STATUS) ? scope_status : scope_filter;
        const uint8_t want = (c->fixed == CTRL_SCOPE_FROM_REQUEST) ? from_request : c->fixed;
        /*
         * A zero scope may or may not be omittable, and the key table already says which.
         *
         * STATUS_REQUEST is declared WIDTH_VARIABLE -- its value is optional, and absent already
         * means "every group", so sending an explicit zero would express that a second way and
         * cost a byte. MESH_FILTER_CLEAR declares a width of 1: it must carry its byte even when
         * the byte is zero, or the frame contradicts the table and a strict reader is entitled to
         * reject it. So the declared width decides, rather than an assumption about zero.
         */
        if (want == 0 && iotdata_node_tlv_key_width(IOTDATA_NODE_TLV_CONTROL, c->key) == IOTDATA_NODE_WIDTH_VARIABLE)
            return iotdata_kvr_add_flag(kv, c->key);
        return iotdata_kvr_add_u8(kv, c->key, want);
    }
    case CTRL_ARG_STATION:
        return ctrl_kvr_mesh_update(kv, c->key, station, c->fixed);
    case CTRL_ARG_STATION_ACTION:
        return ctrl_kvr_mesh_update(kv, c->key, station, action);
    case CTRL_ARG_REPORTS:
        /* everything that can be asked for -- so not CONTENT (nothing implements it) and not
           RECEIVE (a node advertises that, it is not requestable) */
        for (uint8_t type = 0; type <= IOTDATA_TLV_TYPE_SYSTEM_MAX; type++)
            if (iotdata_node_tlv_control_key(type) != IOTDATA_NODE_TLV_NONE && type != IOTDATA_NODE_TLV_CONTENT)
                iotdata_kvr_add_flag(kv, iotdata_node_tlv_control_key(type));
        return !kv->overflow && kv->len > 0;
    default:
        return false;
    }
}

static void ctrl_on_message(const char *topic __attribute__((unused)), const unsigned char *payload, const int len) {
    ctrl_state_t *const st = g_ctrl;

    st->stat_req_rx++;

    cJSON *const root = cJSON_ParseWithLength((const char *)payload, (size_t)len);
    if (root == NULL) {
        st->stat_req_bad++;
        PRINTF_ERROR("ctrl: bad JSON request (%d bytes)\n", len);
        return;
    }

    const uint16_t target = ctrl_parse_target(cJSON_GetObjectItem(root, "target"));
    const uint16_t station = ctrl_parse_station(cJSON_GetObjectItem(root, "station"));
    const cJSON *const jscope = cJSON_GetObjectItem(root, "scope");
    const uint8_t scope_status = ctrl_parse_scope_status(jscope);
    const uint8_t scope_filter = ctrl_parse_scope_filter(jscope);
    const uint8_t action = ctrl_parse_action(cJSON_GetObjectItem(root, "action"));
    const cJSON *const jc = cJSON_GetObjectItem(root, "cmd");
    const char *const cmd = (cJSON_IsString(jc) && jc->valuestring != NULL) ? jc->valuestring : "";

    /*
     * One lookup, one build, one staging. Every command -- a report request, a mesh-management
     * command, the recorder, a reboot -- is a CONTROL key, so there is no longer a reason for the
     * node ones and the mesh ones to travel through different code.
     */
    const ctrl_command_t *const c = ctrl_command_find(cmd);
    if (c == NULL) {
        ctrl_respond_error(st, cmd, "unknown command");
        st->stat_req_bad++;
        cJSON_Delete(root);
        return;
    }

    const buffer_handle_t h = buffer_acquire(st->pool);
    if (h != BUFFER_NONE) {
        iotdata_kvr_t kv;
        iotdata_kvr_init(&kv, buffer_data(st->pool, h), buffer_room(st->pool, h));
        (void)ctrl_command_build(&kv, c, station, scope_status, scope_filter, action);

        const size_t n = kv.overflow ? 0u : kv.len;
        if (n == 0) {
            snprintf(st->_buffer_resp, sizeof(st->_buffer_resp), "payload would not fit (%zu bytes)", kv.len);
            ctrl_respond_error(st, cmd, st->_buffer_resp);
            st->stat_req_bad++;
            buffer_unref(st->pool, h);
            cJSON_Delete(root);
            return;
        }
        cJSON_Delete(root);
        buffer_set_len(st->pool, h, (uint16_t)n);

        /* staged, NOT executed here: this is the mosquitto thread, and node_on_mqtt() transmits
           and mutates node state the main loop owns. Due now, so the queue orders by arrival. */
        const uint32_t now_ms = (uint32_t)__ticks_ms();
        const bool queued = buffer_queue_add(&st->queue, h, now_ms, now_ms + CTRL_QUEUE_TTL_MS, CTRL_TAG_CONTROL, target);
        buffer_unref(st->pool, h); /* the queue took its own reference; this one was ours */
        if (!queued) {
            snprintf(st->_buffer_resp, sizeof(st->_buffer_resp), "busy: %u commands already staged", (unsigned)buffer_queue_count(&st->queue));
            ctrl_respond_error(st, cmd, st->_buffer_resp);
            st->stat_req_bad++;
            return;
        }
        PRINTF_INFO("ctrl: %s target=%04X station=%04X -> %zu byte control (staged %u)\n", cmd, (unsigned)target, (unsigned)station, n, (unsigned)buffer_queue_count(&st->queue));
    } else {
        ctrl_respond_error(st, cmd, "no frame buffer available");
        st->stat_req_bad++;
        cJSON_Delete(root);
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

void ctrl_tick(ctrl_state_t *const st, node_state_t *const ns) {

    /* Age the queue, then take ONE. One per tick and not a drain: each command can put a DOWN
       frame on air, and a burst of them back to back would spend the whole cycle in the radio.
       The tick rate is the pacing. Both calls are individually atomic, which is all this needs --
       a request arriving between them is simply taken on the next tick. */
    const uint32_t now_ms = (uint32_t)__ticks_ms();
    const uint16_t stale = buffer_queue_expire(&st->queue, now_ms);
    if (stale > 0)
        PRINTF_WARN("ctrl: %u staged command(s) expired unsent\n", (unsigned)stale);
    uint32_t target = 0;
    const buffer_handle_t h = buffer_queue_take(&st->queue, now_ms, NULL, &target);
    if (h != BUFFER_NONE) {
        /* runs here, on the main loop, where the node state lives -- it executes locally if
           addressed to us and airs a DOWN frame if it is for anybody else */
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
    snprintf(st->topic_req, sizeof(st->topic_req), "%s" IOTDATA_GATEWAY_TOPIC_REQ, topic_prefix);
    snprintf(st->topic_resp, sizeof(st->topic_resp), "%s" IOTDATA_GATEWAY_TOPIC_RESP, topic_prefix);

    g_ctrl = st;
    if (!mqtt_subscribe(st->topic_req, MQTT_PUBLISH_QOS, ctrl_on_message)) {
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
