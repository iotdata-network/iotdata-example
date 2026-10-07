
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include <cjson/cJSON.h>

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

/* Staged commands waiting for the main loop. Bounded on purpose: they come out of the same pool
   the receive path draws from, so an operator hammering the request topic must not be able to
   starve it. A full queue is an answer ("busy"), not a silent overwrite. */
/* At least IOTDATA_NODE_CONTROL_TARGETS_MAX: one request may name several stations and each becomes its
   own DOWN frame, so a queue shorter than the target limit could never stage a full request. */
#define CTRL_QUEUE_MAX          IOTDATA_NODE_CONTROL_TARGETS_MAX
/* The largest CONTROL payload one command builds: `reports` is the widest, a flag per report type. */
#define CTRL_PAYLOAD_MAX        32
/* A config write is bigger: several [id|type][value] records, bounded by what one frame can carry. */
#define CTRL_PAYLOAD_CONFIG_MAX IOTDATA_TLV_VALUE_MAX
#define CTRL_QUEUE_TTL_MS       30000 /* a command nobody could send in 30s is stale, not queued */

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

/* What a staged payload IS, so the drain knows which TLV to put it in. A command is a CONTROL kvr;
   a config write is a CONFIG record stream, because a write is not a verb and does not need one. */
#define CTRL_TAG_CONTROL        1
#define CTRL_TAG_CONFIG         2

// -----------------------------------------------------------------------------------------------------------------------------------------

static const char *ctrl_tag_name(const uint8_t tag) {
    switch (tag) {
    case CTRL_TAG_CONTROL:
        return "CONTROL";
    case CTRL_TAG_CONFIG:
        return "CONFIG";
    default:
        return "NONE";
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* The TLV a tag travels as. */
static uint8_t ctrl_tag_tlv(const uint8_t tag) {
    return (tag == CTRL_TAG_CONFIG) ? (uint8_t)IOTDATA_NODE_TLV_CONFIG : (uint8_t)IOTDATA_NODE_TLV_CONTROL;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

typedef struct {
    iotdata_node_t *inode;
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
    survey_state_t *survey;
    trace_state_t *trace;
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

/* Management responses are JSON on the one response topic (IOTDATA_NODE_MQTT_MANAGE_TOPIC_RESP), sharing it
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
        cJSON_AddStringToObject(root, "station", snprintf_inline(idbuf, sizeof(idbuf), "%03" PRIX16, iotdata_node_station(st->inode)));
        char *const out = cJSON_PrintUnformatted(root);
        if (out != NULL) {
            (void)mqtt_send(st->topic_resp, out, (int)strlen(out));
            free(out);
        }
        cJSON_Delete(root);
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static cJSON *ctrl_respond_begin(const char *const resp, const char *const cmd) {
    cJSON *const root = cJSON_CreateObject();
    if (root != NULL) {
        cJSON_AddStringToObject(root, "resp", resp);
        cJSON_AddStringToObject(root, "cmd", cmd);
    }
    return root;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void ctrl_respond_diag(ctrl_state_t *const st, const char *const cmd, const char *const text) {
    cJSON *const root = ctrl_respond_begin("diag", cmd);
    if (root != NULL) {
        cJSON_AddStringToObject(root, "source", CTRL_DIAG_SOURCE);
        cJSON_AddStringToObject(root, "text", text);
        ctrl_respond(st, root);
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void ctrl_respond_diag_record(ctrl_state_t *const st, const char *const cmd, const int index, const char *const record) {
    cJSON *const root = ctrl_respond_begin("diag", cmd);
    if (root != NULL) {
        cJSON_AddStringToObject(root, "source", CTRL_DIAG_SOURCE);
        cJSON_AddNumberToObject(root, "index", index);
        cJSON_AddStringToObject(root, "record", record);
        ctrl_respond(st, root);
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void ctrl_respond_error(ctrl_state_t *const st, const char *const cmd, const char *const text) {
    cJSON *const root = ctrl_respond_begin("error", cmd);
    if (root != NULL) {
        cJSON_AddStringToObject(root, "text", text);
        ctrl_respond(st, root);
    }
    PRINTF_ERROR("ctrl: %s: %s\n", cmd, text);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void ctrl_diag_enable(const bool on) {
    ctrl_state_t *const st = g_ctrl;
    blackbox_enable(&st->bbox->handle, on);
    ctrl_respond_diag(st, on ? "diag-enable" : "diag-disable", on ? "enabled" : "disabled");
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void ctrl_diag_clear(void) {
    ctrl_state_t *const st = g_ctrl;
    blackbox_clear(&st->bbox->handle);
    ctrl_respond_diag(st, "diag-clear", "cleared");
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void ctrl_diag_dump(void) {
    ctrl_state_t *const st = g_ctrl;
    size_t cur = 0;
    int n = 0;
    while (blackbox_pull(&st->bbox->handle, &cur, st->_buffer_blackbox_rec, sizeof(st->_buffer_blackbox_rec)) > 0)
        ctrl_respond_diag_record(st, "diag-dump", n++, st->_buffer_blackbox_rec);
    ctrl_respond_diag(st, "diag-dump", snprintf_inline(st->_buffer_resp, sizeof(st->_buffer_resp), "dumped %d records", n));
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* A CONFIG WRITE, built from JSON into the [id|type][value] record stream the node expects.
 *
 * NUMERIC AND EXPLICITLY TYPED, both on purpose. The gateway does not resolve names: an id is what
 * travels, and the client nearest the operator is the one place that turns a name into one. That is
 * what keeps the gateway free of a name table it would otherwise have to keep in step with every
 * firmware in the fleet -- and what leaves the door open to a gateway on an ESP32.
 *
 * The type has to be stated because a write is not self-describing the way a read is: the node sends
 * id+type+value back, but nothing in "0x051 = 20" says whether 20 is a u8 or a u32, and the record
 * carries the type in its own header. `--config-set 0x051:u8=20`.
 *
 * Returns the bytes built, or 0 -- with `why` set to something an operator can act on.
 */
static size_t ctrl_config_build(const cJSON *const root, uint8_t *const out, const size_t size, const char **const why) {
    const cJSON *const set = cJSON_GetObjectItem(root, "set");
    if (!cJSON_IsArray(set) || cJSON_GetArraySize(set) == 0) {
        *why = "conf-set needs a non-empty \"set\" array of {id,type,value}";
        return 0;
    }
    size_t at = 0;
    const cJSON *e = NULL;
    cJSON_ArrayForEach(e, set) {
        const cJSON *const jid = cJSON_GetObjectItem(e, "id");
        const cJSON *const jty = cJSON_GetObjectItem(e, "type");
        const cJSON *const jvl = cJSON_GetObjectItem(e, "value");
        if (!cJSON_IsNumber(jid) || !cJSON_IsNumber(jty) || !cJSON_IsNumber(jvl)) {
            *why = "each entry needs numeric id, type and value";
            return 0;
        }
        const uint16_t id = (uint16_t)jid->valuedouble;
        const uint8_t type = (uint8_t)jty->valuedouble;
        const uint8_t w = iotdata_node_config_type_size(type);
        if (id > IOTDATA_NODE_CONFIG_ID_MAX || w == 0) {
            *why = "id out of range, or a type that has no fixed width";
            return 0;
        }
        if (at + 2u + w > size) {
            *why = "the write would not fit one frame";
            return 0;
        }
        const uint16_t rec = (uint16_t)iotdata_node_config_rec(id, type);
        out[at++] = (uint8_t)(rec >> 8);
        out[at++] = (uint8_t)rec;
        /* Signed and float go in by their own bit pattern; everything else is the integer. The node
           re-reads it through its own table, so a value that disagrees with the row is refused
           THERE -- which is the only place that knows the bounds. */
        uint64_t raw;
        if (type == IOTDATA_NODE_CONFIG_TYPE_FLOAT) {
            const float f = (float)jvl->valuedouble;
            uint32_t bits;
            memcpy(&bits, &f, sizeof(bits));
            raw = bits;
        } else if (iotdata_node_config_type_is_signed(type))
            raw = (uint64_t)(int64_t)jvl->valuedouble;
        else
            raw = (uint64_t)jvl->valuedouble;
        for (uint8_t b = 0; b < w; b++)
            out[at++] = (uint8_t)(raw >> (8u * (w - 1u - b)));
    }
    return at;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void ctrl_from_message(const char *topic __attribute__((unused)), const unsigned char *payload, const int len) {
    ctrl_state_t *const st = g_ctrl;
    st->stat_req_rx++;
    cJSON *const root = cJSON_ParseWithLength((const char *)payload, (size_t)len);
    if (root != NULL) {
        iotdata_node_control_args_t args;
        const iotdata_node_control_command_t *const c = iotdata_node_control_from_json(root, &args);
        const cJSON *const jc = cJSON_GetObjectItem(root, "cmd");
        const char *const cmd = (cJSON_IsString(jc) && jc->valuestring != NULL) ? jc->valuestring : "";
        /* A config write is not a command, so it does not come through iotdata_node_control_from_json --
           it is a CONFIG TLV, and it only borrows this path's targeting and staging. */
        /* A survey is a MESH directive, not a node command: it is not addressed to a station and
           it does not travel as a DOWN frame, so it does not belong on the control path above. It
           only shares this channel, which is the operator's one way in. */
        const bool is_survey = (strcmp(cmd, "survey") == 0);
        const bool is_survey_cancel = (strcmp(cmd, "survey-cancel") == 0);
        const bool is_config_set = (strcmp(cmd, "conf-set") == 0);
        const bool is_trace = (strcmp(cmd, "trace") == 0);
        if (is_trace) {
            /* leg defaults to UP, which is the common question ("how does this node reach me") and
               the one that needs no answer frame: the record ends here and is published. DOWN ends
               at the subject instead, so it defaults to asking for an answer -- without one the
               operator has asked a question of a node that will only write it to its own log. */
            const cJSON *const jst = cJSON_GetObjectItem(root, "station");
            const cJSON *const jleg = cJSON_GetObjectItem(root, "leg");
            const cJSON *const jmode = cJSON_GetObjectItem(root, "mode");
            const cJSON *const jresp = cJSON_GetObjectItem(root, "respond");
            const cJSON *const jid = cJSON_GetObjectItem(root, "id");
            const uint16_t station = cJSON_IsString(jst) ? (uint16_t)strtoul(jst->valuestring, NULL, 16) : (cJSON_IsNumber(jst) ? (uint16_t)jst->valuedouble : 0u);
            const bool down = cJSON_IsString(jleg) && strcmp(jleg->valuestring, "down") == 0;
            const bool one = cJSON_IsString(jmode) && strcmp(jmode->valuestring, "one") == 0;
            const bool respond = cJSON_IsBool(jresp) ? cJSON_IsTrue(jresp) : down;
            const uint16_t id = cJSON_IsNumber(jid) ? (uint16_t)((uint16_t)jid->valuedouble & 0x0FFFu) : (cJSON_IsString(jid) ? (uint16_t)(strtoul(jid->valuestring, NULL, 16) & 0x0FFFu) : trace_next_id(st->trace));
            const char *why = "busy";
            if (trace_request(st->trace, id, station, down, one, respond, &why)) {
                cJSON *const r = ctrl_respond_begin("trace", cmd);
                if (r != NULL) {
                    cJSON_AddStringToObject(r, "text",
                                            snprintf_inline(st->_buffer_resp, sizeof(st->_buffer_resp), "directive %03lX: %s trace of %03lX, %s paths%s", (unsigned long)id, down ? "downward" : "upward", (unsigned long)station,
                                                            one ? "single" : "all", respond ? ", answer requested" : ""));
                    ctrl_respond(st, r);
                }
                PRINTF_INFO("ctrl: trace %03" PRIX16 " -> station=%03" PRIX16 " leg=%s mode=%s respond=%d\n", id, station, down ? "down" : "up", one ? "one" : "all", (int)respond);
            } else {
                ctrl_respond_error(st, cmd, why);
                st->stat_req_bad++;
            }
        } else if (is_survey || is_survey_cancel) {
            const char *why = "busy";
            if (is_survey_cancel) {
                const cJSON *const jid = cJSON_GetObjectItem(root, "id");
                const bool all = !cJSON_IsString(jid) && !cJSON_IsNumber(jid);
                const uint16_t id = all ? 0u : (uint16_t)(cJSON_IsString(jid) ? strtoul(jid->valuestring, NULL, 16) : (unsigned long)jid->valuedouble);
                if (survey_cancel(st->survey, id, all, &why)) {
                    cJSON *const r = ctrl_respond_begin("survey", cmd);
                    if (r != NULL) {
                        cJSON_AddStringToObject(r, "text", all ? "cancelled every survey" : snprintf_inline(st->_buffer_resp, sizeof(st->_buffer_resp), "cancelled directive %03lX", (unsigned long)id));
                        ctrl_respond(st, r);
                    }
                } else {
                    ctrl_respond_error(st, cmd, why);
                    st->stat_req_bad++;
                }
            } else {
                uint16_t stations[GW_SURVEY_STATIONS_MAX];
                int count = 0;
                const cJSON *const js = cJSON_GetObjectItem(root, "stations");
                if (cJSON_IsArray(js)) {
                    const cJSON *e = NULL;
                    cJSON_ArrayForEach(e, js) {
                        if (count >= (int)(sizeof(stations) / sizeof(stations[0])))
                            break;
                        if (cJSON_IsString(e) && e->valuestring != NULL)
                            stations[count++] = (uint16_t)strtoul(e->valuestring, NULL, 0);
                        else if (cJSON_IsNumber(e))
                            stations[count++] = (uint16_t)e->valuedouble;
                    }
                }
                const cJSON *const jd = cJSON_GetObjectItem(root, "duration");
                const cJSON *const jb = cJSON_GetObjectItem(root, "batch");
                const cJSON *const jid = cJSON_GetObjectItem(root, "id");
                const uint16_t duration_m = cJSON_IsNumber(jd) ? (uint16_t)jd->valuedouble : 10u;
                const uint16_t batch_s = cJSON_IsNumber(jb) ? (uint16_t)jb->valuedouble : 30u;
                const uint16_t id = cJSON_IsNumber(jid) ? (uint16_t)((uint16_t)jid->valuedouble & 0x0FFFu) : (cJSON_IsString(jid) ? (uint16_t)(strtoul(jid->valuestring, NULL, 16) & 0x0FFFu) : survey_next_id(st->survey));
                if (survey_request(st->survey, id, duration_m, batch_s, stations, count, &why)) {
                    cJSON *const r = ctrl_respond_begin("survey", cmd);
                    if (r != NULL) {
                        cJSON_AddStringToObject(r, "text",
                                                snprintf_inline(st->_buffer_resp, sizeof(st->_buffer_resp), "directive %03lX: %d station(s), %um lifetime, %us window (%d running here)", (unsigned long)id, count, (unsigned)duration_m,
                                                                (unsigned)batch_s, survey_count(st->survey)));
                        ctrl_respond(st, r);
                    }
                    PRINTF_INFO("ctrl: survey %03" PRIX16 " -> %d station(s), duration=%um batch=%us\n", id, count, (unsigned)duration_m, (unsigned)batch_s);
                } else {
                    ctrl_respond_error(st, cmd, why);
                    st->stat_req_bad++;
                }
            }
        } else if (is_config_set) {
            const char *why = "bad conf-set";
            uint8_t built[CTRL_PAYLOAD_CONFIG_MAX];
            const size_t n = ctrl_config_build(root, built, sizeof(built), &why);
            uint16_t target = IOTDATA_STATION_BROADCAST;
            const cJSON *const jt = cJSON_GetObjectItem(root, "target");
            if (cJSON_IsString(jt) && jt->valuestring != NULL)
                target = (uint16_t)strtoul(jt->valuestring, NULL, 0);
            else if (cJSON_IsNumber(jt))
                target = (uint16_t)jt->valuedouble;
            if (n == 0) {
                ctrl_respond_error(st, cmd, why);
                st->stat_req_bad++;
            } else {
                const buffer_handle_t h = buffer_acquire(st->pool);
                const uint32_t now_ms = hw_time_ms();
                if (h == BUFFER_NONE) {
                    ctrl_respond_error(st, cmd, "no frame buffer");
                    st->stat_req_bad++;
                } else {
                    (void)buffer_write(st->pool, h, built, (uint16_t)n);
                    if (buffer_queue_add(&st->queue, h, now_ms, now_ms + CTRL_QUEUE_TTL_MS, CTRL_TAG_CONFIG, target))
                        PRINTF_INFO("ctrl: conf-set target=%03" PRIX16 " -> %zu byte config\n", target, n);
                    else {
                        ctrl_respond_error(st, cmd, "busy: nothing staged");
                        st->stat_req_bad++;
                    }
                    buffer_unref(st->pool, h);
                }
            }
        } else if (c != NULL) {
            /* Built ONCE: every target gets the same payload and differs only in where the DOWN
               frame is addressed, so the fan-out is a copy per target rather than a rebuild. */
            uint8_t built[CTRL_PAYLOAD_MAX];
            iotdata_kvr_t kv;
            iotdata_kvr_init(&kv, built, sizeof(built));
            (void)iotdata_node_control_build(&kv, c, &args);
            const size_t n = kv.overflow ? 0u : kv.len;
            if (n > 0) {
                /* Room for ALL of them, checked before any are staged: half a fan-out is worse
                   than none, because nothing downstream can say which stations got it. */
                if (args.targets_overflow) {
                    ctrl_respond_error(st, cmd, snprintf_inline(st->_buffer_resp, sizeof(st->_buffer_resp), "too many targets (at most %u)", (unsigned)IOTDATA_NODE_CONTROL_TARGETS_MAX));
                    st->stat_req_bad++;
                } else if (buffer_queue_count(&st->queue) + args.targets_count > CTRL_QUEUE_MAX) {
                    ctrl_respond_error(st, cmd, snprintf_inline(st->_buffer_resp, sizeof(st->_buffer_resp), "busy: %u staged, no room for %u more", (unsigned)buffer_queue_count(&st->queue), (unsigned)args.targets_count));
                    st->stat_req_bad++;
                } else {
                    const uint32_t now_ms = hw_time_ms();
                    unsigned staged = 0;
                    for (uint8_t i = 0; i < args.targets_count; i++) {
                        const buffer_handle_t h = buffer_acquire(st->pool);
                        if (h == BUFFER_NONE)
                            break;
                        (void)buffer_write(st->pool, h, built, (uint16_t)n);
                        if (buffer_queue_add(&st->queue, h, now_ms, now_ms + CTRL_QUEUE_TTL_MS, CTRL_TAG_CONTROL, args.targets[i]))
                            staged++;
                        buffer_unref(st->pool, h);
                    }
                    if (staged == args.targets_count)
                        PRINTF_INFO("ctrl: %s target=%03" PRIX16 "%s station=%03" PRIX16 " -> %zu byte control x%u (staged %u)\n", cmd, args.target, args.targets_count > 1 ? "+" : "", args.station, n, (unsigned)args.targets_count,
                                    (unsigned)buffer_queue_count(&st->queue));
                    else {
                        ctrl_respond_error(st, cmd, snprintf_inline(st->_buffer_resp, sizeof(st->_buffer_resp), "staged %u of %u: no frame buffer", staged, (unsigned)args.targets_count));
                        st->stat_req_bad++;
                    }
                }
            } else {
                ctrl_respond_error(st, cmd, snprintf_inline(st->_buffer_resp, sizeof(st->_buffer_resp), "payload would not fit (%zu bytes)", kv.len));
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
    iotdata_node_order_t ord[sizeof(s->peers) / sizeof(s->peers[0])];
    int ordered = 0;
    for (int i = 0; i < s->peers_count; i++)
        ordered = iotdata_node_order_insert(ord, ordered, (int)(sizeof(ord) / sizeof(ord[0])), s->peers[i].station_id, (uint16_t)i);
    for (int k = 0; k < ordered; k++) {
        const stat_peer_t *const e = &s->peers[ord[k].slot];
        PRINTF_INFO("exec:   %03" PRIX16 " cost=%" PRIu8 " gen=%" PRIu8 " flags=0x%02" PRIX8 " age=%lds\n", e->station_id, e->cost, e->generation, e->flags, (long)(time(NULL) - e->last_seen));
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void ctrl_report_filter(const filter_t *const f) {
    PRINTF_INFO("exec: filters - %d\n", filter_count(f));
    iotdata_node_order_t ord[sizeof(f->e) / sizeof(f->e[0])];
    int ordered = 0;
    for (int i = 0; i < f->count; i++)
        ordered = iotdata_node_order_insert(ord, ordered, (int)(sizeof(ord) / sizeof(ord[0])), f->e[i].station, (uint16_t)i);
    for (int k = 0; k < ordered; k++) {
        const filter_entry_t *const e = &f->e[ord[k].slot];
        PRINTF_INFO("exec:   %03" PRIX16 " %s (%s)\n", e->station, e->action == FILTER_ALLOW ? "allow" : "block", e->source == FILTER_AUTO ? "auto" : "manual");
    }
}

static const uint8_t ctrl_from_iotdata_keys[] = {
    IOTDATA_NODE_TLV_STATUS,      IOTDATA_NODE_ACTION_STATUS_STATIONS_DUMP, IOTDATA_NODE_TLV_STATUS,      IOTDATA_NODE_ACTION_STATUS_FILTERS_UPDATE, IOTDATA_NODE_TLV_STATUS,      IOTDATA_NODE_ACTION_STATUS_FILTERS_CLEAR,
    IOTDATA_NODE_TLV_STATUS,      IOTDATA_NODE_ACTION_STATUS_FILTERS_DUMP,  IOTDATA_NODE_SUBJECT_MESH,    IOTDATA_NODE_ACTION_MESH_PEERS_UPDATE,     IOTDATA_NODE_SUBJECT_MESH,    IOTDATA_NODE_ACTION_MESH_PEERS_CLEAR,
    IOTDATA_NODE_SUBJECT_MESH,    IOTDATA_NODE_ACTION_MESH_PEERS_DUMP,      IOTDATA_NODE_TLV_DIAGNOSTICS, IOTDATA_NODE_ACTION_DIAGNOSTICS_ENABLE,    IOTDATA_NODE_TLV_DIAGNOSTICS, IOTDATA_NODE_ACTION_DIAGNOSTICS_CLEAR,
    IOTDATA_NODE_TLV_DIAGNOSTICS, IOTDATA_NODE_ACTION_DIAGNOSTICS_DUMP,
};

// -----------------------------------------------------------------------------------------------------------------------------------------

static bool ctrl_action_status(process_state_t *const st, const uint8_t action, const uint8_t *const val, const uint8_t vlen) {
    switch (action) {
    case IOTDATA_NODE_ACTION_STATUS_STATIONS_DUMP:
        PRINTF_INFO("exec: CONTROL - MESH_STATIONS_DUMP\n");
        netw_report(&st->network, iotdata_node_station(st->state_mesh->inode));
        return true;
    case IOTDATA_NODE_ACTION_STATUS_FILTERS_DUMP:
        PRINTF_INFO("exec: CONTROL - MESH_FILTERS_DUMP\n");
        ctrl_report_filter(&st->filter);
        return true;
    case IOTDATA_NODE_ACTION_STATUS_FILTERS_UPDATE: {
        int n = 0;
        for (int i = 0; i + IOTDATA_NODE_CONTROL_MESH_UPDATE_ENTRY_SIZE <= (int)vlen; i += IOTDATA_NODE_CONTROL_MESH_UPDATE_ENTRY_SIZE) {
            const uint16_t station = (uint16_t)(((uint16_t)val[i] << 8) | val[i + 1]);
            switch (val[i + 2]) {
            case IOTDATA_NODE_CONTROL_MESH_FILTERS_NONE:
                PRINTF_INFO("exec: CONTROL - MESH_FILTER_UPDATE %03" PRIX16 " none -> %s\n", station, filter_remove(&st->filter, station) ? "removed" : "not present");
                n++;
                break;
            case IOTDATA_NODE_CONTROL_MESH_FILTERS_BLOCK:
            case IOTDATA_NODE_CONTROL_MESH_FILTERS_ALLOW: {
                /* the wire values are NONE/BLOCK/ALLOW = 0/1/2; the table's are BLOCK/ALLOW = 0/1 */
                const bool allow = (val[i + 2] == IOTDATA_NODE_CONTROL_MESH_FILTERS_ALLOW);
                const bool ok = filter_insert(&st->filter, station, allow ? FILTER_ALLOW : FILTER_BLOCK, FILTER_MANUAL);
                PRINTF_INFO("exec: CONTROL - MESH_FILTER_UPDATE %03" PRIX16 " %s -> %s\n", station, allow ? "allow" : "block", ok ? "ok" : "table full");
                if (ok && !allow) /* make the block take effect on what we already believe */
                    (void)stat_mesh_peer_remove(st->state_stat, station);
                n++;
                break;
            }
            default:
                PRINTF_INFO("exec: CONTROL - MESH_FILTER_UPDATE %03" PRIX16 " action=0x%02" PRIX8 " (unknown, ignored)\n", station, val[i + 2]);
                break;
            }
        }
        if (n > 0)
            ctrl_report_filter(&st->filter);
        return true;
    }
    case IOTDATA_NODE_ACTION_STATUS_FILTERS_CLEAR: {
        const filter_scope_t scope = (vlen >= 1) ? (filter_scope_t)val[0] : FILTER_SCOPE_ALL;
        PRINTF_INFO("exec: CONTROL - MESH_FILTER_CLEAR scope=%u -> %d cleared\n", (unsigned)scope, filter_clear(&st->filter, scope));
        ctrl_report_filter(&st->filter);
        return true;
    }

    default:
        return false; /* not ours: the node layer counts it unknown */
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static bool ctrl_action_mesh(process_state_t *const st, const uint8_t action, const uint8_t *const val, const uint8_t vlen) {
    switch (action) {
    case IOTDATA_NODE_ACTION_MESH_PEERS_DUMP:
        PRINTF_INFO("exec: CONTROL - MESH_PEERS_DUMP\n");
        ctrl_report_peers(st->state_stat);
        return true;
    case IOTDATA_NODE_ACTION_MESH_PEERS_UPDATE: {
        int n = 0;
        for (int i = 0; i + IOTDATA_NODE_CONTROL_MESH_UPDATE_ENTRY_SIZE <= (int)vlen; i += IOTDATA_NODE_CONTROL_MESH_UPDATE_ENTRY_SIZE) {
            const uint16_t station = (uint16_t)(((uint16_t)val[i] << 8) | val[i + 1]);
            if (val[i + 2] == IOTDATA_NODE_CONTROL_MESH_PEER_NONE) {
                PRINTF_INFO("exec: CONTROL - MESH_PEERS_UPDATE %03" PRIX16 " none -> %s\n", station, stat_mesh_peer_remove(st->state_stat, station) ? "removed" : "not present");
                n++;
            } else
                PRINTF_INFO("exec: CONTROL - MESH_PEERS_UPDATE %03" PRIX16 " action=0x%02" PRIX8 " (unknown, ignored)\n", station, val[i + 2]);
        }
        if (n > 0)
            ctrl_report_peers(st->state_stat);
        return true;
    }
    case IOTDATA_NODE_ACTION_MESH_PEERS_CLEAR:
        PRINTF_INFO("exec: CONTROL - MESH_PEERS_CLEAR -> forgetting %d peer(s)\n", st->state_stat->peers_count);
        stat_mesh_peers_clear(st->state_stat);
        return true;

    default:
        return false; /* not ours: the node layer counts it unknown */
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static bool ctrl_action_diag(const uint8_t action, const uint8_t *const val, const uint8_t vlen) {
    switch (action) {
    case IOTDATA_NODE_ACTION_DIAGNOSTICS_ENABLE: {
        const bool on = (vlen >= 1) ? (val[0] != 0u) : true;
        PRINTF_INFO("exec: CONTROL - DIAGNOSTICS_ENABLE -> %s\n", on ? "true" : "false");
        ctrl_diag_enable(on);
        return true;
    }
    case IOTDATA_NODE_ACTION_DIAGNOSTICS_CLEAR:
        PRINTF_INFO("exec: CONTROL - DIAGNOSTICS_CLEAR\n");
        ctrl_diag_clear();
        return true;
    case IOTDATA_NODE_ACTION_DIAGNOSTICS_DUMP:
        PRINTF_INFO("exec: CONTROL - DIAGNOSTICS_DUMP\n");
        ctrl_diag_dump();
        return true;

    default:
        return false; /* not ours: the node layer counts it unknown */
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static bool ctrl_from_iotdata(const uint8_t subject, const uint8_t action, const uint8_t *const val, const uint8_t vlen) {
    process_state_t *const st = g_exec;
    if (st == NULL)
        return false;
    switch (subject) {
    case IOTDATA_NODE_TLV_STATUS:
        return ctrl_action_status(st, action, val, vlen);
    case IOTDATA_NODE_SUBJECT_MESH:
        return ctrl_action_mesh(st, action, val, vlen);
    case IOTDATA_NODE_TLV_DIAGNOSTICS:
        return ctrl_action_diag(action, val, vlen); /* the recorder is global: no process state */
    default:
        return false; /* not ours: the node layer counts it unknown */
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void ctrl_tick(ctrl_state_t *const st, node_state_t *const ns) {
    const uint32_t now_ms = hw_time_ms();
    const uint16_t stale = buffer_queue_expire(&st->queue, now_ms);
    if (stale > 0)
        PRINTF_WARN("ctrl: %u staged command(s) expired unsent\n", (unsigned)stale);
    uint32_t target = 0;
    uint8_t tag = 0;
    const buffer_handle_t h = buffer_queue_take(&st->queue, now_ms, &tag, &target);
    if (h != BUFFER_NONE) {
        node_on_mqtt(ns, buffer_data(st->pool, h), buffer_len(st->pool, h), (uint16_t)target, ctrl_tag_tlv(tag));
        buffer_unref(st->pool, h); /* the reference the queue handed over */
    }
    const time_t now = time(NULL);
    if (st->blackbox_tick_last != 0 && st->blackbox_tick_last != now)
        blackbox_tick(&st->bbox->handle, (uint32_t)(now - st->blackbox_tick_last) * 1000u);
    st->blackbox_tick_last = now;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

bool ctrl_begin(ctrl_state_t *st, const char *topic_prefix, iotdata_node_t *inode, bbox_state_t *bbox, survey_state_t *survey, trace_state_t *trace, buffer_pool_t *pool) {
    assert(st && bbox && survey && trace && pool);
    *st = (ctrl_state_t){ 0 };
    st->inode = inode;
    st->bbox = bbox;
    st->survey = survey;
    st->trace = trace;
    st->pool = pool;
    buffer_queue_init(&st->queue, st->queue_slot, CTRL_QUEUE_MAX, pool);
    buffer_queue_set_tag_name(&st->queue, ctrl_tag_name);
    snprintf(st->topic_req, sizeof(st->topic_req), "%s" IOTDATA_NODE_MQTT_MANAGE_TOPIC_REQ, topic_prefix);
    snprintf(st->topic_resp, sizeof(st->topic_resp), "%s" IOTDATA_NODE_MQTT_MANAGE_TOPIC_RESP, topic_prefix);
    g_ctrl = st;
    if (!mqtt_subscribe(st->topic_req, MQTT_PUBLISH_QOS, ctrl_from_message)) {
        PRINTF_ERROR("ctrl: subscribe to '%s' failed\n", st->topic_req);
        return false;
    }
    PRINTF_INFO("ctrl: station=%03" PRIX16 ", request-topic='%s'\n", iotdata_node_station(st->inode), st->topic_req);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void ctrl_end(ctrl_state_t *st) {
    g_ctrl = NULL;
    buffer_queue_clear(&st->queue); /* requests staged but never drained still hold buffers */
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
