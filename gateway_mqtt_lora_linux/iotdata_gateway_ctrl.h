
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include <cjson/cJSON.h>
#include <pthread.h>

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define MQTT_CLIENT_DEFAULT              "iotdata_gateway"
#define MQTT_SERVER_DEFAULT              "mqtt://localhost"
#define MQTT_TLS_DEFAULT                 false
#define MQTT_SYNCHRONOUS_DEFAULT         false
#define MQTT_TOPIC_PREFIX_DEFAULT        "iotdata"
#define MQTT_RECONNECT_DELAY_DEFAULT     5
#define MQTT_RECONNECT_DELAY_MAX_DEFAULT 60

#define CTRL_MANAGE_BUF_MAX              64 /* MANAGE frames are small (STATUS is  8 bytes) */
#define CTRL_MQTT_TOPIC_DEFAULT          "/manage/req"
#define BBOX_MQTT_TOPIC_DEFAULT          "/blackbox/resp"

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

typedef bool (*ctrl_tx_handler_t)(const uint8_t *packet, const int length);

typedef struct {
    uint16_t station_id; /* gateway station id — the MANAGE sender */
    uint16_t seq;        /* MANAGE sender sequence (touched only on the mqtt thread) */
    ctrl_tx_handler_t tx;
    char topic_req[128];
    pthread_mutex_t lock;
    /* Single pending slot: produced by the mqtt-thread callback, drained by the main loop.
       Everything the callback wants done goes through here, because the mosquitto thread must not
       touch the radio or any of the node/mesh state the main loop owns. `kind` says which of the
       two it is -- a packed MANAGE frame to transmit, or a node CONTROL payload to execute. */
    bool pending;
    enum { CTRL_PENDING_MANAGE = 0, CTRL_PENDING_NODE } pending_kind;
    uint8_t pending_buf[CTRL_MANAGE_BUF_MAX];
    int pending_len;
    uint16_t pending_target; /* node only: who the CONTROL is addressed to */
    bbox_state_t *bbox;
    char topic_resp[128];
    char _buffer_resp[244];
    char _buffer_blackbox_rec[BLACKBOX_LINE_MAX];
    time_t blackbox_tick_last; /* for the periodic (batched) flush tick */
    /* stats */
    uint32_t stat_req_rx, stat_req_bad, stat_tx, stat_tx_err, stat_overrun;
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
                return IOTDATA_MESH_MANAGE_TARGET_ALL;
            return (uint16_t)(strtol(jt->valuestring, NULL, 0) & 0x0FFF);
        }
        if (cJSON_IsNumber(jt))
            return (uint16_t)(jt->valueint & 0x0FFF);
    }
    return IOTDATA_MESH_MANAGE_TARGET_ALL;
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

static uint8_t ctrl_parse_scope(const cJSON *js) {
    if (cJSON_IsString(js) && js->valuestring != NULL) {
        if (strcmp(js->valuestring, "manual") == 0)
            return IOTDATA_MESH_MANAGE_FILTER_SCOPE_MANUAL;
        if (strcmp(js->valuestring, "auto") == 0)
            return IOTDATA_MESH_MANAGE_FILTER_SCOPE_AUTO;
    }
    return IOTDATA_MESH_MANAGE_FILTER_SCOPE_ALL;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

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
    const uint8_t scope = ctrl_parse_scope(cJSON_GetObjectItem(root, "scope"));
    const cJSON *const jc = cJSON_GetObjectItem(root, "cmd");
    const char *const cmd = (cJSON_IsString(jc) && jc->valuestring != NULL) ? jc->valuestring : "";

    /* node: the iotdata system TLVs. "node" alone asks for every report; "node-<type>" asks for
       one, where <type> is a name from iotdata_node.h (version/variant/control/status/config/
       diagnostics). We build the same kvr CONTROL payload a remote manager would send, so the
       MQTT and (later) radio paths run through identical code. */
    if (strncmp(cmd, "node", 4) == 0 && (cmd[4] == '\0' || cmd[4] == '-')) {
        uint8_t kvbuf[32]; // XXX
        iotdata_kvr_t kv;
        iotdata_kvr_init(&kv, kvbuf, sizeof(kvbuf));
        if (cmd[4] == '\0') { /* everything that can be asked for -- so not CONTENT, not RECEIVE */
            for (uint8_t type = 0; type <= IOTDATA_TLV_TYPE_SYSTEM_MAX; type++)
                if (iotdata_node_tlv_control_key(type) != IOTDATA_NODE_TLV_NONE && type != IOTDATA_NODE_TLV_CONTENT)
                    iotdata_kvr_add_flag(&kv, iotdata_node_tlv_control_key(type));
        } else {
            const char *const want = &cmd[5];
            uint8_t found = IOTDATA_NODE_TLV_NONE;
            for (uint8_t type = 0; type <= IOTDATA_TLV_TYPE_SYSTEM_MAX; type++) {
                const char *const nm = iotdata_node_tlv_name(type);
                if (nm != NULL && strcmp(nm, want) == 0) {
                    found = type;
                    break;
                }
            }
            if (found != IOTDATA_NODE_TLV_NONE && iotdata_node_tlv_control_key(found) == IOTDATA_NODE_TLV_NONE) {
                snprintf(st->_buffer_resp, sizeof(st->_buffer_resp), "node: '%s' cannot be requested", want);
                (void)mqtt_send(st->topic_resp, st->_buffer_resp, (int)strlen(st->_buffer_resp));
                PRINTF_ERROR("ctrl: %s\n", st->_buffer_resp);
                cJSON_Delete(root);
                return;
            }
            if (found == IOTDATA_NODE_TLV_NONE) {
                snprintf(st->_buffer_resp, sizeof(st->_buffer_resp), "node: unknown tlv '%s'", want);
                (void)mqtt_send(st->topic_resp, st->_buffer_resp, (int)strlen(st->_buffer_resp));
                PRINTF_ERROR("ctrl: %s\n", st->_buffer_resp);
                cJSON_Delete(root);
                return;
            }
            iotdata_kvr_add_flag(&kv, iotdata_node_tlv_control_key(found));
        }
        PRINTF_INFO("ctrl: node cmd='%s' target=%04X -> %zu byte control\n", cmd, (unsigned)target, kv.len);
        /* staged, NOT executed here: this is the mosquitto thread, and node_on_mqtt() transmits
           and mutates node state the main loop owns */
        if (kv.len > 0 && kv.len <= sizeof(st->pending_buf)) {
            pthread_mutex_lock(&st->lock);
            if (st->pending)
                st->stat_overrun++;
            memcpy(st->pending_buf, kvbuf, kv.len);
            st->pending_len = (int)kv.len;
            st->pending_target = target;
            st->pending_kind = CTRL_PENDING_NODE;
            st->pending = true;
            pthread_mutex_unlock(&st->lock);
        } else
            st->stat_req_bad++;
        cJSON_Delete(root);
        return;
    }

    if (strncmp(cmd, "diag", 4) == 0) {
        const bool local = (target == IOTDATA_MESH_MANAGE_TARGET_ALL || target == st->station_id);
        if (local) {
            if (strcmp(cmd, "diag") == 0) {
                blackbox_status_t s;
                blackbox_status(&st->bbox->handle, &s);
                blackbox_status_str(&s, BLACKBOX_STATUS_ALL, st->_buffer_resp, sizeof(st->_buffer_resp));
            } else if (strcmp(cmd, "diag-enable") == 0) {
                blackbox_enable(&st->bbox->handle, true);
                snprintf(st->_buffer_resp, sizeof(st->_buffer_resp), "diag: enabled");
            } else if (strcmp(cmd, "diag-disable") == 0) {
                blackbox_enable(&st->bbox->handle, false);
                snprintf(st->_buffer_resp, sizeof(st->_buffer_resp), "diag: disabled");
            } else if (strcmp(cmd, "diag-clear") == 0) {
                blackbox_clear(&st->bbox->handle);
                snprintf(st->_buffer_resp, sizeof(st->_buffer_resp), "diag: cleared");
            } else if (strcmp(cmd, "diag-dump") == 0) {
                size_t cur = 0;
                int nl = 0;
                while (blackbox_pull(&st->bbox->handle, &cur, st->_buffer_blackbox_rec, sizeof(st->_buffer_blackbox_rec)) > 0) {
                    (void)mqtt_send(st->topic_resp, st->_buffer_blackbox_rec, (int)strlen(st->_buffer_blackbox_rec));
                    nl++;
                }
                snprintf(st->_buffer_resp, sizeof(st->_buffer_resp), "diag: dumped %d records", nl);
            } else {
                snprintf(st->_buffer_resp, sizeof(st->_buffer_resp), "diag: unknown cmd '%s'", cmd);
            }
            (void)mqtt_send(st->topic_resp, st->_buffer_resp, (int)strlen(st->_buffer_resp));
            PRINTF_INFO("ctrl: diag local cmd='%s' target=%04X -> %s\n", cmd, (unsigned)target, st->_buffer_resp);
        }
        if (target == st->station_id) { /* unicast to the gateway itself — done, nothing to air */
            cJSON_Delete(root);
            return;
        }
        /* broadcast or another node → fall through and air the MANAGE frame below */
    }

    const uint16_t seq = st->seq++;
    uint8_t buf[CTRL_MANAGE_BUF_MAX]; // XXX
    int n = 0;
    if (strcmp(cmd, "status") == 0)
        n = iotdata_mesh_pack_manage_status(buf, st->station_id, seq, target);
    else if (strcmp(cmd, "stations") == 0)
        n = iotdata_mesh_pack_manage_stations_report(buf, st->station_id, seq, target);
    else if (strcmp(cmd, "peers") == 0)
        n = iotdata_mesh_pack_manage_peers_report(buf, st->station_id, seq, target);
    else if (strcmp(cmd, "peers-remove") == 0)
        n = iotdata_mesh_pack_manage_peers_remove(buf, st->station_id, seq, target, station);
    else if (strcmp(cmd, "peers-clear") == 0 || strcmp(cmd, "flush") == 0)
        n = iotdata_mesh_pack_manage_peers_clear(buf, st->station_id, seq, target);
    else if (strcmp(cmd, "filters") == 0)
        n = iotdata_mesh_pack_manage_filter_report(buf, st->station_id, seq, target);
    else if (strcmp(cmd, "block") == 0)
        n = iotdata_mesh_pack_manage_filter_insert(buf, st->station_id, seq, target, station, IOTDATA_MESH_MANAGE_FILTER_BLOCK);
    else if (strcmp(cmd, "allow") == 0)
        n = iotdata_mesh_pack_manage_filter_insert(buf, st->station_id, seq, target, station, IOTDATA_MESH_MANAGE_FILTER_ALLOW);
    else if (strcmp(cmd, "unfilter") == 0)
        n = iotdata_mesh_pack_manage_filter_remove(buf, st->station_id, seq, target, station);
    else if (strcmp(cmd, "filter-clear") == 0)
        n = iotdata_mesh_pack_manage_filter_clear(buf, st->station_id, seq, target, scope);
    else if (strcmp(cmd, "diag") == 0) /* air path: broadcast or a node target (a gateway-unicast diag returned above) */
        n = iotdata_mesh_pack_manage_diag_report(buf, st->station_id, seq, target);
    else if (strcmp(cmd, "diag-enable") == 0)
        n = iotdata_mesh_pack_manage_diag_enable(buf, st->station_id, seq, target, 1u);
    else if (strcmp(cmd, "diag-disable") == 0)
        n = iotdata_mesh_pack_manage_diag_enable(buf, st->station_id, seq, target, 0u);
    else if (strcmp(cmd, "diag-clear") == 0)
        n = iotdata_mesh_pack_manage_diag_clear(buf, st->station_id, seq, target);
    else if (strcmp(cmd, "diag-dump") == 0)
        n = iotdata_mesh_pack_manage_diag_dump(buf, st->station_id, seq, target);
    else
        PRINTF_ERROR("ctrl: unknown cmd '%s'\n", cmd);
    if (n > 0)
        PRINTF_INFO("ctrl: request cmd='%s' target=%04X station=%04X -> MANAGE (%d bytes)\n", cmd, (unsigned)target, (unsigned)station, n);
    cJSON_Delete(root);
    if (n <= 0 || n > (int)sizeof(buf)) {
        st->stat_req_bad++; /* unknown command (n==0) or a pack failure */
        return;
    }
    pthread_mutex_lock(&st->lock);
    if (st->pending)
        st->stat_overrun++; /* previous frame not yet sent — overwrite with the newest */
    memcpy(st->pending_buf, buf, (size_t)n);
    st->pending_len = n;
    st->pending_kind = CTRL_PENDING_MANAGE;
    st->pending = true;
    pthread_mutex_unlock(&st->lock);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

void ctrl_tick(ctrl_state_t *const st, node_state_t *const ns) {

    uint8_t buf[CTRL_MANAGE_BUF_MAX]; // XXX
    int n = 0, kind = CTRL_PENDING_MANAGE;
    uint16_t target = 0;
    pthread_mutex_lock(&st->lock);
    if (st->pending) {
        n = st->pending_len;
        memcpy(buf, st->pending_buf, (size_t)n);
        kind = st->pending_kind;
        target = st->pending_target;
        st->pending = false;
    }
    pthread_mutex_unlock(&st->lock);
    if (n > 0) {
        if (kind == CTRL_PENDING_NODE) { /* runs here, on the main loop, where the node state lives */
            node_on_mqtt(ns, buf, (size_t)n, target);
        } else {
            if (st->tx(buf, n)) {
                st->stat_tx++;
                PRINTF_INFO("ctrl: tx MANAGE (%d bytes)\n", n);
            } else {
                st->stat_tx_err++;
                PRINTF_ERROR("ctrl: tx MANAGE failed (%d bytes)\n", n);
            }
        }
    }

    const time_t now = time(NULL);
    if (st->blackbox_tick_last != 0 && st->blackbox_tick_last != now)
        blackbox_tick(&st->bbox->handle, (uint32_t)(now - st->blackbox_tick_last) * 1000u);
    st->blackbox_tick_last = now;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

bool ctrl_begin(ctrl_state_t *st, const char *topic_prefix, uint16_t station_id, bbox_state_t *bbox, ctrl_tx_handler_t tx) {
    assert(st && bbox && tx);

    memset(st, 0, sizeof(*st));
    st->station_id = station_id;
    st->bbox = bbox;
    st->tx = tx;
    if (pthread_mutex_init(&st->lock, NULL) != 0) {
        PRINTF_ERROR("ctrl: mutex init failed\n");
        return false;
    }
    snprintf(st->topic_req, sizeof(st->topic_req), "%s" CTRL_MQTT_TOPIC_DEFAULT, topic_prefix);
    snprintf(st->topic_resp, sizeof(st->topic_resp), "%s" BBOX_MQTT_TOPIC_DEFAULT, topic_prefix);

    g_ctrl = st;
    if (!mqtt_subscribe(st->topic_req, MQTT_PUBLISH_QOS, ctrl_on_message)) {
        PRINTF_ERROR("ctrl: subscribe to '%s' failed\n", st->topic_req);
        return false;
    }
    PRINTF_INFO("ctrl: station=%04" PRIX16 ", request-topic='%s'\n", station_id, st->topic_req);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void ctrl_end(__attribute__((unused)) ctrl_state_t *st) {
    g_ctrl = NULL;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
