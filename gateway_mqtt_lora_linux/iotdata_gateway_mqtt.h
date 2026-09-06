
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include <cjson/cJSON.h>
#include <pthread.h>

// -----------------------------------------------------------------------------------------------------------------------------------------

#define MQTT_CLIENT_DEFAULT              "iotdata_gateway"
#define MQTT_SERVER_DEFAULT              "mqtt://localhost"
#define MQTT_TLS_DEFAULT                 false
#define MQTT_SYNCHRONOUS_DEFAULT         false
#define MQTT_TOPIC_PREFIX_DEFAULT        "iotdata"
#define MQTT_RECONNECT_DELAY_DEFAULT     5
#define MQTT_RECONNECT_DELAY_MAX_DEFAULT 60

// -----------------------------------------------------------------------------------------------------------------------------------------

typedef bool (*gwmqtt_tx_handler_t)(const uint8_t *packet, const int length);

#define GWMQTT_MANAGE_BUF_MAX 64 /* MANAGE frames are small (STATUS is 8 bytes) */
#define GWMQTT_MANAGE_TOPIC   "/manage/req"

typedef struct {
    bool enabled;
    uint16_t station_id; /* gateway station id — the MANAGE sender */
    uint16_t seq;        /* MANAGE sender sequence (touched only on the mqtt thread) */
    gwmqtt_tx_handler_t tx;
    char topic_req[128];
    pthread_mutex_t lock;
    /* single pending slot: produced by the mqtt-thread callback, drained by the main loop */
    bool pending;
    uint8_t pending_buf[GWMQTT_MANAGE_BUF_MAX];
    int pending_len;
    blackbox_handle_t *blackbox;
    char topic_resp[128];
    time_t blackbox_tick_last; /* for the periodic (batched) flush tick */
    /* stats */
    uint32_t stat_req_rx, stat_req_bad, stat_tx, stat_tx_err, stat_overrun;
} gwmqtt_manage_state_t;

/* The mosquitto message callback has no user-data argument, so the state is reached
   through this file-scope pointer, set in gwmqtt_manage_begin. */
static gwmqtt_manage_state_t *g_gwmqtt_manage = NULL;

// -----------------------------------------------------------------------------------------------------------------------------------------

static uint16_t gwmqtt_manage_parse_target(const cJSON *jt) {
    if (jt == NULL)
        return IOTDATA_MESH_MANAGE_TARGET_ALL; /* default: broadcast */
    if (cJSON_IsString(jt) && jt->valuestring != NULL) {
        if (strcmp(jt->valuestring, "all") == 0 || strcmp(jt->valuestring, "broadcast") == 0)
            return IOTDATA_MESH_MANAGE_TARGET_ALL;
        return (uint16_t)(strtol(jt->valuestring, NULL, 0) & 0x0FFF);
    }
    if (cJSON_IsNumber(jt))
        return (uint16_t)(jt->valueint & 0x0FFF);
    return IOTDATA_MESH_MANAGE_TARGET_ALL;
}

/* A station id parameter (for peers-remove / block / allow / unfilter). 0 if absent. */
static uint16_t gwmqtt_manage_parse_station(const cJSON *jt) {
    if (jt == NULL)
        return 0;
    if (cJSON_IsString(jt) && jt->valuestring != NULL)
        return (uint16_t)(strtol(jt->valuestring, NULL, 0) & 0x0FFF);
    if (cJSON_IsNumber(jt))
        return (uint16_t)(jt->valueint & 0x0FFF);
    return 0;
}

/* Filter-clear scope: "manual" / "auto" / else all. */
static uint8_t gwmqtt_manage_parse_scope(const cJSON *js) {
    if (cJSON_IsString(js) && js->valuestring != NULL) {
        if (strcmp(js->valuestring, "manual") == 0)
            return IOTDATA_MESH_MANAGE_FILTER_SCOPE_MANUAL;
        if (strcmp(js->valuestring, "auto") == 0)
            return IOTDATA_MESH_MANAGE_FILTER_SCOPE_AUTO;
    }
    return IOTDATA_MESH_MANAGE_FILTER_SCOPE_ALL;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void gwmqtt_blackbox_tick(void) {
    gwmqtt_manage_state_t *const st = g_gwmqtt_manage;
    if (st != NULL && st->blackbox != NULL) {
        const time_t now = time(NULL);
        if (st->blackbox_tick_last != 0 && st->blackbox_tick_last != now)
            blackbox_tick(st->blackbox, (uint32_t)(now - st->blackbox_tick_last) * 1000u);
        st->blackbox_tick_last = now;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void gwmqtt_manage_on_message(const char *topic __attribute__((unused)), const unsigned char *payload, const int len) {

    gwmqtt_manage_state_t *const st = g_gwmqtt_manage;
    if (st == NULL)
        return;
    st->stat_req_rx++;

    cJSON *const root = cJSON_ParseWithLength((const char *)payload, (size_t)len);
    if (root == NULL) {
        st->stat_req_bad++;
        fprintf(stderr, "manage: bad JSON request (%d bytes)\n", len);
        return;
    }

    const uint16_t target = gwmqtt_manage_parse_target(cJSON_GetObjectItem(root, "target"));
    const uint16_t station = gwmqtt_manage_parse_station(cJSON_GetObjectItem(root, "station"));
    const uint8_t scope = gwmqtt_manage_parse_scope(cJSON_GetObjectItem(root, "scope"));
    const cJSON *const jc = cJSON_GetObjectItem(root, "cmd");
    const char *const cmd = (cJSON_IsString(jc) && jc->valuestring != NULL) ? jc->valuestring : "";

    if (strncmp(cmd, "blackbox-", 9) == 0) {
        char resp[224];
        blackbox_handle_t *const bb = st->blackbox;
        if (bb == NULL) {
            snprintf(resp, sizeof(resp), "blackbox: not configured");
        } else if (strcmp(cmd, "blackbox-status") == 0) {
            blackbox_status_t s;
            blackbox_status(bb, &s);
            blackbox_status_str(&s, BLACKBOX_STATUS_ALL, resp, sizeof(resp));
        } else if (strcmp(cmd, "blackbox-enable") == 0) {
            blackbox_enable(bb, true);
            snprintf(resp, sizeof(resp), "blackbox: enabled");
        } else if (strcmp(cmd, "blackbox-disable") == 0) {
            blackbox_enable(bb, false);
            snprintf(resp, sizeof(resp), "blackbox: disabled");
        } else if (strcmp(cmd, "blackbox-clear") == 0) {
            blackbox_clear(bb);
            snprintf(resp, sizeof(resp), "blackbox: cleared");
        } else if (strcmp(cmd, "blackbox-bound") == 0) {
            const cJSON *const jr = cJSON_GetObjectItem(root, "records");
            const cJSON *const jb = cJSON_GetObjectItem(root, "bytes");
            const uint32_t mr = cJSON_IsNumber(jr) ? (uint32_t)jr->valuedouble : 0u;
            const uint32_t mb = cJSON_IsNumber(jb) ? (uint32_t)jb->valuedouble : 0u;
            blackbox_bound(bb, mr, mb);
            snprintf(resp, sizeof(resp), "blackbox: bound records=%u bytes=%u", (unsigned)mr, (unsigned)mb);
        } else if (strcmp(cmd, "blackbox-dump") == 0) {
            size_t cur = 0;
            char rec[BLACKBOX_LINE_MAX];
            int nl = 0;
            while (blackbox_pull(bb, &cur, rec, sizeof(rec)) > 0) {
                (void)mqtt_send(st->topic_resp, rec, (int)strlen(rec));
                nl++;
            }
            snprintf(resp, sizeof(resp), "blackbox: dumped %d records", nl);
        } else
            snprintf(resp, sizeof(resp), "blackbox: unknown cmd '%s'", cmd);
        (void)mqtt_send(st->topic_resp, resp, (int)strlen(resp));
        printf("manage: blackbox cmd='%s' -> %s\n", cmd, resp);
        cJSON_Delete(root);
        return;
    }

    const uint16_t seq = st->seq++;

    /* NB: `cmd` points into the cJSON tree, so every use of it must precede cJSON_Delete. */
    uint8_t buf[GWMQTT_MANAGE_BUF_MAX];
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
    else
        fprintf(stderr, "manage: unknown cmd '%s'\n", cmd);
    if (n > 0)
        printf("manage: request cmd='%s' target=%04X station=%04X -> MANAGE (%d bytes)\n", cmd, (unsigned)target, (unsigned)station, n);
    cJSON_Delete(root);

    if (n <= 0 || n > GWMQTT_MANAGE_BUF_MAX) {
        st->stat_req_bad++; /* unknown command (n==0) or a pack failure */
        return;
    }

    pthread_mutex_lock(&st->lock);
    if (st->pending)
        st->stat_overrun++; /* previous frame not yet sent — overwrite with the newest */
    memcpy(st->pending_buf, buf, (size_t)n);
    st->pending_len = n;
    st->pending = true;
    pthread_mutex_unlock(&st->lock);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void gwmqtt_manage_pump(void) {

    gwmqtt_manage_state_t *const st = g_gwmqtt_manage;
    if (st == NULL || !st->enabled)
        return;

    uint8_t buf[GWMQTT_MANAGE_BUF_MAX];
    int n = 0;
    pthread_mutex_lock(&st->lock);
    if (st->pending) {
        n = st->pending_len;
        memcpy(buf, st->pending_buf, (size_t)n);
        st->pending = false;
    }
    pthread_mutex_unlock(&st->lock);

    if (n > 0) {
        if (st->tx != NULL && st->tx(buf, n)) {
            st->stat_tx++;
            printf("manage: tx MANAGE (%d bytes)\n", n);
        } else {
            st->stat_tx_err++;
            fprintf(stderr, "manage: tx MANAGE failed (%d bytes)\n", n);
        }
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

bool gwmqtt_manage_begin(gwmqtt_manage_state_t *st, const char *topic_prefix, uint16_t station_id, gwmqtt_tx_handler_t tx) {

    memset(st, 0, sizeof(*st));
    st->station_id = station_id;
    st->tx = tx;
    if (pthread_mutex_init(&st->lock, NULL) != 0) {
        fprintf(stderr, "manage: mutex init failed\n");
        return false;
    }
    snprintf(st->topic_req, sizeof(st->topic_req), "%s" GWMQTT_MANAGE_TOPIC, topic_prefix);
    snprintf(st->topic_resp, sizeof(st->topic_resp), "%s/blackbox/resp", topic_prefix);

    g_gwmqtt_manage = st;
    st->enabled = true;

    if (!mqtt_subscribe(st->topic_req, MQTT_PUBLISH_QOS, gwmqtt_manage_on_message)) {
        fprintf(stderr, "manage: subscribe to '%s' failed\n", st->topic_req);
        st->enabled = false;
        return false;
    }
    printf("manage: enabled, station=%04" PRIX16 ", request-topic='%s'\n", station_id, st->topic_req);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
