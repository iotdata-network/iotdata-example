// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

/*
 * IoT Sensor Telemetry Protocol
 * Copyright(C) 2026 Matthew Gream (https://libiotdata.org)
 *
 * iotdata_gateway.c - E22-900T22U to MQTT gateway
 *
 * Receives iotdata binary frames from E22-900T22U radio, decodes to JSON,
 * and publishes to MQTT topic: <prefix>/<variant_name>/<station_id>
 *
 * Variant definitions are compiled in from the common headers.
 * No routing configuration needed — the variant byte in the iotdata header
 * determines the topic automatically.
 *
 * Mesh support (variant 15):
 *   - FORWARD packets are unwrapped and the inner sensor data processed
 *     as if received directly.
 *   - Duplicate suppression via {station_id, sequence} ring buffer.
 *   - Beacon origination on a configurable interval (when mesh-enable=true).
 *   - ACK transmission to FORWARD senders (stub, ready for implementation).
 *   - All mesh control packets are logged for diagnostics.
 *
 * Dedup support:
 *   - allow incoming, and establish outgoing, UDP streams to specified
 *     other gateways to synchronise station_id/sequence pairs for edge
 *     de-duplication before publishing to MQTT. Operates indepemdently of
 *     Mesh protocol.
 */

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <time.h>
#include <assert.h>

#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#ifndef IOTDATA_CONFIG_LORA_ADDRESS
#define IOTDATA_CONFIG_LORA_ADDRESS LORA_E22_ADDRESS_MONITOR
#endif

#include "iotdata_config.h"
#include "iotdata_hardware.h"

#define IOTDATA_GATEWAY_VERSION IOTDATA_VERSION_SEMVER

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#ifndef BUFFER_LENGTH_FRAME
#define BUFFER_LENGTH_FRAME IOTDATA_MAX_PACKET_SIZE
#endif
#ifndef BUFFER_LENGTH_PREFIX
#define BUFFER_LENGTH_PREFIX 0 /* nothing is prepended: a gateway terminates frames, it does not forward them */
#endif
#ifndef BUFFER_LENGTH_TOTAL
#define BUFFER_LENGTH_TOTAL (BUFFER_LENGTH_PREFIX + BUFFER_LENGTH_FRAME + 1) /* 1 = RSSI byte */
#endif
#ifndef BUFFER_SLOTS_DOWNSTREAM
#define BUFFER_SLOTS_DOWNSTREAM 128
#endif
#ifndef BUFFER_SLOTS_PROCESSING
#define BUFFER_SLOTS_PROCESSING 24 /* one being received, one being sent, the staged control commands */
#endif
#ifndef BUFFER_SLOTS_MARGIN
#define BUFFER_SLOTS_MARGIN 8
#endif
#ifndef BUFFER_SLOTS_TOTAL
#define BUFFER_SLOTS_TOTAL (BUFFER_SLOTS_PROCESSING + BUFFER_SLOTS_DOWNSTREAM + BUFFER_SLOTS_MARGIN)
#endif
// #ifndef BUFFER_QUEUE_TX_MAX
// #define BUFFER_QUEUE_TX_MAX (BUFFER_SLOTS_PROCESSING / 2) /* frames waiting for the radio at once */
// #endif

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

bool _log_enabled = false;

__attribute__((format(printf, 3, 4))) static void _log_write(FILE *const to, const char level, const char *const format, ...) {
    struct timespec ts;
    struct tm tm;
    clock_gettime(CLOCK_REALTIME, &ts);
    localtime_r(&ts.tv_sec, &tm);
    fprintf(to, "%c (%02d:%02d:%02d.%03ld) ", level, tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec / 1000000L);
    va_list args;
    va_start(args, format);
    vfprintf(to, format, args);
    va_end(args);
}

#define PRINTF_INFO(...)  _log_write(stdout, 'I', __VA_ARGS__)
#define PRINTF_WARN(...)  _log_write(stderr, 'W', __VA_ARGS__)
#define PRINTF_ERROR(...) _log_write(stderr, 'E', __VA_ARGS__)
#define PRINTF_DEBUG(...) \
    do { \
        if (_log_enabled) \
            _log_write(stdout, 'D', __VA_ARGS__); \
    } while (0)

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include "d_platform_linux.h"
#include "d_common.h"
#include "d_format.h"

#define BUFFER_LOCK_TYPE       pthread_mutex_t
#define BUFFER_LOCK_INIT(l)    pthread_mutex_init((l), NULL)
#define BUFFER_LOCK_ACQUIRE(l) pthread_mutex_lock(l)
#define BUFFER_LOCK_RELEASE(l) pthread_mutex_unlock(l)
#include "d_module_buffers.h"

#define PIN_DEVICE_UART_TX  GPIO_NUM_NC
#define PIN_DEVICE_UART_RX  GPIO_NUM_NC
#define PIN_DEVICE_LORA_AUX GPIO_NUM_NC
#define PIN_DEVICE_LORA_M0  GPIO_NUM_NC
#define PIN_DEVICE_LORA_M1  GPIO_NUM_NC
#include "d_interface_e22900t22.h"
static bool lora_packet_write(const uint8_t *const packet, const int length) {
    return length > 0 && lora_write(packet, (size_t)length) == ESP_OK;
}

#include "d_module_datastore_linux.h"

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define MQTT_CONNECT_TIMEOUT 60
#define MQTT_PUBLISH_QOS     0
#define MQTT_PUBLISH_RETAIN  false
#include "mqtt_linux.h"

// -----------------------------------------------------------------------------------------------------------------------------------------
// IOTDATA
// -----------------------------------------------------------------------------------------------------------------------------------------

#include "iotdata_variant.h"
#include "iotdata.h"
#include "iotdata.c"
#include "iotdata_mesh.h"
#include "iotdata_node.h"

#include "iotdata_node_utils.h"
#include "iotdata_node_partial.h"
#define IOTDATA_VERSION_HAS_MESH
#define IOTDATA_VERSION_HAS_BLACKBOX
#include "iotdata_node_version.h"
#include "iotdata_node_variant.h"
#include "iotdata_node_status.h"
#include "iotdata_node_config.h"
#include "iotdata_node_control.h"
#define IOTDATA_DIAGNOSTICS IOTDATA_CONFIG_BLACKBOX
#define IOTDATA_BLACKBOX_IMPLEMENTATION
#include "iotdata_node_diagnostics.h"
// #include "iotdata_node_content.h"

#include "iotdata_node_state.h"
#include "iotdata_node_console.h"
#include "iotdata_node_settings.h"
#define IOTDATA_DOWN_SLOTS BUFFER_SLOTS_DOWNSTREAM
#include "iotdata_node_down.h"
#include "iotdata_node_stations.h"
#include "iotdata_node_platform.h"

// -----------------------------------------------------------------------------------------------------------------------------------------

typedef struct {
    bool enabled;
    const char *directory;
    uint32_t max_records, max_seconds, max_bytes;
    uint8_t generations;
} bbox_conf_t;
typedef struct {
    blackbox_handle_t handle;
    blackbox_config_t config;
    char pool[IOTDATA_BLACKBOX_POOL_SZ];
    char path[512];
} bbox_state_t;

static void gateway_blackbox_begin(bbox_state_t *state, const bbox_conf_t *conf) {
    state->config = (blackbox_config_t){
        .pool = state->pool,
        .pool_sz = sizeof(state->pool),
        .flush = (conf->max_seconds > 0u) ? BLACKBOX_FLUSH_BATCH_TIME : BLACKBOX_FLUSH_WRITE_THROUGH,
        .flush_ms = conf->max_seconds * 1000u,
        .max_records = conf->max_records,
        .max_bytes = conf->max_bytes,
        .generations = conf->generations,
        .persist_arg = snprintf_inline(state->path, sizeof(state->path), "%s/iotdata_gateway_blackbox.csv", conf->directory),
        .enabled = conf->enabled,
    };
    if (blackbox_init(&state->handle, &state->config) == 0) {
        (void)iotdata_blackbox_lifecycle(&state->handle, IOTDATA_BB_LC_START, 0);
        PRINTF_INFO("blackbox: %s (path=%s, flush=%s)\n", conf->enabled ? "enabled" : "disabled (default)", state->path, (conf->max_seconds > 0) ? "ram-cache/batched" : "write-through");
    } else
        PRINTF_ERROR("blackbox: init failed (path=%s)\n", state->path);
}

static void gateway_blackbox_end(bbox_state_t *state) {
    if (state->handle.cfg != NULL) {
        (void)iotdata_blackbox_lifecycle(&state->handle, IOTDATA_BB_LC_STOP, 0);
        (void)blackbox_flush(&state->handle);
        blackbox_deinit(&state->handle);
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

typedef struct {
    const char *directory;
} store_conf_t;
typedef struct {
    datastore_t datastore;
    iotdata_node_state_t node;
    bool opened;
} store_state_t;

#define STATE_NODE_TAG 0xB1E28004UL

/*
 * The PROTOCOL's own settings -- station id, reporting schedule, receive schedule.
 *
 * NOT A STATE BLOCK HERE, unlike on a device. They live in `gateway_settings`, seeded and loaded by
 * the config loader out of the SAME FILE the configuration comes from, and written back by the same
 * writer. A host has one place a person looks; a second store, opaque and in a directory beside it,
 * is the thing this removed. `iotdata_settings_saver_attach()` is what swaps the sink.
 */
#define s_settings     gateway_settings

static void gateway_state_begin(store_state_t *state, const store_conf_t *conf) {
    if (datastore_open(&state->datastore, conf->directory)) {
        iotdata_state_init(&state->node, &state->datastore, "state");
        state->opened = true;
        if (!datastore_persistent(&state->datastore))
            PRINTF_WARN("state: '%s' is a tmpfs -- -- nothing will persist across a reboot\n", conf->directory);
        else
            PRINTF_INFO("state: '%s'\n", conf->directory);
    } else
        PRINTF_ERROR("state: cannot open '%s' (%s) -- nothing will persist across a restart\n", conf->directory, strerror(errno));
}

static void gateway_state_restore(store_state_t *state) {
    if (state->opened) {
        const bool restored = iotdata_state_load(&state->node);
        PRINTF_INFO("state: %u block(s), %s\n", (unsigned)state->node.count, restored ? "restored" : "defaulted");
    }
}

static void gateway_state_end(store_state_t *state) {
    if (state->opened) {
        if (state->node.count > 0)
            (void)iotdata_state_flush(&state->node);
        datastore_close(&state->datastore);
        state->opened = false;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool gateway_config_changed(const iotdata_config_row_t *row, const iotdata_config_value_t *was, const iotdata_config_info_t *info);
#define IOTDATA_CONFIG_LORA_NOTIFY gateway_config_changed
#define IOTDATA_CONFIG_MESH_NOTIFY gateway_config_changed
#define IOTDATA_CONFIG_DDUP_NOTIFY gateway_config_changed
#define IOTDATA_CONFIG_MQTT_NOTIFY gateway_config_changed
#define IOTDATA_CONFIG_STAT_NOTIFY gateway_config_changed
#define IOTDATA_GATEWAY_NOTIFY     gateway_config_changed

#include "iotdata_gateway_conf.h"
#include "iotdata_node_config.h"
#include "iotdata_gateway_conf.h"

static const iotdata_console_t s_gateway_cons[] = {
    IOTDATA_CONFIG_CONSOLE_COMMAND,
    IOTDATA_SETTINGS_CONSOLE_COMMAND,
};

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include "iotdata_gateway_util.h"
#include "iotdata_gateway_mesh.h"
#include "iotdata_gateway_ddup.h"
#include "iotdata_gateway_peer.h"
#include "iotdata_gateway_stat.h"
#include "iotdata_gateway_node.h"
#include "iotdata_gateway_netw.h"
#include "iotdata_gateway_survey.h"
#include "iotdata_gateway_trace.h"
#include "iotdata_gateway_state.h"
#include "iotdata_gateway_ctrl.h"
#include "iotdata_gateway_exec.h"

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

BUFFER_POOL_DECLARE(s_pool, BUFFER_SLOTS_TOTAL, BUFFER_LENGTH_TOTAL);

typedef struct {
    iotdata_version_caps_t iotd_caps;
    const char *lora_port;
    lora_config_t lora_config;
    mqtt_config_t mqtt_config;
    mesh_state_t mesh_state;
    ddup_state_t ddup_state;
    peer_presence_t peer_presence;
    stat_state_t stat_state;
    node_state_t node_state;
    ctrl_state_t ctrl_state;
    survey_state_t survey_state;
    trace_state_t trace_state;
    bbox_state_t bbox_state;
    store_state_t store_state;
    idep_node_t inode;
    process_state_t process_state;
    volatile bool running;
} system_t;

static system_t system_state;

// -----------------------------------------------------------------------------------------------------------------------------------------

static bool gateway_config_changed(__attribute__((unused)) const iotdata_config_row_t *const row, __attribute__((unused)) const iotdata_config_value_t *const was, __attribute__((unused)) const iotdata_config_info_t *const info) {
    system_t *const state = &system_state;
    state->lora_config.debug = iotdata_config_bool(LORA_DEBUG);
    _log_enabled = state->lora_config.debug;
    state->mesh_state.enabled = iotdata_config_bool(MESH_ENABLE);
    state->mesh_state.debug = iotdata_config_bool(MESH_DEBUG);
    state->mesh_state.beacon_interval_s = iotdata_config_u16(MESH_BEACON_INTERVAL_S);
    state->ddup_state.hold_ms = iotdata_config_u32(DDUP_HOLD_MS);
    if (state->ddup_state.enabled && state->ddup_state.running != NULL)
        ddup_reconfigure(&state->ddup_state, iotdata_config_string(DDUP_PEERS), iotdata_config_u16(DDUP_DELAY_MS), iotdata_config_bool(DDUP_DEBUG), (unsigned)iotdata_config_u32(DDUP_DEBUG_INJECT_LATENCY_MS));
    else {
        state->ddup_state.delay_ms = iotdata_config_u16(DDUP_DELAY_MS);
        state->ddup_state.debug = iotdata_config_bool(DDUP_DEBUG);
        state->ddup_state.debug_inject_latency_ms = (unsigned)iotdata_config_u32(DDUP_DEBUG_INJECT_LATENCY_MS);
    }
    state->mqtt_config.reconnect_delay = iotdata_config_u16(MQTT_RECONNECT_DELAY_S);
    state->mqtt_config.reconnect_delay_max = iotdata_config_u16(MQTT_RECONNECT_DELAY_S_MAX);
    state->mqtt_config.debug = iotdata_config_bool(MQTT_DEBUG);
    state->process_state.debug = iotdata_config_bool(DEBUG);
    state->process_state.debug_data = _log_enabled;
    return false; /* already true; FLAG_REBOOT on the row is what asks for a restart */
}

// -----------------------------------------------------------------------------------------------------------------------------------------

__attribute__((format(printf, 1, 2))) static void gateway_help_emit(const char *const fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    (void)vprintf(fmt, ap);
    va_end(ap);
}

bool system_config(system_t *state, const int argc, char *argv[]) {

    for (int i = 1; i < argc; i++)
        if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            iotdata_config_help(gateway_help_emit, argv[0]);
            static iotdata_settings_t help_settings;
            iotdata_settings_defaults(&help_settings, iotdata_node_station_from_mac("gateway"));
            iotdata_settings_help(gateway_help_emit, &help_settings);
            exit(EXIT_SUCCESS);
        }

    if (!gateway_config_load(argc, argv))
        return false;
    iotdata_config_saver_attach(gateway_config_save);
    iotdata_settings_saver_attach(gateway_settings_save);
    iotdata_config_console_attach(NULL);
    iotdata_console_init(s_gateway_cons, sizeof s_gateway_cons / sizeof s_gateway_cons[0]);

    state->lora_port = iotdata_config_string(LORA_PORT);
    state->lora_config = (lora_config_t){ 0 };
    iotdata_config_lora_apply(&state->lora_config);
    state->lora_config.rssi_channel = false;
    state->lora_config.rssi_packet = true;

    state->mesh_state = (mesh_state_t){ 0 };

    state->ddup_state = (ddup_state_t){ 0 };
    state->ddup_state.enabled = iotdata_config_bool(DDUP_ENABLE);
    state->ddup_state.port = iotdata_config_u16(DDUP_PORT);
    ddup_peers_parse(&state->ddup_state, iotdata_config_string(DDUP_PEERS));

    state->mqtt_config = (mqtt_config_t){ 0 };
    state->mqtt_config.client = iotdata_config_string(MQTT_CLIENT);
    state->mqtt_config.server = iotdata_config_string(MQTT_SERVER);
    state->mqtt_config.tls_insecure = iotdata_config_bool(MQTT_TLS_INSECURE);
    state->mqtt_config.synchronous = false;

    state->stat_state = (stat_state_t){ 0 };

    state->process_state = (process_state_t){ 0 };

    (void)gateway_config_changed(NULL, NULL, NULL);

    state->running = false;

    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void signal_handler(const int sig __attribute__((unused))) {
    if (system_state.running) {
        PRINTF_INFO("stopping\n");
        system_state.running = false;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

int main(int argc, char *argv[]) {

    int ret = EXIT_FAILURE;

    setbuf(stdout, NULL);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    system_t *state = &system_state;

    if (!system_config(state, argc, argv))
        return ret;

    PRINTF_INFO("starting (iotdata gateway: variants=%d, features=mesh,dedup,stats,mqtt)\n", IOTDATA_VARIANT_MAPS_COUNT);

    iotdata_version_caps_init(&state->iotd_caps);
    iotdata_version_caps_add(&state->iotd_caps, IOTDATA_VERSION_CAPS_RADIO, state->lora_config.module == LORA_MODULE_USB ? IOTDATA_VERSION_RADIO_E22_USB : IOTDATA_VERSION_RADIO_E22_DIP);

    char vbuf[IOTDATA_VERSION_STR_MAX + 1];
    PRINTF_INFO("version: %s\n", iotdata_version_str(vbuf, sizeof(vbuf), &state->iotd_caps));
    if (!iotdata_version_stamp_is_real())
        PRINTF_WARN("version: build stamp is unset -- this binary cannot say when it was built\n");

    BUFFER_POOL_INIT(s_pool, BUFFER_SLOTS_TOTAL, BUFFER_LENGTH_TOTAL, BUFFER_LENGTH_PREFIX);
    gateway_blackbox_begin(&state->bbox_state, &(const bbox_conf_t){ .enabled = iotdata_config_bool(BLACKBOX_ENABLED),
                                                                     .directory = iotdata_config_string(BLACKBOX_FILE_DIRECTORY),
                                                                     .max_records = iotdata_config_u32(BLACKBOX_MAX_RECORDS),
                                                                     .max_seconds = iotdata_config_u32(BLACKBOX_MAX_SECONDS),
                                                                     .max_bytes = iotdata_config_u32(BLACKBOX_MAX_BYTES),
                                                                     .generations = iotdata_config_u8(BLACKBOX_FILE_GENERATIONS) });
    gateway_state_begin(&state->store_state, &(const store_conf_t){ .directory = iotdata_config_string(STATE) });
    idep_node_init(&state->inode, iotdata_settings_station(&gateway_settings), &state->store_state.node, STATE_NODE_TAG);
    iotdata_settings_console_attach(&gateway_settings, &state->store_state.node);
    PRINTF_INFO("node: station=%03" PRIX16 "\n", idep_station(&state->inode));

    // DEVICE (LORA)
    hw_uart_set_device(state->lora_port);
    esp_err_t lerr;
    if ((lerr = lora_setup(&state->lora_config)) != ESP_OK) {
        PRINTF_ERROR("device: setup failure (port=%s): %s\n", state->lora_port, esp_err_to_name(lerr));
        goto end_all;
    }
    if ((lerr = lora_start()) != ESP_OK) {
        PRINTF_ERROR("device: start failure (port=%s): %s\n", state->lora_port, esp_err_to_name(lerr));
        goto end_device;
    }
    PRINTF_INFO("device: connect success (port=%s)\n", state->lora_port);

    /* THE WILL, BEFORE THE CONNECT. A last will is registered as part of CONNECT, so setting it
       afterwards sets it for a session that has already begun without one -- and the whole point of
       it is to clear our presence if this process dies without saying so. Only the topic is needed
       here; the ddup state it reads is already configured. */
    if (state->ddup_state.enabled && state->ddup_state.auto_peer)
        peer_presence_init(&state->peer_presence, &state->ddup_state, idep_station(&state->inode));

    // MQTT BROKER
    if (!mqtt_begin(&state->mqtt_config))
        goto end_device;

    state->running = true;

    // IOTDATA (NETW/NODE/MESH/DDUP/CTRL)
    (void)netw_begin(&state->process_state.network);
    if (!node_begin(&state->node_state, &state->inode, &state->iotd_caps, &state->stat_state, &state->bbox_state, &s_pool, lora_packet_write, ctrl_from_iotdata, exec_node_status_mesh, exec_node_table_count, exec_node_table_row,
                    ctrl_from_iotdata_keys, (uint8_t)(sizeof(ctrl_from_iotdata_keys) / sizeof(ctrl_from_iotdata_keys[0])), iotdata_config_string(MQTT_TOPIC_PREFIX)))
        goto end_mqtt;
    node_settings_attach(&state->node_state, &gateway_settings, &state->store_state.node);
    if (!mesh_begin(&state->mesh_state, &state->inode, &s_pool, lora_packet_write, ddup_insert_handler, (void *)&state->process_state))
        goto end_node;
    /* BY VALUE, not by node: ddup keeps its own copy of the gateway id because the election compares
       it on the receive thread, and reaching through a node from there would be a second reader of
       something the main loop can change. gateway_config_changed() refreshes it -- see there. */
    if (!ddup_begin(&state->ddup_state, idep_station(&state->inode), &state->mesh_state.dedup_ring, &state->running))
        goto end_mesh;
    survey_begin(&state->survey_state, iotdata_config_string(MQTT_TOPIC_PREFIX), &state->inode, &s_pool);
    trace_begin(&state->trace_state, iotdata_config_string(MQTT_TOPIC_PREFIX), &state->inode, &s_pool);
    if (!ctrl_begin(&state->ctrl_state, iotdata_config_string(MQTT_TOPIC_PREFIX), &state->inode, &state->bbox_state, &state->survey_state, &state->trace_state, &s_pool))
        goto end_ddup;
    /* AND THE SUBSCRIPTION AFTER ddup_begin(), not with the will above: a retained presence can be
       delivered the instant we subscribe, and the handler calls into ddup's peer table -- whose
       mutex does not exist until ddup_begin() has run. */
    if (state->ddup_state.enabled && state->ddup_state.auto_peer) {
        peer_presence_start(&state->peer_presence);
        state->process_state.state_peer = &state->peer_presence; /* NULL otherwise: the loop skips it */
    }

    // PROCESS
    gateway_state_restore(&state->store_state);
    stat_begin(&state->stat_state, iotdata_config_string(MQTT_TOPIC_PREFIX), &state->inode, IOTDATA_GATEWAY_VERSION, &state->lora_config, &s_pool);
    state->process_state.pool = &s_pool;
    ret = process_run(&state->process_state, &state->store_state.node, &state->node_state, &state->mesh_state, &state->ddup_state, &state->stat_state, &state->ctrl_state, &state->survey_state, &state->trace_state, &state->running)
              ? EXIT_SUCCESS
              : EXIT_FAILURE;
    stat_end(&state->stat_state);

    /* say we have gone, rather than leaving the broker to notice: a clean exit clears the retained
       presence at once, where the will only fires when the connection drops */
    peer_presence_shutdown(&state->peer_presence);
    ctrl_end(&state->ctrl_state);
    survey_end(&state->survey_state);
    trace_end(&state->trace_state);
end_ddup:
    ddup_end(&state->ddup_state);
end_mesh:
    mesh_end(&state->mesh_state);
end_node:
    node_end(&state->node_state);
end_mqtt:
    mqtt_end();
end_device:
    lora_stop();
end_all:
    gateway_state_end(&state->store_state);
    gateway_blackbox_end(&state->bbox_state);
    return ret;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
