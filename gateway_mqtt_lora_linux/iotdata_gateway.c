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

#ifndef IOTDATA_NODE_CONFIG_LORA_ADDRESS
#define IOTDATA_NODE_CONFIG_LORA_ADDRESS LORA_E22_ADDRESS_MONITOR
#endif

#include "iotdata_config.h"
#include "iotdata_hardware.h"

#define IOTDATA_GATEWAY_VERSION IOTDATA_NODE_VERSION_SEMVER

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
#include "d_module_datastore_linux.h"
#include "d_module_filestore_linux.h"
/* the OTA seams: iotdata_node_otaupdate.h assembles these, and every node that uses the
   framework has an update path, so every node includes them. None depends on an iotdata
   header, so they sit here in the d_ block with the rest. */
#include "d_module_otaslot_linux.h"
#include "d_module_inflate_linux.h"
#include "d_module_hpatch.h"
#define MQTT_CONNECT_TIMEOUT 60
#define MQTT_PUBLISH_QOS     0
#define MQTT_PUBLISH_RETAIN  false
#include "d_module_mqtt_linux.h"
#undef PIN_DEVICE_UART_TX
#undef PIN_DEVICE_UART_RX
#undef PIN_DEVICE_LORA_AUX
#undef PIN_DEVICE_LORA_M0
#undef PIN_DEVICE_LORA_M1
#define PIN_DEVICE_UART_TX  GPIO_NUM_NC
#define PIN_DEVICE_UART_RX  GPIO_NUM_NC
#define PIN_DEVICE_LORA_AUX GPIO_NUM_NC
#define PIN_DEVICE_LORA_M0  GPIO_NUM_NC
#define PIN_DEVICE_LORA_M1  GPIO_NUM_NC
#include "d_interface_e22900t22.h"
static bool lora_packet_write(const uint8_t *const packet, const int length) {
    return length > 0 && lora_write(packet, (size_t)length) == ESP_OK;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// IOTDATA PROTOCOL + PLATFORM
// -----------------------------------------------------------------------------------------------------------------------------------------

#include "iotdata_variant.h"
#include "iotdata.h"
#include "iotdata.c"
#include "iotdata_mesh.h"
#include "iotdata_node.h"

// -----------------------------------------------------------------------------------------------------------------------------------------

#define IOTDATA_NODE_VERSION_HAS_MESH
#define IOTDATA_NODE_DOWN_SLOTS BUFFER_SLOTS_DOWNSTREAM
#include "iotdata_node_platform.h"
#include "iotdata_node_down.h"
#include "iotdata_node_stations.h"

// -----------------------------------------------------------------------------------------------------------------------------------------
// CONFIG
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool gateway_config_changed(const iotdata_node_config_row_t *row, const iotdata_node_config_value_t *was, const iotdata_node_config_info_t *info);
#define IOTDATA_NODE_CONFIG_LORA_NOTIFY gateway_config_changed
#define IOTDATA_NODE_CONFIG_MESH_NOTIFY gateway_config_changed
#define IOTDATA_NODE_CONFIG_DDUP_NOTIFY gateway_config_changed
#define IOTDATA_NODE_CONFIG_MQTT_NOTIFY gateway_config_changed
#define IOTDATA_NODE_CONFIG_STAT_NOTIFY gateway_config_changed
#define IOTDATA_GATEWAY_NOTIFY          gateway_config_changed
#include "iotdata_gateway_conf.h"
#include "iotdata_node_config.h"
#include "iotdata_gateway_conf.h"

// -----------------------------------------------------------------------------------------------------------------------------------------
// CONSOLE
// -----------------------------------------------------------------------------------------------------------------------------------------

static const iotdata_node_console_t s_gateway_cons[] = {
    IOTDATA_NODE_CONFIG_CONSOLE_COMMAND,
    IOTDATA_NODE_SETTINGS_CONSOLE_COMMAND,
};

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static void gateway_blackbox_begin(void) {
    char path[256];
    const uint32_t max_seconds = iotdata_node_config_u32(BLACKBOX_MAX_SECONDS);
    const bool enabled = iotdata_node_config_bool(BLACKBOX_ENABLED);
    if (iotdata_node_diagnostics_begin_conf(
            &(const iotdata_node_diagnostics_conf_t){
                .enabled = enabled,
                .persist_arg = snprintf_inline(path, sizeof(path), "%s/iotdata_gateway_blackbox.csv", iotdata_node_config_string(BLACKBOX_FILE_DIRECTORY)),
                .flush = (max_seconds > 0u) ? BLACKBOX_FLUSH_BATCH_TIME : BLACKBOX_FLUSH_WRITE_THROUGH,
                .flush_ms = max_seconds * 1000u,
                .max_records = iotdata_node_config_u32(BLACKBOX_MAX_RECORDS),
                .max_bytes = iotdata_node_config_u32(BLACKBOX_MAX_BYTES),
                .generations = iotdata_node_config_u8(BLACKBOX_FILE_GENERATIONS),
            },
            IOTDATA_NODE_BB_LC_START, 0))
        PRINTF_INFO("blackbox: %s (path=%s, flush=%s)\n", enabled ? "enabled" : "disabled (default)", iotdata_node_diagnostics_persist_arg(), (max_seconds > 0) ? "ram-cache/batched" : "write-through");
    else
        PRINTF_ERROR("blackbox: init failed (path=%s)\n", path);
}

static void gateway_blackbox_end(void) {
    iotdata_node_diagnostics_end(IOTDATA_NODE_BB_LC_STOP, 0);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
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
 * is the thing this removed. `iotdata_node_settings_saver_attach()` is what swaps the sink.
 */
#define s_settings     gateway_settings

static void gateway_state_begin(store_state_t *state, const store_conf_t *conf) {
    if (datastore_open(&state->datastore, conf->directory)) {
        iotdata_node_state_init(&state->node, &state->datastore, "state");
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
        const bool restored = iotdata_node_state_load(&state->node);
        PRINTF_INFO("state: %u block(s), %s\n", (unsigned)state->node.count, restored ? "restored" : "defaulted");
    }
}

static void gateway_state_end(store_state_t *state) {
    if (state->opened) {
        if (state->node.count > 0)
            (void)iotdata_node_state_flush(&state->node);
        datastore_close(&state->datastore);
        state->opened = false;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

/* THE TIERS, and this order IS the dependency order, not a reading preference:
     utilities -> network (stations) -> node framework + content -> mesh, with survey and trace
     -> ddup -> stats + state -> control + execute
   THE RULE THAT KEEPS IT A DAG: a struct member points DOWN a tier or SIDEWAYS within one; an
   upward edge is a function pointer the lower tier declares and _begin() fills in from here.
   So node declares node_control_handler_t and node_table_*_handler_t and gets ctrl_from_iotdata
   and exec_node_* ; mesh declares mesh_dedup_handler_t and gets ddup_insert_handler, with the
   process state arriving as a void* so its type never reaches mesh; ddup declares ddup_release_fn
   and gets process_release_cb. Downward pointers need none of that and should not have it:
   ctrl holds node_state_t*, survey and trace hold mesh_state_t*, all plain members.
   The member node_state_t once held into stat_state_t was the one pointing UP -- a tier-3 header
   reaching tier 6 for one scalar, start_time -- which is why it became node's own field rather
   than being re-plumbed. Put an upward pointer back in and this list stops being orderable. */

#include "iotdata_gateway_util.h"
#include "iotdata_gateway_netw.h"
#include "iotdata_gateway_node.h"
#include "iotdata_gateway_node_content.h"
#include "iotdata_gateway_mesh.h"
#include "iotdata_gateway_mesh_survey.h"
#include "iotdata_gateway_mesh_trace.h"
#include "iotdata_gateway_ddup.h"

#include "iotdata_gateway_stat.h"
#include "iotdata_gateway_state.h"

#include "iotdata_gateway_ctrl.h"
#include "iotdata_gateway_exec.h"

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

BUFFER_POOL_DECLARE(s_pool, BUFFER_SLOTS_TOTAL, BUFFER_LENGTH_TOTAL);

typedef struct {
    iotdata_node_version_caps_t iotd_caps;
    const char *lora_port;
    lora_config_t lora_config;
    mqtt_config_t mqtt_config;
    mesh_state_t mesh_state;
    ddup_state_t ddup_state;
    ddup_peers_t ddup_peers;
    stat_state_t stat_state;
    node_state_t node_state;
    ctrl_state_t ctrl_state;
    survey_state_t survey_state;
    trace_state_t trace_state;
    store_state_t store_state;
    iotdata_node_t inode;
    process_state_t process_state;
    volatile bool running;
} system_t;

static system_t system_state;

// -----------------------------------------------------------------------------------------------------------------------------------------

static bool gateway_config_changed(__attribute__((unused)) const iotdata_node_config_row_t *const row, __attribute__((unused)) const iotdata_node_config_value_t *const was,
                                   __attribute__((unused)) const iotdata_node_config_info_t *const info) {
    system_t *const state = &system_state;
    state->lora_config.debug = iotdata_node_config_bool(LORA_DEBUG);
    _log_enabled = state->lora_config.debug;
    state->mesh_state.enabled = iotdata_node_config_bool(MESH_ENABLE);
    state->mesh_state.debug = iotdata_node_config_bool(MESH_DEBUG);
    state->mesh_state.beacon_interval_s = iotdata_node_config_u16(MESH_BEACON_INTERVAL_S);
    state->ddup_state.hold_ms = iotdata_node_config_u32(DDUP_HOLD_MS);
    if (state->ddup_state.enabled && state->ddup_state.running)
        ddup_reconfigure(&state->ddup_state, iotdata_node_config_string(DDUP_PEERS), iotdata_node_config_u16(DDUP_DELAY_MS), iotdata_node_config_bool(DDUP_DEBUG), (unsigned)iotdata_node_config_u32(DDUP_DEBUG_INJECT_LATENCY_MS));
    else {
        state->ddup_state.delay_ms = iotdata_node_config_u16(DDUP_DELAY_MS);
        state->ddup_state.debug = iotdata_node_config_bool(DDUP_DEBUG);
        state->ddup_state.debug_inject_latency_ms = (unsigned)iotdata_node_config_u32(DDUP_DEBUG_INJECT_LATENCY_MS);
    }
    state->mqtt_config.reconnect_delay = iotdata_node_config_u16(MQTT_RECONNECT_DELAY_S);
    state->mqtt_config.reconnect_delay_max = iotdata_node_config_u16(MQTT_RECONNECT_DELAY_S_MAX);
    state->mqtt_config.debug = iotdata_node_config_bool(MQTT_DEBUG);
    state->process_state.debug = iotdata_node_config_bool(DEBUG);
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
            iotdata_node_config_help(gateway_help_emit, argv[0]);
            static iotdata_node_settings_t help_settings;
            iotdata_node_settings_defaults(&help_settings, iotdata_node_station_from_mac("gateway"));
            iotdata_node_settings_help(gateway_help_emit, &help_settings);
            exit(EXIT_SUCCESS);
        }

    if (!gateway_config_load(argc, argv))
        return false;
    iotdata_node_config_saver_attach(gateway_config_save);
    iotdata_node_settings_saver_attach(gateway_settings_save);
    iotdata_node_config_console_attach(NULL);
    iotdata_node_console_init(s_gateway_cons, sizeof s_gateway_cons / sizeof s_gateway_cons[0]);

    state->lora_port = iotdata_node_config_string(LORA_PORT);
    state->lora_config = (lora_config_t){ 0 };
    iotdata_node_config_lora_apply(&state->lora_config);
    state->lora_config.rssi_channel = false;
    state->lora_config.rssi_packet = true;

    state->mesh_state = (mesh_state_t){ 0 };

    state->ddup_state = (ddup_state_t){ 0 };
    state->ddup_state.enabled = iotdata_node_config_bool(DDUP_ENABLE);
    state->ddup_state.port = iotdata_node_config_u16(DDUP_PORT);
    ddup_peers_parse(&state->ddup_state, iotdata_node_config_string(DDUP_PEERS));

    state->mqtt_config = (mqtt_config_t){ 0 };
    state->mqtt_config.client = iotdata_node_config_string(MQTT_CLIENT);
    state->mqtt_config.server = iotdata_node_config_string(MQTT_SERVER);
    state->mqtt_config.tls_insecure = iotdata_node_config_bool(MQTT_TLS_INSECURE);
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

    iotdata_node_version_caps_init(&state->iotd_caps);
    iotdata_node_version_caps_add(&state->iotd_caps, IOTDATA_NODE_VERSION_CAPS_RADIO, state->lora_config.module == LORA_MODULE_USB ? IOTDATA_NODE_VERSION_RADIO_E22_USB : IOTDATA_NODE_VERSION_RADIO_E22_DIP);
    char vbuf[IOTDATA_NODE_VERSION_STR_MAX + 1];
    PRINTF_INFO("version: %s\n", iotdata_node_version_str(vbuf, sizeof(vbuf), &(const iotdata_node_version_t){ .caps = &state->iotd_caps }));
    if (!iotdata_node_version_stamp_is_real())
        PRINTF_WARN("version: build stamp is unset -- this binary cannot say when it was built\n");

    BUFFER_POOL_INIT(s_pool, BUFFER_SLOTS_TOTAL, BUFFER_LENGTH_TOTAL, BUFFER_LENGTH_PREFIX);
    gateway_blackbox_begin();
    gateway_content_begin(hw_time_ms());
    gateway_state_begin(&state->store_state, &(const store_conf_t){ .directory = iotdata_node_config_string(STATE) });
    iotdata_node_init(&state->inode, iotdata_node_settings_station(&gateway_settings), &state->store_state.node, STATE_NODE_TAG);
    iotdata_node_settings_console_attach(&gateway_settings, &state->store_state.node);
    PRINTF_INFO("node: station=%03" PRIX16 "\n", iotdata_node_station(&state->inode));

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

    if (state->ddup_state.enabled && state->ddup_state.auto_peer)
        ddup_peers_init(&state->ddup_peers, &state->ddup_state, iotdata_node_station(&state->inode));

    // MQTT BROKER
    if (!mqtt_begin(&state->mqtt_config))
        goto end_device;

    state->running = true;

    // IOTDATA (NETW/NODE/MESH/DDUP/CTRL)
    (void)netw_begin(&state->process_state.network);
    if (!node_begin(&state->node_state, &state->inode, &state->iotd_caps, &s_pool, lora_packet_write, ctrl_from_iotdata, exec_node_status_mesh, exec_node_table_count, exec_node_table_row, ctrl_from_iotdata_keys,
                    CTRL_FROM_IOTDATA_KEYS_COUNT, iotdata_node_config_string(MQTT_TOPIC_PREFIX)))
        goto end_mqtt;
    node_settings_attach(&state->node_state, &gateway_settings, &state->store_state.node);
    mesh_begin(&state->mesh_state, &state->inode, &s_pool, lora_packet_write, ddup_insert_handler, (void *)&state->process_state);
    survey_begin(&state->survey_state, iotdata_node_config_string(MQTT_TOPIC_PREFIX), &state->inode, &state->mesh_state, &s_pool);
    trace_begin(&state->trace_state, iotdata_node_config_string(MQTT_TOPIC_PREFIX), &state->inode, &state->mesh_state, &s_pool);
    if (!ddup_begin(&state->ddup_state, iotdata_node_station(&state->inode), &state->mesh_state.dedup_ring, &state->running))
        goto end_mesh;
    if (state->ddup_state.enabled && state->ddup_state.auto_peer) {
        ddup_peers_begin(&state->ddup_peers);
        state->process_state.peers_ddup = &state->ddup_peers; /* NULL otherwise: the loop skips it */
    }
    if (!ctrl_begin(&state->ctrl_state, iotdata_node_config_string(MQTT_TOPIC_PREFIX), &state->inode, &state->node_state, &state->survey_state, &state->trace_state, &s_pool))
        goto end_ddup;

    // PROCESS
    gateway_state_restore(&state->store_state);
    stat_begin(&state->stat_state, iotdata_node_config_string(MQTT_TOPIC_PREFIX), &state->inode, IOTDATA_GATEWAY_VERSION, &state->lora_config, &s_pool);
    state->process_state.buffer_pool = &s_pool;
    ret = process_run(&state->process_state, &state->store_state.node, &state->node_state, &state->mesh_state, &state->ddup_state, &state->stat_state, &state->ctrl_state, &state->survey_state, &state->trace_state, &state->running)
              ? EXIT_SUCCESS
              : EXIT_FAILURE;
    stat_end(&state->stat_state);

    ctrl_end(&state->ctrl_state);
end_ddup:
    ddup_peers_end(&state->ddup_peers);
    ddup_end(&state->ddup_state);
end_mesh:
    trace_end(&state->trace_state);
    survey_end(&state->survey_state);
    mesh_end(&state->mesh_state);
    node_end(&state->node_state);
end_mqtt:
    mqtt_end();
end_device:
    lora_stop();
end_all:
    gateway_state_end(&state->store_state);
    gateway_content_end();
    gateway_blackbox_end();
    return ret;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
