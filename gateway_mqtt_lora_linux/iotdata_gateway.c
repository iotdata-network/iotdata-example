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

#define MQTT_CONNECT_TIMEOUT    60
#define MQTT_PUBLISH_QOS        0
#define MQTT_PUBLISH_RETAIN     false

#define IOTDATA_GATEWAY_VERSION IOTDATA_VERSION_SEMVER

#define IOTDATA_VERSION_HAS_MESH
#define IOTDATA_VERSION_HAS_BLACKBOX

#define CONFIG_FILE_DEFAULT           "iotdata_gateway.cfg"

#define SERIAL_PORT_DEFAULT           "/dev/e22900t22u"
#define SERIAL_RATE_DEFAULT           9600
#define SERIAL_BITS_DEFAULT           SERIAL_8N1

#define STAT_INTERVAL_DEFAULT         (5 * 60)
#define INTERVAL_RSSI_CHANNEL_DEFAULT 0

#define INTERVAL_BEACON_DEFAULT       60 /* seconds */

#define GATEWAY_STATION_ID_DEFAULT    1

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#ifndef BUFFER_LENGTH_FRAME
#define BUFFER_LENGTH_FRAME 240 /* the E22 sub-packet size: the largest frame that arrives in one piece */
#endif
#ifndef BUFFER_LENGTH_PREFIX
#define BUFFER_LENGTH_PREFIX 0 /* nothing is prepended: a gateway terminates frames, it does not forward them */
#endif
#ifndef BUFFER_LENGTH_TOTAL
#define BUFFER_LENGTH_TOTAL (BUFFER_LENGTH_FRAME + 1 + BUFFER_LENGTH_PREFIX) /* 1 = RSSI byte */
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

/* No pins on a USB dongle. The driver takes these by name and ignores them when module == USB. */
#define PIN_DEVICE_UART_TX  GPIO_NUM_NC
#define PIN_DEVICE_UART_RX  GPIO_NUM_NC
#define PIN_DEVICE_LORA_AUX GPIO_NUM_NC
#define PIN_DEVICE_LORA_M0  GPIO_NUM_NC
#define PIN_DEVICE_LORA_M1  GPIO_NUM_NC

#include "d_interface_e22900t22.h"

/*
 * raw RSSI byte -> dBm. Was get_rssi_dbm() in the depend core; kept here because the stat layer
 * stores the raw byte and converts on output, and the conversion is a pure offset so there is no
 * reason to disturb that. Carried over verbatim, comment included, because it is hard-won:
 *
 *   Both the DIP and USB datasheets specify the *channel RSSI register* formula (DIP: -(256-rssi),
 *   USB: -RSSI/2) but are silent on the per-packet RSSI byte. Empirically the USB packet RSSI is
 *   -(256-rssi), matching the DIP: same SX1262 silicon, so the byte is encoded identically. The
 *   USB "-RSSI/2" is treated as a datasheet error and -(256-rssi) is used wholesale for both
 *   modules / both cases (verified against reciprocity: co-located gateway<->relay links agree).
 */
static inline int get_rssi_dbm(const uint8_t rssi) {
    return -(256 - (int)rssi);
}

/* dBm back to the raw byte, for handing the driver's output to the stat layer unchanged. */
static inline uint8_t rssi_raw_from_dbm(const int dbm) {
    return (uint8_t)(dbm + 256);
}

/* The transmit hook the node/mesh/ctrl layers take: bool(const uint8_t *, int). */
static bool lora_packet_write(const uint8_t *const packet, const int length) {
    return length > 0 && lora_write(packet, (size_t)length) == ESP_OK;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include "mqtt_linux.h"

// -----------------------------------------------------------------------------------------------------------------------------------------
// IOTDATA
// -----------------------------------------------------------------------------------------------------------------------------------------

#include "iotdata_config.h"
#include "iotdata_variant.h"
#include "iotdata.c"
#include "iotdata_mesh.h"
#define IOTDATA_DOWN_SLOTS BUFFER_SLOTS_DOWNSTREAM
#include "iotdata_down.h"
#include "iotdata_node.h"
#include "iotdata_node_version.h"
// variant
#include "iotdata_node_control.h"
// status
// config
// diagnostics
// content
#define IOTDATA_BLACKBOX_IMPLEMENTATION
#include "iotdata_blackbox.h"
#include "iotdata_station_filter.h"

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

typedef struct {
    blackbox_handle_t handle;
    blackbox_config_t config;
    char pool[IOTDATA_BLACKBOX_POOL_SZ];
    char path[512];
} bbox_state_t;

#include "iotdata_gateway_util.h"
#include "iotdata_gateway_mesh.h"
#include "iotdata_gateway_ddup.h"
#include "iotdata_gateway_stat.h"
#include "iotdata_gateway_node.h"
#include "iotdata_gateway_netw.h"
#include "iotdata_gateway_state.h"
#include "iotdata_gateway_ctrl.h"
#include "iotdata_gateway_exec.h"

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include "config_linux.h"

// clang-format off
const struct option config_options [] = {
    {"help",                            no_argument,       0, 'h'},
    {"config",                          required_argument, 0, 0},
    //
    {"lora-port",                       required_argument, 0, 0},
    {"lora-rate",                       required_argument, 0, 0},
    {"lora-bits",                       required_argument, 0, 0},
    {"lora-address",                    required_argument, 0, 0},
    {"lora-network",                    required_argument, 0, 0},
    {"lora-channel",                    required_argument, 0, 0},
    {"lora-crypt",                      required_argument, 0, 0},
    {"lora-packet-size",                required_argument, 0, 0},
    {"lora-packet-rate",                required_argument, 0, 0},
    {"lora-transmit-power",             required_argument, 0, 0},
    {"lora-transmission-method",        required_argument, 0, 0},
    {"lora-rssi-channel",               required_argument, 0, 0},
    {"lora-rssi-packet",                required_argument, 0, 0},
    {"lora-listen-before-transmit",     required_argument, 0, 0},
    {"lora-read-timeout-command",       required_argument, 0, 0},
    {"lora-read-timeout-packet",        required_argument, 0, 0},
    {"lora-debug",                      required_argument, 0, 0},
    //
    {"mqtt-client",                     required_argument, 0, 0},
    {"mqtt-server",                     required_argument, 0, 0},
    {"mqtt-topic-prefix",               required_argument, 0, 0},
    {"mqtt-tls-insecure",               required_argument, 0, 0},
    {"mqtt-reconnect-delay",            required_argument, 0, 0},
    {"mqtt-reconnect-delay-max",        required_argument, 0, 0},
    {"mqtt-debug",                      required_argument, 0, 0},
    //
    {"mesh-enable",                     required_argument, 0, 0},
    {"mesh-station-id",                 required_argument, 0, 0},
    {"mesh-beacon-interval",            required_argument, 0, 0},
    {"mesh-debug",                      required_argument, 0, 0},
    //
    {"ddup-enable",                     required_argument, 0, 0},
    {"ddup-port",                       required_argument, 0, 0},
    {"ddup-peers",                      required_argument, 0, 0},
    {"ddup-delay",                      required_argument, 0, 0},
    {"ddup-debug",                      required_argument, 0, 0},
    //
    {"stat-display-interval",           required_argument, 0, 0},
    {"stat-publish-interval",           required_argument, 0, 0},
    {"stat-network-interval",           required_argument, 0, 0},
    {"stat-publish-mqtt-topic-prefix",  required_argument, 0, 0},
    //
    {"blackbox-enabled",                required_argument, 0, 0},
    {"blackbox-ram-max-records",        required_argument, 0, 0},
    {"blackbox-ram-max-seconds",        required_argument, 0, 0},
    {"blackbox-file-directory",         required_argument, 0, 0},
    {"blackbox-file-max-bytes",         required_argument, 0, 0},
    {"blackbox-file-generations",       required_argument, 0, 0},
    //
    {"debug",                           required_argument, 0, 0},
    {"debug-data",                      required_argument, 0, 0},
    {0, 0, 0, 0}
};

const config_option_help_t config_options_help [] = {
    {"help",                            "Display this help message and exit"},
    {"config",                          "Config file path (default: '" CONFIG_FILE_DEFAULT "')"},
    //
    {"lora-port",                       "Lora E22 serial port device (default: '" SERIAL_PORT_DEFAULT "')"},
    {"lora-rate",                       "Lora E22 serial baud rate (default: 9600)"},
    {"lora-bits",                       "Lora E22 serial data bits (default: 8N1)"},
    {"lora-address",                    "Lora E22 module address (hex) (default: 0x0000)"},
    {"lora-network",                    "Lora E22 module network ID (hex) (default: 0x00)"},
    {"lora-channel",                    "Lora E22 module channel (default: 0)"},
    {"lora-crypt",                      "Lora E22 encryption key (default: 0x0000)"},
    {"lora-packet-size",                "Lora E22 packet size (default: 0)"},
    {"lora-packet-rate",                "Lora E22 packet rate (default: 0)"},
    {"lora-transmit-power",             "Lora E22 transmit power level 0-3 (default: 0)"},
    {"lora-transmission-method",        "Lora E22 transmission method 0=transparent, 1=fixed-point (default: 0)"},
    {"lora-rssi-channel",               "Lora E22 channel (ambient) RSSI poll interval in seconds (default: 0 = disabled)"},
    {"lora-rssi-packet",                "Lora E22 enable packet RSSI capture (default: true)"},
    {"lora-listen-before-transmit",     "Lora E22 enable listen-before-transmit mode (default: true)"},
    {"lora-read-timeout-command",       "Lora E22 command read timeout in milliseconds (default: 1000)"},
    {"lora-read-timeout-packet",        "Lora E22 packet read timeout in milliseconds (default: 5000)"},
    {"lora-debug",                      "Lora E22 debug output (true/false)"},
    //
    {"mqtt-client",                     "MQTT client ID (default: '" MQTT_CLIENT_DEFAULT "')"},
    {"mqtt-server",                     "MQTT server URL (default: '" MQTT_SERVER_DEFAULT "')"},
    {"mqtt-topic-prefix",               "MQTT topic prefix (default: '" MQTT_TOPIC_PREFIX_DEFAULT "')"},
    {"mqtt-tls-insecure",               "MQTT disable TLS verification (default: false)"},
    {"mqtt-reconnect-delay",            "MQTT reconnect delay in seconds (default: 5)"},
    {"mqtt-reconnect-delay-max",        "MQTT max reconnect delay in seconds (default: 60)"},
    {"mqtt-debug",                      "MQTT debug output (true/false)"},
    //
    {"mesh-enable",                     "Mesh enable (iotdata networking) (default: false)"},
    {"mesh-station-id",                 "Mesh gateway (station) ID (default: 1)"},
    {"mesh-beacon-interval",            "Mesh beacon transmission interval in seconds (default: 60)"},
    {"mesh-debug",                      "Mesh debug output (true/false)"},
    //
    {"ddup-enable",                     "Ddup enable (cross-gateway deduplication) (default: false)"},
    {"ddup-port",                       "Ddup UDP port for peer communication (default: 9876)"},
    {"ddup-peers",                      "Ddup peers comma-separated list (host:port)"},
    {"ddup-delay",                      "Ddup batch send delay in milliseconds (default: 20)"},
    {"ddup-debug",                      "Ddup debug output (true/false)"},
    //
    {"stat-display-interval",           "Stat display stdout interval in seconds (default: 300; 0 disables)"},
    {"stat-publish-interval",           "Stat publish MQTT interval in seconds (default: 300; 0 disables)"},
    {"stat-network-interval",           "Network/stations table stdout interval in seconds (default: 300; 0 disables)"},
    {"stat-publish-mqtt-topic-prefix",  "Stat publish MQTT topic prefix (default: 'iotdata/stats')"},
    //
    {"blackbox-enabled",                "Enable the blackbox diagnostic recorder (true/false, default: false)"},
    {"blackbox-ram-max-records",        "RAM pool maximum records (0 = unbounded, default: 0)"},
    {"blackbox-ram-max-seconds",        "RAM pool maximum seconds before flush to file (0 = write-through, default: 0)"},
    {"blackbox-file-directory",         "CSV file directory (default: '.')"},
    {"blackbox-file-max-bytes",         "CSV file maximum bytes, rotates past this (0 = unbounded, default: 0)"},
    {"blackbox-file-generations",       "CSV file maximum rotated generations to keep (<file>.1 .. .N; 0 = overwrite/no backup, default: 10)"},
    {"debug",                           "Debug output (true/false)"},
    {"debug-data",                      "Debug data (hex dump of received packet data) output (true/false)"},
};
// clang-format on

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

/*
 * The `lora-packet-{size,rate}` and `lora-transmit-power` config keys are REGISTER INDICES, which
 * is the right unit for them: both ends of a link must land on the same index, not the same
 * nominal rate. The driver takes real units, so they are converted here.
 *
 * Air rate indices 2..7 mean the same on the U and D modules; 0 and 1 do NOT (they are 0.3/1.2kbps
 * on D and both 2.4kbps on U). Index 2 is 2.4kbps on both and is the portable choice, so 0 and 1
 * are folded to it here rather than silently meaning something different from the sensors.
 */
static uint16_t _lora_rate_bps_from_index(const int idx) {
    switch (idx) {
    case 3:
        return 4800;
    case 4:
        return 9600;
    case 5:
        return 19200;
    case 6:
        return 38400;
    case 7:
        return 62500;
    default:
        return 2400; /* 0, 1, 2 -- see above */
    }
}
static uint8_t _lora_size_bytes_from_index(const int idx) {
    switch (idx) {
    case 1:
        return 128;
    case 2:
        return 64;
    case 3:
        return 32;
    default:
        return 240;
    }
}
static uint8_t _lora_power_dbm_from_index(const int idx) {
    switch (idx) {
    case 1:
        return 17;
    case 2:
        return 13;
    case 3:
        return 10;
    default:
        return 22;
    }
}

void lora_config_populate(const char **cfg_port, lora_config_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));

    *cfg_port = config_get_string("lora-port", SERIAL_PORT_DEFAULT);

    cfg->module = LORA_MODULE_USB; /* this gateway is the USB dongle: no GPIO, software mode switch */
    cfg->e22_address = (uint16_t)config_get_integer("lora-address", IOTDATA_CONFIG_LORA_ADDRESS);
    cfg->e22_network = (uint8_t)config_get_integer("lora-network", IOTDATA_CONFIG_LORA_NETWORK);
    cfg->channel = (uint8_t)config_get_integer("lora-channel", IOTDATA_CONFIG_LORA_CHANNEL);
    cfg->crypt = (uint16_t)config_get_integer("lora-crypt", 0x0000);
    cfg->packet_size = _lora_size_bytes_from_index(config_get_integer("lora-packet-size", 0));
    cfg->air_data_rate = _lora_rate_bps_from_index(config_get_integer("lora-packet-rate", 2));
    cfg->transmit_power = _lora_power_dbm_from_index(config_get_integer("lora-transmit-power", 0));
    cfg->listen_before_transmit = config_get_bool("lora-listen-before-transmit", true);
    /* Same key, same default as the read interval below: off also stops the module computing it. */
    cfg->rssi_channel = config_get_integer("lora-rssi-channel", INTERVAL_RSSI_CHANNEL_DEFAULT) > 0;
    /* This host does not sleep, so the driver should not park the module for it: no sleep command,
       no UART teardown between setup and start, and nothing issued in transparent mode at exit.
       Not a config key -- it is a property of what a gateway is. See lora_config_t.host_sleeps. */
    cfg->host_sleeps = false;
    cfg->rssi_packet = config_get_bool("lora-rssi-packet", true);
    _log_enabled = config_get_bool("lora-debug", false); /* app-side debug logging; the driver has no such flag */

    PRINTF_INFO("config: lora: port=%s, module=USB, "
                "address=0x%04" PRIX16 ", network=0x%02" PRIX8 ", channel=%d, crypt=0x%04" PRIX16 ", "
                "packet-size=%" PRIu8 "B, air-rate=%" PRIu16 "bps, transmit-power=%" PRIu8 "dBm, "
                "listen-before-transmit=%s, rssi-channel=%s, rssi-packet=%s\n",
                *cfg_port, cfg->e22_address, cfg->e22_network, cfg->channel, cfg->crypt, cfg->packet_size, cfg->air_data_rate, cfg->transmit_power, cfg->listen_before_transmit ? "on" : "off", cfg->rssi_channel ? "on" : "off",
                cfg->rssi_packet ? "on" : "off");
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void mqtt_config_populate(mqtt_config_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));

    cfg->client = config_get_string("mqtt-client", MQTT_CLIENT_DEFAULT);
    cfg->server = config_get_string("mqtt-server", MQTT_SERVER_DEFAULT);
    cfg->tls_insecure = config_get_bool("mqtt-tls-insecure", MQTT_TLS_DEFAULT);
    cfg->synchronous = MQTT_SYNCHRONOUS_DEFAULT;
    cfg->reconnect_delay = (unsigned int)config_get_integer("mqtt-reconnect-delay", MQTT_RECONNECT_DELAY_DEFAULT);
    cfg->reconnect_delay_max = (unsigned int)config_get_integer("mqtt-reconnect-delay-max", MQTT_RECONNECT_DELAY_MAX_DEFAULT);
    cfg->debug = config_get_bool("mqtt-debug", false);

    PRINTF_INFO("config: mqtt: client=%s, server=%s, tls=%s, sync=%s, reconnect-delay=%ds, reconnect-delay-max=%ds\n", cfg->client, cfg->server, !cfg->tls_insecure ? "on" : "off", cfg->synchronous ? "on" : "off", cfg->reconnect_delay,
                cfg->reconnect_delay_max);
}

void mesh_config_populate(mesh_state_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));

    cfg->enabled = config_get_bool("mesh-enable", false);
    cfg->station_id = (uint16_t)config_get_integer("mesh-station-id", GATEWAY_STATION_ID_DEFAULT);
    if (!iotdata_station_is_assignable(cfg->station_id)) {
        PRINTF_ERROR("config: mesh-station-id %u is reserved (must be 1..%u) -- using %u\n", (unsigned)cfg->station_id, (unsigned)IOTDATA_STATION_ASSIGNABLE_MAX, (unsigned)GATEWAY_STATION_ID_DEFAULT);
        cfg->station_id = GATEWAY_STATION_ID_DEFAULT;
    }
    cfg->beacon_interval = (time_t)config_get_integer("mesh-beacon-interval", INTERVAL_BEACON_DEFAULT);
    cfg->debug = config_get_bool("mesh-debug", false);

    PRINTF_INFO("config: mesh: enabled=%c, station-id=%04" PRIX16 ", beacon-interval=%" PRIu32 "s, debug=%s\n", cfg->enabled ? 'y' : 'n', cfg->station_id, (uint32_t)cfg->beacon_interval, cfg->debug ? "on" : "off");
}

void ddup_config_populate(ddup_state_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));

    cfg->enabled = config_get_bool("ddup-enable", false);
    cfg->port = (uint16_t)config_get_integer("ddup-port", DDUP_PORT_DEFAULT);
    const char *peers = config_get_string("ddup-peers", "");
    ddup_peers_parse(cfg, peers);
    cfg->delay_ms = (uint32_t)config_get_integer("ddup-delay", DDUP_DELAY_MS_DEFAULT);
    cfg->debug = config_get_bool("ddup-debug", false);

    PRINTF_INFO("config: ddup: enabled=%c, port=%" PRIu16 ", peers=%s, delay=%" PRIu32 "ms, debug=%s\n", cfg->enabled ? 'y' : 'n', cfg->port, peers, cfg->delay_ms, cfg->debug ? "on" : "off");
}

void stat_config_populate(stat_state_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));

    const char *topic = config_get_string("stat-publish-mqtt-topic-prefix", STAT_MQTT_TOPIC_DEFAULT);
    strncpy(cfg->mqtt_topic, topic, sizeof(cfg->mqtt_topic) - 1);
    cfg->mqtt_topic[sizeof(cfg->mqtt_topic) - 1] = '\0';
    size_t len = strlen(cfg->mqtt_topic);
    if (len > 0 && cfg->mqtt_topic[len - 1] == '/')
        cfg->mqtt_topic[len - 1] = '\0';

    PRINTF_INFO("config: stat: mqtt-topic=%s\n", cfg->mqtt_topic);
}

void process_config_populate(process_state_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));

    cfg->mqtt_topic_prefix = config_get_string("mqtt-topic-prefix", MQTT_TOPIC_PREFIX_DEFAULT);
    cfg->interval_rssi_channel = config_get_integer("lora-rssi-channel", INTERVAL_RSSI_CHANNEL_DEFAULT); /* same key, same default as the module flag above */
    cfg->capture_rssi_channel = (cfg->interval_rssi_channel > 0);
    cfg->capture_rssi_packet = config_get_bool("lora-rssi-packet", true); /* same key as the module register bit */
    cfg->stat_display_interval = config_get_integer("stat-display-interval", STAT_INTERVAL_DEFAULT);
    cfg->stat_publish_interval = config_get_integer("stat-publish-interval", STAT_INTERVAL_DEFAULT);
    cfg->stat_netw_interval = config_get_integer("stat-network-interval", STAT_INTERVAL_DEFAULT);
    cfg->debug = config_get_bool("debug", false);
    cfg->debug_data = config_get_bool("debug-data", false);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

BUFFER_POOL_DECLARE(s_pool, BUFFER_SLOTS_TOTAL, BUFFER_LENGTH_TOTAL);

typedef struct {
    const char *lora_port;
    lora_config_t lora_config;
    mqtt_config_t mqtt_config;
    mesh_state_t mesh_state;
    ddup_state_t ddup_state;
    stat_state_t stat_state;
    node_state_t node_state;
    ctrl_state_t ctrl_state;
    bbox_state_t bbox_state;
    process_state_t process_state;
    volatile bool running;
} system_t;

static system_t system_state;

static iotdata_version_caps_t s_caps;

// -----------------------------------------------------------------------------------------------------------------------------------------

bool system_config(system_t *state, const int argc, char *argv[]) {

    for (int i = 1; i < argc; i++)
        if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            config_help(argv[0], config_options, config_options_help, (int)(sizeof(config_options_help) / sizeof(config_options_help[0])));
            exit(EXIT_SUCCESS);
        }

    if (!config_load(CONFIG_FILE_DEFAULT, argc, argv, config_options))
        return false;

    lora_config_populate(&state->lora_port, &state->lora_config);
    mqtt_config_populate(&state->mqtt_config);
    mesh_config_populate(&state->mesh_state);
    ddup_config_populate(&state->ddup_state);
    stat_config_populate(&state->stat_state);
    process_config_populate(&state->process_state);

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

static void gateway_blackbox_begin(bbox_state_t *state) {
    const bool enabled = config_get_bool("blackbox-enabled", false);
    const int max_records = (int)config_get_integer("blackbox-ram-max-records", 0);
    const int max_seconds = (int)config_get_integer("blackbox-ram-max-seconds", 0);
    const int max_bytes = (int)config_get_integer("blackbox-file-max-bytes", 0);
    const char *const dir = config_get_string("blackbox-file-directory", ".");
    const int gens = (int)config_get_integer("blackbox-file-generations", 10);
    snprintf(state->path, sizeof(state->path), "%s/iotdata_gateway_blackbox.csv", dir);
    state->config = (blackbox_config_t){
        .pool = state->pool,
        .pool_sz = sizeof(state->pool),
        .flush = (max_seconds > 0) ? BLACKBOX_FLUSH_BATCH_TIME : BLACKBOX_FLUSH_WRITE_THROUGH,
        .flush_ms = (max_seconds > 0) ? (uint32_t)max_seconds * 1000u : 0u,
        .max_records = (max_records > 0) ? (uint32_t)max_records : 0u,
        .max_bytes = (max_bytes > 0) ? (uint32_t)max_bytes : 0u,
        .generations = (uint8_t)((gens < 0)     ? 0
                                 : (gens > 255) ? 255
                                                : gens),
        .persist_arg = state->path,
        .enabled = enabled,
    };
    if (blackbox_init(&state->handle, &state->config) != 0) {
        PRINTF_ERROR("blackbox: init failed (path=%s)\n", state->path);
        return;
    }
    (void)iotdata_blackbox_lifecycle(&state->handle, IOTDATA_BB_LC_START, 0);
    PRINTF_INFO("blackbox: %s (path=%s, flush=%s)\n", enabled ? "enabled" : "disabled (default)", state->path, (max_seconds > 0) ? "ram-cache/batched" : "write-through");
}

static void gateway_blackbox_end(bbox_state_t *state) {
    if (state->handle.cfg != NULL) {
        (void)iotdata_blackbox_lifecycle(&state->handle, IOTDATA_BB_LC_STOP, 0);
        (void)blackbox_flush(&state->handle);
        blackbox_deinit(&state->handle);
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

int main(int argc, char *argv[]) {

    int ret = EXIT_FAILURE;

    setbuf(stdout, NULL);
    PRINTF_INFO("starting (iotdata gateway: variants=%d, features=mesh,dedup,stats,mqtt)\n", IOTDATA_VARIANT_MAPS_COUNT);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    system_t *state = &system_state;

    if (!system_config(state, argc, argv))
        return ret;
    const uint16_t station_id = state->mesh_state.station_id; // from config

    iotdata_version_caps_init(&s_caps); /* seeds the build-time features */
    (void)iotdata_version_caps_add(&s_caps, IOTDATA_VERSION_CAP_RADIO, state->lora_config.module == LORA_MODULE_USB ? IOTDATA_VERSION_RADIO_E22_USB : IOTDATA_VERSION_RADIO_E22_DIP);
    char vbuf[IOTDATA_VERSION_STR_MAX + 1];
    PRINTF_INFO("version: %s\n", iotdata_version_str(vbuf, sizeof(vbuf), &s_caps));
    if (!iotdata_version_stamp_is_real())
        PRINTF_WARN("version: build stamp is unset -- this binary cannot say when it was built\n");

    BUFFER_POOL_INIT(s_pool, BUFFER_SLOTS_TOTAL, BUFFER_LENGTH_TOTAL, BUFFER_LENGTH_PREFIX);

    gateway_blackbox_begin(&state->bbox_state);

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

    // MQTT BROKER
    if (!mqtt_begin(&state->mqtt_config))
        goto end_device;

    state->running = true;

    // IOTDATA (NETW/NODE/MESH/DDUP/CTRL)
    (void)netw_begin(&state->process_state.network);
    if (!node_begin(&state->node_state, station_id, &s_caps, &state->stat_state, &state->bbox_state, &s_pool, lora_packet_write, ctrl_from_iotdata, exec_node_status_mesh, exec_node_table_count, exec_node_table_row, ctrl_from_iotdata_keys,
                    (uint8_t)(sizeof(ctrl_from_iotdata_keys) / sizeof(ctrl_from_iotdata_keys[0])), state->process_state.mqtt_topic_prefix))
        goto end_mqtt;
    if (!mesh_begin(&state->mesh_state, &s_pool, lora_packet_write, ddup_insert_handler, (void *)&state->process_state))
        goto end_node;
    if (!ddup_begin(&state->ddup_state, station_id, &state->mesh_state.dedup_ring, &state->running))
        goto end_mesh;
    if (!ctrl_begin(&state->ctrl_state, state->process_state.mqtt_topic_prefix, station_id, &state->bbox_state, &s_pool))
        goto end_ddup;

    // PROCESS
    stat_begin(&state->stat_state, state->process_state.mqtt_topic_prefix, station_id, IOTDATA_GATEWAY_VERSION, &state->lora_config, &s_pool);
    state->process_state.pool = &s_pool;
    ret = process_run(&state->process_state, &state->node_state, &state->mesh_state, &state->ddup_state, &state->stat_state, &state->ctrl_state, &state->running) ? EXIT_SUCCESS : EXIT_FAILURE;
    stat_end(&state->stat_state);

    ctrl_end(&state->ctrl_state);
end_ddup:
    ddup_end(&state->ddup_state);
end_mesh:
    mesh_end(&state->mesh_state);
end_node:
    node_end(&state->node_state);
end_mqtt:
    mqtt_end();
end_device:
    (void)lora_stop();
end_all:
    gateway_blackbox_end(&state->bbox_state);
    return ret;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
