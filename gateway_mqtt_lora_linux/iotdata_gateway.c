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
 *
 * Uses EBYTE E22 connector for low-level, but is sufficiently modular to
 * fit onto another driver.
 * https://github.com/matthewgream/e22900t22
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

#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

time_t intervalable(const time_t interval, time_t *last) {
    time_t now = time(NULL);
    if (*last == 0) {
        *last = now;
        return 0;
    }
    if ((now - *last) > interval) {
        const time_t diff = now - *last;
        *last = now;
        return diff;
    }
    return 0;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

void debug_hexdump(const char *prefix, const uint8_t *data, size_t length) {
    for (size_t offset = 0; offset < length; offset += 16) {
        printf("%s[%04X] ", prefix ? prefix : "", (unsigned)offset);
        for (size_t i = 0; i < 16; i++) {
            if (i == 8)
                printf(" ");
            if (offset + i < length)
                printf("%02X ", data[offset + i]);
            else
                printf("   ");
        }
        printf(" ");
        for (size_t i = 0; i < 16; i++) {
            if (i == 8)
                printf(" ");
            if (offset + i < length)
                printf("%c", isprint(data[offset + i]) ? data[offset + i] : '.');
            else
                printf(" ");
        }
        printf("\n");
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include "serial_linux.h"

bool e22_debug = false;
void e22_printf_debug(const char *format, ...) {
    if (!e22_debug)
        return;
    va_list args;
    va_start(args, format);
    vfprintf(stdout, format, args);
    va_end(args);
}
void e22_printf_stdout(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vfprintf(stdout, format, args);
    va_end(args);
}
void e22_printf_stderr(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vfprintf(stderr, format, args);
    va_end(args);
}
#define PRINTF_DEBUG e22_printf_debug
#define PRINTF_ERROR e22_printf_stderr
#define PRINTF_INFO  e22_printf_stdout
#undef E22900T22_SUPPORT_MODULE_DIP
#define E22900T22_SUPPORT_MODULE_USB
#include "e22xxxtxx.h"
void __sleep_ms(const uint32_t ms) {
    usleep((useconds_t)ms * 1000);
}

// bool device_mode_config(void);
//   bool device_info_read(void);
//   bool device_config_read_and_update(void);
// bool device_mode_transfer(void);
// bool device_connect(const e22900t22_module_t config_module, const e22900t22_config_t *config_device);
//   bool device_channel_rssi_read(uint8_t *rssi);
//   bool device_packet_read(uint8_t *packet, const int max_size, int *packet_size, uint8_t *rssi);
//   bool device_packet_write(const uint8_t *packet, const int length);
// void device_disconnect(void);

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define MQTT_CONNECT_TIMEOUT 60
#define MQTT_PUBLISH_QOS     0
#define MQTT_PUBLISH_RETAIN  false
#include "mqtt_linux.h"

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include "iotdata_variant_suite.h"
#include "iotdata.c"
#include "iotdata_mesh.h"

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include "iotdata_gateway_mesh.h"
#include "iotdata_gateway_ddup.h"
#include "iotdata_gateway_stat.h"
#include "iotdata_gateway_exec.h"

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define IOTDATA_GATEWAY_VERSION          "1.0.0"

#define CONFIG_FILE_DEFAULT              "iotdata_gateway.cfg"

#define SERIAL_PORT_DEFAULT              "/dev/e22900t22u"
#define SERIAL_RATE_DEFAULT              9600
#define SERIAL_BITS_DEFAULT              SERIAL_8N1

#define MQTT_CLIENT_DEFAULT              "iotdata_gateway"
#define MQTT_SERVER_DEFAULT              "mqtt://localhost"
#define MQTT_TLS_DEFAULT                 false
#define MQTT_SYNCHRONOUS_DEFAULT         false
#define MQTT_TOPIC_PREFIX_DEFAULT        "iotdata"
#define MQTT_RECONNECT_DELAY_DEFAULT     5
#define MQTT_RECONNECT_DELAY_MAX_DEFAULT 60

#define STAT_INTERVAL_DEFAULT            (5 * 60)
#define INTERVAL_RSSI_DEFAULT            (1 * 60)
#define INTERVAL_BEACON_DEFAULT          60 /* seconds */

#define GATEWAY_STATION_ID_DEFAULT       1

#include "config_linux.h"

// clang-format off
const struct option config_options [] = {
    {"help",                            no_argument,       0, 'h'},
    {"config",                          required_argument, 0, 'c'},
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
    {"stat-publish-mqtt-topic-prefix",  required_argument, 0, 0},
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
    {"lora-rssi-channel",               "Lora E22 enable channel RSSI capture in seconds (default: 60; 0 disables)"},
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
    {"stat-publish-mqtt-topic-prefix",  "Stat publish MQTT topic prefix (default: 'iotdata/stats')"},
    //
    {"debug",                           "Debug output (true/false)"},
    {"debug-data",                      "Debug data (hex dump of received packet data) output (true/false)"},
};
// clang-format on

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

void lora_config_populate(serial_config_t *cfg_serial, e22900t22_config_t *cfg) {
    memset(cfg_serial, 0, sizeof(*cfg_serial));
    memset(cfg, 0, sizeof(*cfg));

    cfg_serial->port = config_get_string("lora-port", SERIAL_PORT_DEFAULT);
    cfg_serial->rate = config_get_integer("lora-rate", SERIAL_RATE_DEFAULT);
    cfg_serial->bits = config_get_bits("lora-bits", SERIAL_BITS_DEFAULT);

    cfg->address = (uint16_t)config_get_integer("lora-address", E22900T22_CONFIG_ADDRESS_DEFAULT);
    cfg->network = (uint8_t)config_get_integer("lora-network", E22900T22_CONFIG_NETWORK_DEFAULT);
    cfg->channel = (uint8_t)config_get_integer("lora-channel", E22900T22_CONFIG_CHANNEL_DEFAULT);
    cfg->crypt = (uint16_t)config_get_integer("lora-crypt", E22900T22_CONFIG_CRYPT_DEFAULT);
    cfg->packet_size = (uint8_t)config_get_integer("lora-packet-size", E22900T22_CONFIG_PACKET_SIZE_DEFAULT);          // index
    cfg->packet_rate = (uint8_t)config_get_integer("lora-packet-rate", E22900T22_CONFIG_PACKET_RATE_DEFAULT);          // index
    cfg->transmit_power = (uint8_t)config_get_integer("lora-transmit-power", E22900T22_CONFIG_TRANSMIT_POWER_DEFAULT); // index
    cfg->transmission_method = (uint8_t)config_get_integer("lora-transmission-method", E22900T22_CONFIG_TRANSMISSION_METHOD_DEFAULT);
    cfg->relay_enabled = E22900T22_CONFIG_RELAY_ENABLED_DEFAULT;
    cfg->listen_before_transmit = config_get_bool("lora-listen-before-transmit", E22900T22_CONFIG_LISTEN_BEFORE_TRANSMIT);
    cfg->rssi_channel = config_get_integer("lora-rssi-channel", 1) > 0;                           // XXX
    cfg->rssi_packet = config_get_bool("lora-rssi-packet", E22900T22_CONFIG_RSSI_PACKET_DEFAULT); // XXX
    cfg->read_timeout_command = (uint32_t)config_get_integer("lora-read-timeout-command", E22900T22_CONFIG_READ_TIMEOUT_COMMAND_DEFAULT);
    cfg->read_timeout_packet = (uint32_t)config_get_integer("lora-read-timeout-packet", E22900T22_CONFIG_READ_TIMEOUT_PACKET_DEFAULT);
    cfg->debug = config_get_bool("lora-debug", false);
    e22_debug = cfg->debug;

    printf("config: lora: port=%s, rate=%d, bits=%s, "
           "address=0x%04" PRIX16 ", network=0x%02" PRIX8 ", channel=%d, crypt=0x%04" PRIX16 ", packet-size=%d, packet-rate=%d, "
           "transmit-power=%" PRIu8 ", transmission-method=%s, mode-relay=%s, mode-listen-before-transmit=%s, "
           "rssi-channel=%s, rssi-packet=%s, read-timeout-command=%" PRIu32 "ms, read-timeout-packet=%" PRIu32 "ms, debug=%s\n",
           cfg_serial->port, cfg_serial->rate, serial_bits_str(cfg_serial->bits), cfg->address, cfg->network, cfg->channel, cfg->crypt, cfg->packet_size, cfg->packet_rate, cfg->transmit_power,
           cfg->transmission_method == E22900T22_CONFIG_TRANSMISSION_METHOD_TRANSPARENT ? "transparent" : "fixed-point", cfg->relay_enabled ? "on" : "off", cfg->listen_before_transmit ? "on" : "off", cfg->rssi_channel ? "on" : "off",
           cfg->rssi_packet ? "on" : "off", cfg->read_timeout_command, cfg->read_timeout_packet, cfg->debug ? "on" : "off");
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void mqtt_config_populate(mqtt_config_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));

    cfg->client = config_get_string("mqtt-client", MQTT_CLIENT_DEFAULT);
    cfg->server = config_get_string("mqtt-server", MQTT_SERVER_DEFAULT);
    cfg->tls_insecure = config_get_bool("mqtt-tls-insecure", MQTT_TLS_DEFAULT);
    cfg->use_synchronous = MQTT_SYNCHRONOUS_DEFAULT;
    cfg->reconnect_delay = (unsigned int)config_get_integer("mqtt-reconnect-delay", MQTT_RECONNECT_DELAY_DEFAULT);
    cfg->reconnect_delay_max = (unsigned int)config_get_integer("mqtt-reconnect-delay-max", MQTT_RECONNECT_DELAY_MAX_DEFAULT);
    cfg->debug = config_get_bool("mqtt-debug", false);

    printf("config: mqtt: client=%s, server=%s, tls-insecure=%s, synchronous=%s, reconnect-delay=%ds, reconnect-delay-max=%ds\n", cfg->client, cfg->server, cfg->tls_insecure ? "on" : "off", cfg->use_synchronous ? "on" : "off",
           cfg->reconnect_delay, cfg->reconnect_delay_max);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void iotdata_mesh_config_populate(mesh_state_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));

    cfg->enabled = config_get_bool("mesh-enable", false);
    cfg->station_id = (uint16_t)config_get_integer("mesh-station-id", GATEWAY_STATION_ID_DEFAULT);
    cfg->beacon_interval = (time_t)config_get_integer("mesh-beacon-interval", INTERVAL_BEACON_DEFAULT);
    cfg->debug = config_get_bool("mesh-debug", false);

    printf("config: mesh: enabled=%c, station-id=0x%04" PRIX16 ", beacon-interval=%" PRIu32 "s, debug=%s\n", cfg->enabled ? 'y' : 'n', cfg->station_id, (uint32_t)cfg->beacon_interval, cfg->debug ? "on" : "off");
}

void iotdata_ddup_config_populate(ddup_state_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));

    cfg->enabled = config_get_bool("ddup-enable", false);
    cfg->port = (uint16_t)config_get_integer("ddup-port", DDUP_PORT_DEFAULT);
    const char *peers = config_get_string("ddup-peers", "");
    ddup_peers_parse(cfg, peers);
    cfg->delay_ms = (uint32_t)config_get_integer("ddup-delay", DDUP_DELAY_MS_DEFAULT);
    cfg->debug = config_get_bool("ddup-debug", false);

    printf("config: ddup: enabled=%c, port=%" PRIu16 ", peers=%s, delay=%" PRIu32 "ms, debug=%s\n", cfg->enabled ? 'y' : 'n', cfg->port, peers, cfg->delay_ms, cfg->debug ? "on" : "off");
}

void stat_config_populate(stat_state_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));

    const char *topic = config_get_string("stat-publish-mqtt-topic-prefix", STAT_MQTT_TOPIC_DEFAULT);
    strncpy(cfg->mqtt_topic, topic, sizeof(cfg->mqtt_topic) - 1);
    cfg->mqtt_topic[sizeof(cfg->mqtt_topic) - 1] = '\0';
    size_t len = strlen(cfg->mqtt_topic);
    if (len > 0 && cfg->mqtt_topic[len - 1] == '/')
        cfg->mqtt_topic[len - 1] = '\0';

    printf("config: stat: mqtt-topic=%s\n", cfg->mqtt_topic);
}

void process_config_populate(process_state_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));

    cfg->mqtt_topic_prefix = config_get_string("mqtt-topic-prefix", MQTT_TOPIC_PREFIX_DEFAULT);
    cfg->interval_rssi_channel = config_get_integer("lora-rssi-channel", INTERVAL_RSSI_DEFAULT);          // XXX
    cfg->capture_rssi_channel = (cfg->interval_rssi_channel > 0);                                         // XXX
    cfg->capture_rssi_packet = config_get_bool("lora-rssi-packet", E22900T22_CONFIG_RSSI_PACKET_DEFAULT); // XXX
    cfg->stat_display_interval = config_get_integer("stat-display-interval", STAT_INTERVAL_DEFAULT);
    cfg->stat_publish_interval = config_get_integer("stat-publish-interval", STAT_INTERVAL_DEFAULT);
    cfg->debug = config_get_bool("debug", false);
    cfg->debug_data = config_get_bool("debug-data", false);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

typedef struct {
    serial_config_t lora_serial_config;
    e22900t22_config_t lora_device_config;
    mqtt_config_t mqtt_config;
    mesh_state_t mesh_state;
    ddup_state_t ddup_state;
    stat_state_t stat_state;
    process_state_t process_state;
    volatile bool running;
} system_t;

static system_t system_state;

// -----------------------------------------------------------------------------------------------------------------------------------------

bool system_config(system_t *state, const int argc, char *argv[]) {

    for (int i = 1; i < argc; i++)
        if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            config_help(argv[0], config_options, config_options_help, (int)(sizeof(config_options_help) / sizeof(config_options_help[0])));
            exit(EXIT_SUCCESS);
        }

    if (!config_load(CONFIG_FILE_DEFAULT, argc, argv, config_options))
        return false;

    lora_config_populate(&state->lora_serial_config, &state->lora_device_config);
    mqtt_config_populate(&state->mqtt_config);
    iotdata_mesh_config_populate(&state->mesh_state);
    iotdata_ddup_config_populate(&state->ddup_state);
    stat_config_populate(&state->stat_state);
    process_config_populate(&state->process_state);

    state->running = false;

    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void signal_handler(const int sig __attribute__((unused))) {
    if (system_state.running) {
        printf("stopping\n");
        system_state.running = false;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

int main(int argc, char *argv[]) {

    int ret = EXIT_FAILURE;

    setbuf(stdout, NULL);
    printf("starting (iotdata gateway: variants=%d, features=mesh,dedup,stats,mqtt)\n", IOTDATA_VARIANT_MAPS_COUNT);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    system_t *state = &system_state;

    if (!system_config(state, argc, argv))
        goto end_all;

    // LORA SERIAL/DEVICE
    if (!serial_begin(&state->lora_serial_config) || !serial_connect()) {
        fprintf(stderr, "device: serial connect failure (port=%s, rate=%d, bits=%s)\n", state->lora_serial_config.port, state->lora_serial_config.rate, serial_bits_str(state->lora_serial_config.bits));
        goto end_all;
    }
    if (!device_connect(E22900T22_MODULE_USB, &state->lora_device_config)) {
        fprintf(stderr, "device: module connect failure (port=%s, rate=%d, bits=%s)\n", state->lora_serial_config.port, state->lora_serial_config.rate, serial_bits_str(state->lora_serial_config.bits));
        goto end_serial;
    }
    printf("device: connect success (port=%s, rate=%d, bits=%s)\n", state->lora_serial_config.port, state->lora_serial_config.rate, serial_bits_str(state->lora_serial_config.bits));
    if (!(device_mode_config() && device_info_read() && device_config_read_and_update() && device_mode_transfer()))
        goto end_device;

    // MQTT BROKER
    if (!mqtt_begin(&state->mqtt_config))
        goto end_device;

    state->running = true;

    // IOTDATA MESH/DDUP
    if (!mesh_begin(&state->mesh_state, device_packet_write, ddup_check_and_add_handler, (void *)state))
        goto end_mqtt;
    if (!ddup_begin(&state->ddup_state, state->mesh_state.station_id, &state->mesh_state.dedup_ring, &state->running))
        goto end_mesh;

    // GATEWAY PROCESSOR
    stat_begin(&state->stat_state, IOTDATA_GATEWAY_VERSION, state->mesh_state.station_id, &state->lora_device_config);
    ret = process_run(&state->process_state, &state->mesh_state, &state->ddup_state, &state->stat_state, &state->running) ? EXIT_SUCCESS : EXIT_FAILURE;
    stat_end(&state->stat_state);

    ddup_end(&state->ddup_state);
end_mesh:
    mesh_end(&state->mesh_state);
end_mqtt:
    mqtt_end();
end_device:
    device_disconnect();
end_serial:
    serial_end();
end_all:
    return ret;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
