#ifndef IOTDATA_GATEWAY_CONF_H
#define IOTDATA_GATEWAY_CONF_H

// -----------------------------------------------------------------------------------------------------------------------------------------
// iotdata_gateway_config.h - the gateway's configuration, as an iotdata_node_config table.
//
// WHAT CHANGED IS THE STORE, NOT THE CALLERS. config_get_string/integer/bool still read the way
// they always did; they just read out of the node table now instead of a parallel key/value list.
// That is deliberate: forty-odd call sites is a lot of surface to disturb in one step, and the
// point of this one is to prove the table, the file loading and the write-back -- not to rewrite
// everything that reads a setting.
//
// THE TABLE OWNS THE DEFAULTS. Every call site still passes one, and it is now ignored: two
// statements of the same default is one too many, and the one in the table is the one the console
// reports and the one a reset returns to. The values here are transcribed from those call sites
// exactly, so this change moves where a default lives and not what it is.
//
// WHERE A VALUE COMES FROM, weakest first:
//
//   the table       what this build believes, below
//   the .cfg file   the operator's, hand-edited, and never written back to
//   the overlay     what was changed at runtime, beside the file rather than in it
//   the command line for this run only, and never persisted
//
// -----------------------------------------------------------------------------------------------------------------------------------------

#ifndef CONFIG_FILE_DEFAULT
#define CONFIG_FILE_DEFAULT "iotdata_gateway.cfg"
#endif
#ifndef CONFIG_OVERLAY_DEFAULT
#define CONFIG_OVERLAY_DEFAULT "iotdata_gateway.runtime.cfg"
#endif
#ifndef STATE_PATH_DEFAULT
#define STATE_PATH_DEFAULT "/var/lib/iotdata"
#endif
#ifndef SERIAL_PORT_DEFAULT
#define SERIAL_PORT_DEFAULT "/dev/e22900t22u"
#endif
#ifndef STAT_INTERVAL_DEFAULT
#define STAT_INTERVAL_DEFAULT (5 * 60)
#endif
#ifndef INTERVAL_RSSI_CHANNEL_DEFAULT
#define INTERVAL_RSSI_CHANNEL_DEFAULT 0
#endif
#ifndef INTERVAL_BEACON_DEFAULT
#define INTERVAL_BEACON_DEFAULT 60 /* seconds */
#endif
#ifndef GATEWAY_STATION_ID_DEFAULT
#define GATEWAY_STATION_ID_DEFAULT 1
#endif

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

/* The E22's own validators, from the shared block. The radio settings are REAL VALUES here -- bps,
   bytes, dBm -- and not the module's register indices, which is what they used to be: the driver
   took an index, the config file said "2", and only a lookup table three functions long knew that
   meant 2400bps. The sensors always stated the real thing; now the gateway does too, and the same
   validators refuse a value the module cannot actually take. */
#include "iotdata_node_config_device_e22900t22.h"

/* The string maxima are the storage, so they are sized for what each actually holds: a URL and a
   peer list want room, a client id does not. See THE EXPANSION in iotdata_node_config.h. */
#define IOTDATA_CONFIG_ENTRIES(X) \
    /*  NAME                           id     TYPE    min max                   default                            flags                      validate notify */ \
    X(CONFIG, 0x001, STRING, 1, 127, CONFIG_FILE_DEFAULT, IOTDATA_CONFIG_FLAG_LOCAL | IOTDATA_CONFIG_FLAG_STARTUP, NULL, NULL, "the settings file to read") \
    X(OVERLAY, 0x002, STRING, 1, 127, CONFIG_OVERLAY_DEFAULT, IOTDATA_CONFIG_FLAG_LOCAL | IOTDATA_CONFIG_FLAG_STARTUP, NULL, NULL, "where a setting changed at runtime is written, beside that file") \
    X(LORA_PORT, 0x101, STRING, 1, 127, SERIAL_PORT_DEFAULT, IOTDATA_CONFIG_FLAG_LOCAL, NULL, NULL, "the serial device the E22 radio is on") \
    X(LORA_ADDRESS, 0x102, U16, 0, 0xFFFF, IOTDATA_CONFIG_LORA_ADDRESS, 0, NULL, NULL, "Lora E22 module address (hex)") \
    X(LORA_NETWORK, 0x103, U8, 0, 0xFF, IOTDATA_CONFIG_LORA_NETWORK, 0, NULL, NULL, "Lora E22 module network ID (hex)") \
    X(LORA_CHANNEL, 0x104, U8, 0, 83, IOTDATA_CONFIG_LORA_CHANNEL, 0, NULL, NULL, "Lora E22 module channel") \
    X(LORA_CRYPT, 0x105, U16, 0, 0xFFFF, 0x0000, 0, NULL, NULL, "Lora E22 encryption key") \
    X(LORA_PACKET_SIZE, 0x106, U8, 32, 240, IOTDATA_CONFIG_LORA_PACKET_SIZE, 0, iotdata_config_lora_packet_size_ok, NULL, "the E22 sub-packet size in bytes: 32, 64, 128 or 240") \
    X(LORA_AIR_RATE, 0x107, U16, 300, 62500, IOTDATA_CONFIG_LORA_AIR_RATE, 0, iotdata_config_lora_air_rate_ok, NULL, "on-air bit rate in bps; lower reaches further and takes longer") \
    X(LORA_TX_POWER, 0x108, U8, 10, 22, IOTDATA_CONFIG_LORA_TX_POWER, 0, iotdata_config_lora_tx_power_ok, NULL, "transmit power in dBm: 10, 13, 17 or 22") \
    X(LORA_RSSI_CHANNEL, 0x109, U16, 0, 3600, INTERVAL_RSSI_CHANNEL_DEFAULT, 0, NULL, NULL, "Lora E22 channel (ambient) RSSI poll interval in seconds") \
    X(LORA_RSSI_PACKET, 0x10A, BOOL, 0, 1, true, 0, NULL, NULL, "Lora E22 enable packet RSSI capture") \
    X(LORA_LBT, 0x10B, BOOL, 0, 1, IOTDATA_CONFIG_LORA_LBT, 0, NULL, NULL, "listen before transmitting, so two radios do not talk over each other") \
    X(LORA_DEBUG, 0x10C, BOOL, 0, 1, false, IOTDATA_CONFIG_FLAG_LOCAL, NULL, NULL, "Lora E22 debug output (true/false)") \
\
    X(MQTT_CLIENT, 0x110, STRING, 1, 63, MQTT_CLIENT_DEFAULT, IOTDATA_CONFIG_FLAG_LOCAL, NULL, NULL, "the client id to connect to the broker as") \
    X(MQTT_SERVER, 0x111, STRING, 1, 191, MQTT_SERVER_DEFAULT, IOTDATA_CONFIG_FLAG_LOCAL, NULL, NULL, "the broker URL to publish telemetry to") \
    X(MQTT_TOPIC_PREFIX, 0x112, STRING, 1, 63, MQTT_TOPIC_PREFIX_DEFAULT, IOTDATA_CONFIG_FLAG_LOCAL, NULL, NULL, "what every published topic starts with") \
    X(MQTT_TLS_INSECURE, 0x113, BOOL, 0, 1, MQTT_TLS_DEFAULT, IOTDATA_CONFIG_FLAG_LOCAL, NULL, NULL, "MQTT disable TLS verification") \
    X(MQTT_RECONNECT_DELAY, 0x114, U16, 1, 3600, MQTT_RECONNECT_DELAY_DEFAULT, 0, NULL, NULL, "MQTT reconnect delay in seconds") \
    X(MQTT_RECONNECT_DELAY_MAX, 0x115, U16, 1, 3600, MQTT_RECONNECT_DELAY_MAX_DEFAULT, 0, NULL, NULL, "MQTT max reconnect delay in seconds") \
    X(MQTT_DEBUG, 0x116, BOOL, 0, 1, false, IOTDATA_CONFIG_FLAG_LOCAL, NULL, NULL, "MQTT debug output (true/false)") \
\
    X(MESH_ENABLE, 0x120, BOOL, 0, 1, false, IOTDATA_CONFIG_FLAG_REBOOT, NULL, NULL, "Mesh enable (iotdata networking)") \
    /* NOT a mesh row, despite the 0x12x realm it still sits in: it is what this node is CALLED, and \
       it applies whether or not the mesh is enabled. It is the default under the SETTINGS station -- \
       the host's equivalent of a device's MAC-derived id -- so a commissioned station overrides it, \
       and this is what a node that was never commissioned comes up as. (The id belongs in a node \
       realm rather than the mesh one; moving it is a registry decision, not this change's.) */ \
    X(STATION_ID, 0x121, U16, 1, IOTDATA_STATION_ASSIGNABLE_MAX, GATEWAY_STATION_ID_DEFAULT, IOTDATA_CONFIG_FLAG_REBOOT, NULL, NULL, "Station ID this node comes up as, before commissioning") \
    X(MESH_BEACON_INTERVAL, 0x122, U16, 5, 3600, INTERVAL_BEACON_DEFAULT, 0, NULL, NULL, "Mesh beacon transmission interval in seconds") \
    X(MESH_DEBUG, 0x123, BOOL, 0, 1, false, IOTDATA_CONFIG_FLAG_LOCAL, NULL, NULL, "Mesh debug output (true/false)") \
\
    X(DDUP_ENABLE, 0x130, BOOL, 0, 1, false, IOTDATA_CONFIG_FLAG_REBOOT, NULL, NULL, "Ddup enable (cross-gateway deduplication)") \
    X(DDUP_PORT, 0x131, U16, 1, 65535, DDUP_PORT_DEFAULT, IOTDATA_CONFIG_FLAG_REBOOT, NULL, NULL, "Ddup UDP port for peer communication") \
    X(DDUP_PEERS, 0x132, STRING, 0, 255, "", IOTDATA_CONFIG_FLAG_LOCAL, NULL, NULL, "Ddup peers comma-separated list (host:port)") \
    X(DDUP_DELAY, 0x133, U16, 0, 10000, DDUP_DELAY_MS_DEFAULT, 0, NULL, NULL, "Ddup batch send delay in milliseconds") \
    X(DDUP_DEBUG, 0x134, BOOL, 0, 1, false, IOTDATA_CONFIG_FLAG_LOCAL, NULL, NULL, "Ddup debug output (true/false)") \
\
    X(STAT_DISPLAY_INTERVAL, 0x140, U16, 0, 3600, STAT_INTERVAL_DEFAULT, 0, NULL, NULL, "Stat display stdout interval in seconds") \
    X(STAT_PUBLISH_INTERVAL, 0x141, U16, 0, 3600, STAT_INTERVAL_DEFAULT, 0, NULL, NULL, "Stat publish MQTT interval in seconds") \
    X(STAT_NETWORK_INTERVAL, 0x142, U16, 0, 3600, STAT_INTERVAL_DEFAULT, 0, NULL, NULL, "Network/stations table stdout interval in seconds") \
    X(STAT_PUBLISH_MQTT_TOPIC_PREFIX, 0x143, STRING, 1, 63, STAT_MQTT_TOPIC_DEFAULT, IOTDATA_CONFIG_FLAG_LOCAL, NULL, NULL, "Stat publish MQTT topic prefix") \
\
    X(BLACKBOX_ENABLED, 0x150, BOOL, 0, 1, false, 0, NULL, NULL, "Enable the blackbox diagnostic recorder (true/false, default: false)") \
    X(BLACKBOX_RAM_MAX_RECORDS, 0x151, U32, 0, 1000000, 0, 0, NULL, NULL, "RAM pool maximum records (0 = unbounded, default: 0)") \
    X(BLACKBOX_RAM_MAX_SECONDS, 0x152, U32, 0, 31536000, 0, 0, NULL, NULL, "RAM pool maximum seconds before flush to file (0 = write-through, default: 0)") \
    X(BLACKBOX_FILE_DIRECTORY, 0x153, STRING, 1, 127, ".", IOTDATA_CONFIG_FLAG_LOCAL, NULL, NULL, "CSV file directory") \
    X(BLACKBOX_FILE_MAX_BYTES, 0x154, U32, 0, 0xFFFFFFFF, 0, 0, NULL, NULL, "CSV file maximum bytes, rotates past this (0 = unbounded, default: 0)") \
    X(BLACKBOX_FILE_GENERATIONS, 0x155, U8, 0, 255, 10, 0, NULL, NULL, "CSV file maximum rotated generations to keep (<file>.1 .. .N; 0 = overwrite/no backup, default: 10)") \
\
    X(STATE, 0x160, STRING, 1, 127, STATE_PATH_DEFAULT, IOTDATA_CONFIG_FLAG_LOCAL, NULL, NULL, "the directory this gateway keeps its state in") \
    X(DEBUG, 0x161, BOOL, 0, 1, false, IOTDATA_CONFIG_FLAG_LOCAL, NULL, NULL, "Debug output (true/false)") \
    X(DEBUG_DATA, 0x162, BOOL, 0, 1, false, IOTDATA_CONFIG_FLAG_LOCAL, NULL, NULL, "Debug data (hex dump of received packet data) output (true/false)")

#endif /* IOTDATA_GATEWAY_CONF_H */

/* --- the second include, once the table has been expanded ------------------------------------ */

#if defined(IOTDATA_NODE_CONFIG_EXPANDED) && !defined(IOTDATA_GATEWAY_CONFIG_APPLIED)
#define IOTDATA_GATEWAY_CONFIG_APPLIED

/* Which rows a later write must persist: the ones changed at RUNTIME, and the ones a previous run
   already put in the overlay. Not the ones that came out of the operator's file -- those are theirs
   and stay theirs -- and not the command line, which is for this run. */
static bool gateway_config_owned[IOTDATA_CFG_COUNT];
static const char *gateway_config_overlay_path = NULL;

/*
 * THE SAME FILE CARRIES THE SETTINGS, which is the whole point of the fusion: on a host there is
 * one place a person looks, and the protocol's own values are not hidden in a state blob beside it.
 * A line is routed by its NAME -- "node-" means settings, anything else is a config row -- and the
 * reader, the writer, the precedence and the ownership rule are shared.
 *
 * `gateway_settings` is the live struct the whole gateway reads; the loader fills it and the saver
 * writes it back. Ownership is tracked exactly as it is for config rows and for the same reason:
 * "differs from the default" would sweep up every value the operator put in their own file and then
 * shadow the file they were editing.
 */
static iotdata_settings_t gateway_settings;
static bool gateway_settings_owned[IOTDATA_SETTINGS_KEY_COUNT];
/* AS LOADED, which is what "ours" is measured against. Not the build's default: every value the
   operator put in their own file differs from that too, so measuring there would copy their file
   into the overlay and then -- the overlay being read second -- shadow the file they were editing.
   That is the exact mistake the config side already made once. */
static iotdata_settings_t gateway_settings_loaded;

/* THE FILE IS READ TWICE, and this says which pass is running.
 *
 * Not for tidiness: the settings' DEFAULTS depend on the configuration -- a host's station default
 * is whatever `station-id` says -- so the settings cannot be seeded until the config is complete,
 * and a "node-" line cannot be applied before the seeding that would overwrite it. Two passes over
 * a small text file is the honest way to say that, and cheaper than a pending list that would have
 * to reproduce the precedence rules a second time. */
typedef enum { GATEWAY_PASS_CONFIG, GATEWAY_PASS_SETTINGS } gateway_pass_t;
static gateway_pass_t gateway_config_pass = GATEWAY_PASS_CONFIG;

// -----------------------------------------------------------------------------------------------------------------------------------------
// THE ADAPTERS
//
// The same three functions the gateway has always called, reading out of the table. The `default`
// argument is accepted and IGNORED: the table states it, which is the whole point of moving it
// there -- two statements of one default is how they come to disagree. A key that is not in the
// table is a typo in the code rather than a missing setting, so it says so loudly and then falls
// back, which is what the old ones did.
// -----------------------------------------------------------------------------------------------------------------------------------------

static const iotdata_config_row_t *gateway_config_lookup(const char *const key) {
    const int i = iotdata_config_index_by_name(key);
    if (i < 0) {
        PRINTF_ERROR("config: no such entry '%s' -- the caller and the table disagree\n", key);
        return NULL;
    }
    return &iotdata_config_table[i];
}

const char *config_get_string(const char *const key, const char *const default_value) {
    const iotdata_config_row_t *const row = gateway_config_lookup(key);
    if (row == NULL)
        return default_value;
    if (row->type != IOTDATA_CONFIG_TYPE_STRING) {
        PRINTF_ERROR("config: '%s' is a %s, not a string\n", key, "scalar");
        return default_value;
    }
    iotdata_config_value_t v;
    return (iotdata_config_get(row->id, &v) && v.s.p != NULL) ? v.s.p : default_value;
}

int config_get_integer(const char *const key, const int default_value) {
    const iotdata_config_row_t *const row = gateway_config_lookup(key);
    iotdata_config_value_t v;
    if (row == NULL || !iotdata_config_get(row->id, &v))
        return default_value;
    return iotdata_config_type_is_signed(row->type) ? (int)v.i : (int)v.u;
}

bool config_get_bool(const char *const key, const bool default_value) {
    const iotdata_config_row_t *const row = gateway_config_lookup(key);
    iotdata_config_value_t v;
    if (row == NULL || !iotdata_config_get(row->id, &v))
        return default_value;
    return (row->type == IOTDATA_CONFIG_TYPE_BOOL) ? v.b : (v.u != 0u);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// LOADING
//
// Straight into the table, with no key/value list in between. There used to be one -- config_linux.h
// parsed a file and a command line into a pool and the accessors searched it -- and once the table
// became the store it was a second copy of everything, kept only so that a lookup could happen
// twice. A line is parsed against the ROW now, so a value that is not a number where a number goes
// is caught at the line that says it rather than becoming a zero nobody notices.
//
// NO OPTION TABLE EITHER. Every setting is already a command-line option by virtue of being a row,
// and its --help text is the row's own description, so the parallel arrays of names and help
// strings are gone with the list. A row added without either is not possible.
// -----------------------------------------------------------------------------------------------------------------------------------------

/* One key=value, whatever said it. Parsed against the row's type so the complaint names the line. */
static bool gateway_config_set(const char *const key, const char *const text, const bool from_overlay) {
    /* ROUTED BY NAME, and settings first because their prefix is the narrower claim. A "node-" key
       this build does not have is an error here rather than a config lookup that would fail with a
       misleading message about config. */
    if (iotdata_settings_key_is_ours(key)) {
        if (gateway_config_pass != GATEWAY_PASS_SETTINGS)
            return true; /* pass one: the settings are not seeded yet, so this line is not ours to read */
        const int k = iotdata_settings_key_index(key);
        if (k < 0) {
            PRINTF_ERROR("config: '%s' is not a protocol setting this build has\n", key);
            return false;
        }
        if (!iotdata_settings_key_write(&gateway_settings, k, text)) {
            PRINTF_ERROR("config: '%s' is not a value for '%s'\n", text, key);
            return false;
        }
        if (from_overlay)
            gateway_settings_owned[k] = true;
        return true;
    }
    if (gateway_config_pass != GATEWAY_PASS_CONFIG)
        return true; /* pass two reads only the settings; the rows were done in pass one */
    const int i = iotdata_config_index_by_name(key);
    if (i < 0) {
        PRINTF_ERROR("config: '%s' is not a setting this build has\n", key);
        return false;
    }
    const iotdata_config_row_t *const row = &iotdata_config_table[i];
    iotdata_config_value_t v;
    if (!_iotdata_config_parse(row, text, &v)) {
        PRINTF_ERROR("config: '%s' is not a %s, for '%s'\n", text, _iotdata_config_type_name(row->type), key);
        return false;
    }
    iotdata_config_update_t u;
    iotdata_config_update_begin(&u); /* LOCAL: a file and a command line are both a person */
    if (!iotdata_config_update_stage(&u, row->id, &v) || !iotdata_config_update_commit(&u, NULL, NULL)) {
        PRINTF_ERROR("config: '%s' refused for '%s' -- out of range, or read-only\n", text, key);
        return false;
    }
    if (from_overlay)
        gateway_config_owned[i] = true; /* a previous run's runtime change: still ours to keep */
    return true;
}

/* `key = value`, `#` and `;` to end of line, blanks ignored. */
static bool gateway_config_read(const char *const path, const bool required, const bool from_overlay) {
    FILE *const f = fopen(path, "r");
    if (f == NULL) {
        if (required) /* an absent OVERLAY is the ordinary case: nothing has been changed yet */
            PRINTF_ERROR("config: could not read '%s'\n", path);
        return false;
    }
    char line[512];
    while (fgets(line, sizeof(line), f) != NULL) {
        char *const hash = strpbrk(line, "#;");
        if (hash != NULL)
            *hash = '\0';
        char *const eq = strchr(line, '=');
        if (eq == NULL)
            continue;
        *eq = '\0';
        char *key = line, *val = eq + 1;
        while (*key != '\0' && isspace((unsigned char)*key))
            key++;
        for (char *e = key + strlen(key); e > key && isspace((unsigned char)e[-1]);)
            *--e = '\0';
        while (*val != '\0' && isspace((unsigned char)*val))
            val++;
        for (char *e = val + strlen(val); e > val && isspace((unsigned char)e[-1]);)
            *--e = '\0';
        if (*key == '\0')
            continue;
        const int row = iotdata_config_index_by_name(key);
        if (row >= 0 && (iotdata_config_table[row].flags & IOTDATA_CONFIG_FLAG_STARTUP) != 0u) {
            /* By the time this line is read, the file it is trying to choose is already open. */
            PRINTF_WARN("config: '%s' has no effect from inside a settings file -- it is a command-line option\n", key);
            continue;
        }
        (void)gateway_config_set(key, val, from_overlay);
    }
    (void)fclose(f);
    return true;
}

/* `--setting value` or `--setting=value`. Not getopt, because getopt wants a table of options and
   the table of options is the config table -- building one from the other to hand it back would be
   the same list twice again. */
static bool gateway_config_argv(const int argc, char **const argv) {
    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "--", 2) != 0) {
            PRINTF_ERROR("config: '%s' is not an option\n", argv[i]);
            return false;
        }
        char name[80];
        const char *val = NULL;
        const char *const eq = strchr(argv[i] + 2, '=');
        if (eq != NULL) {
            const size_t n = (size_t)(eq - (argv[i] + 2));
            if (n >= sizeof(name))
                return false;
            memcpy(name, argv[i] + 2, n);
            name[n] = '\0';
            val = eq + 1;
        } else {
            snprintf(name, sizeof(name), "%s", argv[i] + 2);
            if (i + 1 >= argc) {
                PRINTF_ERROR("config: '--%s' needs a value\n", name);
                return false;
            }
            val = argv[++i];
        }
        if (strcmp(name, "config") == 0 || strcmp(name, "overlay") == 0)
            continue; /* handled before the file was read */
        if (!gateway_config_set(name, val, false))
            return false;
        /* PINNED, and pinned AFTER it was applied -- pinning first would make the command line
           refuse its own argument. From here the value is immutable for this run: the console and
           a manager on the radio are both told no, and the way out is to restart without it. */
        if (iotdata_settings_key_is_ours(name)) {
            if (gateway_config_pass == GATEWAY_PASS_SETTINGS)
                iotdata_settings_key_pin(iotdata_settings_key_index(name));
        } else if (gateway_config_pass == GATEWAY_PASS_CONFIG) {
            const int row = iotdata_config_index_by_name(name);
            if (row >= 0)
                (void)iotdata_config_pin(iotdata_config_table[row].id);
        }
    }
    return true;
}

/* An option's value from argv WITHOUT applying it, for the two that decide where to read from. */
static const char *gateway_config_argv_peek(const int argc, char **const argv, const char *const want) {
    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "--", 2) != 0)
            continue;
        const char *const eq = strchr(argv[i] + 2, '=');
        if (eq != NULL) {
            if ((size_t)(eq - (argv[i] + 2)) == strlen(want) && strncmp(argv[i] + 2, want, strlen(want)) == 0)
                return eq + 1;
        } else if (strcmp(argv[i] + 2, want) == 0 && i + 1 < argc)
            return argv[i + 1];
    }
    return NULL;
}

/* defaults < the operator's file < the runtime overlay < the command line.
 *
 * The overlay sits between the file and the command line on purpose: a value changed at runtime has
 * to outlive a restart, and the only other place to put it would be the operator's own file, which
 * would mean rewriting something they hand-edited and losing their comments and their ordering. The
 * command line still wins over both, because that is for this run and a person is watching. */
static bool gateway_config_load(const int argc, char **const argv) {
    const char *const named = gateway_config_argv_peek(argc, argv, "config");
    const char *const file = (named != NULL) ? named : CONFIG_FILE_DEFAULT;
    const char *const over = gateway_config_argv_peek(argc, argv, "overlay");
    gateway_config_overlay_path = (over != NULL) ? over : CONFIG_OVERLAY_DEFAULT;
    /* A file named explicitly must be readable: falling back to defaults would silently apply the
       wrong settings and, for a gateway, put the attached radio on the wrong channel. */
    /* Recorded before the read, so that `conf config` answers with the file actually in use rather
       than with the default it might not be. */
    (void)gateway_config_set("config", file, false);
    (void)gateway_config_set("overlay", gateway_config_overlay_path, false);

    /* THE ROUTING RULE HAS TO STAY UNAMBIGUOUS. A config row named "node-something" would be read
       as a setting and never reach its own table, so the build says so instead of the box behaving
       strangely in the field. */
    for (int i = 0; i < (int)IOTDATA_CFG_COUNT; i++)
        if (iotdata_settings_key_is_ours(iotdata_config_table[i].name)) {
            PRINTF_ERROR("config: row '%s' collides with the '%s' settings prefix -- rename it\n", iotdata_config_table[i].name, IOTDATA_SETTINGS_KEY_PREFIX);
            return false;
        }

    gateway_config_pass = GATEWAY_PASS_CONFIG;
    if (!gateway_config_read(file, named != NULL, false) && named != NULL)
        return false;
    (void)gateway_config_read(gateway_config_overlay_path, false, true);
    if (!gateway_config_argv(argc, argv))
        return false;

    /* SEEDED FROM THE CONFIGURATION, between the passes: on a host "what this node is called before
       anybody commissioned it" is what `station-id` says, which is the same role a MAC-derived id
       plays on a device. Frozen straight after, because the console's "(default N)" has to come
       from somewhere and on a host there is no state block doing it. */
    iotdata_settings_defaults(&gateway_settings, iotdata_config_u16(STATION_ID));
    iotdata_settings_freeze(&gateway_settings);

    gateway_config_pass = GATEWAY_PASS_SETTINGS;
    (void)gateway_config_read(file, false, false);
    (void)gateway_config_read(gateway_config_overlay_path, false, true);
    if (!gateway_config_argv(argc, argv))
        return false;
    gateway_config_pass = GATEWAY_PASS_CONFIG;
    gateway_settings_loaded = gateway_settings; /* the baseline a later write is a change FROM */

    PRINTF_INFO("config: %d settings + %d node value(s) from '%s' (+ overlay '%s')\n", (int)IOTDATA_CFG_COUNT, (int)IOTDATA_SETTINGS_KEY_COUNT, file, gateway_config_overlay_path);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// WRITING BACK
//
// To the overlay, never to the operator's file. Rewriting that would mean reformatting something a
// person wrote and losing their comments and their ordering, to save a value they did not set.
// -----------------------------------------------------------------------------------------------------------------------------------------

/* ONE WRITER FOR BOTH, because there is one file. Whichever sink was called has already recorded
   what it now owns; this puts the whole picture back on disk in a single atomic replace, so a
   config write can never drop the settings lines and a settings write can never drop the rows. */
static bool gateway_overlay_write(void) {
    if (gateway_config_overlay_path == NULL) {
        PRINTF_ERROR("config: nowhere to write to -- no overlay path was set\n");
        return false;
    }
    char tmp[PATH_MAX];
    snprintf(tmp, sizeof(tmp), "%s.new", gateway_config_overlay_path);
    FILE *const f = fopen(tmp, "w");
    if (f == NULL) {
        PRINTF_ERROR("config: cannot create '%s' (%s)\n", tmp, strerror(errno));
        return false;
    }
    fprintf(f, "# iotdata gateway -- values changed at runtime.\n");
    fprintf(f, "# Written by the gateway; edit '%s' instead, this file is overwritten.\n\n", CONFIG_FILE_DEFAULT);
    int written = 0;
    for (int i = 0; i < (int)IOTDATA_CFG_COUNT; i++) {
        if (!gateway_config_owned[i] || (iotdata_config_table[i].flags & IOTDATA_CONFIG_FLAG_STARTUP) != 0u)
            continue;
        iotdata_config_value_t v;
        if (!iotdata_config_get(iotdata_config_table[i].id, &v))
            continue;
        char buf[64];
        const iotdata_config_row_t *const row = &iotdata_config_table[i];
        /* the key as the file and the command line spell it: lower case, dashes */
        for (const char *p = row->name; *p != '\0'; p++)
            fputc((*p == '_') ? '-' : (*p | 0x20), f);
        if (row->type == IOTDATA_CONFIG_TYPE_STRING)
            fprintf(f, " = %s\n", (v.s.p != NULL) ? v.s.p : "");
        else if (row->type == IOTDATA_CONFIG_TYPE_BOOL)
            fprintf(f, " = %s\n", v.b ? "true" : "false");
        else
            fprintf(f, " = %s\n", _iotdata_config_fmt(row, &v, buf, sizeof(buf)));
        written++;
    }
    /* the settings, in the same file and by the same rule: what we own, and nothing the operator
       stated and never changed */
    int node_written = 0;
    for (int k = 0; k < IOTDATA_SETTINGS_KEY_COUNT; k++) {
        char key[IOTDATA_SETTINGS_KEY_MAX], val[32];
        if (!gateway_settings_owned[k] || !iotdata_settings_key_name(k, key, sizeof(key)) || !iotdata_settings_key_read(&gateway_settings, k, val, sizeof(val)))
            continue;
        if (node_written == 0)
            fprintf(f, "\n");
        fprintf(f, "%s = %s\n", key, val);
        node_written++;
    }
    const bool flushed = (fflush(f) == 0);
    /* TRIED, NOT REQUIRED. fsync is a durability nicety and not every filesystem has one -- a CIFS
       or NFS mount can refuse it outright, and treating that as a failed write threw away a file
       that had in fact been written perfectly well. The bytes are in the page cache either way. */
    if (fsync(fileno(f)) != 0)
        PRINTF_WARN("config: '%s' not synced to disk (%s) -- written, but not yet durable\n", tmp, strerror(errno));
    if (fclose(f) != 0 || !flushed) {
        PRINTF_ERROR("config: cannot write '%s' (%s)\n", tmp, strerror(errno));
        (void)remove(tmp);
        return false;
    }
    /* Renamed into place, so a reader never sees half a file and a crash mid-write leaves the
       previous one intact. */
    if (rename(tmp, gateway_config_overlay_path) != 0) {
        PRINTF_ERROR("config: cannot replace '%s' with '%s' (%s)\n", gateway_config_overlay_path, tmp, strerror(errno));
        (void)remove(tmp);
        return false;
    }
    PRINTF_INFO("config: wrote %d changed value(s) and %d node setting(s) to '%s'\n", written, node_written, gateway_config_overlay_path);
    return true;
}

/* THE CONFIG SINK. Records what the update moved, then writes the whole file. */
static bool gateway_config_save(const struct iotdata_config_update *const u) {
    /* WHAT JUST MOVED is ours to keep, and nothing else. This used to ask "does it differ from the
       default", which is a different question with the same answer most of the time and a ruinous
       one the rest: every value an operator had put in their OWN file differs from the default too,
       so the overlay filled up with their settings and then, being read after the file, overrode
       the file they were editing. */
    for (int i = 0; i < (int)IOTDATA_CFG_COUNT; i++)
        if (u != NULL && u->touched[i] && (iotdata_config_table[i].flags & IOTDATA_CONFIG_FLAG_STARTUP) == 0u)
            gateway_config_owned[i] = true;
    return gateway_overlay_write();
}

/* THE SETTINGS SINK, attached in place of the state-block flush a device uses.
 *
 * It cannot be handed a delta the way the config sink is -- a settings write is a struct assignment
 * and not a staged update -- so "what moved" is answered by comparing against the image AS LOADED.
 * Deliberately not against the build's default: that is the same question with the same answer most
 * of the time and a ruinous one the rest, because a value the operator wrote in their own file
 * differs from the default too, and copying it here would shadow the file they were editing. */
static bool gateway_settings_save(const iotdata_settings_t *const s) {
    if (s == NULL)
        return false;
    for (int k = 0; k < IOTDATA_SETTINGS_KEY_COUNT; k++) {
        char now[32], was[32];
        if (!iotdata_settings_key_read(s, k, now, sizeof(now)) || !iotdata_settings_key_read(&gateway_settings_loaded, k, was, sizeof(was)))
            continue;
        if (strcmp(now, was) != 0)
            gateway_settings_owned[k] = true;
    }
    return gateway_overlay_write();
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#endif /* IOTDATA_NODE_CONFIG_EXPANDED && !IOTDATA_GATEWAY_CONFIG_APPLIED */
