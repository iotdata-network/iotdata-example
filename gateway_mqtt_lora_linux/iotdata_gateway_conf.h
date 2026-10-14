#ifndef IOTDATA_GATEWAY_CONF_H
#define IOTDATA_GATEWAY_CONF_H

// -----------------------------------------------------------------------------------------------------------------------------------------
// iotdata_gateway_config.h - the gateway's configuration, as an iotdata_node_config table.
// -----------------------------------------------------------------------------------------------------------------------------------------

#ifndef MQTT_CLIENT_DEFAULT
#define MQTT_CLIENT_DEFAULT "iotdata_gateway"
#endif
#ifndef CONFIG_FILE_DEFAULT
#define CONFIG_FILE_DEFAULT "iotdata_gateway.cfg"
#endif
#ifndef CONFIG_OVERLAY_DEFAULT
#define CONFIG_OVERLAY_DEFAULT "iotdata_gateway.runtime.cfg"
#endif
#ifndef STATE_PATH_DEFAULT
#define STATE_PATH_DEFAULT "/var/lib/iotdata"
#endif
#ifndef LORA_PORT_DEFAULT
#define LORA_PORT_DEFAULT "/dev/e22900t22u"
#endif

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define IOTDATA_NODE_CONFIG_LORA_MODULE LORA_MODULE_USB
#include "iotdata_node_config_device_e22900t22.h"
#define IOTDATA_NODE_CONFIG_MESH_ENABLE 0
#include "iotdata_node_config_mesh.h"
#include "iotdata_node_config_ddup.h"
#define IOTDATA_NODE_CONFIG_MQTT_CLIENT MQTT_CLIENT_DEFAULT
#include "iotdata_node_config_mqtt.h"
#include "iotdata_node_config_bbox.h"
#include "iotdata_node_config_content.h"
#include "iotdata_node_config_stat.h"

/* The application's live-refresh handler, for rows it CACHES rather than reads each time. The
   gateway names its own before including this; a translation unit that only wants the table -- the
   test does -- has nothing cached to refresh and takes the do-nothing default. */
#ifndef IOTDATA_GATEWAY_NOTIFY
#define IOTDATA_GATEWAY_NOTIFY NULL
#endif

#define IOTDATA_NODE_CONFIG_ENTRIES(X) \
    /*  NAME                           id     TYPE    min max                   default                            flags                      validate notify */ \
    X(CONFIG, 0x001, STRING, 1, 127, CONFIG_FILE_DEFAULT, IOTDATA_NODE_CONFIG_FLAG_LOCAL | IOTDATA_NODE_CONFIG_FLAG_STARTUP, NULL, NULL, "the settings file to read") \
    X(OVERLAY, 0x002, STRING, 1, 127, CONFIG_OVERLAY_DEFAULT, IOTDATA_NODE_CONFIG_FLAG_LOCAL | IOTDATA_NODE_CONFIG_FLAG_STARTUP, NULL, NULL, "where a setting changed at runtime is written, beside that file") \
    X(STATE, 0x003, STRING, 1, 127, STATE_PATH_DEFAULT, IOTDATA_NODE_CONFIG_FLAG_LOCAL, NULL, NULL, "the directory this gateway keeps its state in") \
    X(DEBUG, 0x004, BOOL, 0, 1, false, IOTDATA_NODE_CONFIG_FLAG_LOCAL, NULL, IOTDATA_GATEWAY_NOTIFY, "Debug output (true/false)") \
\
    X(LORA_PORT, 0x101, STRING, 1, 127, LORA_PORT_DEFAULT, IOTDATA_NODE_CONFIG_FLAG_LOCAL, NULL, NULL, "the serial device the E22 radio is on") \
    IOTDATA_NODE_CONFIG_ENTRIES_LORA(X) \
    IOTDATA_NODE_CONFIG_ENTRIES_MESH(X) \
    IOTDATA_NODE_CONFIG_ENTRIES_MESH_GATEWAY(X) \
    IOTDATA_NODE_CONFIG_ENTRIES_DDUP(X) \
    IOTDATA_NODE_CONFIG_ENTRIES_MQTT(X) \
\
    IOTDATA_NODE_CONFIG_ENTRIES_STAT_DISPLAY(X) \
    IOTDATA_NODE_CONFIG_ENTRIES_STAT_DISPLAY_NETW(X) \
    IOTDATA_NODE_CONFIG_ENTRIES_STAT_PUBLISH_NETW(X) \
    IOTDATA_NODE_CONFIG_ENTRIES_STAT_PUBLISH(X) \
    IOTDATA_NODE_CONFIG_ENTRIES_STAT_PUBLISH_MQTT(X) \
\
    IOTDATA_NODE_CONFIG_ENTRIES_BBOX(X) \
    IOTDATA_NODE_CONFIG_ENTRIES_BBOX_FILE(X) \
\
    IOTDATA_NODE_CONFIG_ENTRIES_CONTENT(X) \
    IOTDATA_NODE_CONFIG_ENTRIES_CONTENT_FILE(X)

#endif /* IOTDATA_GATEWAY_CONF_H */

/* --- the second include, once the table has been expanded ------------------------------------ */

#if defined(IOTDATA_NODE_CONFIG_EXPANDED) && !defined(IOTDATA_GATEWAY_CONFIG_APPLIED)
#define IOTDATA_GATEWAY_CONFIG_APPLIED

#include "iotdata_node_config_device_e22900t22.h"
#include "iotdata_node_config_mesh.h"
#include "iotdata_node_config_mqtt.h"
#include "iotdata_node_config_ddup.h"
#include "iotdata_node_config_mqtt.h"
#include "iotdata_node_config_bbox.h"
#include "iotdata_node_config_content.h"
#include "iotdata_node_config_stat.h"

/* Which rows a later write must persist: the ones changed at RUNTIME, and the ones a previous run
   already put in the overlay. Not the ones that came out of the operator's file -- those are theirs
   and stay theirs -- and not the command line, which is for this run. */
static bool gateway_config_owned[IOTDATA_NODE_CFG_COUNT];
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
static iotdata_node_settings_t gateway_settings;
static bool gateway_settings_owned[IOTDATA_NODE_SETTINGS_KEY_COUNT];
/* AS LOADED, which is what "ours" is measured against. Not the build's default: every value the
   operator put in their own file differs from that too, so measuring there would copy their file
   into the overlay and then -- the overlay being read second -- shadow the file they were editing.
   That is the exact mistake the config side already made once. */
static iotdata_node_settings_t gateway_settings_loaded;

/* ONE PASS, since STATION_ID went away.
 *
 * It used to be two. The settings' defaults depended on the configuration -- the station's default
 * was whatever a `station-id` row said -- so nothing "node-" could be applied until the config was
 * complete, or the seeding would overwrite it. That row is gone: the station now defaults the way
 * it does on a device, from the hardware, which needs nothing read first. So the settings are
 * seeded before the file is opened and both kinds of line are applied in the one walk. */

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
    if (iotdata_node_settings_key_is_ours(key)) {
        const int k = iotdata_node_settings_key_index(key);
        if (k < 0) {
            PRINTF_ERROR("config: '%s' is not a protocol setting this build has\n", key);
            return false;
        }
        if (!iotdata_node_settings_key_write(&gateway_settings, k, text)) {
            PRINTF_ERROR("config: '%s' is not a value for '%s'\n", text, key);
            return false;
        }
        if (from_overlay)
            gateway_settings_owned[k] = true;
        return true;
    }
    const int i = iotdata_node_config_index_by_name(key);
    if (i < 0) {
        PRINTF_ERROR("config: '%s' is not a setting this build has\n", key);
        return false;
    }
    const iotdata_node_config_row_t *const row = &iotdata_node_config_table[i];
    iotdata_node_config_value_t v;
    if (!_iotdata_node_config_parse(row, text, &v)) {
        PRINTF_ERROR("config: '%s' is not a %s, for '%s'\n", text, _iotdata_node_config_type_name(row->type), key);
        return false;
    }
    iotdata_node_config_update_t u;
    iotdata_node_config_update_begin(&u, false);
    if (!iotdata_node_config_update_stage(&u, row->id, &v) || !iotdata_node_config_update_commit(&u, NULL, NULL)) {
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
    if (!f) {
        if (required) /* an absent OVERLAY is the ordinary case: nothing has been changed yet */
            PRINTF_ERROR("config: could not read '%s'\n", path);
        return false;
    }
    char line[512];
    while (fgets(line, sizeof(line), f)) {
        char *const hash = strpbrk(line, "#;");
        if (hash)
            *hash = '\0';
        char *const eq = strchr(line, '=');
        if (eq) {
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
            if (*key != '\0') {
                const int row = iotdata_node_config_index_by_name(key);
                if (row >= 0 && (iotdata_node_config_table[row].flags & IOTDATA_NODE_CONFIG_FLAG_STARTUP) != 0u) {
                    /* By the time this line is read, the file it is trying to choose is already open. */
                    PRINTF_WARN("config: '%s' has no effect from inside a settings file -- it is a command-line option\n", key);
                } else
                    (void)gateway_config_set(key, val, from_overlay);
            }
        }
    }
    (void)fclose(f);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

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
        if (eq) {
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
        if (strcmp(name, "config") != 0 && strcmp(name, "overlay") != 0) {
            if (!gateway_config_set(name, val, false))
                return false;
            /* PINNED, and pinned AFTER it was applied -- pinning first would make the command line
            refuse its own argument. From here the value is immutable for this run: the console and
            a manager on the radio are both told no, and the way out is to restart without it. */
            if (iotdata_node_settings_key_is_ours(name))
                iotdata_node_settings_key_pin(iotdata_node_settings_key_index(name));
            else {
                const int row = iotdata_node_config_index_by_name(name);
                if (row >= 0)
                    (void)iotdata_node_config_pin(iotdata_node_config_table[row].id);
            }
        }
    }
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* An option's value from argv WITHOUT applying it, for the two that decide where to read from. */
static const char *gateway_config_argv_peek(const int argc, char **const argv, const char *const want) {
    for (int i = 1; i < argc; i++)
        if (strncmp(argv[i], "--", 2) == 0) {
            const char *const eq = strchr(argv[i] + 2, '=');
            if (eq) {
                if ((size_t)(eq - (argv[i] + 2)) == strlen(want) && strncmp(argv[i] + 2, want, strlen(want)) == 0)
                    return eq + 1;
            } else if (strcmp(argv[i] + 2, want) == 0 && i + 1 < argc)
                return argv[i + 1];
        }
    return NULL;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* defaults < the operator's file < the runtime overlay < the command line.
 *
 * The overlay sits between the file and the command line on purpose: a value changed at runtime has
 * to outlive a restart, and the only other place to put it would be the operator's own file, which
 * would mean rewriting something they hand-edited and losing their comments and their ordering. The
 * command line still wins over both, because that is for this run and a person is watching. */
static bool gateway_config_load(const int argc, char **const argv) {
    const char *const named = gateway_config_argv_peek(argc, argv, "config");
    const char *const file = (named) ? named : CONFIG_FILE_DEFAULT;
    const char *const over = gateway_config_argv_peek(argc, argv, "overlay");
    gateway_config_overlay_path = (over) ? over : CONFIG_OVERLAY_DEFAULT;
    /* A file named explicitly must be readable: falling back to defaults would silently apply the
       wrong settings and, for a gateway, put the attached radio on the wrong channel. */
    /* Recorded before the read, so that `conf config` answers with the file actually in use rather
       than with the default it might not be. */
    (void)gateway_config_set("config", file, false);
    (void)gateway_config_set("overlay", gateway_config_overlay_path, false);

    /* THE ROUTING RULE HAS TO STAY UNAMBIGUOUS. A config row named "node-something" would be read
       as a setting and never reach its own table, so the build says so instead of the box behaving
       strangely in the field. */
    for (int i = 0; i < (int)IOTDATA_NODE_CFG_COUNT; i++)
        if (iotdata_node_settings_key_is_ours(iotdata_node_config_table[i].name)) {
            PRINTF_ERROR("config: row '%s' collides with the '%s' settings prefix -- rename it\n", iotdata_node_config_table[i].name, IOTDATA_NODE_SETTINGS_KEY_PREFIX);
            return false;
        }

    /* SEEDED BEFORE ANYTHING IS READ, from the hardware -- the same answer a device gets, by the
       same call. A host has no config row saying who it is any more: that was one more place for
       an identity to live, and a gateway commissioned over the air would have had a file still
       insisting otherwise. The station is a SETTING, so `node-station` in the file below overrides
       this, and a `--node-station` argument overrides and pins it.
       Frozen straight after, because the console's "(default N)" has to come from somewhere and on
       a host there is no state block doing it. */
    iotdata_node_settings_defaults(&gateway_settings, iotdata_node_station_from_mac("gateway"));
    iotdata_node_settings_freeze(&gateway_settings);

    if (!gateway_config_read(file, named != NULL, false) && named)
        return false;
    (void)gateway_config_read(gateway_config_overlay_path, false, true);
    if (!gateway_config_argv(argc, argv))
        return false;
    gateway_settings_loaded = gateway_settings; /* the baseline a later write is a change FROM */

    PRINTF_INFO("config: %d settings + %d node value(s) from '%s' (+ overlay '%s')\n", (int)IOTDATA_NODE_CFG_COUNT, (int)IOTDATA_NODE_SETTINGS_KEY_COUNT, file, gateway_config_overlay_path);
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
    if (!gateway_config_overlay_path) {
        PRINTF_ERROR("config: nowhere to write to -- no overlay path was set\n");
        return false;
    }
    char tmp[PATH_MAX];
    FILE *const f = fopen(snprintf_inline(tmp, sizeof(tmp), "%s.new", gateway_config_overlay_path), "w");
    if (!f) {
        PRINTF_ERROR("config: cannot create '%s' (%s)\n", tmp, strerror(errno));
        return false;
    }
    fprintf(f, "# iotdata gateway -- values changed at runtime.\n");
    fprintf(f, "# Written by the gateway; edit '%s' instead, this file is overwritten.\n\n", CONFIG_FILE_DEFAULT);
    int written = 0;
    for (int i = 0; i < (int)IOTDATA_NODE_CFG_COUNT; i++)
        if (gateway_config_owned[i] && (iotdata_node_config_table[i].flags & IOTDATA_NODE_CONFIG_FLAG_STARTUP) == 0u) {
            iotdata_node_config_value_t v;
            if (iotdata_node_config_get(iotdata_node_config_table[i].id, &v)) {
                char buf[64];
                const iotdata_node_config_row_t *const row = &iotdata_node_config_table[i];
                /* the key as the file and the command line spell it: lower case, dashes */
                for (const char *p = row->name; *p != '\0'; p++)
                    fputc((*p == '_') ? '-' : (*p | 0x20), f);
                if (row->type == IOTDATA_NODE_CONFIG_TYPE_STRING)
                    fprintf(f, " = %s\n", (v.s.p) ? v.s.p : "");
                else if (row->type == IOTDATA_NODE_CONFIG_TYPE_BOOL)
                    fprintf(f, " = %s\n", v.b ? "true" : "false");
                else
                    fprintf(f, " = %s\n", _iotdata_node_config_fmt(row, &v, buf, sizeof(buf)));
                written++;
            }
        }
    /* the settings, in the same file and by the same rule: what we own, and nothing the operator
       stated and never changed */
    int node_written = 0;
    for (int k = 0; k < IOTDATA_NODE_SETTINGS_KEY_COUNT; k++) {
        char key[IOTDATA_NODE_SETTINGS_KEY_MAX], val[32];
        if (gateway_settings_owned[k] && iotdata_node_settings_key_name(k, key, sizeof(key)) && iotdata_node_settings_key_read(&gateway_settings, k, val, sizeof(val))) {
            if (node_written == 0)
                fprintf(f, "\n");
            fprintf(f, "%s = %s\n", key, val);
            node_written++;
        }
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

// -----------------------------------------------------------------------------------------------------------------------------------------

/* THE CONFIG SINK. Records what the update moved, then writes the whole file. */
static bool gateway_config_save(const struct iotdata_node_config_update *const u) {
    /* WHAT JUST MOVED is ours to keep, and nothing else. This used to ask "does it differ from the
       default", which is a different question with the same answer most of the time and a ruinous
       one the rest: every value an operator had put in their OWN file differs from the default too,
       so the overlay filled up with their settings and then, being read after the file, overrode
       the file they were editing. */
    for (int i = 0; i < IOTDATA_NODE_CFG_COUNT; i++)
        if (u && u->touched[i] && (iotdata_node_config_table[i].flags & IOTDATA_NODE_CONFIG_FLAG_STARTUP) == 0u)
            gateway_config_owned[i] = true;
    return gateway_overlay_write();
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* THE SETTINGS SINK, attached in place of the state-block flush a device uses.
 *
 * It cannot be handed a delta the way the config sink is -- a settings write is a struct assignment
 * and not a staged update -- so "what moved" is answered by comparing against the image AS LOADED.
 * Deliberately not against the build's default: that is the same question with the same answer most
 * of the time and a ruinous one the rest, because a value the operator wrote in their own file
 * differs from the default too, and copying it here would shadow the file they were editing. */
static bool gateway_settings_save(const iotdata_node_settings_t *const s) {
    if (!s)
        return false;
    for (int k = 0; k < IOTDATA_NODE_SETTINGS_KEY_COUNT; k++) {
        char now[32], was[32];
        if (iotdata_node_settings_key_read(s, k, now, sizeof(now)) && iotdata_node_settings_key_read(&gateway_settings_loaded, k, was, sizeof(was)))
            if (strcmp(now, was) != 0)
                gateway_settings_owned[k] = true;
    }
    return gateway_overlay_write();
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#endif /* IOTDATA_NODE_CONFIG_EXPANDED && !IOTDATA_GATEWAY_CONFIG_APPLIED */
