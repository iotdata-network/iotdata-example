// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

/*
 * IoT Sensor Telemetry Protocol
 * Copyright(C) 2026 Matthew Gream (https://libiotdata.org)
 *
 * sensor_lora.c - BME280/LTR390 weather sensor on esp32
 *
 * Reads a Bosch BME280 (temperature / pressure / humidity) amd/or an LTR390
 * (solar irradiance and ultraviolet) over I2C, encodes the reading as an
 * iotdata packet using the "weather_station" variant, transmits it via the
 * E22 LoRa radio module. It LISTENS CONTINUOUSLY and speaks only when told to -- see app_survey.h.
 *
 * Every wake is a complete, self-contained cycle: measure, transmit, sleep.
 * Nothing carries over except a small block of RTC-retained state (station id,
 * packet sequence, BME280 factory calibration), and that block is purely a
 * cache — anything missing is re-acquired on the next wake. So a cycle can be
 * repeated, or missed, without disturbing the ones around it.
 *
 */

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wnested-externs"
#pragma GCC diagnostic ignored "-Wredundant-decls"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#pragma GCC diagnostic pop

// -----------------------------------------------------------------------------------------------------------------------------------------
// EXAMPLE config + hardware
// -----------------------------------------------------------------------------------------------------------------------------------------

#include "iotdata_config.h"
#include "iotdata_hardware.h"

// -----------------------------------------------------------------------------------------------------------------------------------------
// TUNABLE
// -----------------------------------------------------------------------------------------------------------------------------------------

#ifndef SURVEY_AUTO_MS
#define SURVEY_AUTO_MS ((uint32_t)iotdata_config_u16(SURVEY_AUTO_S) * 1000u)
#endif
#ifndef CONSOLE_DRAIN_MS
#define CONSOLE_DRAIN_MS 100 /* let the console flush before deep sleep    */
#endif
#ifndef RECEIVE_POLL_MS
#define RECEIVE_POLL_MS 10 /* polling during the recieve window */
#endif
#ifndef RESTART_MS
#define RESTART_MS 30000 /* duration of restart on failure */
#endif

// -----------------------------------------------------------------------------------------------------------------------------------------
// HARDWARE
// -----------------------------------------------------------------------------------------------------------------------------------------

// AE GEN LORA I2C --> BME280
// (A) SDA -> (4) SDA
// (B) SCL -> (3) SCL
// (C) VCC -> (1) VCC + (5) CSB
// (D) GND -> (2) GND + (6) SD0

#define PIN_DEVICE_UART_TX    PIN_E22_RXD
#define PIN_DEVICE_UART_RX    PIN_E22_TXD
#define PIN_DEVICE_LORA_AUX   PIN_E22_AUX
#define PIN_DEVICE_LORA_M0    PIN_E22_M0
#define PIN_DEVICE_LORA_M1    PIN_E22_M1
#define PIN_DEVICE_I2C_SDA    PIN_I2C_SDA
#define PIN_DEVICE_I2C_SCL    PIN_I2C_SCL

#define GNSS_UART_PORT        UART_NUM_0

#define PIN_GNSS_UART_TX      GPIO_NUM_1 /* -> module RX */
#define PIN_GNSS_UART_RX      GPIO_NUM_0 /* <- module TX */

/* The button. Circuit is the SDS's: VCC -> 10k -> GPIO <- button <- GND, so released reads HIGH and
   pressed reads LOW, and the module leaves the internal pull-up off because the external one does
   the work. GPIO3 is the last pin free on this board.
     single press   send a position packet, survey or no survey -- marking a spot is useful
                    on its own, and the packet is the same one either way
     double press   start a survey, or stop the one running
     long  (2s)     start automatic position reporting, or stop it -- also independent of
                    whether a survey is running
     xlong (8s)     cancel everything and reboot -- the only physical recovery on a node
                    with no console

   The SDS's 10s/20s long presses are retuned here: that was a sealed outdoor unit where an
   accidental press mattered more than a convenient one. This is a handheld being operated on
   purpose, and waiting ten seconds to do something deliberate is its own kind of wrong. */
#define PIN_BUTTON            GPIO_NUM_3

/* PIN_I2C_SDA/SCL are UNUSED on this build -- there is no I2C sensor here, and the board's pull-ups
   are optional solder. That leaves two pins free for indication, which is what this node most
   wants next: a survey is started and stopped blind, and the only confirmation today is the
   monitor on a phone. */

/* The directive is short-lived and RE-ISSUED while the walk continues, rather than issued once for
   an optimistic duration. Re-issue is idempotent by design -- a relay already holding the id passes
   it on and carries on unchanged -- and the useful consequence is the one that matters in a field:
   switch this node off, or walk out of range, and every relay stops surveying by itself within
   SURVEY_DURATION_M. Nobody has to remember to tell them.

   THE LIFETIME IS DERIVED, not chosen. It only has to outlast a few missed re-issues, and a walk
   goes to the edge of coverage on purpose -- so the relay most likely to miss one is exactly the
   relay worth keeping. Three intervals tolerates two consecutive misses. What this number really
   governs is how long relays keep spending airtime AFTER the walk ends; the cadence that actually
   costs airtime during it is SURVEY_BATCH_S. */
#define SURVEY_LOOP_MS        20u    /* how often the loop comes round: receive, fix, button */
#define SURVEY_FIX_REFRESH_MS 5000u  /* how often to look for a newer fix */
#define SURVEY_FIX_STALE_MS   60000u /* after this, report NO position rather than an old one */
#define SURVEY_LIVENESS_MS    30000u

#include "d_platform_esp32.h"
#include "d_common.h"
#include "d_format.h"
#include "d_readings.h"
#include "d_hardware_gpio.h"
#include "d_hardware_uart.h"
#include "d_interface_e22900t22.h"
#include "d_interface_at6558.h"
#include "d_interface_button.h"
#include "d_module_datastore_esp32.h"

// -----------------------------------------------------------------------------------------------------------------------------------------
// IOTDATA PROTOCOL + COMMONS
// -----------------------------------------------------------------------------------------------------------------------------------------

/*
 * Strip everything except the encoder for a minimal ESP32 build:
 *   - NO_JSON:     no cJSON dependency
 *   - NO_DUMP:     no dump output
 *   - NO_PRINT:    no print output
 *   - NO_FLOATING: iotdata_float_t = int32_t (value * 100)
 */
#define IOTDATA_NO_JSON
#define IOTDATA_NO_DUMP
#define IOTDATA_NO_PRINT
#define IOTDATA_NO_FLOATING
#include "iotdata_variant.h"
#include "iotdata.h"
#include "iotdata.c"
#include "iotdata_mesh.h" /* this node issues its own DIAG SURVEY directive */
#include "iotdata_node.h"

#include "iotdata_node_utils.h"
#include "iotdata_node_partial.h"
#define IOTDATA_VERSION_HAS_BLACKBOX
#include "iotdata_node_version.h"
#include "iotdata_node_variant.h"
#include "iotdata_node_status.h"
#include "iotdata_node_config.h"
#include "iotdata_node_control.h"
#define IOTDATA_DIAGNOSTICS       IOTDATA_CONFIG_BLACKBOX
#define IOTDATA_DIAGNOSTICS_FLUSH BLACKBOX_FLUSH_MANUAL /* a sleeping node flushes before it sleeps */
#define IOTDATA_BLACKBOX_IMPLEMENTATION
static void _node_diagnostics_emit(const char *const line) {
    ESP_LOGI("app", "%s", line); // XXX
}
#include "iotdata_node_diagnostics.h"
// #include "iotdata_node_content.h"

#include "iotdata_node_state.h"
#include "iotdata_node_settings.h"
#include "iotdata_node_platform.h"

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool on_lora_changed(const iotdata_config_row_t *row, const iotdata_config_value_t *was, const iotdata_config_info_t *info);

#define IOTDATA_CONFIG_LORA_NOTIFY on_lora_changed
#include "iotdata_node_config_device_e22900t22.h"

#define IOTDATA_CONFIG_ENTRIES(X) \
    /*NAME  id  TYPE  min  max  default  flags  validate  notify */ \
    X(SURVEY_AUTO_S, 0x001, U16, 5, 3600, 30, 0, NULL, NULL, "seconds between automatic reports, once a long press turns them on") \
    IOTDATA_CONFIG_ENTRIES_LORA(X)

#include "iotdata_node_config.h"
#include "iotdata_node_config_device_e22900t22.h"

static bool on_lora_changed(const iotdata_config_row_t *row, const iotdata_config_value_t *was, __attribute__((unused)) const iotdata_config_info_t *info) {
    ESP_LOGW("config", "%s changed (was %llu); the radio is configured once, so this needs a restart", row->name, (unsigned long long)was->u);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static void _app_node_status(const uint16_t station, iotdata_node_status_t *const out);
static bool _app_node_transmit(const uint8_t *const packet, const size_t len);
static int _app_config_build(uint8_t *buf, size_t size, iotdata_partial_t *partial);
static bool _app_config_apply(const uint8_t *buf, size_t len, bool *reboot_out);

// -----------------------------------------------------------------------------------------------------------------------------------------
// APP STATE
// -----------------------------------------------------------------------------------------------------------------------------------------

static const char *__tag_app = "app";

/* Before the state struct that holds a survey_t. It takes this board's answers -- how a packet is
   built, how the radio is brought up -- as function POINTERS, resolved where the config literal is
   initialised in app_init(), so it needs none of them in scope here. */
#include "app_survey.h"

#define STATE_MAGIC 0xB1E28003UL
#define STATE_TAG   0xB1E28004UL

typedef struct {
    _RTC_DATA_STAMP_ENTRY;
    iotdata_version_caps_t caps;
    idep_node_t node;
    uint32_t cycles;    /* wake cycles since the last restart              */
    uint32_t tx_count;  /* packets transmitted                             */
    uint32_t tx_errors; /* packets the radio would not take                */
    int16_t battery_mv; /* last reading, for the charging trend bit        */
    bool lora_present;  /* E22 NVM configuration verified this power cycle */
    bool gnss_present;  /* the AT6558 answered a product query at cold boot  */
    uint32_t rx_count;  /* frames heard, of anything, from anyone */
    bool fix_valid;
    uint32_t fix_at_ms, fix_last_ms; /* when it was taken, and when we last looked */
    iotdata_mesh_diag_position_t fix;
    survey_t survey; /* the instrument: see app_survey.h */
} state_operating_t;

typedef struct {
    datastore_t datastore;
    iotdata_node_state_t node;
    iotdata_settings_t settings; /* the PROTOCOL's own: station id, reporting, receive */
    const idep_config_t idep_cfg;
} state_running_t;

static _RTC_DATA_STRUCT state_operating_t _state_operating;
static state_running_t _state_running = { .idep_cfg = {
                                              .caps = &_state_operating.caps,
                                              .status = _app_node_status,
                                              .tx = _app_node_transmit,
                                              .control = IOTDATA_DIAGNOSTICS_CONTROL,
                                              .control_actions = iotdata_diagnostics_control_actions,
                                              .control_actions_count = IOTDATA_DIAGNOSTICS_CONTROL_ACTIONS_COUNT,
                                              .diag = IOTDATA_DIAGNOSTICS_PULL,
                                              .receive_every_ms = IDEP_RECEIVE_EVERY_MS,
                                              .receive_window_ms = IDEP_RECEIVE_WINDOW_MS,
                                              .settings = &_state_running.settings,
                                              .state = &_state_running.node,
                                              .config_build = _app_config_build,
                                              .config_apply = _app_config_apply,
                                              .packet_max = LORA_PACKET_SIZE_MAX,
                                          } };

static const struct {
    state_operating_t *const operating;
    state_running_t *const running;
} app_state = { .operating = &_state_operating, .running = &_state_running };

// -----------------------------------------------------------------------------------------------------------------------------------------

static int _app_config_build(uint8_t *const buf, const size_t size, iotdata_partial_t *const partial) {
    return iotdata_config_pack(buf, size, partial);
}

static bool _app_config_apply(const uint8_t *const buf, const size_t len, bool *const reboot_out) {
    return iotdata_config_apply(buf, len, &app_state.running->datastore, reboot_out);
}

static void _app_node_status(__attribute__((unused)) const uint16_t station, iotdata_node_status_t *const out) {
    const state_operating_t *const s = app_state.operating;
    out->uptime_s = (uint32_t)(esp_timer_get_time() / 1000000);
    out->reason = iotdata_node_reason_reset();
    out->has_heap = true;
    out->heap_free = (uint32_t)esp_get_free_heap_size();
    out->heap_min = (uint32_t)esp_get_minimum_free_heap_size();
    out->has_restarts = true;
    out->restarts = (uint16_t)s->cycles;
}

static bool _app_node_transmit(const uint8_t *const packet, const size_t len) {
    return app_state.operating->lora_present && lora_write_complete(packet, len, /*wait_complete=*/true) == ESP_OK;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool _app_exec_transmit(const uint8_t *const buf, const size_t len) {
    return app_state.operating->lora_present && lora_write_complete(buf, len, /*wait_complete=*/true) == ESP_OK;
}

static bool _app_exec_position(iotdata_mesh_diag_position_t *const p) {
    if (!app_state.operating->fix_valid)
        return false;
    *p = app_state.operating->fix;
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static void app_exec_receive(void) {
    for (;;) {
        static uint8_t buf[IOTDATA_MAX_PACKET_SIZE]; // XXX
        int len = 0, rssi_dbm = LORA_RSSI_NONE;
        if (!app_state.operating->lora_present || lora_read(buf, sizeof(buf), &len, &rssi_dbm, 0) != ESP_OK || len <= 0)
            return;
        uint8_t variant = 0;
        uint16_t station = 0, sequence = 0;
        if (iotdata_mesh_peek_header(buf, len, &variant, &station, &sequence)) {
            if (station != idep_station(&app_state.operating->node)) {
                survey_observe(&app_state.operating->survey, station, sequence, rssi_dbm);
                app_state.operating->rx_count++;
            }
        }
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* Stricter than the driver's default, because a survey's whole worth is the position being right:
 *   quality >= 1  rejects GGA quality 6, the receiver's own ESTIMATE, which looks like a fix and
 *                 is not one -- the first fixes on this bench were exactly that;
 *   sats >= 4     a 3D solution needs four, and fewer is arithmetic rather than measurement;
 *   hdop_max 0    deliberately OFF. A poor fix is still evidence the node was there, and the HDOP
 *                 rides along in the TLV so the decision can be made later, with the whole map in
 *                 view, rather than here with one point in view.
 */
static const gnss_strategy_t survey_gnss_strategy = {
    .fix_timeout_ms = GNSS_FIX_TIMEOUT_MS_DEFAULT,
    .nmea_timeout_ms = GNSS_NMEA_TIMEOUT_MS_DEFAULT,
    .max_fix_retries = 0,
    .min_satellites = 4,
    .min_quality = GNSS_QUALITY_GPS,
    .hdop_max = 0.0f,
};

static void app_exec_refresh_gnss(void) {
    if (app_state.operating->gnss_present && (hw_time_ms() - app_state.operating->fix_last_ms) >= SURVEY_FIX_REFRESH_MS) {
        app_state.operating->fix_last_ms = hw_time_ms();
        gnss_reading_t r;
        if (gnss_read(&r, GNSS_READ_POSN, &survey_gnss_strategy) == ESP_OK) {
            app_state.operating->fix.latitude_e7 = (int32_t)(r.latitude * 1e7);
            app_state.operating->fix.longitude_e7 = (int32_t)(r.longitude * 1e7);
            app_state.operating->fix.altitude_m = (int16_t)(r.altitude + (r.altitude < 0.0f ? -0.5f : 0.5f));
            app_state.operating->fix_at_ms = hw_time_ms();
            if (!app_state.operating->fix_valid) {
                app_state.operating->fix_valid = true;
                ESP_LOGI(__tag_app, "gnss: fix %.6f,%.6f alt=%dm sats=%u hdop=%.1f", r.latitude, r.longitude, (int)app_state.operating->fix.altitude_m, (unsigned)r.satellites, (double)r.hdop);
            }
        } else {
            if (app_state.operating->fix_valid && (hw_time_ms() - app_state.operating->fix_at_ms) >= SURVEY_FIX_STALE_MS) {
                app_state.operating->fix_valid = false;
                ESP_LOGW(__tag_app, "gnss: no fix for %us -- reports will carry no position rather than an old one", (unsigned)(SURVEY_FIX_STALE_MS / 1000u));
            }
        }
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void app_exec_status(void) {
    static uint32_t last_ms = 0;
    const uint32_t now = hw_time_ms();
    if (last_ms == 0 || (now - last_ms) >= SURVEY_LIVENESS_MS) {
        last_ms = now;
        ESP_LOGI(__tag_app, "alive: rx=%" PRIu32 " heard=%" PRIu32 " (dropped %" PRIu32 ") tx=%" PRIu32 " errors=%" PRIu32 " | survey=%s auto=%s fix=%s", app_state.operating->rx_count, app_state.operating->survey.obs_total,
                 app_state.operating->survey.obs_dropped, app_state.operating->tx_count, app_state.operating->tx_errors, app_state.operating->survey.active ? "on" : "off", app_state.operating->survey.auto_report ? "on" : "off",
                 app_state.operating->fix_valid ? "yes" : "no");
        iotdata_diagnostics_flush();
        iotdata_state_flush(&app_state.running->node);
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

bool app_exec(void) {
    for (;;) {
        app_exec_receive();
        app_exec_refresh_gnss();
        survey_poll(&app_state.operating->survey);
        app_exec_status();
        (void)iotdata_diagnostics_pump();
        app_state.operating->cycles++;
        hw_delay_ms_yieldable(SURVEY_LOOP_MS);
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

bool app_init(void) {

    const esp_reset_reason_t reset_reason = esp_reset_reason();
    const bool restarted = (reset_reason != ESP_RST_DEEPSLEEP);

    ESP_LOGI(__tag_app, "iotdata lora survey instrument: listens continuously, reports on demand");
    ESP_LOGI(__tag_app, "boot: reset_reason=%d %s", (int)reset_reason, reset_reason_str(reset_reason));

    iotdata_diagnostics_emit_set(_node_diagnostics_emit);
    if (!iotdata_diagnostics_begin((uint8_t)reset_reason, restarted))
        ESP_LOGW(__tag_app, "diag: the recorder did not start");
    iotdata_diagnostics_flush();

    if (reset_reason == ESP_RST_BROWNOUT && _RTC_DATA_VALID(app_state.operating, STATE_MAGIC) && app_state.operating->battery_mv > 0)
        ESP_LOGE(__tag_app, "boot: BROWNOUT -- the last reading before it was %dmV", (int)app_state.operating->battery_mv);

    if (!_RTC_DATA_VALID(app_state.operating, STATE_MAGIC)) {
        _RTC_DATA_INIT(app_state.operating, STATE_MAGIC);
        idep_node_init(&app_state.operating->node, iotdata_node_station_from_mac(__tag_app), NULL, 0u);
        iotdata_version_caps_init(&app_state.operating->caps);
        if ((app_state.operating->gnss_present = (gnss_setup(&gnss_config_default) == ESP_OK)))
            iotdata_version_caps_add(&app_state.operating->caps, IOTDATA_VERSION_CAPS_SENSOR, IOTDATA_VERSION_SENSOR_GNSS);
        else
            ESP_LOGW(__tag_app, "gnss: no module answered on tx=GPIO%d rx=GPIO%d", (int)PIN_GNSS_UART_TX, (int)PIN_GNSS_UART_RX);
    }

    char vbuf[IOTDATA_VERSION_STR_MAX + 1];
    ESP_LOGI(__tag_app, "version: %s", iotdata_version_str(vbuf, sizeof(vbuf), &app_state.operating->caps));
    if (!iotdata_version_stamp_is_real())
        ESP_LOGW(__tag_app, "version: build stamp is unset -- this binary cannot say when it was built");

    if (datastore_open(&app_state.running->datastore, "iotdata")) {
        iotdata_state_init(&app_state.running->node, &app_state.running->datastore, "state");
        iotdata_settings_defaults(&app_state.running->settings, idep_station(&app_state.operating->node));
        if (!iotdata_settings_attach(&app_state.running->settings, &app_state.running->node))
            ESP_LOGW(__tag_app, "state: settings block not registered -- protocol settings will not persist");
        if (!idep_node_bind(&app_state.operating->node, &app_state.running->node, STATE_TAG))
            ESP_LOGW(__tag_app, "state: persistence failed (bind STATE_TAG)");
        const bool restored = iotdata_state_load(&app_state.running->node);
        ESP_LOGI(__tag_app, "state: %u block(s), %s", (unsigned)app_state.running->node.count, restored ? "restored" : "defaulted");
        if (idep_node_restore_station(&app_state.operating->node, &app_state.running->settings))
            ESP_LOGI(__tag_app, "node: station=%" PRIu16 " restored from settings", idep_station(&app_state.operating->node));
        if (!iotdata_config_load(&app_state.running->datastore))
            ESP_LOGI(__tag_app, "config: nothing stored yet -- every row at its default");
    } else
        ESP_LOGW(__tag_app, "state: persistence disabled (no datastore)");

    static const survey_config_t survey_config = {
        .node = &_state_operating.node,
        .button_gpio = PIN_BUTTON,
        .send = _app_exec_transmit,
        .position = _app_exec_position,
        .tx_count = &_state_operating.tx_count,
        .tx_errors = &_state_operating.tx_errors,
    };
    (void)survey_init(&app_state.operating->survey, &survey_config);

    lora_config_t cfg;
    iotdata_config_lora_apply(&cfg);
    esp_log_level_set(__tag_device_e22900t22, cfg.debug ? ESP_LOG_DEBUG : ESP_LOG_INFO);
    if ((app_state.operating->lora_present = (lora_setup(&cfg) == ESP_OK)) && lora_start() == ESP_OK)
        ESP_LOGI(__tag_app, "lora: listening continuously on uart%d", (int)E22_UART_PORT);
    else {
        app_state.operating->lora_present = false;
        ESP_LOGE(__tag_app, "lora: would not start -- this node can neither hear nor be heard");
    }

    if (app_state.operating->gnss_present) {
        if (gnss_start() == ESP_OK) {
            ESP_LOGI(__tag_app, "gnss: running continuously on uart%d (tx=GPIO%d rx=GPIO%d)", (int)GNSS_UART_PORT, (int)PIN_GNSS_UART_TX, (int)PIN_GNSS_UART_RX);
        } else {
            app_state.operating->gnss_present = false;
            ESP_LOGW(__tag_app, "gnss: would not start -- carrying on without a fix");
        }
    }

    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void app_fail(void) {
    ESP_LOGE(__tag_app, "failed");
    iotdata_diagnostics_event(IOTDATA_BB_LC_STOP, 0);
    hw_delay_ms_yieldable(RESTART_MS);
    esp_restart();
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void app_main(void) {

    setbuf(stdout, NULL);

    const esp_err_t wdt_err = esp_task_wdt_add(NULL);
    if (wdt_err != ESP_OK)
        ESP_LOGE(__tag_app, "task watchdog: subscribe failed: %s", esp_err_to_name(wdt_err));
    else
        ESP_LOGI(__tag_app, "task watchdog: subscribed (timeout=%ds)", CONFIG_ESP_TASK_WDT_TIMEOUT_S);

    if (!app_init() || !app_exec())
        app_fail();
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
