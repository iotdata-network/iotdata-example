

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include "iotdata_config.h"
#include "iotdata_hardware.h"

// AE GEN LORA I2C --> BME280
// (A) SDA -> (4) SDA
// (B) SCL -> (3) SCL
// (C) VCC -> (1) VCC + (5) CSB
// (D) GND -> (2) GND + (6) SD0

#define PIN_DEVICE_UART_TX  PIN_E22_RXD
#define PIN_DEVICE_UART_RX  PIN_E22_TXD
#define PIN_DEVICE_LORA_AUX PIN_E22_AUX
#define PIN_DEVICE_LORA_M0  PIN_E22_M0
#define PIN_DEVICE_LORA_M1  PIN_E22_M1

// -----------------------------------------------------------------------------------------------------------------------------------------

typedef struct {
    bool bme280_present; /* bme280_setup succeeded this power cycle         */
    bool ltr390_present; /* ltr390_setup succeeded this power cycle         */
} app_operating_t;

typedef struct {
} app_running_t;

#define APP_CONFIG_ENTRIES(X) X(TRANSMIT_PERIOD_S, 0x001, U16, 10, 3600, (IOTDATA_NODE_CONFIG_SENSOR_TRANSMIT_PERIOD_MS / 1000), 0, NULL, NULL, "how often a reading goes out")

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

/*
 * IoT Sensor Telemetry Protocol
 * Copyright(C) 2026 Matthew Gream (https://libiotdata.org)
 *
 * app_simple_lora_esp32.c - BME280/LTR390 weather sensor on esp32
 *
 * Battery + Lora
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
// TUNABLE
// -----------------------------------------------------------------------------------------------------------------------------------------

#ifndef BATTERY_SELFTEST_MS
#define BATTERY_SELFTEST_MS 0
#endif

// -----------------------------------------------------------------------------------------------------------------------------------------
// HARDWARE
// -----------------------------------------------------------------------------------------------------------------------------------------

#include "d_platform_esp32.h"
#include "d_common.h"
#include "d_format.h"
#include "d_module_datastore_esp32.h"
#include "d_readings.h"
#include "d_hardware_gpio.h"
#include "d_hardware_adc.h"
#include "d_hardware_uart.h"
#include "d_interface_e22900t22.h"
#include "d_interface_batt.h"

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
#include "iotdata_node.h"

#include "iotdata_node_utils.h"
#include "iotdata_node_partial.h"
#define IOTDATA_NODE_VERSION_HAS_BLACKBOX
#include "iotdata_node_version.h"
#include "iotdata_node_variant.h"
#include "iotdata_node_status.h"
#include "iotdata_node_config.h"
#include "iotdata_node_control.h"
#define IOTDATA_NODE_DIAGNOSTICS       IOTDATA_NODE_CONFIG_BLACKBOX
#define IOTDATA_NODE_DIAGNOSTICS_FLUSH BLACKBOX_FLUSH_MANUAL /* a sleeping node flushes before it sleeps */
#define IOTDATA_NODE_BLACKBOX_IMPLEMENTATION
static void _iotd_diagnostics_emit(const char *const line) {
    ESP_LOGI("app", "%s", line); // XXX
}
#include "iotdata_node_diagnostics.h"
// #include "iotdata_node_content.h"

#include "iotdata_node_state.h"
#include "iotdata_node_settings.h"
#include "iotdata_node_platform.h"

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool _iotd_on_lora_changed(const iotdata_node_config_row_t *row, const iotdata_node_config_value_t *was, const iotdata_node_config_info_t *info);
#define IOTDATA_NODE_CONFIG_LORA_NOTIFY _iotd_on_lora_changed
#include "iotdata_node_config_device_e22900t22.h"

static bool _iotd_on_batt_changed(const iotdata_node_config_row_t *row, const iotdata_node_config_value_t *was, const iotdata_node_config_info_t *info);
#define IOTDATA_NODE_CONFIG_BATT_NOTIFY _iotd_on_batt_changed
#include "iotdata_node_config_device_batt.h"

#define IOTDATA_NODE_CONFIG_ENTRIES(X) \
    /*NAME  id  TYPE  min  max  default  flags  validate  notify */ \
    APP_CONFIG_ENTRIES(X) \
    IOTDATA_NODE_CONFIG_ENTRIES_LORA(X) \
    IOTDATA_NODE_CONFIG_ENTRIES_BATT(X)

#include "iotdata_node_config.h"

#include "iotdata_node_config_device_e22900t22.h"

static bool _iotd_on_lora_changed(const iotdata_node_config_row_t *row, const iotdata_node_config_value_t *was, __attribute__((unused)) const iotdata_node_config_info_t *info) {
    ESP_LOGW("config", "lora %s changed (was %llu), restart required", row->name, (unsigned long long)was->u);
    return true;
}

#include "iotdata_node_config_device_batt.h"

static bool _iotd_on_batt_changed(const iotdata_node_config_row_t *row, __attribute__((unused)) const iotdata_node_config_value_t *was, __attribute__((unused)) const iotdata_node_config_info_t *info) {
    if (!iotdata_node_config_batt_apply())
        ESP_LOGW("config", "batt %s changed, but values not accepted", row->name);
    return false;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#ifndef POWER_DIAGNOSTICS
#define POWER_DIAGNOSTICS 1
#endif
#include "iotdata_node_diagnostics_device_batt.h"

// -----------------------------------------------------------------------------------------------------------------------------------------

static void _iotd_node_status(const uint16_t station, iotdata_node_status_t *const out);
static bool _iotd_node_transmit(const uint8_t *const packet, const size_t len);
static int _iotd_node_config_build(uint8_t *buf, size_t size, iotdata_node_partial_t *partial);
static bool _iotd_node_config_apply(const uint8_t *buf, size_t len, bool *reboot_out);

// -----------------------------------------------------------------------------------------------------------------------------------------
// APP STATE
// -----------------------------------------------------------------------------------------------------------------------------------------

#define IOTD_STATE_MAGIC 0xB1E28003UL
#define IOTD_STATE_TAG   0xB1E28004UL

typedef struct {
    _RTC_DATA_STAMP_ENTRY;
    iotdata_node_version_caps_t caps;
    iotdata_node_t node;
    uint32_t restarts;
    uint32_t tx_cycles;   /* wake cycles since the last restart              */
    uint32_t tx_count;    /* packets transmitted                             */
    uint32_t tx_errors;   /* packets the radio would not take                */
    int16_t battery_mv;   /* last reading, for the charging trend bit        */
    bool battery_present; /* a divider answered the probe at restart         */
    bool lora_present;    /* E22 NVM configuration verified this power cycle */
    app_operating_t app;
} iotd_operating_t;

typedef struct {
    datastore_t datastore;
    iotdata_node_state_t node;
    iotdata_node_settings_t node_settings; /* the PROTOCOL's own: station id, reporting, receive */
    const iotdata_node_params_t node_config;
    uint8_t packet_buffer[IOTDATA_MAX_PACKET_SIZE];
    app_running_t app;
} iotd_running_t;

static _RTC_DATA_STRUCT iotd_operating_t _iotd_operating;
static iotd_running_t _iotd_running = { .node_config = {
                                            .caps = &_iotd_operating.caps,
                                            .status = _iotd_node_status,
                                            .tx = _iotd_node_transmit,
                                            .control = IOTDATA_NODE_DIAGNOSTICS_CONTROL,
                                            .control_actions = iotdata_node_diagnostics_control_actions,
                                            .control_actions_count = IOTDATA_NODE_DIAGNOSTICS_CONTROL_ACTIONS_COUNT,
                                            .diag = IOTDATA_NODE_DIAGNOSTICS_PULL,
                                            .receive_every_ms = IOTDATA_NODE_RECEIVE_EVERY_MS,
                                            .receive_window_ms = IOTDATA_NODE_RECEIVE_WINDOW_MS,
                                            .settings = &_iotd_running.node_settings,
                                            .state = &_iotd_running.node,
                                            .config_build = _iotd_node_config_build,
                                            .config_apply = _iotd_node_config_apply,
                                            .packet_max = LORA_PACKET_SIZE_MAX,
                                        } };

typedef struct {
    iotd_operating_t *const operating;
    iotd_running_t *const running;
} iotd_state_t;
typedef struct {
    app_operating_t *const operating;
    app_running_t *const running;
} app_state_t;

static iotd_state_t iotd_state = { .operating = &_iotd_operating, .running = &_iotd_running };
static app_state_t app_state = { .operating = &_iotd_operating.app, .running = &_iotd_running.app };

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

typedef enum {
    IOTD_EV_OK,
    IOTD_EV_SLEEP_DEEP,
    IOTD_EV_SLEEP_LIGHT,
    IOTD_EV_FAIL
} iotd_event_t;

typedef void (*app_func_init_log_t)(app_state_t *const as, iotdata_node_t *const node, char *const buf, const size_t len);
typedef iotd_event_t (*app_func_init_entry_t)(app_state_t *const as, iotdata_node_t *const node, const esp_reset_reason_t reset_reason);
typedef iotd_event_t (*app_func_init_rtc_t)(app_state_t *const as, iotdata_node_t *const node, iotdata_node_version_caps_t *const caps);
typedef iotd_event_t (*app_func_init_datastore_t)(app_state_t *const as, iotdata_node_t *const node, datastore_t *const ds);
typedef iotd_event_t (*app_func_init_final_t)(app_state_t *const as, iotdata_node_t *const node, const esp_reset_reason_t reset_reason);
typedef iotd_event_t (*app_func_sleep_t)(app_state_t *const as, iotdata_node_t *const node, uint32_t *const sleep_ms, const uint32_t start_ms, const uint32_t awake_ms, bool deepsleep);
typedef iotd_event_t (*app_func_exec_packet_t)(app_state_t *const as, iotdata_node_t *const node, const uint32_t cycle, const bool battery_present, const battery_reading_t *const battery, const int16_t battery_mv_was, uint8_t *const buf,
                                               const size_t len, const bool advertise, size_t *const transmit);
typedef iotd_event_t (*app_func_exec_final_t)(app_state_t *const as, iotdata_node_t *const node, const uint32_t start_ms);
typedef void (*app_func_fail_t)(app_state_t *const as, iotdata_node_t *const node);

typedef struct {
    const char *name;
    uint8_t variant;
    app_func_init_log_t func_init_log;
    app_func_init_entry_t func_init_entry;
    app_func_init_rtc_t func_init_rtc;
    app_func_init_datastore_t func_init_datastore;
    app_func_init_final_t func_init_final;
    app_func_sleep_t func_sleep;
    app_func_exec_packet_t func_exec_packet;
    app_func_exec_final_t func_exec_final;
    app_func_fail_t func_fail;
} app_descr_t;

extern const app_descr_t *app_spec;

// -----------------------------------------------------------------------------------------------------------------------------------------
// IOTDATA COMMONS
// -----------------------------------------------------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#ifndef CONSOLE_DRAIN_MS
#define CONSOLE_DRAIN_MS 100 /* let the console flush before deep sleep    */
#endif
#ifndef RECEIVE_POLL_MS
#define RECEIVE_POLL_MS 10 /* polling during the recieve window */
#endif
#ifndef RESTART_MS
#define RESTART_MS 30000 /* duration of restart on failure */
#endif

static const char *__tag_iotd = "iotd";

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static void _iotd_node_status(__attribute__((unused)) const uint16_t station, iotdata_node_status_t *const out) {
    const iotd_operating_t *const s = iotd_state.operating;

    out->uptime_s = (uint32_t)(esp_timer_get_time() / 1000000);
    out->reason = iotdata_node_reason_reset();
    out->has_heap = true;
    out->heap_free = (uint32_t)esp_get_free_heap_size();
    out->heap_min = (uint32_t)esp_get_minimum_free_heap_size();
    out->has_restarts = true;
    out->restarts = (uint16_t)s->restarts;
    if (s->battery_present && s->battery_mv > 0) {
        out->has_supply = true;
        out->supply_mv = (uint16_t)s->battery_mv;
    }
}

static bool _iotd_node_transmit(const uint8_t *const packet, const size_t len) {
    return lora_write_complete(packet, len, /*wait_complete=*/true) == ESP_OK;
}

static int _iotd_node_config_build(uint8_t *const buf, const size_t size, iotdata_node_partial_t *const partial) {
    return iotdata_node_config_pack(buf, size, partial);
}
static bool _iotd_node_config_apply(const uint8_t *const buf, const size_t len, bool *const reboot_out) {
    return iotdata_node_config_apply(buf, len, &iotd_state.running->datastore, reboot_out);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static void iotd_cycle_elapsed(iotd_state_t *const as, const uint32_t cycle_ms) {
    (void)iotdata_node_window_advance(&as->operating->node, cycle_ms);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void iotd_node_receive(iotd_state_t *const as) {
    iotdata_node_t *const n = &as->operating->node;

    ESP_LOGI(__tag_iotd, "receive: window opened for %us (station=%03" PRIX16 ")", (unsigned)(IOTDATA_NODE_RECEIVE_WINDOW_MS / 1000), iotdata_node_station(n));
    iotdata_node_window_begin(n, (uint32_t)hw_time_ms());
    iotdata_node_diagnostics_event(IOTDATA_NODE_BB_LC_WAKE, 0);
    bool reboot = false;
    unsigned frames = 0, acted = 0;
    while (iotdata_node_window_active(n, (uint32_t)hw_time_ms())) {
        int len = 0, rssi_dbm = 0;
        if (lora_read(as->running->packet_buffer, sizeof(as->running->packet_buffer), &len, &rssi_dbm, 0) == ESP_OK && len > 0) {
            frames++;
            acted += iotdata_node_on_frame(n, as->running->packet_buffer, (size_t)len, &reboot) ? 1 : 0;
        }
        iotdata_node_diagnostics_pump();        /* a DUMP that arrived in this window drains inside it */
        hw_delay_ms_yieldable(RECEIVE_POLL_MS); /* yield and pat the watchdog: the window is long by MCU standards */
    }
    iotdata_node_window_end(n);
    ESP_LOGI(__tag_iotd, "receive: window closed (%u frame(s) heard, %u for us)", frames, acted);

    if (reboot) {
        ESP_LOGW(__tag_iotd, "node: REBOOT commanded -- restarting");
        iotdata_node_diagnostics_event(IOTDATA_NODE_BB_LC_STOP, 0);
        iotdata_node_diagnostics_flush();
        hw_delay_ms_yieldable(CONSOLE_DRAIN_MS);
        esp_restart();
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static void iotd_fail(iotd_state_t *const as) {

    const app_descr_t *const ad = app_spec;

    if (ad->func_fail)
        ad->func_fail(&app_state, &as->operating->node);

    // don't close the datastore or do anything: we don't know what kind of bad situation the app is in
    ESP_LOGE(__tag_iotd, "failure, restarting in %ds", RESTART_MS);
    iotdata_node_diagnostics_event(IOTDATA_NODE_BB_LC_STOP, 0);
    hw_delay_ms_yieldable(RESTART_MS);
    esp_restart();
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static bool iotd_init(iotd_state_t *const as) {

    const app_descr_t *const ad = app_spec;
    iotdata_node_t *const node = &as->operating->node;

    // capture reset reason
    const esp_reset_reason_t reset_reason = esp_reset_reason();
    const bool restarted = (reset_reason != ESP_RST_DEEPSLEEP);
    if (restarted)
        as->operating->restarts++;

    const bool cold_boot = !_RTC_DATA_VALID(as->operating, IOTD_STATE_MAGIC);
    const int16_t brownout_mv = cold_boot ? 0 : as->operating->battery_mv;
    if (cold_boot) {
        _RTC_DATA_INIT(as->operating, IOTD_STATE_MAGIC);
        iotdata_node_init(node, iotdata_node_station_from_mac(__tag_iotd), NULL, 0u); // station_id may be changed in persistence load
    }
    iotdata_node_attach(node, &as->running->node_config);

    // initial banner, before anything else
    char buffer[80] = { 0 };
    if (ad->func_init_log)
        ad->func_init_log(&app_state, node, buffer, sizeof(buffer));
    ESP_LOGI(__tag_iotd, "app :: %s :: variant: %s%s%s", ad->name, iotdata_vsuite_name(ad->variant), buffer[0] != '\0' ? ", details: " : "", buffer);
    ESP_LOGI(__tag_iotd, "boot: reset_reason=%d %s", (int)reset_reason, reset_reason_str(reset_reason));

    // bring up diagnostics and flush any pending records if in RTC
    iotdata_node_diagnostics_emit_set(_iotd_diagnostics_emit);
    if (!iotdata_node_diagnostics_begin((uint8_t)reset_reason, restarted))
        ESP_LOGW(__tag_iotd, "diag: the recorder did not start");
    iotdata_node_diagnostics_flush();

    // deal with power reset issues
    if (reset_reason == ESP_RST_BROWNOUT && brownout_mv > 0) {
        power_startup(brownout_mv);
        ESP_LOGE(__tag_iotd, "boot: BROWNOUT -- the last reading before it was %dmV", (int)brownout_mv);
    }

    // any app specific entry activities, failable
    if (ad->func_init_entry && ad->func_init_entry(&app_state, node, reset_reason) != IOTD_EV_OK)
        return false;

#if BATTERY_SELFTEST_MS > 0
    (void)battery_test(BATTERY_SELFTEST_MS);
#endif

    // capabilities and the fitted-hardware probe, cold boot only
    if (cold_boot) {
        iotdata_node_version_caps_init(&as->operating->caps);
        as->operating->battery_present = battery_probe();
        if (ad->func_init_rtc)
            ad->func_init_rtc(&app_state, node, &as->operating->caps);
    }

    // dump out version information
    char vbuf[IOTDATA_NODE_VERSION_STR_MAX + 1];
    ESP_LOGI(__tag_iotd, "version: %s", iotdata_node_version_str(vbuf, sizeof(vbuf), &as->operating->caps));
    if (!iotdata_node_version_stamp_is_real())
        ESP_LOGW(__tag_iotd, "version: build stamp is unset -- this binary cannot say when it was built");

    // open the datastore and hydrate all forms of persistent storage
    if (datastore_open(&as->running->datastore, "iotdata")) {
        iotdata_node_state_init(&as->running->node, &as->running->datastore, "state");
        iotdata_node_settings_defaults(&as->running->node_settings, iotdata_node_station(&as->operating->node));
        if (!iotdata_node_settings_attach(&as->running->node_settings, &as->running->node))
            ESP_LOGW(__tag_iotd, "state: persistence not available, protocol settings will not persist");
        if (!iotdata_node_bind(&as->operating->node, &as->running->node, IOTD_STATE_TAG))
            ESP_LOGW(__tag_iotd, "state: persistence binding failed");
        const bool restored = iotdata_node_state_load(&as->running->node);
        ESP_LOGI(__tag_iotd, "state: persistence %s %u block(s)", restored ? "restored" : "defaulted", (unsigned)as->running->node.count);
        if (iotdata_node_restore_station(&as->operating->node, &as->running->node_settings))
            ESP_LOGI(__tag_iotd, "node: station=%" PRIu16 " restored from persisted settings", iotdata_node_station(&as->operating->node));
        if (!iotdata_node_config_load(&as->running->datastore))
            ESP_LOGI(__tag_iotd, "config: persistence empty, using defaults");
        if (!iotdata_node_config_batt_apply())
            ESP_LOGW(__tag_iotd, "battery: persisted profile refused, using the built-in one");
        if (ad->func_init_datastore && ad->func_init_datastore(&app_state, node, &as->running->datastore) != IOTD_EV_OK) {
            datastore_close(&as->running->datastore);
            return false;
        }
    } else
        ESP_LOGW(__tag_iotd, "state: persistence disabled (no datastore)");

    // record power state
    if (cold_boot)
        power_describe(as->operating->battery_present);

    // any app specific final activities, failable
    if (ad->func_init_final && ad->func_init_final(&app_state, node, reset_reason) != IOTD_EV_OK)
        return false;

    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static bool iotd_sleep(iotd_state_t *const as, const uint32_t start_ms, bool deep) {

    const app_descr_t *const ad = app_spec;
    iotdata_node_t *const node = &as->operating->node;

    const uint32_t awake_ms = hw_time_ms() - start_ms;
    uint32_t sleep_ms = 0;
    if (!deep) {
        if (ad->func_sleep && ad->func_sleep(&app_state, node, &sleep_ms, start_ms, awake_ms, false) != IOTD_EV_OK)
            return false;
        iotd_cycle_elapsed(as, awake_ms + sleep_ms);
        iotdata_node_diagnostics_event(IOTDATA_NODE_BB_LC_SLEEP, iotdata_node_diagnostics_reason(sleep_ms / 1000));
        iotdata_node_diagnostics_flush();
        ESP_LOGI(__tag_iotd, "sleep light %" PRIu32 "ms (awake %" PRIu32 "ms) -- cycle %" PRIu32 ", tx=%" PRIu32 " errors=%" PRIu32, sleep_ms, awake_ms, as->operating->tx_cycles, as->operating->tx_count, as->operating->tx_errors);
        hw_delay_ms_yieldable(CONSOLE_DRAIN_MS);
        hw_delay_ms_yieldable(sleep_ms);
    } else {
        if (ad->func_sleep && ad->func_sleep(&app_state, node, &sleep_ms, start_ms, awake_ms, true) != IOTD_EV_OK)
            return false;
        iotd_cycle_elapsed(as, awake_ms + sleep_ms);
        iotdata_node_diagnostics_event(IOTDATA_NODE_BB_LC_SLEEP, iotdata_node_diagnostics_reason(sleep_ms / 1000));
        iotdata_node_diagnostics_flush();
        lora_hold();
        datastore_close(&as->running->datastore);
        ESP_LOGI(__tag_iotd, "sleep deep %" PRIu32 "ms (awake %" PRIu32 "ms) -- cycle %" PRIu32 ", tx=%" PRIu32 " errors=%" PRIu32, sleep_ms, awake_ms, as->operating->tx_cycles, as->operating->tx_count, as->operating->tx_errors);
        hw_delay_ms_yieldable(CONSOLE_DRAIN_MS);
        esp_deep_sleep((uint64_t)sleep_ms * 1000); /* does not return */
    }
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static bool iotd_exec(iotd_state_t *const as) {

    const app_descr_t *const ad = app_spec;
    iotdata_node_t *const node = &as->operating->node;

    do {
        const uint32_t start_ms = hw_time_ms();

        const bool receive = iotdata_node_window_pending(&as->operating->node);

        const int16_t battery_mv_was = as->operating->battery_mv;
        battery_reading_t battery_reading = { 0 };
        const battery_reading_t *battery = NULL;
        if (as->operating->battery_present && battery_begin()) {
            if (battery_read(&battery_reading, &as->operating->battery_mv))
                battery = &battery_reading;
            else
                ESP_LOGE(__tag_iotd, "battery: unable to read");
            battery_end();
        }

        size_t transmit = 0;
        iotd_event_t ev = ad->func_exec_packet != NULL
                              ? ad->func_exec_packet(&app_state, node, as->operating->tx_cycles, as->operating->battery_present, battery, battery_mv_was, as->running->packet_buffer, sizeof(as->running->packet_buffer), receive, &transmit)
                              : IOTD_EV_OK;
        if (ev == IOTD_EV_FAIL)
            return false;
        as->operating->tx_cycles++;

        if (transmit || receive) {
            if (!as->operating->lora_present) {
                lora_config_t lora_cfg;
                iotdata_node_config_lora_apply(&lora_cfg);
                esp_log_level_set(__tag_device_e22900t22, lora_cfg.debug ? ESP_LOG_DEBUG : ESP_LOG_INFO);
                as->operating->lora_present = lora_setup(&lora_cfg) == ESP_OK;
            }
            const bool lora_started = as->operating->lora_present && lora_start() == ESP_OK;
            if (!lora_started)
                ESP_LOGE(__tag_iotd, "lora: start failed, no transmissions this cycle");
            if (transmit) {
                if (lora_started && lora_write_complete(as->running->packet_buffer, transmit, /*wait_complete=*/true) == ESP_OK) {
                    ESP_LOGI(__tag_iotd, "packet: transmit success (%zu bytes)", transmit);
                    iotdata_node_sequence_used(&as->operating->node);
                    as->operating->tx_count++;
                } else {
                    ESP_LOGE(__tag_iotd, "packet: transmit failed (%zu bytes)", transmit);
                    as->operating->tx_errors++;
                    iotdata_node_diagnostics_event(IOTDATA_NODE_BB_LC_ERROR, iotdata_node_diagnostics_reason(as->operating->tx_errors));
                }
            }
            if (lora_started && receive)
                iotd_node_receive(as);
            if (lora_started)
                lora_stop();
        }

        ev = ad->func_exec_final != NULL ? ad->func_exec_final(&app_state, node, start_ms) : IOTD_EV_OK;
        if (ev == IOTD_EV_FAIL)
            return false;
        iotdata_node_state_flush(&as->running->node);
        if (ev == IOTD_EV_SLEEP_LIGHT || ev == IOTD_EV_SLEEP_DEEP) {
            if (!iotd_sleep(as, start_ms, ev == IOTD_EV_SLEEP_DEEP))
                return false;
        } else
            iotd_cycle_elapsed(as, hw_time_ms() - start_ms); /* a cycle that did not sleep still took time */

        esp_task_wdt_reset();

    } while (true);

    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void app_main(void) {

    setbuf(stdout, NULL);

    const esp_err_t wdt_err = esp_task_wdt_add(NULL);
    if (wdt_err != ESP_OK)
        ESP_LOGE(__tag_iotd, "task watchdog: subscribe failed: %s", esp_err_to_name(wdt_err));
    else
        ESP_LOGI(__tag_iotd, "task watchdog: subscribed (timeout=%ds)", CONFIG_ESP_TASK_WDT_TIMEOUT_S);

    iotd_state_t *const as = &iotd_state;
    if (!iotd_init(as) || !iotd_exec(as))
        iotd_fail(as);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#ifndef TRANSMIT_PERIOD_MS
#define TRANSMIT_PERIOD_MS ((uint32_t)iotdata_node_config_u16(TRANSMIT_PERIOD_S) * 1000u)
#endif

#ifndef TRANSMIT_PERIOD_MIN_MS
#define TRANSMIT_PERIOD_MIN_MS 1000 /* floor, if a cycle overruns the period  */
#endif

#ifndef PACKET_VARIANT
#define PACKET_VARIANT IOTDATA_VSUITE_WEATHER_STATION
#endif

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static const char *__tag_iapp = "iapp";

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define PIN_DEVICE_I2C_SDA PIN_I2C_SDA
#define PIN_DEVICE_I2C_SCL PIN_I2C_SCL

#include "d_hardware_i2c.h"
#include "d_interface_bme280.h"
#include "d_interface_ltr390.h"

static inline uint16_t ltr390_wm2_16bit(const float lux) {
#define LUX_PER_WM2_DAYLIGHT 126.7f
#define SOLAR_IRRADIANCE_MAX 1023.0f /* the field is 10 bits */
    const float wm2 = lux / LUX_PER_WM2_DAYLIGHT;
    return (uint16_t)lroundf(wm2 < 0.0f ? 0.0f : (wm2 > SOLAR_IRRADIANCE_MAX ? SOLAR_IRRADIANCE_MAX : wm2));
}
static inline uint8_t ltr390_uvi_8bit(const float uvi) {
#define SOLAR_UV_INDEX_MAX 15.0f /* the field is 4 bits  */
    return (uint8_t)lroundf(uvi < 0.0f ? 0.0f : (uvi > SOLAR_UV_INDEX_MAX ? SOLAR_UV_INDEX_MAX : uvi));
}

static bool packet_build(app_state_t *const as, const iotdata_node_t *const node, uint8_t *const buf, size_t len, size_t *out, const uint16_t station, const uint16_t sequence, const bme280_reading_t *const reading_environment,
                         const ltr390_reading_t *const reading_ltr390, const battery_reading_t *const battery, uint8_t flags, const bool advertise_receive) {

    static iotdata_encoder_t enc;

    iotdata_status_t rc;
    if ((rc = iotdata_encode_begin(&enc, buf, len, PACKET_VARIANT, station, sequence)) != IOTDATA_OK) {
        ESP_LOGE(__tag_iapp, "encode_begin: %s", iotdata_strerror(rc));
        return false;
    }

    if (battery != NULL && (rc = iotdata_encode_battery(&enc, battery->percent, battery->charging)) != IOTDATA_OK)
        ESP_LOGW(__tag_iapp, "encode_battery: %s", iotdata_strerror(rc));

    if (reading_environment != NULL && (rc = iotdata_encode_environment(&enc, (iotdata_float_t)lroundf(reading_environment->temperature_c * 100.0f), (uint16_t)lroundf(reading_environment->pressure_hpa),
                                                                        (uint8_t)lroundf(reading_environment->humidity_pct))) != IOTDATA_OK) {
        ESP_LOGW(__tag_iapp, "encode_environment: %s", iotdata_strerror(rc));
        flags |= (uint8_t)(1U << VSUITE_FLAG_SENSOR_FAULTS);
    }

    if (reading_ltr390 != NULL && (rc = iotdata_encode_solar(&enc, ltr390_wm2_16bit(reading_ltr390->lux), ltr390_uvi_8bit(reading_ltr390->uvi))) != IOTDATA_OK) {
        ESP_LOGW(__tag_iapp, "encode_solar: %s", iotdata_strerror(rc));
        flags |= (uint8_t)(1U << VSUITE_FLAG_SENSOR_FAULTS);
    }

    if (flags != 0 && (rc = iotdata_encode_flags(&enc, flags)) != IOTDATA_OK)
        ESP_LOGW(__tag_iapp, "encode_flags: %s", iotdata_strerror(rc));

    if (advertise_receive && !iotdata_node_receive_append(node, &enc))
        ESP_LOGW(__tag_iapp, "encode_receive: no room");

    if ((rc = iotdata_encode_end(&enc, out)) != IOTDATA_OK) {
        ESP_LOGE(__tag_iapp, "encode_end: %s", iotdata_strerror(rc));
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static uint16_t app_board_id(void) {
    static uint16_t cached = 0u;
    return cached == 0u ? (cached = iotdata_node_station_from_mac(__tag_iapp)) : cached;
}

#define QUIRK_BOARD_BME280_ADDR_77 0x769

static const bme280_config_t *app_bme280_config(void) {
    static bme280_config_t config;
    config = bme280_config_default;
    if (app_board_id() == QUIRK_BOARD_BME280_ADDR_77) {
        config.i2c_addr = 0x77;
        ESP_LOGW(__tag_iapp, "bme280: quirk for board %03X -- i2c address 0x%02X, not 0x%02X", QUIRK_BOARD_BME280_ADDR_77, config.i2c_addr, BME280_I2C_ADDR_DEFAULT);
    }
    return &config;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

iotd_event_t app_exec__build_sensor_packet(app_state_t *const as, iotdata_node_t *const node, const uint32_t cycle, const bool battery_present, const battery_reading_t *const battery_reading, const int16_t battery_mv_was,
                                           uint8_t *const buf, const size_t len, const bool receive, size_t *const transmit) {

    uint8_t flags = 0;
    if (cycle == 0)
        flags |= (uint8_t)(1U << VSUITE_FLAG_RESTART_RECENT);

    /* --- sensor: bme280 (enviro) ---  */
    if (!as->operating->bme280_present) {
        as->operating->bme280_present = (bme280_setup(app_bme280_config()) == ESP_OK);
        if (!as->operating->bme280_present && cycle == 0)
            bme280_diagnose();
    }
    bme280_reading_t reading_bme280_, *reading_bme280 = NULL;
    if (as->operating->bme280_present) {
        if (bme280_start() == ESP_OK) {
            reading_bme280 = bme280_read(&reading_bme280_, 0, NULL) == ESP_OK ? &reading_bme280_ : NULL;
            (void)bme280_stop();
        }
        if (!reading_bme280) {
            ESP_LOGE(__tag_iapp, "bme280: detected, but no reading this cycle");
            flags |= (uint8_t)(1U << VSUITE_FLAG_SENSOR_FAULTS);
        }
    }

    /* --- sensor: ltr390 (solar) ---  */
    if (!as->operating->ltr390_present) {
        as->operating->ltr390_present = (ltr390_setup(&ltr390_config_default) == ESP_OK);
        if (!as->operating->ltr390_present && cycle == 0)
            ltr390_diagnose();
    }
    ltr390_reading_t reading_ltr390_, *reading_ltr390 = NULL;
    if (as->operating->ltr390_present) {
        if (ltr390_start() == ESP_OK) {
            reading_ltr390 = ltr390_read(&reading_ltr390_, 0, NULL) == ESP_OK ? &reading_ltr390_ : NULL;
            (void)ltr390_stop();
        }
        if (!reading_ltr390) {
            ESP_LOGE(__tag_iapp, "ltr390: detected, but no reading this cycle");
            flags |= (uint8_t)(1U << VSUITE_FLAG_SENSOR_FAULTS);
        }
    }

    const bool measured = reading_bme280 != NULL || reading_ltr390 != NULL;
    if (!measured) {
        ESP_LOGE(__tag_iapp, "sensors: nothing answered this cycle");
        flags |= (uint8_t)(1U << VSUITE_FLAG_SENSOR_FAULTS);
    }

    /* --- battery --- */
    if (battery_present && battery_reading == NULL)
        flags |= (uint8_t)(1U << VSUITE_FLAG_SENSOR_FAULTS);
    if (battery_reading != NULL) {
        power_sample(battery_reading, reading_bme280 ? &reading_bme280->temperature_c : NULL, cycle, battery_mv_was, battery_reading->percent);
        if (battery_reading->percent <= BATTERY_PCT_LOW)
            flags |= (uint8_t)(1U << VSUITE_FLAG_BATTERY_DRAINING);
    }

    if (packet_build(as, node, buf, len, transmit, iotdata_node_station(node), iotdata_node_sequence(node), reading_bme280, reading_ltr390, battery_reading, flags, receive))
        ESP_LOGI(__tag_iapp, "packet: variant=%s station=%03" PRIX16 " sequence=%" PRIu16 " flags=0x%02" PRIX8 "%s", iotdata_vsuite_name(PACKET_VARIANT), iotdata_node_station(node), iotdata_node_sequence(node), flags,
                 receive ? " +receive" : "");
    else
        *transmit = 0;

    return IOTD_EV_OK;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

void app_init_log__timing_and_build_info(__attribute__((unused)) app_state_t *const as, __attribute__((unused)) iotdata_node_t *const node, char *const buf, const size_t len) {
    snprintf(buf, len,
             "transmit every %us"
#if defined(BENCH_BUSY_SLEEP)
             " (BENCH_BUSY_SLEEP -- NO DEEP SLEEP, DO NOT SHIP)"
#endif
             ,
             (unsigned)(TRANSMIT_PERIOD_MS / 1000));
}

iotd_event_t app_init_rtc__add_sensors_to_caps(app_state_t *const as, iotdata_node_t *const node, iotdata_node_version_caps_t *const caps) {
    if ((as->operating->bme280_present = (bme280_setup(app_bme280_config()) == ESP_OK)))
        iotdata_node_version_caps_add(caps, IOTDATA_NODE_VERSION_CAPS_SENSOR, IOTDATA_NODE_VERSION_SENSOR_BME280);
    if ((as->operating->ltr390_present = (ltr390_setup(&ltr390_config_default) == ESP_OK)))
        iotdata_node_version_caps_add(caps, IOTDATA_NODE_VERSION_CAPS_SENSOR, IOTDATA_NODE_VERSION_SENSOR_LTR390);
    return IOTD_EV_OK;
}

iotd_event_t app_sleep__delay(__attribute__((unused)) app_state_t *const as, __attribute__((unused)) iotdata_node_t *const node, uint32_t *const sleep_ms, __attribute__((unused)) const uint32_t start_ms, const uint32_t awake_ms,
                              __attribute__((unused)) const bool deep) {
    *sleep_ms = MAX_INT(TRANSMIT_PERIOD_MS - awake_ms, TRANSMIT_PERIOD_MIN_MS);
    return IOTD_EV_OK;
}

iotd_event_t app_exec__sleep_style(__attribute__((unused)) app_state_t *const as, __attribute__((unused)) iotdata_node_t *const node, __attribute__((unused)) const uint32_t start_ms) {
#ifdef BENCH_BUSY_SLEEP
    return IOTD_EV_SLEEP_LIGHT;
#else
    return IOTD_EV_SLEEP_DEEP;
#endif
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static const app_descr_t _app_spec = {
    .name = "lora/esp32 sensor[bme280|ltr390]",
    .variant = PACKET_VARIANT,
    .func_init_log = app_init_log__timing_and_build_info,
    .func_init_rtc = app_init_rtc__add_sensors_to_caps,
    .func_sleep = app_sleep__delay,
    .func_exec_packet = app_exec__build_sensor_packet,
    .func_exec_final = app_exec__sleep_style,
};

const app_descr_t *app_spec = &_app_spec;

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
