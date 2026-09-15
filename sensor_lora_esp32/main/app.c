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
 * E22 LoRa radio module, then sleeps until the next cycle — every TX_PERIOD_MS.
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

#define TX_PERIOD_MS        IOTDATA_CONFIG_SENSOR_TX_PERIOD_MS /* one measure+transmit cycle per period */
#define TX_PERIOD_MIN_MS    1000                               /* floor, if a cycle overruns the period  */
#define CONSOLE_DRAIN_MS    100                                /* let the console flush before deep sleep    */
#define RECEIVE_WINDOW_MS   10
#define RESTART_MS          30000
#define PACKET_MAX          64 /* the packets built here are a dozen bytes   */
#define PACKET_VARIANT      IOTDATA_VSUITE_WEATHER_STATION

// -----------------------------------------------------------------------------------------------------------------------------------------
// HARDWARE
// -----------------------------------------------------------------------------------------------------------------------------------------

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
#define PIN_DEVICE_I2C_SDA  PIN_I2C_SDA
#define PIN_DEVICE_I2C_SCL  PIN_I2C_SCL

#include "d_platform_esp32.h"
#include "d_common.h"
#include "d_format.h"
#include "d_readings.h"
#include "d_hardware_gpio.h"
#include "d_hardware_uart.h"
#include "d_hardware_i2c.h"
#include "d_interface_e22900t22.h"
#include "d_interface_bme280.h"
#include "d_interface_ltr390.h"
#include "d_interface_batt.h" // XXX does not use d_hardware_adc.h
#include "d_module_datastore_esp32.h"

static const lora_config_t lora_cfg = {
    .e22_address = IOTDATA_CONFIG_LORA_ADDRESS,
    .e22_network = IOTDATA_CONFIG_LORA_NETWORK,
    .channel = IOTDATA_CONFIG_LORA_CHANNEL,
    .transmit_power = 22,  // dBm
    .air_data_rate = 2400, // bps
    .packet_size = 240,
    .listen_before_transmit = true,
    .crypt = 0x0000,
    .module = LORA_MODULE_DIP,
    .rssi_packet = true,
    .rssi_channel = false,
};

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
#include "iotdata_node_version.h"
// #include "iotdata_node_variant.h"
#include "iotdata_node_status.h"
// #include "iotdata_node_config.h"
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
#include "iotdata_node_platform.h"

// -----------------------------------------------------------------------------------------------------------------------------------------

static void sensor_node_status(const uint16_t station, iotdata_node_status_t *const out);
static bool sensor_node_transmit(const uint8_t *const packet, const size_t len);

// -----------------------------------------------------------------------------------------------------------------------------------------
// APP STATE
// -----------------------------------------------------------------------------------------------------------------------------------------

static const char *__tag_app = "app";

#define STATE_MAGIC 0xB1E28003UL
#define STATE_TAG   0xB1E28004UL

typedef struct {
    _RTC_DATA_STAMP_ENTRY;
    iotdata_version_caps_t caps;
    idep_node_t node;
    uint32_t cycles;      /* wake cycles since the last restart              */
    uint32_t tx_count;    /* packets transmitted                             */
    uint32_t tx_errors;   /* packets the radio would not take                */
    int16_t battery_mv;   /* last reading, for the charging trend bit        */
    bool lora_present;    /* E22 NVM configuration verified this power cycle */
    bool bme280_present;  /* bme280_setup succeeded this power cycle         */
    bool ltr390_present;  /* ltr390_setup succeeded this power cycle         */
    bool battery_present; /* a divider answered the probe at restart         */
} state_operating_t;

typedef struct {
    datastore_t datastore;
    iotdata_node_state_t node;
    const idep_config_t idep_cfg;
} state_running_t;

static _RTC_DATA_STRUCT state_operating_t _state_operating;
static state_running_t _state_running = { .idep_cfg = {
                                              .caps = &_state_operating.caps,
                                              .status = sensor_node_status,
                                              .tx = sensor_node_transmit,
                                              .control = IOTDATA_DIAGNOSTICS_CONTROL,
                                              .control_keys = iotdata_diagnostics_control_keys,
                                              .control_keys_count = IOTDATA_DIAGNOSTICS_CONTROL_KEYS_COUNT,
                                              .diag = IOTDATA_DIAGNOSTICS_PULL,
                                              .receive_every_ms = IDEP_RECEIVE_EVERY_MS,
                                              .receive_window_ms = IDEP_RECEIVE_WINDOW_MS,
                                              .packet_max = LORA_PACKET_SIZE_MAX,
                                          } };

static const struct {
    state_operating_t *const operating;
    state_running_t *const running;
} app_state = { .operating = &_state_operating, .running = &_state_running };

// -----------------------------------------------------------------------------------------------------------------------------------------
// IOTDATA COMMONS
// -----------------------------------------------------------------------------------------------------------------------------------------

static void sensor_node_status(__attribute__((unused)) const uint16_t station, iotdata_node_status_t *const out) {
    const state_operating_t *const s = app_state.operating;

    out->uptime_s = (uint32_t)(esp_timer_get_time() / 1000000);
    out->reason = iotdata_node_reason_reset();
    out->has_heap = true;
    out->heap_free = (uint32_t)esp_get_free_heap_size();
    out->heap_min = (uint32_t)esp_get_minimum_free_heap_size();
    out->has_restarts = true;
    out->restarts = (uint16_t)s->cycles;
    if (s->battery_present && s->battery_mv > 0) {
        out->has_supply = true;
        out->supply_mv = (uint16_t)s->battery_mv;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static bool sensor_node_transmit(const uint8_t *const packet, const size_t len) {
    return lora_write_complete(packet, len, /*wait_complete=*/true) == ESP_OK;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

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

static bool packet_build(uint8_t *const buf, size_t len, size_t *out, const uint16_t station, const uint16_t sequence, const bme280_reading_t *const reading_environment, const ltr390_reading_t *const reading_ltr390,
                         const battery_reading_t *const battery, uint8_t flags, const bool advertise_receive) {

    static iotdata_encoder_t enc;

    iotdata_status_t rc;
    if ((rc = iotdata_encode_begin(&enc, buf, len, PACKET_VARIANT, station, sequence)) != IOTDATA_OK) {
        ESP_LOGE(__tag_app, "encode_begin: %s", iotdata_strerror(rc));
        return false;
    }

    if (battery != NULL && (rc = iotdata_encode_battery(&enc, battery->percent, battery->charging)) != IOTDATA_OK)
        ESP_LOGW(__tag_app, "encode_battery: %s", iotdata_strerror(rc));

    if (reading_environment != NULL && (rc = iotdata_encode_environment(&enc, (iotdata_float_t)lroundf(reading_environment->temperature_c * 100.0f), (uint16_t)lroundf(reading_environment->pressure_hpa),
                                                                        (uint8_t)lroundf(reading_environment->humidity_pct))) != IOTDATA_OK) {
        ESP_LOGW(__tag_app, "encode_environment: %s", iotdata_strerror(rc));
        flags |= (uint8_t)(1U << VSUITE_FLAG_SENSOR_FAULTS);
    }

    if (reading_ltr390 != NULL && (rc = iotdata_encode_solar(&enc, ltr390_wm2_16bit(reading_ltr390->lux), ltr390_uvi_8bit(reading_ltr390->uvi))) != IOTDATA_OK) {
        ESP_LOGW(__tag_app, "encode_solar: %s", iotdata_strerror(rc));
        flags |= (uint8_t)(1U << VSUITE_FLAG_SENSOR_FAULTS);
    }

    if (flags != 0 && (rc = iotdata_encode_flags(&enc, flags)) != IOTDATA_OK)
        ESP_LOGW(__tag_app, "encode_flags: %s", iotdata_strerror(rc));

    if (advertise_receive && !idep_receive_append(&app_state.running->idep_cfg, &enc))
        ESP_LOGW(__tag_app, "encode_receive: no room");

    if ((rc = iotdata_encode_end(&enc, out)) != IOTDATA_OK) {
        ESP_LOGE(__tag_app, "encode_end: %s", iotdata_strerror(rc));
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

// app_state should be here

// -----------------------------------------------------------------------------------------------------------------------------------------

static void app_receive(void) {
    const idep_config_t *const c = &app_state.running->idep_cfg;
    idep_node_t *const n = &app_state.operating->node;

    ESP_LOGI(__tag_app, "receive: window opening for %us (station=%" PRIu16 ")", (unsigned)(IDEP_RECEIVE_WINDOW_MS / 1000), idep_station(n));
    idep_window_begin(c, n, (uint32_t)hw_time_ms());
    iotdata_diagnostics_event(IOTDATA_BB_LC_WAKE, 0);
    bool reboot = false;
    unsigned frames = 0, acted = 0;
    while (idep_window_active(c, n, (uint32_t)hw_time_ms())) {
        uint8_t buf[IOTDATA_MAX_PACKET_SIZE]; // XXX
        int len = 0, rssi_dbm = 0;
        if (lora_read(buf, sizeof(buf), &len, &rssi_dbm, 0) == ESP_OK && len > 0) {
            frames++;
            acted += idep_on_frame(c, n, buf, (size_t)len, &reboot) ? 1 : 0;
        }
        iotdata_diagnostics_pump();               /* a DUMP that arrived in this window drains inside it */
        hw_delay_ms_yieldable(RECEIVE_WINDOW_MS); /* yield and pat the watchdog: the window is long by MCU standards */
    }
    idep_window_end(n);
    ESP_LOGI(__tag_app, "receive: window closed (%u frame(s) heard, %u for us)", frames, acted);

    if (reboot) {
        ESP_LOGW(__tag_app, "node: REBOOT commanded -- restarting");
        iotdata_diagnostics_event(IOTDATA_BB_LC_STOP, 0);
        iotdata_diagnostics_flush();
        hw_delay_ms_yieldable(CONSOLE_DRAIN_MS);
        esp_restart();
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

size_t app_transmit_gather(uint8_t *const buf, const size_t len, bool advertise) {

    uint8_t flags = 0;
    if (app_state.operating->cycles == 0)
        flags |= (uint8_t)(1U << VSUITE_FLAG_RESTART_RECENT);

    /* --- sensor: bme280 (enviro) ---  */
    if (!app_state.operating->bme280_present) {
        app_state.operating->bme280_present = (bme280_setup(&bme280_config_default) == ESP_OK);
        if (!app_state.operating->bme280_present && app_state.operating->cycles == 0)
            bme280_diagnose();
    }
    bme280_reading_t reading_bme280_, *reading_bme280 = NULL;
    if (app_state.operating->bme280_present) {
        if (bme280_start() == ESP_OK) {
            reading_bme280 = bme280_read(&reading_bme280_, 0, NULL) == ESP_OK ? &reading_bme280_ : NULL;
            (void)bme280_stop();
        }
        if (!reading_bme280) {
            ESP_LOGE(__tag_app, "bme280: detected, but no reading this cycle");
            flags |= (uint8_t)(1U << VSUITE_FLAG_SENSOR_FAULTS);
        }
    }

    /* --- sensor: ltr390 (solar) ---  */
    if (!app_state.operating->ltr390_present) {
        app_state.operating->ltr390_present = (ltr390_setup(&ltr390_config_default) == ESP_OK);
        if (!app_state.operating->ltr390_present && app_state.operating->cycles == 0)
            ltr390_diagnose();
    }
    ltr390_reading_t reading_ltr390_, *reading_ltr390 = NULL;
    if (app_state.operating->ltr390_present) {
        if (ltr390_start() == ESP_OK) {
            reading_ltr390 = ltr390_read(&reading_ltr390_, 0, NULL) == ESP_OK ? &reading_ltr390_ : NULL;
            (void)ltr390_stop();
        }
        if (!reading_ltr390) {
            ESP_LOGE(__tag_app, "ltr390: detected, but no reading this cycle");
            flags |= (uint8_t)(1U << VSUITE_FLAG_SENSOR_FAULTS);
        }
    }

    const bool measured = reading_bme280 != NULL || reading_ltr390 != NULL;
    if (!measured) {
        ESP_LOGE(__tag_app, "sensors: nothing answered this cycle");
        flags |= (uint8_t)(1U << VSUITE_FLAG_SENSOR_FAULTS);
    }

    /* --- battery --- */
    battery_reading_t reading_battery_ = { 0 }, *reading_battery = NULL;
    if (app_state.operating->battery_present) {
        if (battery_begin())
            reading_battery = battery_read(&reading_battery_, &app_state.operating->battery_mv) ? &reading_battery_ : NULL;
        battery_end();
        if (!reading_battery) {
            ESP_LOGE(__tag_app, "battery: no reading this cycle");
            flags |= (uint8_t)(1U << VSUITE_FLAG_SENSOR_FAULTS);
        } else {
            if (reading_battery->percent <= BATTERY_PCT_LOW)
                flags |= (uint8_t)(1U << VSUITE_FLAG_BATTERY_DRAINING);
        }
    }

    size_t out;
    if (packet_build(buf, len, &out, idep_station(&app_state.operating->node), idep_sequence(&app_state.operating->node), reading_bme280, reading_ltr390, reading_battery, flags, advertise)) {
        ESP_LOGI(__tag_app, "packet: variant=%s station=%" PRIu16 " sequence=%" PRIu16 " flags=0x%02" PRIX8 "%s", iotdata_vsuite_name(PACKET_VARIANT), idep_station(&app_state.operating->node), idep_sequence(&app_state.operating->node),
                 flags, advertise ? " +receive" : "");
        return out;
    }

    return 0;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void app_cycle(void) {

    uint8_t buf[PACKET_MAX];

    const bool receive = idep_window_advance(&app_state.running->idep_cfg, &app_state.operating->node, TX_PERIOD_MS);
    const size_t transmit = app_transmit_gather(buf, sizeof(buf), receive);

    if (transmit || receive) {
        if (!app_state.operating->lora_present)
            app_state.operating->lora_present = lora_setup(&lora_cfg) == ESP_OK;
        const bool lora_started = app_state.operating->lora_present && lora_start() == ESP_OK;
        if (lora_started && transmit && lora_write_complete(buf, transmit, /*wait_complete=*/true) == ESP_OK) {
            idep_sequence_used(&app_state.operating->node);
            app_state.operating->tx_count++;
        } else {
            app_state.operating->tx_errors++;
            iotdata_diagnostics_event(IOTDATA_BB_LC_ERROR, iotdata_diagnostics_reason(app_state.operating->tx_errors));
        }
        if (lora_started && receive)
            app_receive();
        if (lora_started)
            lora_stop();
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

bool app_init(void) {

    const esp_reset_reason_t reset_reason = esp_reset_reason();
    const bool restarted = (reset_reason != ESP_RST_DEEPSLEEP);

    ESP_LOGI(__tag_app, "iotdata lora sensor+bme280/ltr390: %s variant, every %us", iotdata_vsuite_name(PACKET_VARIANT), (unsigned)(TX_PERIOD_MS / 1000));
    ESP_LOGI(__tag_app, "boot: reset_reason=%d %s", (int)reset_reason, reset_reason_str(reset_reason));
#if defined(BENCH_BUSY_SLEEP)
    ESP_LOGW(__tag_app, "boot: BENCH_BUSY_SLEEP -- NO DEEP SLEEP, DO NOT SHIP");
#endif

    iotdata_diagnostics_emit_set(_node_diagnostics_emit);
    if (!iotdata_diagnostics_begin((uint8_t)reset_reason, restarted))
        ESP_LOGW(__tag_app, "diag: the recorder did not start");
    iotdata_diagnostics_flush();

    if (!_RTC_DATA_VALID(app_state.operating, STATE_MAGIC)) {
        _RTC_DATA_INIT(app_state.operating, STATE_MAGIC);
        idep_node_init(&app_state.operating->node, iotdata_node_station_from_mac(__tag_app), NULL, 0u);
        iotdata_version_caps_init(&app_state.operating->caps);
        app_state.operating->battery_present = battery_probe();
        if ((app_state.operating->bme280_present = (bme280_setup(&bme280_config_default) == ESP_OK)))
            iotdata_version_caps_add(&app_state.operating->caps, IOTDATA_VERSION_CAP_SENSOR, IOTDATA_VERSION_SENSOR_BME280);
        if ((app_state.operating->ltr390_present = (ltr390_setup(&ltr390_config_default) == ESP_OK)))
            iotdata_version_caps_add(&app_state.operating->caps, IOTDATA_VERSION_CAP_SENSOR, IOTDATA_VERSION_SENSOR_LTR390);
    }

    char vbuf[IOTDATA_VERSION_STR_MAX + 1];
    ESP_LOGI(__tag_app, "version: %s", iotdata_version_str(vbuf, sizeof(vbuf), &app_state.operating->caps));
    if (!iotdata_version_stamp_is_real())
        ESP_LOGW(__tag_app, "version: build stamp is unset -- this binary cannot say when it was built");

    if (datastore_open(&app_state.running->datastore, "iotdata")) {
        iotdata_state_init(&app_state.running->node, &app_state.running->datastore, "state");
        if (!idep_node_bind(&app_state.operating->node, &app_state.running->node, STATE_TAG))
            ESP_LOGW(__tag_app, "state: persistence failed (bind STATE_TAG)");
        if (restarted) {
            const bool restored = iotdata_state_load(&app_state.running->node);
            ESP_LOGI(__tag_app, "state: %u block(s), %s", (unsigned)app_state.running->node.count, restored ? "restored" : "defaulted");
        }
    } else
        ESP_LOGW(__tag_app, "state: persistence disabled (no datastore)");

    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void app_sleep(const uint32_t start_ms) {
#define MAX_INT(a, b) ((a) > (b) ? (a) : (b))
    const uint32_t awake_ms = hw_time_ms() - start_ms;
    const uint32_t sleep_ms = MAX_INT(TX_PERIOD_MS - awake_ms - CONSOLE_DRAIN_MS, TX_PERIOD_MIN_MS);
    ESP_LOGI(__tag_app, "cycle %" PRIu32 " done: tx=%" PRIu32 " errors=%" PRIu32 " awake=%" PRIu32 "ms, sleeping %" PRIu32 "ms", app_state.operating->cycles, app_state.operating->tx_count, app_state.operating->tx_errors, awake_ms,
             sleep_ms);
    iotdata_diagnostics_event(IOTDATA_BB_LC_SLEEP, iotdata_diagnostics_reason(sleep_ms / 1000));
    iotdata_diagnostics_flush();
    hw_delay_ms_yieldable(CONSOLE_DRAIN_MS);
#if defined(BENCH_BUSY_SLEEP)
    hw_delay_ms_yieldable(sleep_ms);
#else
    lora_hold();
    datastore_close(&app_state.running->datastore);
    esp_deep_sleep((uint64_t)sleep_ms * 1000); /* does not return */
#endif
}

// -----------------------------------------------------------------------------------------------------------------------------------------

bool app_exec(void) {
    do {
        const uint32_t start_ms = hw_time_ms();
        app_cycle();
        app_state.operating->cycles++;
        iotdata_state_flush(&app_state.running->node);
        app_sleep(start_ms); // if deep sleep, will not return
    } while (true);
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
