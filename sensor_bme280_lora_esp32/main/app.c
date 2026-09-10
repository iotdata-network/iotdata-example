// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

/*
 * IoT Sensor Telemetry Protocol
 * Copyright(C) 2026 Matthew Gream (https://libiotdata.org)
 *
 * sensor_bme280_lora.c - BME280 weather sensor on esp32
 *
 * Reads a Bosch BME280 (temperature / pressure / humidity) over I2C, encodes
 * the reading as an iotdata packet using the "weather_station" variant,
 * transmits it via the E22 LoRa radio module, then deep sleeps until the next
 * cycle — one transmission every TX_PERIOD_MS.
 *
 * Every wake is a complete, self-contained cycle: measure, transmit, sleep.
 * Nothing carries over except a small block of RTC-retained state (station id,
 * packet sequence, BME280 factory calibration), and that block is purely a
 * cache — anything missing is re-acquired on the next wake. So a cycle can be
 * repeated, or missed, without disturbing the ones around it.
 *
 * Note that deep sleep takes the USB-Serial-JTAG console down with it, so a
 * board watched over USB drops and re-enumerates its serial port once a minute;
 * that is the sleep working, not a fault.
 *
 * Depends upon EBYTE E22 connector
 * https://github.com/matthewgream/e22900t22u
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

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_rom_sys.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#pragma GCC diagnostic pop

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include "iotdata_config.h"
#include "iotdata_hardware.h"

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define TX_PERIOD_MS     IOTDATA_CONFIG_SENSOR_TX_PERIOD_MS /* one measure+transmit cycle per period */
#define TX_PERIOD_MIN_MS 1000                               /* floor, if a cycle overruns the period  */
#define STARTUP_DELAY_MS (5 * 1000)                         /* cold boot only: let the USB console attach */
#define CONSOLE_DRAIN_MS 100                                /* let the console flush before deep sleep    */
#define PACKET_MAX       64                                 /* the packets built here are a dozen bytes   */

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static const char *__tag_app = "app";

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

/* Fixed-point rendering for logs: 2135 -> "21.35". Avoids pulling in float printf. */
#define CENTI_STR_MAX 16
static const char *centi_str(char *const buf, const size_t size, const int32_t centi) {
    const int32_t abs = centi < 0 ? -centi : centi;
    (void)snprintf(buf, size, "%s%" PRId32 ".%02" PRId32, centi < 0 ? "-" : "", abs / 100, abs % 100);
    return buf;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include "d_common.h"

#define PIN_DEVICE_UART_TX  PIN_E22_RXD
#define PIN_DEVICE_UART_RX  PIN_E22_TXD
#define PIN_DEVICE_LORA_AUX PIN_E22_AUX
#define PIN_DEVICE_LORA_M0  PIN_E22_M0
#define PIN_DEVICE_LORA_M1  PIN_E22_M1

#include "d_hardware_gpio.h"
#include "d_hardware_uart.h"
#include "d_interface_e22900t22.h"

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

#define PIN_DEVICE_I2C_SDA PIN_BME280_SDA
#define PIN_DEVICE_I2C_SCL PIN_BME280_SCL

#include "d_hardware_i2c.h"
#include "d_readings.h"
#include "d_interface_bme280.h"
#include "d_interface_batt.h"

// -----------------------------------------------------------------------------------------------------------------------------------------
// iotdata — variant suite (unity build, encode-only)
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
#include "iotdata.c"
#include "iotdata_variant.h"
#include "iotdata_node.h"
#include "iotdata_node_endpoint.h"

// -----------------------------------------------------------------------------------------------------------------------------------------

/*
 * The weather_station variant is the full outdoor station map — battery, link,
 * environment, wind, rain, solar, and more. This node populates environment
 * (temperature + pressure + humidity), which is all a BME280 has to offer, and
 * battery on the carriers that have a divider fitted; presence bits mean the
 * unused slots cost nothing on air — a node without a divider sends the same
 * packet it always did — and the same station can later grow wind/rain/solar
 * without changing the variant or the gateway. Flags are sent only when
 * something is worth reporting.
 */
#define PACKET_VARIANT IOTDATA_VSUITE_WEATHER_STATION

typedef struct {
    uint8_t buf[PACKET_MAX];
    size_t len;
} packet_t;

static void sensor_node_status(const uint16_t station, iotdata_kvr_t *const kv);
static bool sensor_node_tx(const uint8_t *const packet, const size_t len);
static void sensor_receive_window(void); /* defined below app_cycle, which opens it */

static const idep_version_t sensor_version = {
    .firmware = "0.01",
    .application = "iotdata_sensor_bme280",
    .platform = CONFIG_IDF_TARGET,
    .build = __DATE__,
};

static const idep_config_t idep_cfg = {
    .version = &sensor_version,
    .status = sensor_node_status,
    .tx = sensor_node_tx,
    .receive_every_ms = IDEP_RECEIVE_EVERY_MS,
    .receive_window_ms = IDEP_RECEIVE_WINDOW_MS,
};

static bool packet_build(packet_t *const out, const uint16_t station, const uint16_t sequence, const bme280_reading_t *const reading, const battery_reading_t *const battery, uint8_t flags, const bool advertise_receive) {

    static iotdata_encoder_t enc;

    iotdata_status_t rc;
    if ((rc = iotdata_encode_begin(&enc, out->buf, sizeof(out->buf), PACKET_VARIANT, station, sequence)) != IOTDATA_OK) {
        ESP_LOGE(__tag_app, "encode_begin: %s", iotdata_strerror(rc));
        return false;
    }

    if (battery != NULL && (rc = iotdata_encode_battery(&enc, battery->percent, battery->charging)) != IOTDATA_OK)
        ESP_LOGW(__tag_app, "encode_battery: %s", iotdata_strerror(rc));

    if (reading != NULL) {
        /* Rounded to the units the field carries: whole hPa, whole %RH. */
        /* The driver reports floats; the encoder is built NO_FLOATING and takes centi-units, so
           the conversion happens here -- once, at the boundary, rather than through the app. */
        const iotdata_float_t temperature_c100 = (iotdata_float_t)lroundf(reading->temperature_c * 100.0f);
        const uint16_t pressure_hpa = (uint16_t)lroundf(reading->pressure_hpa);
        const uint8_t humidity_pct = (uint8_t)lroundf(reading->humidity_pct);
        if ((rc = iotdata_encode_environment(&enc, temperature_c100, pressure_hpa, humidity_pct)) != IOTDATA_OK) {
            /* In range for the sensor but out of range for the protocol: report the fault rather than an implausible value. */
            ESP_LOGW(__tag_app, "encode_environment: %s", iotdata_strerror(rc));
            flags |= (uint8_t)(1U << VSUITE_FLAG_SENSOR_FAULTS);
        }
    }

    if (flags != 0 && (rc = iotdata_encode_flags(&enc, flags)) != IOTDATA_OK)
        ESP_LOGW(__tag_app, "encode_flags: %s", iotdata_strerror(rc));

    if (advertise_receive && !idep_receive_append(&idep_cfg, &enc))
        ESP_LOGW(__tag_app, "encode_receive: no room");

    if ((rc = iotdata_encode_end(&enc, &out->len)) != IOTDATA_OK) {
        ESP_LOGE(__tag_app, "encode_end: %s", iotdata_strerror(rc));
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define STATE_MAGIC 0xB1E28002U

typedef struct {
    uint32_t magic;
    uint16_t station_id;    /* derived from the factory MAC: stable per board  */
    uint16_t sequence;      /* rolling packet counter, skips the reserved 0xFFFF */
    idep_node_t node;       /* node personality: sequence, receive window, stats */
    uint32_t cycles;        /* wake cycles since the last restart              */
    uint32_t tx_count;      /* packets transmitted                             */
    uint32_t tx_errors;     /* packets the radio would not take                */
    int16_t battery_mv;     /* last reading, for the charging trend bit        */
    bool radio_configured;  /* E22 NVM configuration verified this power cycle */
    bool sensor_calibrated; /* bme280_setup succeeded this power cycle         */
    bool battery_present;   /* a divider answered the probe at restart         */
} state_t;

static RTC_NOINIT_ATTR state_t state;

static uint16_t state_station_id(void) {
    uint8_t mac[6] = { 0 };
    (void)esp_efuse_mac_get_default(mac);
    const uint32_t mac32 = ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8) | mac[5];
    const uint16_t station_id = iotdata_station_from_id(mac32); /* 1..4094: never 0, never broadcast */
    ESP_LOGI(__tag_app, "board: mac=%02X:%02X:%02X:%02X:%02X:%02X station=%" PRIu16, (unsigned)mac[0], (unsigned)mac[1], (unsigned)mac[2], (unsigned)mac[3], (unsigned)mac[4], (unsigned)mac[5], station_id);
    return station_id;
}

static void state_reset(void) {
    memset(&state, 0, sizeof(state));
    state.magic = STATE_MAGIC;
    state.station_id = state_station_id();
    idep_node_init(&state.node, state.station_id);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#if IOTDATA_CONFIG_BLACKBOX
#ifndef IOTDATA_BLACKBOX_POOL_SZ
#define IOTDATA_BLACKBOX_POOL_SZ 1024u
#endif
#if IOTDATA_CONFIG_BLACKBOX >= 2
#define BLACKBOX_PERSIST BLACKBOX_PERSIST_ESP_FLASH
#else
#define BLACKBOX_PERSIST BLACKBOX_PERSIST_NONE
#endif
#define IOTDATA_BLACKBOX_IMPLEMENTATION
#include "iotdata_blackbox.h"
static RTC_NOINIT_ATTR char blackbox_pool[IOTDATA_BLACKBOX_POOL_SZ];
static blackbox_handle_t blackbox;
static const blackbox_config_t blackbox_config = {
    .pool = blackbox_pool,
    .pool_sz = sizeof(blackbox_pool),
    .flush = BLACKBOX_FLUSH_MANUAL, /* flushed explicitly before sleep; a no-op under PERSIST_NONE */
    .persist_arg = "diag",          /* ESP_FLASH: the partition label; ignored by PERSIST_NONE */
    .enabled = true,                /* compiled in == collecting; the compile-time knob is the gate */
};

static void blackbox_start(const esp_reset_reason_t reason, const bool restarted) {
    iotdata_blackbox_begin();
    if (blackbox_init(&blackbox, &blackbox_config) != 0) {
        ESP_LOGW(__tag_app, "blackbox: init failed -- diagnostics disabled");
        return;
    }
    (void)iotdata_blackbox_lifecycle(&blackbox, restarted ? IOTDATA_BB_LC_BOOT : IOTDATA_BB_LC_WAKE, (uint8_t)reason);
}
#define BLACKBOX_START(reason, restarted) blackbox_start((reason), (restarted))
#define BLACKBOX_EVENT(ev, reason)        (void)iotdata_blackbox_lifecycle(&blackbox, (ev), (uint8_t)(reason))
#define BLACKBOX_FLUSH()                  (void)blackbox_flush(&blackbox)
#else
#define BLACKBOX_START(reason, restarted) ((void)0)
#define BLACKBOX_EVENT(ev, reason)        ((void)0)
#define BLACKBOX_FLUSH()                  ((void)0)
#endif

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static uint8_t sensor_node_reason(void) {
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:
        return IOTDATA_NODE_REASON_POWER_ON;
    case ESP_RST_SW:
        return IOTDATA_NODE_REASON_SOFTWARE;
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
        return IOTDATA_NODE_REASON_WATCHDOG;
    case ESP_RST_BROWNOUT:
        return IOTDATA_NODE_REASON_BROWNOUT;
    case ESP_RST_PANIC:
        return IOTDATA_NODE_REASON_PANIC;
    case ESP_RST_DEEPSLEEP:
        return IOTDATA_NODE_REASON_DEEPSLEEP;
    case ESP_RST_EXT:
        return IOTDATA_NODE_REASON_EXTERNAL;
    default:
        return IOTDATA_NODE_REASON_UNKNOWN;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static void sensor_node_status(__attribute__((unused)) const uint16_t station, iotdata_kvr_t *const kv) {
    iotdata_kvr_add_u32(kv, IOTDATA_NODE_STATUS_UPTIME, (uint32_t)(esp_timer_get_time() / 1000000));
    iotdata_kvr_add_u8(kv, IOTDATA_NODE_STATUS_REASON, sensor_node_reason());
    iotdata_kvr_add_u32(kv, IOTDATA_NODE_STATUS_HEAP_FREE, (uint32_t)esp_get_free_heap_size());
    iotdata_kvr_add_u32(kv, IOTDATA_NODE_STATUS_HEAP_MIN, (uint32_t)esp_get_minimum_free_heap_size());
    iotdata_kvr_add_u16(kv, IOTDATA_NODE_STATUS_RESTARTS, (uint16_t)state.cycles);
    if (state.battery_present && state.battery_mv > 0)
        iotdata_kvr_add_u16(kv, IOTDATA_NODE_STATUS_SUPPLY, (uint16_t)state.battery_mv);
}

static bool sensor_node_tx(const uint8_t *const packet, const size_t len) {
    return lora_write_complete(packet, len, /*wait_complete=*/true) == ESP_OK;
}

static void sensor_receive_window(void) {

    const uint32_t opened = (uint32_t)(esp_timer_get_time() / 1000);
    idep_window_begin(&idep_cfg, &state.node, opened);
    ESP_LOGI(__tag_app, "receive: window open for %ums (station=%" PRIu16 ")", (unsigned)IDEP_RECEIVE_WINDOW_MS, state.station_id);
    BLACKBOX_EVENT(IOTDATA_BB_LC_WAKE, 0);

    bool reboot = false;
    unsigned frames = 0, acted = 0;
    while (idep_window_active(&state.node, (uint32_t)(esp_timer_get_time() / 1000))) {
        uint8_t buf[IDEP_PACKET_MAX];
        int len = 0, rssi_dbm = 0;
        /* 0ms first-byte timeout: this polls inside the window loop rather than blocking in it. */
        if (lora_read(buf, sizeof(buf), &len, &rssi_dbm, 0) == ESP_OK && len > 0) {
            frames++;
            if (idep_on_frame(&idep_cfg, &state.node, buf, (size_t)len, &reboot))
                acted++;
        }
        __SLEEP_MS(10); /* yield and pat the watchdog: the window is long by MCU standards */
    }
    idep_window_end(&state.node);
    ESP_LOGI(__tag_app, "receive: window closed (%u frame(s) heard, %u for us)", frames, acted);

    if (reboot) {
        ESP_LOGW(__tag_app, "node: REBOOT commanded -- restarting");
        BLACKBOX_EVENT(IOTDATA_BB_LC_STOP, 0);
        BLACKBOX_FLUSH();
        __SLEEP_MS(CONSOLE_DRAIN_MS);
        esp_restart();
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool app_cycle(void) {

    uint8_t flags = 0;
    if (state.cycles == 0)
        flags |= (uint8_t)(1U << VSUITE_FLAG_RESTART_RECENT);

    /* --- sensor ---
     *
     * bme280_setup reads the chip id and the factory calibration and keeps them in RTC memory, so
     * like the radio it only needs doing once per power cycle and survives deep sleep. Each wake
     * then goes straight to start/read/stop. */
    bme280_reading_t reading;
    bool measured = false;
    if (!state.sensor_calibrated)
        state.sensor_calibrated = bme280_setup(&bme280_config_default) == ESP_OK;
    if (state.sensor_calibrated && bme280_start() == ESP_OK) {
        measured = bme280_read(&reading, 0, NULL) == ESP_OK;
        (void)bme280_stop();
    }
    if (!measured) {
        ESP_LOGE(__tag_app, "sensor: no reading this cycle");
        flags |= (uint8_t)(1U << VSUITE_FLAG_SENSOR_FAULTS);
    }

    /* --- battery --- */
    battery_reading_t battery = { 0 };
    bool powered = false;
    if (state.battery_present) {
        if (battery_begin())
            powered = battery_read(&battery, &state.battery_mv);
        battery_end();
        if (!powered) {
            ESP_LOGE(__tag_app, "battery: no reading this cycle");
            flags |= (uint8_t)(1U << VSUITE_FLAG_SENSOR_FAULTS);
        } else if (battery.percent <= BATTERY_PCT_LOW)
            flags |= (uint8_t)(1U << VSUITE_FLAG_BATTERY_DRAINING);
    }

    /* --- packet --- */
    packet_t packet;
    const bool advertise = idep_window_advance(&idep_cfg, &state.node, TX_PERIOD_MS);
    if (!packet_build(&packet, state.station_id, state.sequence, measured ? &reading : NULL, powered ? &battery : NULL, flags, advertise))
        return false;
    ESP_LOGI(__tag_app, "packet: variant=%s station=%" PRIu16 " sequence=%" PRIu16 " flags=0x%02" PRIX8 "%s", iotdata_vsuite_name(PACKET_VARIANT), state.station_id, state.sequence, flags, advertise ? " +receive" : "");
    state.sequence = iotdata_sequence_next(state.sequence);

    /* --- radio ---
     *
     * lora_setup writes and verifies the module's register block; it only needs doing once per
     * power cycle, since the E22 holds that config in its own NVM and the driver keeps its copy in
     * RTC memory (which survives deep sleep). Every wake after that goes straight to lora_start,
     * saving a second or so of command traffic. lora_stop parks the module. */
    bool transmitted = false;
    bool ready = true;
    if (!state.radio_configured) {
        if (lora_setup(&lora_cfg) != ESP_OK)
            ready = false;
        else
            state.radio_configured = true;
    }
    if (ready && lora_start() != ESP_OK)
        ready = false;
    if (ready) {
        transmitted = lora_write_complete(packet.buf, packet.len, /*wait_complete=*/true) == ESP_OK;
        if (transmitted && advertise)
            sensor_receive_window();
    }
    (void)lora_stop(); /* the module must not be left awake for the sleep ahead */

    if (transmitted)
        state.tx_count++;
    else
        state.tx_errors++;
    return transmitted && measured;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void app_sleep(const int64_t time_start_us) {

    const int64_t awake_ms = (esp_timer_get_time() - time_start_us) / 1000;
    int64_t sleep_ms = (int64_t)TX_PERIOD_MS - awake_ms - CONSOLE_DRAIN_MS;
    if (sleep_ms < TX_PERIOD_MIN_MS)
        sleep_ms = TX_PERIOD_MIN_MS;

    ESP_LOGI(__tag_app, "cycle %" PRIu32 " done: tx=%" PRIu32 " errors=%" PRIu32 " awake=%" PRId64 "ms, sleeping %" PRId64 "ms", state.cycles, state.tx_count, state.tx_errors, awake_ms, sleep_ms);

    BLACKBOX_EVENT(IOTDATA_BB_LC_SLEEP, (uint8_t)(sleep_ms / 1000));
    BLACKBOX_FLUSH(); /* one flash append per cycle when persisting; a no-op in RAM-only mode */

    /* The USB-Serial-JTAG console goes down with the chip and takes anything still
       queued with it, so give the host a moment to collect this cycle's output. */
    __SLEEP_MS(CONSOLE_DRAIN_MS);

    lora_hold();
    esp_deep_sleep((uint64_t)sleep_ms * 1000);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void app_main(void) {

    const int64_t time_start = esp_timer_get_time();

    setbuf(stdout, NULL);

    const esp_err_t err = esp_task_wdt_add(NULL);
    if (err != ESP_OK)
        ESP_LOGE(__tag_app, "task watchdog: subscribe failed: %s", esp_err_to_name(err));

    const esp_reset_reason_t reset_reason = esp_reset_reason();
    const bool restarted = (reset_reason != ESP_RST_DEEPSLEEP) || state.magic != STATE_MAGIC;

    BLACKBOX_START(reset_reason, restarted);

    if (restarted) {
        ESP_LOGI(__tag_app, "iotdata bme280 lora sensor: %s variant, every %us", iotdata_vsuite_name(PACKET_VARIANT), (unsigned)(TX_PERIOD_MS / 1000));
        ESP_LOGI(__tag_app, "boot: reset_reason=%d %s", (int)reset_reason, reset_reason_str(reset_reason));
        state_reset();
        state.battery_present = battery_probe(); /* restart only; see battery_probe() */
        __SLEEP_MS(STARTUP_DELAY_MS);            /* cold boot only, so it costs nothing per cycle */
    }

    if (!app_cycle()) {
        ESP_LOGE(__tag_app, "cycle %" PRIu32 " incomplete", state.cycles);
        BLACKBOX_EVENT(IOTDATA_BB_LC_ERROR, state.tx_errors);
    }
    state.cycles++;

    app_sleep(time_start);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
