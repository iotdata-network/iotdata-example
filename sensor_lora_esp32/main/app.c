
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

/*
 * IoT Sensor Telemetry Protocol
 * Copyright(C) 2026 Matthew Gream (https://libiotdata.org)
 *
 * BME280/LTR390 weather sensor on esp32
 *
 */

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include "iotdata_config.h"
#include "iotdata_hardware.h"

// AE GEN LORA I2C --> BME280/LTR390
// (A) SDA -> (4) SDA
// (B) SCL -> (3) SCL
// (C) VCC -> (1) VCC + (5) CSB
// (D) GND -> (2) GND + (6) SD0

// -----------------------------------------------------------------------------------------------------------------------------------------

typedef struct {
    bool bme280_present; /* bme280_setup succeeded this power cycle */
    bool ltr390_present; /* ltr390_setup succeeded this power cycle */
} app_operating_t;

typedef struct {
} app_running_t;

#define APP_CONFIG_ENTRIES(X) X(TRANSMIT_PERIOD_S, 0x001, U16, 10, 43200, (IOTDATA_NODE_CONFIG_SENSOR_TRANSMIT_PERIOD_MS / 1000), 0, NULL, NULL, "how often a reading goes out")

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include "app_template_simple_lora_esp32.c"

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

static const char *__tag_app = "app";

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#if !defined(PIN_DEVICE_I2C_SDA) || !defined(PIN_DEVICE_I2C_SCL)
#error "must define all of PIN_DEVICE_I2C_SDA/PIN_DEVICE_I2C_SCL"
#endif
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

    if (advertise_receive && !iotdata_node_receive_append(node, &enc))
        ESP_LOGW(__tag_app, "encode_receive: no room");

    if ((rc = iotdata_encode_end(&enc, out)) != IOTDATA_OK) {
        ESP_LOGE(__tag_app, "encode_end: %s", iotdata_strerror(rc));
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static uint16_t app_board_id(void) {
    static uint16_t cached = 0u;
    return cached == 0u ? (cached = iotdata_node_station_from_mac(__tag_app)) : cached;
}

#define QUIRK_BOARD_BME280_ADDR_77 0x769

static const bme280_config_t *app_bme280_config(void) {
    static bme280_config_t config;
    config = bme280_config_default;
    if (app_board_id() == QUIRK_BOARD_BME280_ADDR_77) {
        config.i2c_addr = 0x77;
        ESP_LOGW(__tag_app, "bme280: quirk for board %03X -- i2c address 0x%02X, not 0x%02X", QUIRK_BOARD_BME280_ADDR_77, config.i2c_addr, BME280_I2C_ADDR_DEFAULT);
    }
    return &config;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

iapp_event_t app_exec__build_sensor_packet(app_state_t *const as, iotdata_node_t *const node, const uint32_t cycle, const bool battery_present, const battery_reading_t *const battery_reading, const int16_t battery_mv_was,
                                           uint8_t *const buf, const size_t len, const bool receive, size_t *const transmit) {

    uint8_t flags = 0;
    if (cycle == 0)
        flags |= (uint8_t)(1U << VSUITE_FLAG_RESTART_RECENT);

    /* --- sensor: bme280 (enviro) ---  */
    if (!as->operating->bme280_present && cycle == 0)
        if (!(as->operating->bme280_present = (bme280_setup(app_bme280_config()) == ESP_OK)))
            bme280_diagnose();
    bme280_reading_t reading_bme280_, *reading_bme280 = NULL;
    if (as->operating->bme280_present) {
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
    if (!as->operating->ltr390_present && cycle == 0)
        if (!(as->operating->ltr390_present = (ltr390_setup(&ltr390_config_default) == ESP_OK)))
            ltr390_diagnose();
    ltr390_reading_t reading_ltr390_, *reading_ltr390 = NULL;
    if (as->operating->ltr390_present) {
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
    if (battery_present && battery_reading == NULL)
        flags |= (uint8_t)(1U << VSUITE_FLAG_SENSOR_FAULTS);
    if (battery_reading != NULL) {
        power_sample(battery_reading, reading_bme280 ? &reading_bme280->temperature_c : NULL, cycle, battery_mv_was, battery_reading->percent);
        if (battery_reading->percent <= BATTERY_PCT_LOW)
            flags |= (uint8_t)(1U << VSUITE_FLAG_BATTERY_DRAINING);
    }

    if (packet_build(as, node, buf, len, transmit, iotdata_node_station(node), iotdata_node_sequence(node), reading_bme280, reading_ltr390, battery_reading, flags, receive))
        ESP_LOGI(__tag_app, "packet: variant=%s station=%03" PRIX16 " sequence=%" PRIu16 " flags=0x%02" PRIX8 "%s", iotdata_vsuite_name(PACKET_VARIANT), iotdata_node_station(node), iotdata_node_sequence(node), flags,
                 receive ? " +receive" : "");
    else
        *transmit = 0;

    return IAPP_EV_OK;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

void app_init_log__timing_and_build_info(__attribute__((unused)) app_state_t *const as, __attribute__((unused)) iotdata_node_t *const node, char *const buf, const size_t len) {
    snprintf(buf, len,
             "transmit every %us"
#if defined(NO_SLEEP_DEEP)
             " (NO_SLEEP_DEEP -- DO NOT SHIP)"
#endif
             ,
             (unsigned)(TRANSMIT_PERIOD_MS / 1000));
}

iapp_event_t app_init_rtc__add_sensors_to_caps(app_state_t *const as, iotdata_node_t *const node, iotdata_node_version_caps_t *const caps) {
    if ((as->operating->bme280_present = (bme280_setup(app_bme280_config()) == ESP_OK)))
        iotdata_node_version_caps_add(caps, IOTDATA_NODE_VERSION_CAPS_SENSOR, IOTDATA_NODE_VERSION_SENSOR_BME280);
    if ((as->operating->ltr390_present = (ltr390_setup(&ltr390_config_default) == ESP_OK)))
        iotdata_node_version_caps_add(caps, IOTDATA_NODE_VERSION_CAPS_SENSOR, IOTDATA_NODE_VERSION_SENSOR_LTR390);
    return IAPP_EV_OK;
}

iapp_event_t app_sleep__delay(__attribute__((unused)) app_state_t *const as, __attribute__((unused)) iotdata_node_t *const node, uint32_t *const sleep_ms, __attribute__((unused)) const uint32_t start_ms, const uint32_t awake_ms,
                              __attribute__((unused)) const bool deep) {
    *sleep_ms = MAX_INT(TRANSMIT_PERIOD_MS - awake_ms, TRANSMIT_PERIOD_MIN_MS);
    return IAPP_EV_OK;
}

iapp_event_t app_exec__sleep_style(__attribute__((unused)) app_state_t *const as, __attribute__((unused)) iotdata_node_t *const node, __attribute__((unused)) const uint32_t start_ms) {
#ifdef NO_SLEEP_DEEP
    return IAPP_EV_SLEEP_LIGHT;
#else
    return IAPP_EV_SLEEP_DEEP;
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
