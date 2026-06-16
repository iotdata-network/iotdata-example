/*
 * IoT Sensor Telemetry Protocol
 * Copyright(C) 2026 Matthew Gream (https://libiotdata.org)
 *
 * iotdata_variant_simulator.c - multi-sensor simulator implementation
 *
 * Internal state uses integer representations for clean RNG-based drift.
 * Converted to iotdata_float_t at the encoder boundary.
 *
 * May be utilised as a library, or build as a standalone tool (using
 * -DTEST_MAIN).
 */

#ifdef TEST_MAIN
#define IOTDATA_NO_JSON
#endif
#include "iotdata_variant_simulator.h"

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>

/* =========================================================================
 * RNG — xorshift32 (fast, deterministic, good enough for simulation)
 * ========================================================================= */

static inline uint32_t _rng(iotsim_t *sim) {
    uint32_t x = sim->rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    sim->rng_state = x;
    return x;
}

/* Uniform in [lo, hi] inclusive (rejection sampling to eliminate modulo bias) */
static inline int32_t _rng_range(iotsim_t *sim, int32_t lo, int32_t hi) {
    if (lo >= hi)
        return lo;
    uint32_t range = (uint32_t)(hi - lo + 1);
    uint32_t limit = (UINT32_MAX / range) * range;
    uint32_t r;
    do {
        r = _rng(sim);
    } while (r >= limit);
    return lo + (int32_t)(r % range);
}

/* Small signed jitter in [-mag, +mag] */
static inline int32_t _jitter(iotsim_t *sim, int32_t mag) {
    return _rng_range(sim, -mag, mag);
}

/* Clamp */
static inline int32_t _clamp(int32_t v, int32_t lo, int32_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* =========================================================================
 * Unit conversion helpers
 *
 * Internal sim state:
 *   temperature:  centi-degrees C (int16_t, e.g. 2150 = 21.50°C)
 *   wind speed:   centi-m/s       (uint16_t, e.g. 850 = 8.50 m/s)
 *   wind gust:    centi-m/s
 *   snr:          tenths of dB    (int16_t, e.g. 50 = 5.0 dB)
 *   rad_dose:     centi-µSv/h     (uint16_t, e.g. 10 = 0.10 µSv/h)
 *
 * iotdata encoder (with float): uses °C, m/s, dB, µSv/h directly.
 * iotdata encoder (no float):   uses centi-units (int32_t * 100).
 * ========================================================================= */

static inline iotdata_float_t _to_temp(int16_t centi_c) {
#if defined(IOTDATA_NO_FLOATING)
    return (iotdata_float_t)centi_c;
#else
    return (iotdata_float_t)(centi_c / 100.0f);
#endif
}

static inline iotdata_float_t _to_speed(uint16_t centi_ms) {
#if defined(IOTDATA_NO_FLOATING)
    return (iotdata_float_t)centi_ms;
#else
    return (iotdata_float_t)(centi_ms / 100.0f);
#endif
}

static inline iotdata_float_t _to_snr(int16_t tenths_db) {
#if defined(IOTDATA_NO_FLOATING)
    return (iotdata_float_t)(tenths_db * 10); /* tenths → centi for int32 */
#else
    return (iotdata_float_t)(tenths_db / 10.0f);
#endif
}

static inline iotdata_float_t _to_dose(uint16_t centi_usvh) {
#if defined(IOTDATA_NO_FLOATING)
    return (iotdata_float_t)centi_usvh;
#else
    return (iotdata_float_t)(centi_usvh / 100.0f);
#endif
}

/* =========================================================================
 * Sensor initialisation — realistic baseline per variant
 * ========================================================================= */

static void _init_common(iotsim_t *sim, iotsim_sensor_t *s) {
    s->battery = (uint8_t)_rng_range(sim, 40, 100);
    s->flags = 1;
    s->sequence = 0;
    s->tx_count = 0;
}

static void _init_sensor(iotsim_t *sim, iotsim_sensor_t *s) {
    _init_common(sim, s);

    switch (s->variant) {

    case IOTDATA_VSUITE_WEATHER_STATION:
        s->temperature = (int16_t)_rng_range(sim, 500, 3000); /* 5-30°C */
        s->pressure = (uint16_t)_rng_range(sim, 980, 1040);
        s->humidity = (uint8_t)_rng_range(sim, 30, 80);
        s->wind_speed = (uint16_t)_rng_range(sim, 0, 1500); /* 0-15 m/s */
        s->wind_dir = (uint16_t)_rng_range(sim, 0, 355);
        s->wind_gust = (uint16_t)_clamp(s->wind_speed + _rng_range(sim, 100, 500), 0, 6350);
        s->rain_rate = (_rng(sim) % 4 == 0) ? (uint8_t)_rng_range(sim, 1, 20) : 0;
        s->rain_size = s->rain_rate ? (uint8_t)_rng_range(sim, 2, 8) : 0;
        s->solar_irr = (uint16_t)_rng_range(sim, 0, 800);
        s->solar_uv = (uint8_t)_rng_range(sim, 0, 10);
        s->clouds = (uint8_t)_rng_range(sim, 0, 8);
        s->aq_index = (uint16_t)_rng_range(sim, 20, 150);
        s->rad_cpm = (uint16_t)_rng_range(sim, 10, 50);
        s->rad_dose = (uint16_t)_rng_range(sim, 5, 20); /* 0.05-0.20 µSv/h */
        break;

    case IOTDATA_VSUITE_AIR_QUALITY:
        s->temperature = (int16_t)_rng_range(sim, 1800, 2800); /* 18-28°C indoor */
        s->pressure = (uint16_t)_rng_range(sim, 990, 1030);
        s->humidity = (uint8_t)_rng_range(sim, 30, 65);
        s->aq_index = (uint16_t)_rng_range(sim, 20, 200);
        s->aq_pm_present = 0x0F;                          /* all four channels */
        s->aq_pm[0] = (uint16_t)_rng_range(sim, 5, 50);   /* PM1 */
        s->aq_pm[1] = (uint16_t)_rng_range(sim, 10, 80);  /* PM2.5 */
        s->aq_pm[2] = (uint16_t)_rng_range(sim, 15, 100); /* PM4 */
        s->aq_pm[3] = (uint16_t)_rng_range(sim, 20, 120); /* PM10 */
        /* SEN55-style: VOC + NOx */
        s->aq_gas_present = 0x03;
        s->aq_gas[0] = (uint16_t)_rng_range(sim, 50, 300); /* VOC idx */
        s->aq_gas[1] = (uint16_t)_rng_range(sim, 10, 100); /* NOx idx */
        /* ~30% chance of SEN66 (adds CO2) */
        if (_rng(sim) % 10 < 3) {
            s->aq_gas_present |= 0x04;
            s->aq_gas[2] = (uint16_t)_rng_range(sim, 400, 1200); /* CO2 ppm */
        }
        break;

    case IOTDATA_VSUITE_SOIL_MOISTURE:
        s->temperature = (int16_t)_rng_range(sim, 800, 2200); /* 8-22°C soil */
        s->humidity = (uint8_t)_rng_range(sim, 15, 80);       /* soil moisture % */
        s->depth = (uint16_t)_rng_range(sim, 15, 60);         /* burial depth cm */
        break;

    case IOTDATA_VSUITE_WATER_LEVEL:
        s->temperature = (int16_t)_rng_range(sim, 200, 2000); /* 2-20°C water */
        s->depth = (uint16_t)_rng_range(sim, 50, 500);        /* water level cm */
        break;

    case IOTDATA_VSUITE_SNOW_DEPTH:
        s->temperature = (int16_t)_rng_range(sim, -2000, 500); /* -20 to 5°C */
        s->pressure = (uint16_t)_rng_range(sim, 850, 1000);    /* high altitude */
        s->humidity = (uint8_t)_rng_range(sim, 50, 95);
        s->depth = (uint16_t)_rng_range(sim, 0, 300); /* snow cm */
        s->solar_irr = (uint16_t)_rng_range(sim, 0, 600);
        s->solar_uv = (uint8_t)_rng_range(sim, 0, 8);
        break;

    case IOTDATA_VSUITE_ENVIRONMENT:
        s->temperature = (int16_t)_rng_range(sim, 1500, 3000);
        s->pressure = (uint16_t)_rng_range(sim, 990, 1040);
        s->humidity = (uint8_t)_rng_range(sim, 25, 75);
        break;

    case IOTDATA_VSUITE_WIND_STATION:
        s->wind_speed = (uint16_t)_rng_range(sim, 0, 2000);
        s->wind_dir = (uint16_t)_rng_range(sim, 0, 355);
        s->wind_gust = (uint16_t)_clamp(s->wind_speed + _rng_range(sim, 100, 800), 0, 6350);
        s->solar_irr = (uint16_t)_rng_range(sim, 0, 700);
        s->solar_uv = (uint8_t)_rng_range(sim, 0, 10);
        break;

    case IOTDATA_VSUITE_RAIN_GAUGE:
        s->temperature = (int16_t)_rng_range(sim, 0, 2500);
        s->rain_rate = (uint8_t)_rng_range(sim, 0, 30);
        s->rain_size = (uint8_t)_rng_range(sim, 0, 12);
        break;

    case IOTDATA_VSUITE_RADIATION_MONITOR:
        s->temperature = (int16_t)_rng_range(sim, 1000, 2800);
        s->pressure = (uint16_t)_rng_range(sim, 990, 1030);
        s->humidity = (uint8_t)_rng_range(sim, 30, 70);
        s->rad_cpm = (uint16_t)_rng_range(sim, 10, 80);
        s->rad_dose = (uint16_t)_rng_range(sim, 3, 30);
        break;

    default:
        break;
    }
}

/* =========================================================================
 * Drift — small random walk each transmission, clamped to valid range
 * ========================================================================= */

static void _drift_sensor(iotsim_t *sim, iotsim_sensor_t *s) {

    /* Battery drain: ~0.1% per TX on average */
    if (_rng(sim) % 10 == 0 && s->battery > 5)
        s->battery--;

    switch (s->variant) {

    case IOTDATA_VSUITE_WEATHER_STATION:
        s->temperature = (int16_t)_clamp(s->temperature + _jitter(sim, 30), -4000, 8000);
        s->pressure = (uint16_t)_clamp(s->pressure + _jitter(sim, 2), 850, 1100);
        s->humidity = (uint8_t)_clamp(s->humidity + _jitter(sim, 3), 5, 100);
        s->wind_speed = (uint16_t)_clamp(s->wind_speed + _jitter(sim, 80), 0, 6000);
        s->wind_dir = (uint16_t)((s->wind_dir + 360 + _jitter(sim, 15)) % 360);
        s->wind_gust = (uint16_t)_clamp(s->wind_speed + _rng_range(sim, 50, 400), 0, 6350);
        if (_rng(sim) % 20 == 0)
            s->rain_rate = (uint8_t)_clamp(s->rain_rate + _jitter(sim, 5), 0, 200);
        s->rain_size = s->rain_rate ? (uint8_t)_clamp(s->rain_size + _jitter(sim, 1), 0, 24) : 0;
        s->solar_irr = (uint16_t)_clamp(s->solar_irr + _jitter(sim, 30), 0, 1023);
        s->solar_uv = (uint8_t)_clamp(s->solar_uv + _jitter(sim, 1), 0, 15);
        s->clouds = (uint8_t)_clamp(s->clouds + _jitter(sim, 1), 0, 8);
        s->aq_index = (uint16_t)_clamp(s->aq_index + _jitter(sim, 10), 0, 500);
        s->rad_cpm = (uint16_t)_clamp(s->rad_cpm + _jitter(sim, 3), 0, 500);
        s->rad_dose = (uint16_t)_clamp(s->rad_dose + _jitter(sim, 2), 0, 200);
        break;

    case IOTDATA_VSUITE_AIR_QUALITY:
        s->temperature = (int16_t)_clamp(s->temperature + _jitter(sim, 15), -4000, 8000);
        s->pressure = (uint16_t)_clamp(s->pressure + _jitter(sim, 1), 850, 1100);
        s->humidity = (uint8_t)_clamp(s->humidity + _jitter(sim, 2), 5, 100);
        s->aq_index = (uint16_t)_clamp(s->aq_index + _jitter(sim, 8), 0, 500);
        for (int i = 0; i < 4; i++)
            if (s->aq_pm_present & (1U << i))
                s->aq_pm[i] = (uint16_t)_clamp(s->aq_pm[i] + _jitter(sim, 5), 0, 1000);
        for (int i = 0; i < 8; i++)
            if (s->aq_gas_present & (1U << i)) {
                int mag = (i < 2) ? 8 : 25;
                s->aq_gas[i] = (uint16_t)_clamp(s->aq_gas[i] + _jitter(sim, mag), 0, 40000);
            }
        break;

    case IOTDATA_VSUITE_SOIL_MOISTURE:
        s->temperature = (int16_t)_clamp(s->temperature + _jitter(sim, 10), -2000, 5000);
        s->humidity = (uint8_t)_clamp(s->humidity + _jitter(sim, 2), 0, 100);
        break;

    case IOTDATA_VSUITE_WATER_LEVEL:
        s->temperature = (int16_t)_clamp(s->temperature + _jitter(sim, 5), -500, 4000);
        s->depth = (uint16_t)_clamp(s->depth + _jitter(sim, 3), 0, 1023);
        break;

    case IOTDATA_VSUITE_SNOW_DEPTH:
        s->temperature = (int16_t)_clamp(s->temperature + _jitter(sim, 20), -4000, 2000);
        s->pressure = (uint16_t)_clamp(s->pressure + _jitter(sim, 1), 850, 1100);
        s->humidity = (uint8_t)_clamp(s->humidity + _jitter(sim, 2), 10, 100);
        s->depth = (uint16_t)_clamp(s->depth + _jitter(sim, 2), 0, 800);
        s->solar_irr = (uint16_t)_clamp(s->solar_irr + _jitter(sim, 20), 0, 1023);
        s->solar_uv = (uint8_t)_clamp(s->solar_uv + _jitter(sim, 1), 0, 15);
        break;

    case IOTDATA_VSUITE_ENVIRONMENT:
        s->temperature = (int16_t)_clamp(s->temperature + _jitter(sim, 15), -4000, 8000);
        s->pressure = (uint16_t)_clamp(s->pressure + _jitter(sim, 1), 850, 1100);
        s->humidity = (uint8_t)_clamp(s->humidity + _jitter(sim, 2), 5, 100);
        break;

    case IOTDATA_VSUITE_WIND_STATION:
        s->wind_speed = (uint16_t)_clamp(s->wind_speed + _jitter(sim, 100), 0, 6000);
        s->wind_dir = (uint16_t)((s->wind_dir + 360 + _jitter(sim, 20)) % 360);
        s->wind_gust = (uint16_t)_clamp(s->wind_speed + _rng_range(sim, 50, 600), 0, 6350);
        s->solar_irr = (uint16_t)_clamp(s->solar_irr + _jitter(sim, 25), 0, 1023);
        s->solar_uv = (uint8_t)_clamp(s->solar_uv + _jitter(sim, 1), 0, 15);
        break;

    case IOTDATA_VSUITE_RAIN_GAUGE:
        s->temperature = (int16_t)_clamp(s->temperature + _jitter(sim, 15), -2000, 5000);
        if (_rng(sim) % 10 == 0)
            s->rain_rate = (uint8_t)_clamp(s->rain_rate + _jitter(sim, 8), 0, 200);
        s->rain_size = s->rain_rate ? (uint8_t)_clamp(s->rain_size + _jitter(sim, 1), 0, 24) : 0;
        break;

    case IOTDATA_VSUITE_RADIATION_MONITOR:
        s->temperature = (int16_t)_clamp(s->temperature + _jitter(sim, 10), -4000, 8000);
        s->pressure = (uint16_t)_clamp(s->pressure + _jitter(sim, 1), 850, 1100);
        s->humidity = (uint8_t)_clamp(s->humidity + _jitter(sim, 2), 5, 100);
        s->rad_cpm = (uint16_t)_clamp(s->rad_cpm + _jitter(sim, 5), 0, 1000);
        s->rad_dose = (uint16_t)_clamp(s->rad_dose + _jitter(sim, 2), 0, 500);
        break;

    default:
        break;
    }
}

/* =========================================================================
 * Encode — build iotdata packet from current sensor state
 *
 * Converts internal integer units to iotdata_float_t at the boundary.
 * ========================================================================= */

static bool _encode_sensor(iotsim_t *sim, iotsim_sensor_t *s, iotsim_packet_t *out, uint32_t time_now_ms) {
    iotdata_encoder_t enc;
    bool extras = (s->tx_count % IOTSIM_EXTRA_FIELDS_EVERY == 0);

    if (iotdata_encode_begin(&enc, out->buf, sizeof(out->buf), s->variant, s->station_id, s->sequence) != IOTDATA_OK)
        return false;

    /* Battery + link always present */
    iotdata_encode_battery(&enc, s->battery, 0);
    iotdata_encode_link(&enc, (int8_t)_rng_range(sim, -100, -60), _to_snr((int16_t)_rng_range(sim, -100, 80)));

    switch (s->variant) {

    case IOTDATA_VSUITE_WEATHER_STATION:
        iotdata_encode_environment(&enc, _to_temp(s->temperature), s->pressure, s->humidity);
        iotdata_encode_wind(&enc, _to_speed(s->wind_speed), s->wind_dir, _to_speed(s->wind_gust));
        iotdata_encode_rain(&enc, s->rain_rate, s->rain_size);
        iotdata_encode_solar(&enc, s->solar_irr, s->solar_uv);
        if (extras) {
            iotdata_encode_clouds(&enc, s->clouds);
            iotdata_encode_air_quality_index(&enc, s->aq_index);
            iotdata_encode_radiation(&enc, s->rad_cpm, _to_dose(s->rad_dose));
            iotdata_encode_position(&enc, 5933459, 1806323);
            iotdata_encode_flags(&enc, s->flags);
        }
        break;

    case IOTDATA_VSUITE_AIR_QUALITY:
        iotdata_encode_environment(&enc, _to_temp(s->temperature), s->pressure, s->humidity);
        iotdata_encode_air_quality(&enc, s->aq_index, s->aq_pm_present, s->aq_pm, s->aq_gas_present, s->aq_gas);
        if (extras)
            iotdata_encode_flags(&enc, s->flags);
        break;

    case IOTDATA_VSUITE_SOIL_MOISTURE:
        iotdata_encode_temperature(&enc, _to_temp(s->temperature));
        iotdata_encode_humidity(&enc, s->humidity);
        iotdata_encode_depth(&enc, s->depth);
        if (extras)
            iotdata_encode_flags(&enc, s->flags);
        break;

    case IOTDATA_VSUITE_WATER_LEVEL:
        iotdata_encode_temperature(&enc, _to_temp(s->temperature));
        iotdata_encode_depth(&enc, s->depth);
        if (extras)
            iotdata_encode_flags(&enc, s->flags);
        break;

    case IOTDATA_VSUITE_SNOW_DEPTH:
        iotdata_encode_depth(&enc, s->depth);
        iotdata_encode_environment(&enc, _to_temp(s->temperature), s->pressure, s->humidity);
        iotdata_encode_solar(&enc, s->solar_irr, s->solar_uv);
        if (extras) {
            iotdata_encode_position(&enc, 6120000, 1500000);
            iotdata_encode_flags(&enc, s->flags);
        }
        break;

    case IOTDATA_VSUITE_ENVIRONMENT:
        iotdata_encode_environment(&enc, _to_temp(s->temperature), s->pressure, s->humidity);
        if (extras)
            iotdata_encode_flags(&enc, s->flags);
        break;

    case IOTDATA_VSUITE_WIND_STATION:
        iotdata_encode_wind(&enc, _to_speed(s->wind_speed), s->wind_dir, _to_speed(s->wind_gust));
        iotdata_encode_solar(&enc, s->solar_irr, s->solar_uv);
        if (extras)
            iotdata_encode_flags(&enc, s->flags);
        break;

    case IOTDATA_VSUITE_RAIN_GAUGE:
        iotdata_encode_rain(&enc, s->rain_rate, s->rain_size);
        iotdata_encode_temperature(&enc, _to_temp(s->temperature));
        if (extras)
            iotdata_encode_flags(&enc, s->flags);
        break;

    case IOTDATA_VSUITE_RADIATION_MONITOR:
        iotdata_encode_radiation(&enc, s->rad_cpm, _to_dose(s->rad_dose));
        iotdata_encode_environment(&enc, _to_temp(s->temperature), s->pressure, s->humidity);
        if (extras)
            iotdata_encode_flags(&enc, s->flags);
        break;

    default:
        break;
    }

    /* Datetime on extras */
    (void)time_now_ms;
    if (extras)
        iotdata_encode_datetime(&enc, s->tx_count * 10);

    size_t len = 0;
    iotdata_status_t rc = iotdata_encode_end(&enc, &len);
    if (rc != IOTDATA_OK)
        return false;

    out->len = len;
    out->sensor_index = 0; /* filled by caller */
    out->variant = s->variant;
    out->station_id = s->station_id;
    out->sequence = s->sequence;
    return true;
}

/* =========================================================================
 * Public API
 * ========================================================================= */

void iotsim_init(iotsim_t *sim, uint32_t seed, uint32_t time_now_ms) {
    memset(sim, 0, sizeof(*sim));
    sim->rng_state = seed ? seed : 0xDEADBEEF;
    sim->time_base = time_now_ms;

    /* Ensure at least one of each variant type, then fill remaining
     * slots randomly.  Total = IOTSIM_NUM_SENSORS (16). */
    uint8_t variants[IOTSIM_NUM_SENSORS];
    int idx = 0;

    /* One of each (9 variants) */
    for (int v = 0; v < IOTDATA_VSUITE_COUNT && idx < IOTSIM_NUM_SENSORS; v++)
        variants[idx++] = (uint8_t)v;

    /* Fill remaining 7 slots randomly */
    while (idx < IOTSIM_NUM_SENSORS)
        variants[idx++] = (uint8_t)(_rng(sim) % IOTDATA_VSUITE_COUNT);

    /* Shuffle (Fisher-Yates) for random ordering */
    for (int i = IOTSIM_NUM_SENSORS - 1; i > 0; i--) {
        int j = (int)(_rng(sim) % (uint32_t)(i + 1));
        uint8_t tmp = variants[i];
        variants[i] = variants[j];
        variants[j] = tmp;
    }

    /* Initialise each sensor */
    for (int i = 0; i < IOTSIM_NUM_SENSORS; i++) {
        iotsim_sensor_t *s = &sim->sensors[i];
        memset(s, 0, sizeof(*s));
        s->variant = variants[i];
        s->station_id = (uint16_t)(i + 1);

        _init_sensor(sim, s);

        /* Stagger initial transmissions over the first interval window */
        s->tx_interval_ms = (uint32_t)_rng_range(sim, IOTSIM_TX_MIN_MS, IOTSIM_TX_MAX_MS);
        s->next_tx_ms = time_now_ms + (uint32_t)_rng_range(sim, 0, (int32_t)s->tx_interval_ms);
    }
}

bool iotsim_poll(iotsim_t *sim, uint32_t time_now_ms, iotsim_packet_t *out) {
    for (int n = 0; n < IOTSIM_NUM_SENSORS; n++) {
        int i = (sim->poll_next + n) % IOTSIM_NUM_SENSORS;
        iotsim_sensor_t *s = &sim->sensors[i];
        if ((int32_t)(time_now_ms - s->next_tx_ms) < 0)
            continue;

        _drift_sensor(sim, s);

        if (!_encode_sensor(sim, s, out, time_now_ms))
            continue;

        out->sensor_index = (uint8_t)i;

        s->sequence++;
        s->tx_count++;
        s->tx_interval_ms = (uint32_t)_rng_range(sim, IOTSIM_TX_MIN_MS, IOTSIM_TX_MAX_MS);
        s->next_tx_ms = time_now_ms + s->tx_interval_ms;

        sim->poll_next = (i + 1) % IOTSIM_NUM_SENSORS;
        return true;
    }
    return false;
}

const iotsim_sensor_t *iotsim_sensor(const iotsim_t *sim, int index) {
    if (index < 0 || index >= IOTSIM_NUM_SENSORS)
        return NULL;
    return &sim->sensors[index];
}

/* =========================================================================
 * TEST_MAIN — standalone Linux test
 *
 * Runs simulation, decodes each packet and dumps fields.
 * Usage: ./test_sim [seed] [packet_count]
 * ========================================================================= */

#ifdef TEST_MAIN

#define IOTDATA_NO_JSON
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wredundant-decls"
#include "iotdata.c"
#pragma GCC diagnostic pop

static void _print_decoded(const iotdata_decoder_t *d, uint8_t variant) {
    /* Common fields */
    if (IOTDATA_FIELD_PRESENT(d->fields, IOTDATA_FIELD_BATTERY))
        printf("  bat=%" PRIu8 "%%%s", d->battery_level, d->battery_charging ? "(chg)" : "");
    if (IOTDATA_FIELD_PRESENT(d->fields, IOTDATA_FIELD_LINK))
        printf("  rssi=%" PRId16 " snr=%.0f", d->link_rssi, (double)d->link_snr);

    /* Environment */
    if (IOTDATA_FIELD_PRESENT(d->fields, IOTDATA_FIELD_ENVIRONMENT))
        printf("  T=%.2f P=%" PRIu16 " H=%" PRIu8, (double)d->temperature, d->pressure, d->humidity);
    else if (IOTDATA_FIELD_PRESENT(d->fields, IOTDATA_FIELD_TEMPERATURE))
        printf("  T=%.2f", (double)d->temperature);

    /* Wind */
    if (IOTDATA_FIELD_PRESENT(d->fields, IOTDATA_FIELD_WIND))
        printf("  W=%.1f/%" PRIu16 "/%.1f", (double)d->wind_speed, d->wind_direction, (double)d->wind_gust);

    /* Rain */
    if (IOTDATA_FIELD_PRESENT(d->fields, IOTDATA_FIELD_RAIN))
        printf("  R=%" PRIu8 "/%" PRIu8, d->rain_rate, d->rain_size10);

    /* Solar */
    if (IOTDATA_FIELD_PRESENT(d->fields, IOTDATA_FIELD_SOLAR))
        printf("  S=%" PRIu16 "/UV%" PRIu8, d->solar_irradiance, d->solar_ultraviolet);

    /* Depth */
    if (IOTDATA_FIELD_PRESENT(d->fields, IOTDATA_FIELD_DEPTH))
        printf("  D=%" PRIu16, d->depth);

    /* Humidity standalone (soil moisture) */
    if (IOTDATA_FIELD_PRESENT(d->fields, IOTDATA_FIELD_HUMIDITY))
        printf("  H=%" PRIu8 "%%", d->humidity);

    /* Air quality */
    if (IOTDATA_FIELD_PRESENT(d->fields, IOTDATA_FIELD_AIR_QUALITY)) {
        printf("  AQ=%" PRIu16, d->aq_index);
        if (d->aq_pm_present)
            printf(" PM[%" PRIu16 "/%" PRIu16 "/%" PRIu16 "/%" PRIu16 "]", d->aq_pm[0], d->aq_pm[1], d->aq_pm[2], d->aq_pm[3]);
        if (d->aq_gas_present & 0x01)
            printf(" VOC=%" PRIu16, d->aq_gas[0]);
        if (d->aq_gas_present & 0x02)
            printf(" NOx=%" PRIu16, d->aq_gas[1]);
        if (d->aq_gas_present & 0x04)
            printf(" CO2=%" PRIu16, d->aq_gas[2]);
    } else if (IOTDATA_FIELD_PRESENT(d->fields, IOTDATA_FIELD_AIR_QUALITY_INDEX)) {
        printf("  AQI=%" PRIu16, d->aq_index);
    }

    /* Radiation */
    if (IOTDATA_FIELD_PRESENT(d->fields, IOTDATA_FIELD_RADIATION))
        printf("  rad=%" PRIu16 "/%.2f", d->radiation_cpm, (double)d->radiation_dose);

    /* Clouds */
    if (IOTDATA_FIELD_PRESENT(d->fields, IOTDATA_FIELD_CLOUDS))
        printf("  C=%" PRIu8, d->clouds);

    /* Flags */
    if (IOTDATA_FIELD_PRESENT(d->fields, IOTDATA_FIELD_FLAGS))
        printf("  F=%" PRIu8, d->flags);

    (void)variant;
}

int main(int argc, char *argv[]) {
    uint32_t seed = 12345;
    int target = 100;
    if (argc > 1)
        seed = (uint32_t)strtoul(argv[1], NULL, 0);
    if (argc > 2)
        target = atoi(argv[2]);

    iotsim_t sim;
    iotsim_init(&sim, seed, 0);

    /* Print sensor allocation */
    printf("=== Simulator: %d sensors, seed=%" PRIu32 " ===\n\n", IOTSIM_NUM_SENSORS, seed);
    printf("  ID  Variant             Station\n");
    printf("  --  ------------------  -------\n");
    for (int i = 0; i < IOTSIM_NUM_SENSORS; i++) {
        const iotsim_sensor_t *s = iotsim_sensor(&sim, i);
        printf("  %2d  %-18s  %" PRIu16 "\n", i, iotdata_vsuite_name(s->variant), s->station_id);
    }
    printf("\n");

    /* Run simulation */
    uint32_t t = 0;
    int packets = 0;

    while (packets < target && t < 600000) {
        iotsim_packet_t pkt;
        while (iotsim_poll(&sim, t, &pkt)) {
            packets++;
            iotdata_decoder_t dec;
            iotdata_status_t rc = iotdata_decode(pkt.buf, pkt.len, &dec);
            printf("[%5" PRIu32 ".%" PRIu32 "s] #%-4d stn=%3" PRIu16 " %-18s seq=%-3" PRIu16 " bytes=%-2" PRIu32 " [", t / 1000, (t % 1000) / 100, packets, pkt.station_id, iotdata_vsuite_name(pkt.variant), pkt.sequence, (uint32_t)pkt.len);
            for (size_t i = 0; i < pkt.len; i++)
                printf("%02" PRIX8, pkt.buf[i]);
            printf("]: ");

            if (rc == IOTDATA_OK) {
                _print_decoded(&dec, pkt.variant);
                printf("\n");
            } else {
                printf("  ERR: %s\n", iotdata_strerror(rc));
            }
            if (packets >= target)
                break;
        }
        t += 100;
    }

    /* Summary */
    printf("\n=== %d packets in %.1fs simulated ===\n\n", packets, (double)t / 1000.0);

    printf("  ID  Variant             TXs  Bat%%  Last seq\n");
    printf("  --  ------------------  ---  ----  --------\n");
    for (int i = 0; i < IOTSIM_NUM_SENSORS; i++) {
        const iotsim_sensor_t *s = iotsim_sensor(&sim, i);
        printf("  %2d  %-18s  %3" PRIu32 "  %3" PRIu8 "%%  %" PRIu16 "\n", i, iotdata_vsuite_name(s->variant), s->tx_count, s->battery, s->sequence);
    }

    return 0;
}

#endif /* TEST_MAIN */
