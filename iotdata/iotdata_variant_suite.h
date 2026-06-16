/*
 * IoT Sensor Telemetry Protocol
 * Copyright(C) 2026 Matthew Gream (https://libiotdata.org)
 *
 * iotdata_variant_suite.h - multi-sensor variant definitions
 *
 * Defines 9 sensor variants inspired by common real-world sensor
 * configurations (Ecowitt, Sensirion SEN55/SEN66, agricultural
 * probes, hydrological gauges, etc).
 */

#ifndef IOTDATA_VARIANT_SUITE_H
#define IOTDATA_VARIANT_SUITE_H

/* ---------------------------------------------------------------------------
 * Variant indices
 * -------------------------------------------------------------------------*/

#define IOTDATA_VSUITE_WEATHER_STATION   0 /* Full outdoor weather station        */
#define IOTDATA_VSUITE_AIR_QUALITY       1 /* Indoor/outdoor AQ (SEN55/SEN66)     */
#define IOTDATA_VSUITE_SOIL_MOISTURE     2 /* Agricultural soil probe             */
#define IOTDATA_VSUITE_WATER_LEVEL       3 /* River/tank level gauge              */
#define IOTDATA_VSUITE_SNOW_DEPTH        4 /* Ultrasonic snow depth sensor        */
#define IOTDATA_VSUITE_ENVIRONMENT       5 /* Simple T/H/P (BME280)              */
#define IOTDATA_VSUITE_WIND_STATION      6 /* Standalone anemometer               */
#define IOTDATA_VSUITE_RAIN_GAUGE        7 /* Standalone rain collector           */
#define IOTDATA_VSUITE_RADIATION_MONITOR 8 /* Geiger counter station              */

#define IOTDATA_VSUITE_COUNT             9

/* ---------------------------------------------------------------------------
 * Configure iotdata for external variant maps, all fields enabled
 * -------------------------------------------------------------------------*/

#define IOTDATA_VARIANT_MAPS             iotdata_variant_suite
#define IOTDATA_VARIANT_MAPS_COUNT       IOTDATA_VSUITE_COUNT

/* Not selective — all field types compiled in */
#include "iotdata.h"

/* ---------------------------------------------------------------------------
 * FLAGS used across the suite
 * -------------------------------------------------------------------------*/

#define VSUITE_FLAG_RESTART_RECENT          (0)
#define VSUITE_FLAG_BATTERY_DRAINING        (1)
#define VSUITE_FLAG_SENSOR_FAULTS           (2)
#define VSUITE_FLAG_THERMAL_EXCEPTION       (3)
#define VSUITE_FLAG_USER_INTERACTION_RECENT (4)
#define VSUITE_FLAG_BIT_5                   (5)
#define VSUITE_FLAG_BIT_6                   (6)
#define VSUITE_FLAG_BIT_7                   (7)

/* ---------------------------------------------------------------------------
 * Variant map definitions
 *
 * Slot layout:
 *   pres0: slots [0]-[5]   (6 data slots)
 *   pres1: slots [6]-[12]  (7 data slots)
 *
 * Every variant includes battery + link in slots 0-1 for consistent
 * telemetry.  Unused slots within the pres byte range MUST be filled
 * with _VS_NONE since IOTDATA_FIELD_NONE = -1 and zero-init would
 * alias to IOTDATA_FIELD_BATTERY (enum 0).
 * -------------------------------------------------------------------------*/

#define _VS_NONE                            { IOTDATA_FIELD_NONE, NULL }

const iotdata_variant_def_t iotdata_variant_suite[IOTDATA_VSUITE_COUNT] = {

    /* -----------------------------------------------------------------
     * Variant 0: weather_station
     *
     * Full outdoor station (e.g. Ecowitt WS90 + HP2560 console).
     * All environmental parameters, 2 presence bytes.
     * ----------------------------------------------------------------- */
    [IOTDATA_VSUITE_WEATHER_STATION] = {
        .name = "weather_station",
        .num_pres_bytes = 2,
        .fields = {
            /* pres0 [0..5] */
            { IOTDATA_FIELD_BATTERY,             "battery"     },  /* S0 */
            { IOTDATA_FIELD_LINK,                "link"        },  /* S1 */
            { IOTDATA_FIELD_ENVIRONMENT,         "environment" },  /* S2 */
            { IOTDATA_FIELD_WIND,                "wind"        },  /* S3 */
            { IOTDATA_FIELD_RAIN,                "rain"        },  /* S4 */
            { IOTDATA_FIELD_SOLAR,               "solar"       },  /* S5 */
            /* pres1 [6..12] */
            { IOTDATA_FIELD_CLOUDS,              "clouds"      },  /* S6 */
            { IOTDATA_FIELD_AIR_QUALITY_INDEX,   "air_quality" },  /* S7 */
            { IOTDATA_FIELD_RADIATION,           "radiation"   },  /* S8 */
            { IOTDATA_FIELD_POSITION,            "position"    },  /* S9 */
            { IOTDATA_FIELD_DATETIME,            "datetime"    },  /* S10 */
            { IOTDATA_FIELD_FLAGS,               "flags"       },  /* S11 */
            _VS_NONE,                                              /* S12 */
        },
    },

    /* -----------------------------------------------------------------
     * Variant 1: air_quality
     *
     * Indoor/outdoor air quality monitor (e.g. Sensirion SEN55/SEN66,
     * PurpleAir).  Uses the AQ bundle for AQI + PM + gas in one slot.
     * ----------------------------------------------------------------- */
    [IOTDATA_VSUITE_AIR_QUALITY] = {
        .name = "air_quality",
        .num_pres_bytes = 1,
        .fields = {
            /* pres0 [0..5] */
            { IOTDATA_FIELD_BATTERY,             "battery"     },  /* S0 */
            { IOTDATA_FIELD_LINK,                "link"        },  /* S1 */
            { IOTDATA_FIELD_ENVIRONMENT,         "environment" },  /* S2 */
            { IOTDATA_FIELD_AIR_QUALITY,         "air_quality" },  /* S3 */
            { IOTDATA_FIELD_DATETIME,            "datetime"    },  /* S4 */
            { IOTDATA_FIELD_FLAGS,               "flags"       },  /* S5 */
        },
    },

    /* -----------------------------------------------------------------
     * Variant 2: soil_moisture
     *
     * Agricultural soil probe (e.g. Ecowitt WH51, Teros-12).
     * Temperature = soil temperature, humidity = soil moisture %,
     * depth = sensor burial depth in cm.
     * ----------------------------------------------------------------- */
    [IOTDATA_VSUITE_SOIL_MOISTURE] = {
        .name = "soil_moisture",
        .num_pres_bytes = 1,
        .fields = {
            /* pres0 [0..5] */
            { IOTDATA_FIELD_BATTERY,             "battery"     },  /* S0 */
            { IOTDATA_FIELD_LINK,                "link"        },  /* S1 */
            { IOTDATA_FIELD_TEMPERATURE,         "soil_temp"   },  /* S2 */
            { IOTDATA_FIELD_HUMIDITY,            "moisture"    },  /* S3 */
            { IOTDATA_FIELD_DEPTH,               "depth"       },  /* S4 */
            { IOTDATA_FIELD_FLAGS,               "flags"       },  /* S5 */
        },
    },

    /* -----------------------------------------------------------------
     * Variant 3: water_level
     *
     * River/tank level gauge (e.g. ultrasonic distance sensor).
     * Depth = water level cm, temperature for compensation.
     * ----------------------------------------------------------------- */
    [IOTDATA_VSUITE_WATER_LEVEL] = {
        .name = "water_level",
        .num_pres_bytes = 1,
        .fields = {
            /* pres0 [0..5] */
            { IOTDATA_FIELD_BATTERY,             "battery"     },  /* S0 */
            { IOTDATA_FIELD_LINK,                "link"        },  /* S1 */
            { IOTDATA_FIELD_TEMPERATURE,         "water_temp"  },  /* S2 */
            { IOTDATA_FIELD_DEPTH,               "level"       },  /* S3 */
            { IOTDATA_FIELD_DATETIME,            "datetime"    },  /* S4 */
            { IOTDATA_FIELD_FLAGS,               "flags"       },  /* S5 */
        },
    },

    /* -----------------------------------------------------------------
     * Variant 4: snow_depth
     *
     * Ultrasonic snow depth sensor with environmental context and
     * solar for power monitoring (often solar-powered remote sites).
     * ----------------------------------------------------------------- */
    [IOTDATA_VSUITE_SNOW_DEPTH] = {
        .name = "snow_depth",
        .num_pres_bytes = 2,
        .fields = {
            /* pres0 [0..5] */
            { IOTDATA_FIELD_BATTERY,             "battery"     },  /* S0 */
            { IOTDATA_FIELD_LINK,                "link"        },  /* S1 */
            { IOTDATA_FIELD_DEPTH,               "snow_depth"  },  /* S2 */
            { IOTDATA_FIELD_ENVIRONMENT,         "environment" },  /* S3 */
            { IOTDATA_FIELD_SOLAR,               "solar"       },  /* S4 */
            { IOTDATA_FIELD_BITS32,              "flags"       },  /* S5 */
            /* pres1 [6..12] */
            { IOTDATA_FIELD_POSITION,            "position"    },  /* S6 */
            { IOTDATA_FIELD_DATETIME,            "datetime"    },  /* S7 */
            _VS_NONE,                                              /* S8 */
            _VS_NONE,                                              /* S9 */
            _VS_NONE,                                              /* S10 */
            _VS_NONE,                                              /* S11 */
            _VS_NONE,                                              /* S12 */
        },
    },

    /* -----------------------------------------------------------------
     * Variant 5: environment
     *
     * Simple indoor/outdoor T/H/P sensor (e.g. BME280/BME680 node,
     * Ecowitt WN30).  Minimal fields, single presence byte.
     * ----------------------------------------------------------------- */
    [IOTDATA_VSUITE_ENVIRONMENT] = {
        .name = "environment",
        .num_pres_bytes = 1,
        .fields = {
            /* pres0 [0..5] */
            { IOTDATA_FIELD_BATTERY,             "battery"     },  /* S0 */
            { IOTDATA_FIELD_LINK,                "link"        },  /* S1 */
            { IOTDATA_FIELD_ENVIRONMENT,         "environment" },  /* S2 */
            { IOTDATA_FIELD_DATETIME,            "datetime"    },  /* S3 */
            { IOTDATA_FIELD_FLAGS,               "flags"       },  /* S4 */
            _VS_NONE,                                              /* S5 */
        },
    },

    /* -----------------------------------------------------------------
     * Variant 6: wind_station
     *
     * Standalone anemometer (e.g. Ecowitt WS80, Calypso ultrasonic).
     * Wind bundle + solar (often solar-powered mast-mounted).
     * ----------------------------------------------------------------- */
    [IOTDATA_VSUITE_WIND_STATION] = {
        .name = "wind_station",
        .num_pres_bytes = 1,
        .fields = {
            /* pres0 [0..5] */
            { IOTDATA_FIELD_BATTERY,             "battery"     },  /* S0 */
            { IOTDATA_FIELD_LINK,                "link"        },  /* S1 */
            { IOTDATA_FIELD_WIND,                "wind"        },  /* S2 */
            { IOTDATA_FIELD_SOLAR,               "solar"       },  /* S3 */
            { IOTDATA_FIELD_DATETIME,            "datetime"    },  /* S4 */
            { IOTDATA_FIELD_FLAGS,               "flags"       },  /* S5 */
        },
    },

    /* -----------------------------------------------------------------
     * Variant 7: rain_gauge
     *
     * Standalone rain collector (e.g. Ecowitt WH40, tipping bucket).
     * Temperature for freeze detection.
     * ----------------------------------------------------------------- */
    [IOTDATA_VSUITE_RAIN_GAUGE] = {
        .name = "rain_gauge",
        .num_pres_bytes = 1,
        .fields = {
            /* pres0 [0..5] */
            { IOTDATA_FIELD_BATTERY,             "battery"     },  /* S0 */
            { IOTDATA_FIELD_LINK,                "link"        },  /* S1 */
            { IOTDATA_FIELD_RAIN,                "rain"        },  /* S2 */
            { IOTDATA_FIELD_TEMPERATURE,         "temperature" },  /* S3 */
            { IOTDATA_FIELD_DATETIME,            "datetime"    },  /* S4 */
            { IOTDATA_FIELD_FLAGS,               "flags"       },  /* S5 */
        },
    },

    /* -----------------------------------------------------------------
     * Variant 8: radiation_monitor
     *
     * Geiger counter station (e.g. RadMon, GQ GMC-320).
     * Radiation bundle (CPM + dose) with environmental context.
     * ----------------------------------------------------------------- */
    [IOTDATA_VSUITE_RADIATION_MONITOR] = {
        .name = "radiation_monitor",
        .num_pres_bytes = 1,
        .fields = {
            /* pres0 [0..5] */
            { IOTDATA_FIELD_BATTERY,             "battery"     },  /* S0 */
            { IOTDATA_FIELD_LINK,                "link"        },  /* S1 */
            { IOTDATA_FIELD_RADIATION,           "radiation"   },  /* S2 */
            { IOTDATA_FIELD_ENVIRONMENT,         "environment" },  /* S3 */
            { IOTDATA_FIELD_DATETIME,            "datetime"    },  /* S4 */
            { IOTDATA_FIELD_FLAGS,               "flags"       },  /* S5 */
        },
    },
};

/* ---------------------------------------------------------------------------
 * Variant name lookup (for debug/logging)
 * -------------------------------------------------------------------------*/

static inline const char *iotdata_vsuite_name(uint8_t variant) {
    return (variant < IOTDATA_VSUITE_COUNT) ? iotdata_variant_suite[variant].name : "unknown";
}

#endif /* IOTDATA_VARIANT_SUITE_H */
