#ifndef IOTDATA_EXAMPLE_CONFIG_H
#define IOTDATA_EXAMPLE_CONFIG_H

// ---------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------
//
// iotdata_config.h - shared compile-time config for the iotdata-example device apps (simulator, sensors).
//
// Host-local overrides: the project Makefile locates a gitignored iotdata_config.<hostname>.h -- the shared
// iotdata-example/include/ one if present, else the project-local one -- and passes its path as
// -DIOTDATA_CONFIG=... . That header just #defines the IOTDATA_CONFIG_* knobs it wants to change; every
// knob it leaves alone falls through to the committed default below. So one gitignored header per host carries
// all local test tuning, never committed.
//
// To add a knob: add one #ifndef block here + use IOTDATA_CONFIG_<...> in the app. No Makefile/CMake per-knob
// wiring (that was the point of moving off the make-variable/-D-foreach mechanism).
//
// ---------------------------------------------------------------------------------------------------------------------------

#ifdef IOTDATA_CONFIG
#include IOTDATA_CONFIG /* gitignored iotdata_config.<host>.h, path injected by the Makefile */
#endif

// --- hardware pin-map variant ----------------------------------------------------------------------------------------------

#ifndef IOTDATA_CONFIG_CARRIER
#define IOTDATA_CONFIG_CARRIER 1 /* 1 = AE SDC carrier PCB pin map (default); 0 = original bench wiring */
#endif

// --- LoRa radio (E22 today; the names are radio-agnostic for the RAK3272/WM1302 ports) -------------------------------------
// Channel is the one hard fleet-wide match (freq); network/address do NOT gate RX in E22 transparent mode.

#ifndef IOTDATA_CONFIG_LORA_CHANNEL
#define IOTDATA_CONFIG_LORA_CHANNEL 0x0A
#endif
#ifndef IOTDATA_CONFIG_LORA_NETWORK
#define IOTDATA_CONFIG_LORA_NETWORK 0x00
#endif
#ifndef IOTDATA_CONFIG_LORA_ADDRESS
#define IOTDATA_CONFIG_LORA_ADDRESS 0x0008
#endif

// --- simulator (simulator_sensor_lora_esp32) -------------------------------------------------------------------------------

#ifndef IOTDATA_CONFIG_SIMULATOR_NUM_SENSORS
#define IOTDATA_CONFIG_SIMULATOR_NUM_SENSORS 8 /* sensors/board — keeps the shared channel sane across ~4 boards */
#endif
#ifndef IOTDATA_CONFIG_SIMULATOR_VARIANT_TYPES
#define IOTDATA_CONFIG_SIMULATOR_VARIANT_TYPES 4 /* distinct types/board; a fleet spreads the 9-type suite out */
#endif
#ifndef IOTDATA_CONFIG_SIMULATOR_TX_MIN_MS
#define IOTDATA_CONFIG_SIMULATOR_TX_MIN_MS 20000 /* 20s min tx interval */
#endif
#ifndef IOTDATA_CONFIG_SIMULATOR_TX_MAX_MS
#define IOTDATA_CONFIG_SIMULATOR_TX_MAX_MS 40000 /* 40s max tx interval */
#endif
#ifndef IOTDATA_CONFIG_SIMULATOR_EXTRA_FIELDS_EVERY
#define IOTDATA_CONFIG_SIMULATOR_EXTRA_FIELDS_EVERY 10 /* add extra fields every ~Nth tx */
#endif
#ifndef IOTDATA_CONFIG_SIMULATOR_MAX_PACKET
#define IOTDATA_CONFIG_SIMULATOR_MAX_PACKET 128
#endif

// --- bme280 sensor (sensor_bme280_lora_esp32) ------------------------------------------------------------------------------

#ifndef IOTDATA_CONFIG_SENSOR_TX_PERIOD_MS
#define IOTDATA_CONFIG_SENSOR_TX_PERIOD_MS (60 * 1000) /* one measure+transmit cycle per minute */
#endif

// ---------------------------------------------------------------------------------------------------------------------------

#endif /* IOTDATA_EXAMPLE_CONFIG_H */
