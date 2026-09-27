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
#ifndef IOTDATA_CONFIG_LORA_CRYPT
#define IOTDATA_CONFIG_LORA_CRYPT 0x0000 /* E22 scrambling key; 0 = off */
#endif
#ifndef IOTDATA_CONFIG_LORA_LBT
#define IOTDATA_CONFIG_LORA_LBT 1 /* listen before transmit */
#endif
#ifndef IOTDATA_CONFIG_LORA_AIR_RATE
#define IOTDATA_CONFIG_LORA_AIR_RATE 2400 /* bps on air */
#endif
#ifndef IOTDATA_CONFIG_LORA_PACKET_SIZE
#define IOTDATA_CONFIG_LORA_PACKET_SIZE 240 /* the E22 sub-packet */
#endif
#ifndef IOTDATA_CONFIG_LORA_TX_POWER
#define IOTDATA_CONFIG_LORA_TX_POWER 22 /* dBm */
#endif

// --- the receive window ----------------------------------------------------------------------------------------------------

#ifndef IOTDATA_CONFIG_RECEIVE_WINDOW_MS
#define IOTDATA_CONFIG_RECEIVE_WINDOW_MS 5000
#endif

// --- simulator (simulator_lora_esp32) --------------------------------------------------------------------------------------

#ifndef IOTDATA_CONFIG_SIMULATOR_SENSORS_MAX
#define IOTDATA_CONFIG_SIMULATOR_SENSORS_MAX 16 /* what the arrays are dimensioned for */
#endif
#ifndef IOTDATA_CONFIG_SIMULATOR_SENSORS
#define IOTDATA_CONFIG_SIMULATOR_SENSORS 8 /* sensors/board — keeps the shared channel sane across ~4 boards */
#endif
#ifndef IOTDATA_CONFIG_SIMULATOR_VARIANTS_MAX
#define IOTDATA_CONFIG_SIMULATOR_VARIANTS_MAX IOTDATA_VARIANT_MAX /* the whole telemetry variant space */
#endif
#ifndef IOTDATA_CONFIG_SIMULATOR_VARIANTS
#define IOTDATA_CONFIG_SIMULATOR_VARIANTS 4 /* distinct types/board; a fleet spreads the 9-type suite out */
#endif
#ifndef IOTDATA_CONFIG_SIMULATOR_TRANSMIT_MIN_MS
#define IOTDATA_CONFIG_SIMULATOR_TRANSMIT_MIN_MS 20000 /* 20s min tx interval */
#endif
#ifndef IOTDATA_CONFIG_SIMULATOR_TRANSMIT_MAX_MS
#define IOTDATA_CONFIG_SIMULATOR_TRANSMIT_MAX_MS 40000 /* 40s max tx interval */
#endif
#ifndef IOTDATA_CONFIG_SIMULATOR_EXTRA_FIELDS
#define IOTDATA_CONFIG_SIMULATOR_EXTRA_FIELDS 10 /* add extra fields every ~Nth tx */
#endif

// --- bme280 sensor (sensor_lora_esp32) -------------------------------------------------------------------------------------

#ifndef IOTDATA_CONFIG_SENSOR_TRANSMIT_PERIOD_MS
#define IOTDATA_CONFIG_SENSOR_TRANSMIT_PERIOD_MS (60 * 1000) /* one measure+transmit cycle per minute */
#endif

// ---------------------------------------------------------------------------------------------------------------------------

#ifndef IOTDATA_CONFIG_BLACKBOX
// Blackbox diagnostics:
//  0 = off (compiled out entirely)
//  1 = RAM only    — store in RTC_NOINIT ring; survives deep sleep and watchdog/panic resets, but not a power cycle.
//  2 = RAM + flash — the ring is backed into the `diag` partition (partitions.csv), so records outlive a power loss.
#define IOTDATA_CONFIG_BLACKBOX 2
#endif

// ---------------------------------------------------------------------------------------------------------------------------

#endif /* IOTDATA_EXAMPLE_CONFIG_H */
