#ifndef IOTDATA_HARDWARE_CONFIG_H
#define IOTDATA_HARDWARE_CONFIG_H

// -----------------------------------------------------------------------------------------------------------------------------------------
// iotdata_hardware_config.h - CONFIG rows for the hardware, defined ONCE and shared by the example
// apps that carry it. The sibling of iotdata_hardware.h: that one is the pin map and the board
// constants, this one is what of the hardware is SETTABLE at runtime and what a valid setting is.
//
//     #include "iotdata_node_config.h"        // the types
//     #include "iotdata_hardware_config.h"    // the validators and the row block
//     #define IOTDATA_CONFIG_ENTRIES(X)
//         X(...this app's own...)
//         IOTDATA_CONFIG_ENTRIES_LORA(X)
//     #include "iotdata_node_config.h"        // expand
//     #include "iotdata_hardware_config.h"    // now iotdata_config_lora_apply()
// -----------------------------------------------------------------------------------------------------------------------------------------

static inline bool iotdata_config_lora_rate_known(const uint64_t bps) {
    return bps == 300u || bps == 1200u || bps == 2400u || bps == 4800u || bps == 9600u || bps == 19200u || bps == 38400u || bps == 62500u;
}

static inline bool iotdata_config_lora_air_rate_ok(__attribute__((unused)) const iotdata_config_row_t *const row, const iotdata_config_value_t *const v, __attribute__((unused)) const struct iotdata_config_update *const u) {
    return iotdata_config_lora_rate_known(v->u);
}

static inline bool iotdata_config_lora_tx_power_ok(__attribute__((unused)) const iotdata_config_row_t *const row, const iotdata_config_value_t *const v, __attribute__((unused)) const struct iotdata_config_update *const u) {
    return v->u == 30u || v->u == 22u || v->u == 17u || v->u == 13u || v->u == 10u;
}

static inline bool iotdata_config_lora_packet_size_ok(__attribute__((unused)) const iotdata_config_row_t *const row, const iotdata_config_value_t *const v, __attribute__((unused)) const struct iotdata_config_update *const u) {
    return v->u == 240u || v->u == 128u || v->u == 64u || v->u == 32u;
}

#ifndef IOTDATA_CONFIG_LORA_NOTIFY
static inline bool iotdata_config_lora_changed(__attribute__((unused)) const iotdata_config_row_t *const row, __attribute__((unused)) const iotdata_config_value_t *const was,
                                               __attribute__((unused)) const iotdata_config_info_t *const info) {
    return true;
}
#define IOTDATA_CONFIG_LORA_NOTIFY iotdata_config_lora_changed
#endif

#define IOTDATA_CONFIG_ENTRIES_LORA(X) \
    X(LORA_ADDRESS, 0x030, U16, 0, 0xFFFF, IOTDATA_CONFIG_LORA_ADDRESS, IOTDATA_CONFIG_FLAG_REBOOT, NULL, IOTDATA_CONFIG_LORA_NOTIFY) \
    X(LORA_NETWORK, 0x031, U8, 0, 0xFF, IOTDATA_CONFIG_LORA_NETWORK, IOTDATA_CONFIG_FLAG_REBOOT, NULL, IOTDATA_CONFIG_LORA_NOTIFY) \
    X(LORA_CHANNEL, 0x032, U8, 0, 83, IOTDATA_CONFIG_LORA_CHANNEL, IOTDATA_CONFIG_FLAG_REBOOT, NULL, IOTDATA_CONFIG_LORA_NOTIFY) \
    X(LORA_TX_POWER, 0x033, U8, 10, 22, IOTDATA_CONFIG_LORA_TX_POWER, IOTDATA_CONFIG_FLAG_REBOOT, iotdata_config_lora_tx_power_ok, IOTDATA_CONFIG_LORA_NOTIFY) \
    X(LORA_AIR_RATE, 0x034, U16, 300, 62500, IOTDATA_CONFIG_LORA_AIR_RATE, IOTDATA_CONFIG_FLAG_REBOOT, iotdata_config_lora_air_rate_ok, IOTDATA_CONFIG_LORA_NOTIFY) \
    X(LORA_PACKET_SIZE, 0x035, U8, 32, 240, IOTDATA_CONFIG_LORA_PACKET_SIZE, IOTDATA_CONFIG_FLAG_REBOOT, iotdata_config_lora_packet_size_ok, IOTDATA_CONFIG_LORA_NOTIFY) \
    X(LORA_LBT, 0x036, BOOL, 0, 1, IOTDATA_CONFIG_LORA_LBT, IOTDATA_CONFIG_FLAG_REBOOT, NULL, IOTDATA_CONFIG_LORA_NOTIFY) \
    X(LORA_CRYPT, 0x037, U16, 0, 0xFFFF, IOTDATA_CONFIG_LORA_CRYPT, IOTDATA_CONFIG_FLAG_REBOOT, NULL, IOTDATA_CONFIG_LORA_NOTIFY)

#endif /* IOTDATA_HARDWARE_CONFIG_H */

/* --- the second include, once the table has been expanded ---------------------------------- */

#if defined(IOTDATA_NODE_CONFIG_EXPANDED) && !defined(IOTDATA_HARDWARE_CONFIG_APPLIED)
#define IOTDATA_HARDWARE_CONFIG_APPLIED

static inline void iotdata_config_lora_apply(lora_config_t *const c) {
    if (c == NULL)
        return;
    c->module = LORA_MODULE_DIP;
    c->rssi_packet = true;
    c->rssi_channel = false;
    c->e22_address = iotdata_config_u16(LORA_ADDRESS);
    c->e22_network = iotdata_config_u8(LORA_NETWORK);
    c->channel = iotdata_config_u8(LORA_CHANNEL);
    c->transmit_power = iotdata_config_u8(LORA_TX_POWER);
    c->air_data_rate = iotdata_config_u16(LORA_AIR_RATE);
    c->packet_size = iotdata_config_u8(LORA_PACKET_SIZE);
    c->listen_before_transmit = iotdata_config_bool(LORA_LBT);
    c->crypt = iotdata_config_u16(LORA_CRYPT);
}

#endif /* IOTDATA_NODE_CONFIG_EXPANDED && !IOTDATA_HARDWARE_CONFIG_APPLIED */
