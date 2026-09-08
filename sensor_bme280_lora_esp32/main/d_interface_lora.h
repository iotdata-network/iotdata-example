
// -----------------------------------------------------------------------------------------------------------------------------------------
// EBYTE E22-xxxTxx serial transport
// -----------------------------------------------------------------------------------------------------------------------------------------

static const char *__tag_lora = "lora";

// -----------------------------------------------------------------------------------------------------------------------------------------

static bool debug_e22 = false;
#define PRINTF_DEBUG(fmt, ...) \
    do { \
        if (debug_e22) \
            printf(fmt, ##__VA_ARGS__); \
    } while (0)
#define PRINTF_INFO  printf
#define PRINTF_ERROR printf

// -----------------------------------------------------------------------------------------------------------------------------------------

bool serial_connect(void) {
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err;
    if ((err = uart_driver_install(E22_UART, E22_UART_BUF_SIZE, E22_UART_BUF_SIZE, 0, NULL, 0)) != ESP_OK) {
        ESP_LOGE(__tag_lora, "uart_driver_install: %s", esp_err_to_name(err));
        return false;
    }
    if ((err = uart_param_config(E22_UART, &uart_config)) != ESP_OK) {
        ESP_LOGE(__tag_lora, "uart_param_config: %s", esp_err_to_name(err));
        (void)uart_driver_delete(E22_UART);
        return false;
    }
    if ((err = uart_set_pin(E22_UART, PIN_E22_RXD, PIN_E22_TXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)) != ESP_OK) {
        ESP_LOGE(__tag_lora, "uart_set_pin: %s", esp_err_to_name(err));
        (void)uart_driver_delete(E22_UART);
        return false;
    }
    return true;
}
void serial_disconnect(void) {
    (void)uart_driver_delete(E22_UART);
}
void serial_flush(void) {
    (void)uart_flush(E22_UART);
}
int serial_write(const uint8_t *buffer, const int length) {
    __SLEEP_MS(50); // XXX
    return uart_write_bytes(E22_UART, buffer, (size_t)length);
}
int serial_read(uint8_t *buffer, const int length, const uint32_t timeout_ms) {
    __SLEEP_MS(50); // XXX
    return uart_read_bytes(E22_UART, buffer, (size_t)length, pdMS_TO_TICKS(timeout_ms));
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// EBYTE E22-xxxTxx software interface (DIP variant)
// -----------------------------------------------------------------------------------------------------------------------------------------

#define E22900T22_SUPPORT_MODULE_DIP
#undef E22900T22_SUPPORT_MODULE_USB
#include "e22xxxtxx.h"
inline void __sleep_ms(const uint32_t ms) {
    __SLEEP_MS(ms);
}

static void e22_set_pin_mx(const bool pin_m0, const bool pin_m1) {
    (void)gpio_set_level(PIN_E22_M0, pin_m0 ? 1 : 0);
    (void)gpio_set_level(PIN_E22_M1, pin_m1 ? 1 : 0);
}
static bool e22_get_pin_aux(void) {
    return gpio_get_level(PIN_E22_AUX) == 1;
}
static e22900t22_config_t e22_config = {
    .address = IOTDATA_CONFIG_LORA_ADDRESS,
    .network = IOTDATA_CONFIG_LORA_NETWORK,
    .channel = IOTDATA_CONFIG_LORA_CHANNEL,
    .packet_size = E22900T22_CONFIG_PACKET_SIZE_DEFAULT,
    .packet_rate = E22900T22_CONFIG_PACKET_RATE_DEFAULT,
    .crypt = E22900T22_CONFIG_CRYPT_DEFAULT,
    .wor_enabled = E22900T22_CONFIG_WOR_ENABLED_DEFAULT,
    .wor_cycle = E22900T22_CONFIG_WOR_CYCLE_DEFAULT,
    .transmit_power = E22900T22_CONFIG_TRANSMIT_POWER_DEFAULT,
    .transmission_method = E22900T22_CONFIG_TRANSMISSION_METHOD_DEFAULT,
    .relay_enabled = E22900T22_CONFIG_RELAY_ENABLED_DEFAULT,
    .listen_before_transmit = true, // XXX
    .rssi_packet = true,
    .rssi_channel = true,
    .read_timeout_command = E22900T22_CONFIG_READ_TIMEOUT_COMMAND_DEFAULT,
    .read_timeout_packet = E22900T22_CONFIG_READ_TIMEOUT_PACKET_DEFAULT,
    .set_pin_mx = e22_set_pin_mx,
    .get_pin_aux = e22_get_pin_aux,
    .debug = false,
};

// -----------------------------------------------------------------------------------------------------------------------------------------
// LoRa radio — bring up, transmit one packet, put back to sleep
// -----------------------------------------------------------------------------------------------------------------------------------------

#define LORA_DRAIN_TIMEOUT_MS 1000 /* UART TX FIFO drain before the pins go quiet */
#define LORA_SETTLE_MS        20   /* let the module pull AUX low on the packet it just received */

static void lora_gpio_init(void) {
    /* Release the M0/M1 hold applied before deep sleep (see lora_hold); a no-op on a cold boot. */
    gpio_deep_sleep_hold_dis();
    (void)gpio_hold_dis(PIN_E22_M0);
    (void)gpio_hold_dis(PIN_E22_M1);

    (void)gpio_set_direction(PIN_E22_M0, GPIO_MODE_OUTPUT);
    (void)gpio_set_level(PIN_E22_M0, 1);
    (void)gpio_set_direction(PIN_E22_M1, GPIO_MODE_OUTPUT);
    (void)gpio_set_level(PIN_E22_M1, 1);
    (void)gpio_set_direction(PIN_E22_AUX, GPIO_MODE_INPUT);
    (void)gpio_pullup_en(PIN_E22_AUX);
}

/*
 * The E22 holds its own configuration in module NVM, so it only needs writing
 * once per power cycle — `configure` is true on a cold boot (or whenever the
 * cached state was lost) and false on an ordinary deep sleep wake, where going
 * straight to transfer mode saves a second or so of UART command traffic.
 */
static bool lora_begin(const bool configure) {

    lora_gpio_init();
    if (!serial_connect()) {
        ESP_LOGE(__tag_lora, "serial_connect failed");
        return false;
    }
    if (!device_connect(E22900T22_MODULE_DIP, &e22_config)) {
        ESP_LOGE(__tag_lora, "device_connect failed");
        return false;
    }
    if (configure) {
        if (!(device_mode_config() && device_info_read() && device_config_read_and_update())) {
            ESP_LOGE(__tag_lora, "device_mode/info/config failed");
            return false;
        }
        ESP_LOGI(__tag_lora, "configured: address=0x%04" PRIX16 " network=0x%02" PRIX8 " channel=0x%02" PRIX8, e22_config.address, e22_config.network, e22_config.channel);
    }
    if (!device_mode_transfer()) {
        ESP_LOGE(__tag_lora, "device_mode_transfer failed");
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static bool lora_transmit(const uint8_t *const packet, const size_t length) {

    char hex[(PACKET_MAX * 2) + 1] = { '\0' };
    for (size_t i = 0, o = 0; i < length && o < (sizeof(hex) - 1); i++)
        o += (size_t)snprintf(&hex[o], (sizeof(hex) - o) - 1, "%02" PRIX8, packet[i]);

    if (!device_packet_write(packet, (int)length)) {
        ESP_LOGE(__tag_lora, "tx failed: len=%u hex=%s", (unsigned)length, hex);
        return false;
    }
    ESP_LOGI(__tag_lora, "tx: len=%u hex=%s", (unsigned)length, hex);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/*
 * Park the module in its own sleep mode. `active` says whether lora_begin got
 * far enough to talk to it: if so the packet is still sitting in the UART TX
 * buffer, so drain it, give the module a moment to react, then switch mode —
 * device_mode_switch waits on AUX, so that returns only once the packet has
 * gone out on air. If not, drive M0/M1 directly rather than asking a module we
 * never established contact with.
 */
static void lora_end(const bool active) {

    if (active) {
        (void)uart_wait_tx_done(E22_UART, pdMS_TO_TICKS(LORA_DRAIN_TIMEOUT_MS));
        __SLEEP_MS(LORA_SETTLE_MS);
        if (!device_mode_deepsleep())
            ESP_LOGW(__tag_lora, "device_mode_deepsleep failed");
    } else
        e22_set_pin_mx(true, true);
    serial_disconnect();
}

// -----------------------------------------------------------------------------------------------------------------------------------------
/*
 * Deep sleep leaves the pads floating, which would drop the module out of the
 * sleep mode just selected; hold M0/M1 so it stays there (~2uA) for as long as
 * we do. The hold is released on the next wake, at the top of lora_gpio_init.
 */
static void lora_hold(void) {
    (void)gpio_hold_en(PIN_E22_M0);
    (void)gpio_hold_en(PIN_E22_M1);
    gpio_deep_sleep_hold_en();
}

// -----------------------------------------------------------------------------------------------------------------------------------------
