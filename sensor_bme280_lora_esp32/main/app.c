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

#include "iotdata_config.h" // shared IOTDATA_CONFIG_* knobs (+ host-local iotdata_config.<host>.h override)

// -----------------------------------------------------------------------------------------------------------------------------------------
// Application configuration
// -----------------------------------------------------------------------------------------------------------------------------------------

#define TX_PERIOD_MS     IOTDATA_CONFIG_SENSOR_TX_PERIOD_MS /* one measure+transmit cycle per period */
#define TX_PERIOD_MIN_MS 1000                               /* floor, if a cycle overruns the period  */
#define STARTUP_DELAY_MS (5 * 1000)                         /* cold boot only: let the USB console attach */
#define CONSOLE_DRAIN_MS 100                                /* let the console flush before deep sleep    */
#define PACKET_MAX       64                                 /* the packets built here are a dozen bytes   */

// -----------------------------------------------------------------------------------------------------------------------------------------
// GPIO and UART configuration
// -----------------------------------------------------------------------------------------------------------------------------------------

#if IOTDATA_CONFIG_CARRIER

// GEN-LORA-I2C carrier PCB. Avoids USB-Serial-JTAG (GPIO18/19) and the strapping pins (GPIO2/8/9).
// The BME280 goes on the carrier's 4-pin I2C sensor header, which carries 4.7k pull-ups to 3V3.
#define PIN_E22_M0      GPIO_NUM_4  /* E22 pin (1) */
#define PIN_E22_M1      GPIO_NUM_21 /* E22 pin (2) */
#define PIN_E22_RXD     GPIO_NUM_20 /* E22 pin (3) ESP TX -> module RXD */
#define PIN_E22_TXD     GPIO_NUM_10 /* E22 pin (4) module TXD -> ESP RX */
#define PIN_E22_AUX     GPIO_NUM_7  /* E22 pin (5) */
#define PIN_E22_VCC                 /* E22 pin (6) */
#define PIN_E22_GND                 /* E22 pin (7) */
#define PIN_BME280_SDA  GPIO_NUM_5  /* I2C header pin (3) */
#define PIN_BME280_SCL  GPIO_NUM_6  /* I2C header pin (4) */
#define PIN_BATTERY_ADC GPIO_NUM_3  /* patch header — divider midpoint (ADC1_CH3) */
#define PIN_BATTERY_EN  GPIO_NUM_1  /* patch header — divider enable, high = on    */

#else

// Original bench wiring.
#define PIN_E22_M0      GPIO_NUM_5 /* E22 pin (1) */
#define PIN_E22_M1      GPIO_NUM_6 /* E22 pin (2) */
#define PIN_E22_RXD     GPIO_NUM_7 /* E22 pin (3) ESP TX -> module RXD */
#define PIN_E22_TXD     GPIO_NUM_8 /* E22 pin (4) module TXD -> ESP RX */
#define PIN_E22_AUX     GPIO_NUM_9 /* E22 pin (5) */
#define PIN_E22_VCC                /* E22 pin (6) */
#define PIN_E22_GND                /* E22 pin (7) */
#define PIN_BME280_SDA  GPIO_NUM_3
#define PIN_BME280_SCL  GPIO_NUM_4
#define PIN_BATTERY_ADC GPIO_NUM_0 /* divider midpoint (ADC1_CH0) */
#define PIN_BATTERY_EN  GPIO_NUM_1 /* divider enable, high = on   */

#endif

#define E22_UART          UART_NUM_1
#define E22_UART_BUF_SIZE 512

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static const char *__tag_app = "app";
static const char *__tag_lora = "lora";
static const char *__tag_bme280 = "bme280";
static const char *__tag_battery = "battery";

/*
 * Every cooperative wait in this app funnels through __SLEEP_MS — directly, or
 * via __sleep_ms() which the e22 driver calls (its AUX wait loop and serial
 * read timeouts are sleep-based, not busy loops). Patting the Task Watchdog
 * Timer here therefore feeds it from within any blocking-but-yielding wait, so
 * the TWDT only fires on a genuine no-yield hang. Long sleeps are chunked so
 * even a single multi-second delay keeps patting inside the timeout window.
 */
#define __WDT_FEED_MS 1000U
#define __SLEEP_MS(ms) \
    do { \
        uint32_t __wdt_remain = (uint32_t)(ms); \
        do { \
            const uint32_t __wdt_chunk = __wdt_remain < __WDT_FEED_MS ? __wdt_remain : __WDT_FEED_MS; \
            (void)esp_task_wdt_reset(); \
            vTaskDelay(pdMS_TO_TICKS(__wdt_chunk)); \
            __wdt_remain -= __wdt_chunk; \
        } while (__wdt_remain > 0); \
    } while (0)

/* Fixed-point rendering for logs: 2135 -> "21.35". Avoids pulling in float printf. */
#define CENTI_STR_MAX 16
static const char *centi_str(char *const buf, const size_t size, const int32_t centi) {
    const int32_t abs = centi < 0 ? -centi : centi;
    (void)snprintf(buf, size, "%s%" PRId32 ".%02" PRId32, centi < 0 ? "-" : "", abs / 100, abs % 100);
    return buf;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// EBYTE E22-xxxTxx serial transport
// -----------------------------------------------------------------------------------------------------------------------------------------

/*
 * e22xxxtxx.h builds multi-part output lines (no trailing \n on some calls),
 * so we use printf rather than ESP_LOG which adds prefixes and newlines.
 */
static bool debug_e22 = false;
#define PRINTF_DEBUG(fmt, ...) \
    do { \
        if (debug_e22) \
            printf(fmt, ##__VA_ARGS__); \
    } while (0)
#define PRINTF_INFO  printf
#define PRINTF_ERROR printf

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
// BME280 — Temperature / Pressure / Humidity (I2C, forced mode)
// https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
// -----------------------------------------------------------------------------------------------------------------------------------------

#define BME280_I2C_ADDR             0x76
#define BME280_I2C_FREQ             100000
#define BME280_I2C_TIMEOUT_MS       100

#define BME280_CHIP_ID_BME280       0x60
#define BME280_CHIP_ID_BMP280       0x58

#define BME280_REG_CHIP_ID          0xD0
#define BME280_REG_CTRL_HUMI        0xF2
#define BME280_REG_STATUS           0xF3
#define BME280_REG_CTRL_MEAS        0xF4
#define BME280_REG_DATA             0xF7 /* 8 bytes: press[2:0] temp[2:0] humi[1:0] */
#define BME280_REG_CALIB_T_P        0x88 /* 26 bytes */
#define BME280_REG_CALIB_H1         0xA1 /* 1 byte   */
#define BME280_REG_CALIB_H2         0xE1 /* 7 bytes  */

#define BME280_STATUS_BIT_IM_UPDATE 0x01 /* calibration copy in progress */
#define BME280_STATUS_BIT_MEASURING 0x08
#define BME280_MODE_SLEEP           0x00
#define BME280_MODE_FORCED          0x01
#define BME280_OSRS_1X              0x01
#define BME280_CTRL_HUMI            (BME280_OSRS_1X)
#define BME280_CTRL_MEAS            ((BME280_OSRS_1X << 5) | (BME280_OSRS_1X << 2) | BME280_MODE_FORCED)

#define BME280_READY_DELAY_MS       2
#define BME280_READY_CYCLES         30 /* startup: wait for im_update to clear    */
#define BME280_MEASURE_DELAY_MS     2
#define BME280_MEASURE_CYCLES       10 /* forced 1x on all channels takes ~8ms    */

/*
 * Multi-sample strategy: take N measurements, drop the first few (forced mode settles by the second
 * reading), discard anything outside a gross sanity range, average the rest. The full version there
 * also does median-relative outlier rejection; at 1x oversampling over a few hundred milliseconds
 * the plain mean is good enough for an example.
 */
#define BME280_SAMPLES              5
#define BME280_SAMPLES_DISCARD      2

#define BME280_TEMP_MIN             (-5000) /* centi-degC */
#define BME280_TEMP_MAX             (8000)
#define BME280_PRES_MIN             (30000) /* Pa */
#define BME280_PRES_MAX             (110000)
#define BME280_HUMI_MIN             (0) /* centi-%RH */
#define BME280_HUMI_MAX             (10000)

// -----------------------------------------------------------------------------------------------------------------------------------------

typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4, dig_H5;
    int8_t dig_H6;
    int32_t t_fine; /* shared state between the T and the P/H compensation */
} bme280_calib_t;

/*
 * Readings stay in the sensor's own fixed-point units all the way to the
 * encoder: the Bosch compensation formulas are integer, and iotdata is built
 * here with IOTDATA_NO_FLOATING (centi-units), so no float ever appears.
 */
typedef struct {
    int32_t temperature_c100; /* centi-degC   */
    int32_t pressure_pa;      /* Pa           */
    int32_t humidity_pct100;  /* centi-%RH    */
} bme280_reading_t;

static i2c_master_bus_handle_t bme280_i2c_bus = NULL;
static i2c_master_dev_handle_t bme280_i2c_dev = NULL;

// -----------------------------------------------------------------------------------------------------------------------------------------

static bool bme280_read(const uint8_t reg, uint8_t *const data, const size_t length) {
    const esp_err_t err = i2c_master_transmit_receive(bme280_i2c_dev, &reg, 1, data, length, BME280_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(__tag_bme280, "i2c read (reg=0x%02" PRIX8 ", len=%u): %s", reg, (unsigned)length, esp_err_to_name(err));
        return false;
    }
    return true;
}

static bool bme280_write(const uint8_t reg, const uint8_t value) {
    const esp_err_t err = i2c_master_transmit(bme280_i2c_dev, (uint8_t[2]){ reg, value }, 2, BME280_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(__tag_bme280, "i2c write (reg=0x%02" PRIX8 ", val=0x%02" PRIX8 "): %s", reg, value, esp_err_to_name(err));
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static bool bme280_begin(void) {

    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0, .sda_io_num = PIN_BME280_SDA, .scl_io_num = PIN_BME280_SCL, .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7, .flags.enable_internal_pullup = true, /* harmless next to the carrier's 4.7k externals */
    };
    esp_err_t err;
    if ((err = i2c_new_master_bus(&bus_config, &bme280_i2c_bus)) != ESP_OK) {
        ESP_LOGE(__tag_bme280, "i2c_new_master_bus: %s", esp_err_to_name(err));
        bme280_i2c_bus = NULL;
        return false;
    }
    const i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME280_I2C_ADDR,
        .scl_speed_hz = BME280_I2C_FREQ,
    };
    if ((err = i2c_master_bus_add_device(bme280_i2c_bus, &dev_config, &bme280_i2c_dev)) != ESP_OK) {
        ESP_LOGE(__tag_bme280, "i2c_master_bus_add_device: %s", esp_err_to_name(err));
        (void)i2c_del_master_bus(bme280_i2c_bus);
        bme280_i2c_bus = NULL;
        bme280_i2c_dev = NULL;
        return false;
    }
    return true;
}

static void bme280_end(void) {

    /* Forced mode returns to sleep by itself after each measurement, but a failed
       cycle can leave it elsewhere; make the low power state explicit either way.
     */
    if (bme280_i2c_dev) {
        (void)bme280_write(BME280_REG_CTRL_MEAS, BME280_MODE_SLEEP);
        (void)i2c_master_bus_rm_device(bme280_i2c_dev);
        bme280_i2c_dev = NULL;
    }
    if (bme280_i2c_bus) {
        (void)i2c_del_master_bus(bme280_i2c_bus);
        bme280_i2c_bus = NULL;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* Chip identity and factory calibration: read once per power cycle, then cached
 * in RTC memory. */
static bool bme280_identify(uint8_t *const chip_id, bme280_calib_t *const calib) {

    if (!bme280_read(BME280_REG_CHIP_ID, chip_id, 1))
        return false;
    if (*chip_id != BME280_CHIP_ID_BME280) {
        ESP_LOGE(__tag_bme280, "chip-id invalid: 0x%02" PRIX8 " (expected 0x%02" PRIX8 "%s)", *chip_id, BME280_CHIP_ID_BME280, *chip_id == BME280_CHIP_ID_BMP280 ? ", this is a bmp280: no humidity" : "");
        return false;
    }

    /* The sensor copies its calibration from NVM to registers at power on;
       im_update is set while that is in flight, so wait it out before reading. */
    bool ready = false;
    for (int i = 0; i < BME280_READY_CYCLES && !ready; i++) {
        uint8_t status;
        if (!bme280_read(BME280_REG_STATUS, &status, 1))
            return false;
        if ((status & BME280_STATUS_BIT_IM_UPDATE) == 0)
            ready = true;
        else
            esp_rom_delay_us(BME280_READY_DELAY_MS * 1000);
    }
    if (!ready) {
        ESP_LOGE(__tag_bme280, "timeout waiting for calibration copy (im_update)");
        return false;
    }

    /* Temperature and pressure calibration (0x88..0xA1) */
    uint8_t buf1[26];
    if (!bme280_read(BME280_REG_CALIB_T_P, buf1, sizeof(buf1)))
        return false;
    calib->dig_T1 = (uint16_t)(buf1[1] << 8 | buf1[0]);
    calib->dig_T2 = (int16_t)(buf1[3] << 8 | buf1[2]);
    calib->dig_T3 = (int16_t)(buf1[5] << 8 | buf1[4]);
    calib->dig_P1 = (uint16_t)(buf1[7] << 8 | buf1[6]);
    calib->dig_P2 = (int16_t)(buf1[9] << 8 | buf1[8]);
    calib->dig_P3 = (int16_t)(buf1[11] << 8 | buf1[10]);
    calib->dig_P4 = (int16_t)(buf1[13] << 8 | buf1[12]);
    calib->dig_P5 = (int16_t)(buf1[15] << 8 | buf1[14]);
    calib->dig_P6 = (int16_t)(buf1[17] << 8 | buf1[16]);
    calib->dig_P7 = (int16_t)(buf1[19] << 8 | buf1[18]);
    calib->dig_P8 = (int16_t)(buf1[21] << 8 | buf1[20]);
    calib->dig_P9 = (int16_t)(buf1[23] << 8 | buf1[22]);

    /* Humidity calibration, part 1 (0xA1) and part 2 (0xE1..0xE7) */
    if (!bme280_read(BME280_REG_CALIB_H1, &calib->dig_H1, 1))
        return false;
    uint8_t buf2[7];
    if (!bme280_read(BME280_REG_CALIB_H2, buf2, sizeof(buf2)))
        return false;
    calib->dig_H2 = (int16_t)(buf2[1] << 8 | buf2[0]);
    calib->dig_H3 = buf2[2];
    calib->dig_H4 = (int16_t)((int16_t)buf2[3] << 4 | (buf2[4] & 0x0F));
    calib->dig_H5 = (int16_t)((int16_t)buf2[5] << 4 | (buf2[4] >> 4));
    calib->dig_H6 = (int8_t)buf2[6];

    ESP_LOGI(__tag_bme280, "identified: chip-id=0x%02" PRIX8 ", calibration read", *chip_id);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// Compensation — the Bosch reference integer algorithms, kept in fixed point
// -----------------------------------------------------------------------------------------------------------------------------------------

/* Returns centi-degC, and sets calib->t_fine for the pressure/humidity compensation. */
static int32_t bme280_compensate_temperature(bme280_calib_t *const calib, const int32_t adc_T) {
    int64_t var1 = (((int64_t)(adc_T >> 3) - ((int64_t)calib->dig_T1 << 1)) * (int64_t)calib->dig_T2) >> 11;
    const int64_t diff = (int64_t)(adc_T >> 4) - (int64_t)calib->dig_T1;
    int64_t var2 = (int64_t)((uint64_t)diff * (uint64_t)diff) >> 12;
    var2 = (var2 * (int64_t)calib->dig_T3) >> 14;
    calib->t_fine = (int32_t)(var1 + var2);
    return (calib->t_fine * 5 + 128) >> 8;
}

/* Returns Pa (the reference algorithm yields Q24.8 Pa; the fraction is well below our resolution). */
static int32_t bme280_compensate_pressure(const bme280_calib_t *const calib, const int32_t adc_P) {
    int64_t var1 = (int64_t)calib->t_fine - 128000;
    int64_t var2 = (int64_t)((uint64_t)var1 * (uint64_t)var1) * (int64_t)calib->dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib->dig_P5) << 17);
    var2 = var2 + (((int64_t)calib->dig_P4) << 35);
    var1 = ((int64_t)((uint64_t)var1 * (uint64_t)var1) * (int64_t)calib->dig_P3 >> 8) + ((var1 * (int64_t)calib->dig_P2) << 12);
    var1 = ((((int64_t)1) << 47) + var1) * ((int64_t)calib->dig_P1) >> 33;
    if (var1 == 0)
        return 0; /* rejected by the range check */
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)calib->dig_P9 * (int64_t)((uint64_t)(p >> 13) * (uint64_t)(p >> 13))) >> 25;
    var2 = ((int64_t)calib->dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib->dig_P7) << 4);
    return (int32_t)((uint32_t)p >> 8);
}

/* Returns centi-%RH (the reference algorithm yields Q22.10 %RH). */
static int32_t bme280_compensate_humidity(const bme280_calib_t *const calib, const int32_t adc_H) {
    int32_t v = calib->t_fine - 76800;
    v = (((((adc_H << 14) - (((int32_t)calib->dig_H4) << 20) - (((int32_t)calib->dig_H5) * v)) + 16384) >> 15) *
         (((((((v * ((int32_t)calib->dig_H6)) >> 10) * (((v * ((int32_t)calib->dig_H3)) >> 11) + 32768)) >> 10) + 2097152) * ((int32_t)calib->dig_H2) + 8192) >> 14));
    const int32_t sq = (int32_t)((uint32_t)(v >> 15) * (uint32_t)(v >> 15));
    v = v - (((sq >> 7) * ((int32_t)calib->dig_H1)) >> 4);
    v = (v < 0) ? 0 : v;
    v = (v > 419430400) ? 419430400 : v;
    return (int32_t)(((uint32_t)v >> 12) * 100 / 1024);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* One forced-mode conversion: trigger, wait for the measuring bit to clear, read the raw ADC words. */
static bool bme280_measure_raw(int32_t *const adc_T, int32_t *const adc_P, int32_t *const adc_H) {

    /* ctrl_hum only takes effect on the following ctrl_meas write, so order matters */
    if (!bme280_write(BME280_REG_CTRL_HUMI, BME280_CTRL_HUMI) || !bme280_write(BME280_REG_CTRL_MEAS, BME280_CTRL_MEAS))
        return false;
    for (int i = 0; i < BME280_MEASURE_CYCLES; i++) {
        esp_rom_delay_us(BME280_MEASURE_DELAY_MS * 1000);
        uint8_t status;
        if (!bme280_read(BME280_REG_STATUS, &status, 1))
            return false;
        if ((status & BME280_STATUS_BIT_MEASURING) == 0)
            break;
    }

    uint8_t data[8];
    if (!bme280_read(BME280_REG_DATA, data, sizeof(data)))
        return false;
    *adc_P = (int32_t)((uint32_t)data[0] << 12 | (uint32_t)data[1] << 4 | data[2] >> 4);
    *adc_T = (int32_t)((uint32_t)data[3] << 12 | (uint32_t)data[4] << 4 | data[5] >> 4);
    *adc_H = (int32_t)((uint32_t)data[6] << 8 | data[7]);
    return true;
}

/* Discard the settling samples, drop anything outside the sanity range, average the rest. */
static bool bme280_reduce(const int32_t *const values, const int count, const int discard, const int32_t min, const int32_t max, int32_t *const out) {
    int64_t sum = 0;
    int accepted = 0;
    for (int i = discard; i < count; i++)
        if (values[i] >= min && values[i] <= max) {
            sum += values[i];
            accepted++;
        } else
            ESP_LOGW(__tag_bme280, "sample %d rejected: %" PRId32 " outside %" PRId32 "..%" PRId32, i, values[i], min, max);
    if (accepted == 0)
        return false;
    *out = (int32_t)(sum / accepted);
    return true;
}

static bool bme280_measure(bme280_calib_t *const calib, bme280_reading_t *const out) {

    int32_t temperature[BME280_SAMPLES], pressure[BME280_SAMPLES], humidity[BME280_SAMPLES];
    int count = 0;
    for (int i = 0; i < BME280_SAMPLES; i++) {
        int32_t adc_T, adc_P, adc_H;
        if (!bme280_measure_raw(&adc_T, &adc_P, &adc_H)) {
            ESP_LOGW(__tag_bme280, "sample %d failed", i);
            continue;
        }
        /* temperature first: it computes t_fine, which the other two compensations use */
        temperature[count] = bme280_compensate_temperature(calib, adc_T);
        pressure[count] = bme280_compensate_pressure(calib, adc_P);
        humidity[count] = bme280_compensate_humidity(calib, adc_H);
        count++;
    }
    if (count == 0) {
        ESP_LOGE(__tag_bme280, "no samples completed");
        return false;
    }

    const int discard = (count > BME280_SAMPLES_DISCARD) ? BME280_SAMPLES_DISCARD : 0;
    if (!bme280_reduce(temperature, count, discard, BME280_TEMP_MIN, BME280_TEMP_MAX, &out->temperature_c100) || !bme280_reduce(pressure, count, discard, BME280_PRES_MIN, BME280_PRES_MAX, &out->pressure_pa) ||
        !bme280_reduce(humidity, count, discard, BME280_HUMI_MIN, BME280_HUMI_MAX, &out->humidity_pct100)) {
        ESP_LOGE(__tag_bme280, "no acceptable readings from %d samples", count);
        return false;
    }

    char sb1[CENTI_STR_MAX], sb2[CENTI_STR_MAX];
    ESP_LOGI(__tag_bme280, "measured: T=%s degC P=%" PRId32 " Pa H=%s %%RH (%d/%d samples)", centi_str(sb1, sizeof(sb1), out->temperature_c100), out->pressure_pa, centi_str(sb2, sizeof(sb2), out->humidity_pct100), count - discard,
             BME280_SAMPLES);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// Battery voltage divider (optional)
// -----------------------------------------------------------------------------------------------------------------------------------------

/*
 * A switched resistive divider from the battery to an ADC pin. Not every carrier has one
 * fitted, so the first boot after a restart probes for it and records the answer in the
 * RTC-retained state; when it is absent the ADC is never brought up again and the packet
 * simply carries no battery field. Nothing else in the app changes.
 *
 *   BAT+ --[P-FET]-- R1 --+-- R2 -- GND        EN --[R]-- NPN base (base pulled down to GND)
 *                         |
 *                         +-- C1 -- GND
 *                         |
 *                       ADC in
 *
 * R1 = R2 = 330k gives a ratio of 2 and draws 6.4uA, but only while enabled. 330k alone
 * cannot source the ADC's sample-and-hold charge, so C1 = 100nF sits across R2 and supplies
 * it; the pair settle with tau = (R1||R2) * C1 = 16.5ms, hence the 50ms wait below.
 *
 * The NPN inverts, so EN high = divider powered. Its base needs an external pull-down: the
 * pin is an input during boot and floats in deep sleep, and a divider left switched on would
 * drain the pack for exactly as long as nobody was looking.
 */

#define BATTERY_DIVIDER_RATIO_X100     200 /* (R1 + R2) / R2, x100 — 330k / 330k = 2.00       */
#define BATTERY_SETTLE_MS              50  /* 3 tau of (R1||R2) * C1, i.e. 95% settled        */
#define BATTERY_SAMPLE_COUNT           8
#define BATTERY_SAMPLE_INTERVAL_US     2000
#define BATTERY_ADC_ATTEN              ADC_ATTEN_DB_12 /* ~0-2500mV, so a 4.2V cell reads 2.1V */
#define BATTERY_ADC_BITWIDTH           ADC_BITWIDTH_12
#define BATTERY_ADC_MAX_MV             2500
#define BATTERY_ADC_MAX_RAW            4095

/* Li-ion / LiPo. LiFePO4 would be 2500..3650 with a much flatter curve between them. */
#define BATTERY_MV_MIN                 3000
#define BATTERY_MV_KNEE                3500 /* the two points where a Li-ion discharge curve bends */
#define BATTERY_MV_MID                 3700
#define BATTERY_MV_MAX                 4200

/* Below this the node is close enough to the end that the gateway should hear about it. */
#define BATTERY_PCT_LOW                20

/* A reading this much above the last one is taken as charging; see battery_read(). */
#define BATTERY_CHARGING_HYSTERESIS_MV 20

/* Probe limits. With the divider fitted and off, R2 holds the pin at ground; with it on,
   the pin sits at half a plausible battery. Absent, the pin floats and reads arbitrarily,
   but it cannot follow the enable line — which is what the delta test looks for. */
#define BATTERY_PROBE_OFF_MAX_RAW      200
#define BATTERY_PROBE_DELTA_MIN_RAW    500
#define BATTERY_PROBE_MV_MIN           2000
#define BATTERY_PROBE_MV_MAX           4500

typedef struct {
    int voltage_mv;
    uint8_t percent;
    bool charging;
} battery_reading_t;

static adc_oneshot_unit_handle_t battery_adc = NULL;
static adc_cali_handle_t battery_cali = NULL;
static adc_channel_t battery_channel;

// -----------------------------------------------------------------------------------------------------------------------------------------

static bool battery_begin(void) {

    const gpio_config_t en_config = {
        .pin_bit_mask = BIT64(PIN_BATTERY_EN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE, /* belt and braces with the external base pull-down */
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err;
    if ((err = gpio_config(&en_config)) != ESP_OK) {
        ESP_LOGE(__tag_battery, "gpio_config(en): %s", esp_err_to_name(err));
        return false;
    }
    (void)gpio_set_level(PIN_BATTERY_EN, 0);

    adc_unit_t unit;
    if ((err = adc_oneshot_io_to_channel(PIN_BATTERY_ADC, &unit, &battery_channel)) != ESP_OK) {
        ESP_LOGE(__tag_battery, "gpio %d is not an ADC pin: %s", (int)PIN_BATTERY_ADC, esp_err_to_name(err));
        return false;
    }
    if (unit != ADC_UNIT_1) {
        /* ADC2 shares its hardware with the radio on some parts; this app only uses ADC1. */
        ESP_LOGE(__tag_battery, "gpio %d is on ADC%d, expected ADC1", (int)PIN_BATTERY_ADC, (int)unit + 1);
        return false;
    }

    const adc_oneshot_unit_init_cfg_t unit_config = { .unit_id = ADC_UNIT_1 };
    if ((err = adc_oneshot_new_unit(&unit_config, &battery_adc)) != ESP_OK) {
        ESP_LOGE(__tag_battery, "adc_oneshot_new_unit: %s", esp_err_to_name(err));
        battery_adc = NULL;
        return false;
    }
    const adc_oneshot_chan_cfg_t chan_config = { .atten = BATTERY_ADC_ATTEN, .bitwidth = BATTERY_ADC_BITWIDTH };
    if ((err = adc_oneshot_config_channel(battery_adc, battery_channel, &chan_config)) != ESP_OK) {
        ESP_LOGE(__tag_battery, "adc_oneshot_config_channel: %s", esp_err_to_name(err));
        (void)adc_oneshot_del_unit(battery_adc);
        battery_adc = NULL;
        return false;
    }

    /* Raw ADC counts are a few percent out and vary chip to chip; the eFuse calibration
       corrects that. Without it the readings still track, they just sit off by an offset. */
    battery_cali = NULL;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    const adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = BATTERY_ADC_ATTEN,
        .bitwidth = BATTERY_ADC_BITWIDTH,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_config, &battery_cali) != ESP_OK)
        battery_cali = NULL;
#endif
    if (battery_cali == NULL)
        ESP_LOGW(__tag_battery, "no eFuse calibration: voltages are approximate");

    return true;
}

static void battery_end(void) {

    (void)gpio_set_level(PIN_BATTERY_EN, 0);
    /* Deliberately not gpio_reset_pin(): that re-enables the internal pull-up, which would
       hold the divider on and quietly drain the pack. The pin is left driving low, and deep
       sleep floats it onto the external base pull-down. */

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (battery_cali != NULL) {
        (void)adc_cali_delete_scheme_curve_fitting(battery_cali);
        battery_cali = NULL;
    }
#endif
    if (battery_adc != NULL) {
        (void)adc_oneshot_del_unit(battery_adc);
        battery_adc = NULL;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static bool battery_adc_raw(int *const raw) {
    const esp_err_t err = adc_oneshot_read(battery_adc, battery_channel, raw);
    if (err != ESP_OK) {
        ESP_LOGE(__tag_battery, "adc_oneshot_read: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

static int battery_adc_mv(const int raw) {
    int mv;
    if (battery_cali != NULL && adc_cali_raw_to_voltage(battery_cali, raw, &mv) == ESP_OK)
        return mv;
    return (raw * BATTERY_ADC_MAX_MV) / BATTERY_ADC_MAX_RAW;
}

/* Enable the divider, let the RC settle, average a burst, switch it off again.
   Returns the battery voltage in mV, or -1 if the ADC would not answer. */
static int battery_measure_mv(void) {

    (void)gpio_set_level(PIN_BATTERY_EN, 1);
    __SLEEP_MS(BATTERY_SETTLE_MS);

    int sum = 0, count = 0;
    for (int i = 0; i < BATTERY_SAMPLE_COUNT; i++) {
        int raw;
        esp_rom_delay_us(BATTERY_SAMPLE_INTERVAL_US);
        if (battery_adc_raw(&raw)) {
            sum += battery_adc_mv(raw);
            count++;
        }
    }

    (void)gpio_set_level(PIN_BATTERY_EN, 0);

    if (count == 0)
        return -1;
    return ((sum / count) * BATTERY_DIVIDER_RATIO_X100) / 100;
}

/* Piecewise, because a Li-ion cell does not discharge linearly: it falls quickly off the
   top, sits on a long plateau, then drops away below the knee. Straight-line interpolation
   over the whole range would read ~50% for most of the life and then fall off a cliff. */
static uint8_t battery_percent(const int mv) {

    if (mv <= BATTERY_MV_MIN)
        return 0;
    if (mv >= BATTERY_MV_MAX)
        return 100;

    int pct;
    if (mv > BATTERY_MV_MID)
        pct = 50 + ((mv - BATTERY_MV_MID) * 50) / (BATTERY_MV_MAX - BATTERY_MV_MID);
    else if (mv > BATTERY_MV_KNEE)
        pct = 20 + ((mv - BATTERY_MV_KNEE) * 30) / (BATTERY_MV_MID - BATTERY_MV_KNEE);
    else
        pct = ((mv - BATTERY_MV_MIN) * 20) / (BATTERY_MV_KNEE - BATTERY_MV_MIN);

    return (uint8_t)(pct < 0 ? 0 : (pct > 100 ? 100 : pct));
}

/*
 * One reading, plus the charging bit. There is no charge-detect line, so charging is
 * inferred from the pack reading higher than it did last cycle — on a solar node that
 * tracks daylight well enough to be worth a bit, but it is a trend, not a measurement,
 * and it will flicker on noise near the threshold. Drop it to a constant false if that
 * bothers you more than the missing information does.
 */
static bool battery_read(battery_reading_t *const out, int16_t *const previous_mv) {

    const int mv = battery_measure_mv();
    if (mv < 0)
        return false;

    out->voltage_mv = mv;
    out->percent = battery_percent(mv);
    out->charging = (*previous_mv > 0) && (mv > (int)*previous_mv + BATTERY_CHARGING_HYSTERESIS_MV);
    *previous_mv = (int16_t)mv;

    char sb[CENTI_STR_MAX];
    ESP_LOGI(__tag_battery, "measured: %sV (%u%%)%s", centi_str(sb, sizeof(sb), mv / 10), (unsigned)out->percent, out->charging ? " charging" : "");
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/*
 * Is a divider actually fitted? Read with it off, on, and off again. Fitted, the pin is
 * held at ground by R2 and rises to half the pack when switched; unfitted, it floats and
 * cannot track the enable line. Run once per restart, because it costs 150ms of settling
 * and the answer cannot change without someone holding a soldering iron.
 */
static bool battery_probe(void) {

    if (!battery_begin()) {
        battery_end();
        return false;
    }

    int raw_off1 = 0, raw_on = 0, raw_off2 = 0;
    bool read = true;
    (void)gpio_set_level(PIN_BATTERY_EN, 0);
    __SLEEP_MS(BATTERY_SETTLE_MS);
    read = read && battery_adc_raw(&raw_off1);
    (void)gpio_set_level(PIN_BATTERY_EN, 1);
    __SLEEP_MS(BATTERY_SETTLE_MS);
    read = read && battery_adc_raw(&raw_on);
    (void)gpio_set_level(PIN_BATTERY_EN, 0);
    __SLEEP_MS(BATTERY_SETTLE_MS);
    read = read && battery_adc_raw(&raw_off2);

    const int mv = read ? ((battery_adc_mv(raw_on) * BATTERY_DIVIDER_RATIO_X100) / 100) : 0;
    battery_end();

    if (!read) {
        ESP_LOGW(__tag_battery, "probe: ADC unreadable, assuming no divider");
        return false;
    }

    ESP_LOGI(__tag_battery, "probe: raw off=%d on=%d off=%d (%dmV at the pack)", raw_off1, raw_on, raw_off2, mv);

    const char *why = NULL;
    if (raw_off1 > BATTERY_PROBE_OFF_MAX_RAW || raw_off2 > BATTERY_PROBE_OFF_MAX_RAW)
        why = "pin not pulled to ground when off (R2 missing, or the switch is stuck on)";
    else if (raw_on - raw_off1 < BATTERY_PROBE_DELTA_MIN_RAW)
        why = "pin does not follow the enable line (no divider fitted, no cell connected, or the switch is stuck off)";
    else if (mv < BATTERY_PROBE_MV_MIN || mv > BATTERY_PROBE_MV_MAX)
        why = "voltage outside any plausible pack (wrong divider ratio, or no battery)";

    if (why != NULL) {
        ESP_LOGW(__tag_battery, "probe: no usable divider — %s", why);
        return false;
    }

    ESP_LOGI(__tag_battery, "probe: divider present");
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// iotdata — variant suite (unity build, encode-only)
// -----------------------------------------------------------------------------------------------------------------------------------------

/*
 * Strip everything except the encoder for a minimal ESP32 build:
 *   - NO_DECODE:   no decoder (encoder-only)
 *   - NO_JSON:     no cJSON dependency
 *   - NO_DUMP:     no dump output
 *   - NO_PRINT:    no print output
 *   - NO_FLOATING: iotdata_float_t = int32_t (value * 100)
 *
 * NO_FLOATING is what lets the BME280 readings go straight to the encoder in
 * their native fixed point: centi-degC is exactly what the temperature field
 * wants, so nothing is converted and no float support is linked in.
 */
#define IOTDATA_NO_DECODE
#define IOTDATA_NO_JSON
#define IOTDATA_NO_DUMP
#define IOTDATA_NO_PRINT
#define IOTDATA_NO_FLOATING
#include "iotdata.c"
#include "iotdata_variant.h"

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

static bool packet_build(packet_t *const out, const uint16_t station, const uint16_t sequence, const bme280_reading_t *const reading, const battery_reading_t *const battery, uint8_t flags) {

    static iotdata_encoder_t enc; /* too large for a comfortable stack frame */

    iotdata_status_t rc;
    if ((rc = iotdata_encode_begin(&enc, out->buf, sizeof(out->buf), PACKET_VARIANT, station, sequence)) != IOTDATA_OK) {
        ESP_LOGE(__tag_app, "encode_begin: %s", iotdata_strerror(rc));
        return false;
    }

    if (battery != NULL && (rc = iotdata_encode_battery(&enc, battery->percent, battery->charging)) != IOTDATA_OK)
        ESP_LOGW(__tag_app, "encode_battery: %s", iotdata_strerror(rc));

    if (reading != NULL) {
        /* Rounded to the units the field carries: whole hPa, whole %RH. */
        const uint16_t pressure_hpa = (uint16_t)((reading->pressure_pa + 50) / 100);
        const uint8_t humidity_pct = (uint8_t)((reading->humidity_pct100 + 50) / 100);
        if ((rc = iotdata_encode_environment(&enc, (iotdata_float_t)reading->temperature_c100, pressure_hpa, humidity_pct)) != IOTDATA_OK) {
            /* In range for the sensor but out of range for the protocol: report the fault rather than an implausible value. */
            ESP_LOGW(__tag_app, "encode_environment: %s", iotdata_strerror(rc));
            flags |= (uint8_t)(1U << VSUITE_FLAG_SENSOR_FAULTS);
        }
    }

    if (flags != 0 && (rc = iotdata_encode_flags(&enc, flags)) != IOTDATA_OK)
        ESP_LOGW(__tag_app, "encode_flags: %s", iotdata_strerror(rc));

    if ((rc = iotdata_encode_end(&enc, &out->len)) != IOTDATA_OK) {
        ESP_LOGE(__tag_app, "encode_end: %s", iotdata_strerror(rc));
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// State retained across deep sleep
// -----------------------------------------------------------------------------------------------------------------------------------------

#define STATE_MAGIC 0xB1E28002U

typedef struct {
    uint32_t magic;
    uint16_t station_id;    /* derived from the factory MAC: stable per board  */
    uint16_t sequence;      /* rolling packet counter, wraps at 16 bits        */
    uint32_t cycles;        /* wake cycles since the last restart              */
    uint32_t tx_count;      /* packets transmitted                             */
    uint32_t tx_errors;     /* packets the radio would not take                */
    int16_t battery_mv;     /* last reading, for the charging trend bit        */
    bool radio_configured;  /* E22 NVM configuration verified this power cycle */
    bool sensor_calibrated; /* chip_id and calib below are this sensor's       */
    bool battery_present;   /* a divider answered the probe at restart         */
    uint8_t chip_id;
    bme280_calib_t calib;
} state_t;

static RTC_NOINIT_ATTR state_t state;

/* Station ids must fit iotdata's 12-bit station field (1..IOTDATA_STATION_MAX).
   Taking it from the factory MAC makes each board unique and stable across
   reboots, so the same firmware image can be flashed to a whole fleet. */
static uint16_t state_station_id(void) {
    uint8_t mac[6] = { 0 };
    (void)esp_efuse_mac_get_default(mac);
    const uint32_t mac32 = ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8) | mac[5];
    const uint16_t station_id = (uint16_t)((mac32 % IOTDATA_STATION_MAX) + 1);
    ESP_LOGI(__tag_app, "board: mac=%02X:%02X:%02X:%02X:%02X:%02X station=%" PRIu16, (unsigned)mac[0], (unsigned)mac[1], (unsigned)mac[2], (unsigned)mac[3], (unsigned)mac[4], (unsigned)mac[5], station_id);
    return station_id;
}

static void state_reset(void) {
    memset(&state, 0, sizeof(state));
    state.magic = STATE_MAGIC;
    state.station_id = state_station_id();
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// Application
// -----------------------------------------------------------------------------------------------------------------------------------------

static const char *reset_reason_str(esp_reset_reason_t r) {
    switch (r) {
    case ESP_RST_POWERON:
        return "POWERON (cold boot / power cycle)";
    case ESP_RST_EXT:
        return "EXT (external reset pin)";
    case ESP_RST_SW:
        return "SW (esp_restart / software)";
    case ESP_RST_PANIC:
        return "PANIC (exception/abort crash)";
    case ESP_RST_INT_WDT:
        return "INT_WDT (interrupt watchdog)";
    case ESP_RST_TASK_WDT:
        return "TASK_WDT (task watchdog)";
    case ESP_RST_WDT:
        return "WDT (other watchdog)";
    case ESP_RST_DEEPSLEEP:
        return "DEEPSLEEP (wake from deep sleep)";
    case ESP_RST_BROWNOUT:
        return "BROWNOUT (power dip — check USB/cable/supply)";
    case ESP_RST_SDIO:
        return "SDIO";
    case ESP_RST_USB:
        return "USB (reset over USB peripheral)";
    case ESP_RST_JTAG:
        return "JTAG";
    default:
        return "UNKNOWN";
    }
}

/*
 * One cycle: measure, encode, transmit. Each step is allowed to fail without
 * taking the rest down — a sensor that does not answer still produces a packet
 * (flagged as faulty, so the gateway can tell a blind node from a dead one),
 * and a radio that does not answer still leaves the sequence advanced.
 */
static bool app_cycle(void) {

    uint8_t flags = 0;
    if (state.cycles == 0)
        flags |= (uint8_t)(1U << VSUITE_FLAG_RESTART_RECENT);

    /* --- sensor --- */
    bme280_reading_t reading;
    bool measured = false;
    if (bme280_begin()) {
        if (!state.sensor_calibrated)
            state.sensor_calibrated = bme280_identify(&state.chip_id, &state.calib);
        if (state.sensor_calibrated)
            measured = bme280_measure(&state.calib, &reading);
        bme280_end();
    }
    if (!measured) {
        ESP_LOGE(__tag_app, "sensor: no reading this cycle");
        flags |= (uint8_t)(1U << VSUITE_FLAG_SENSOR_FAULTS);
    }

    /* --- battery --- */
    /* Skipped entirely on a carrier with no divider: the probe at restart settled that,
       and the packet goes out without the field rather than with a fabricated one. */
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
    if (!packet_build(&packet, state.station_id, state.sequence, measured ? &reading : NULL, powered ? &battery : NULL, flags))
        return false;
    ESP_LOGI(__tag_app, "packet: variant=%s station=%" PRIu16 " sequence=%" PRIu16 " flags=0x%02" PRIX8, iotdata_vsuite_name(PACKET_VARIANT), state.station_id, state.sequence, flags);
    state.sequence++;

    /* --- radio --- */
    bool transmitted = false;
    const bool ready = lora_begin(!state.radio_configured);
    if (ready) {
        state.radio_configured = true;
        transmitted = lora_transmit(packet.buf, packet.len);
    }
    lora_end(ready); /* on every path: the module must not be left awake for the sleep ahead */

    if (transmitted)
        state.tx_count++;
    else
        state.tx_errors++;
    return transmitted && measured;
}

/* Sleep out the remainder of the period, so the cycle time is TX_PERIOD_MS
   rather than TX_PERIOD_MS plus however long a cycle happened to take. */
static void app_sleep(const int64_t time_start_us) {

    const int64_t awake_ms = (esp_timer_get_time() - time_start_us) / 1000;
    int64_t sleep_ms = (int64_t)TX_PERIOD_MS - awake_ms - CONSOLE_DRAIN_MS;
    if (sleep_ms < TX_PERIOD_MIN_MS)
        sleep_ms = TX_PERIOD_MIN_MS;

    ESP_LOGI(__tag_app, "cycle %" PRIu32 " done: tx=%" PRIu32 " errors=%" PRIu32 " awake=%" PRId64 "ms, sleeping %" PRId64 "ms", state.cycles, state.tx_count, state.tx_errors, awake_ms, sleep_ms);

    /* The USB-Serial-JTAG console goes down with the chip and takes anything still
       queued with it, so give the host a moment to collect this cycle's output. */
    __SLEEP_MS(CONSOLE_DRAIN_MS);

    lora_hold();
    esp_deep_sleep((uint64_t)sleep_ms * 1000);
}

void app_main(void) {

    const int64_t time_start = esp_timer_get_time();

    setbuf(stdout, NULL);

    /*
     * Subscribe this task to the Task Watchdog Timer (the TWDT itself is started
     * at boot via CONFIG_ESP_TASK_WDT_INIT). From here on __SLEEP_MS pats it; if
     * the app stops yielding for CONFIG_ESP_TASK_WDT_TIMEOUT_S the TWDT panics
     * and the chip resets (CONFIG_ESP_TASK_WDT_PANIC), recovering a wedged
     * unattended sensor — the reset reason is logged on the way back up, and the
     * cycle simply repeats.
     */
    const esp_err_t err = esp_task_wdt_add(NULL);
    if (err != ESP_OK)
        ESP_LOGE(__tag_app, "task watchdog: subscribe failed: %s", esp_err_to_name(err));

    const esp_reset_reason_t reset_reason = esp_reset_reason();
    const bool restarted = (reset_reason != ESP_RST_DEEPSLEEP) || state.magic != STATE_MAGIC;

    if (restarted) {
        ESP_LOGI(__tag_app, "iotdata bme280 lora sensor: %s variant, every %us", iotdata_vsuite_name(PACKET_VARIANT), (unsigned)(TX_PERIOD_MS / 1000));
        ESP_LOGI(__tag_app, "boot: reset_reason=%d %s", (int)reset_reason, reset_reason_str(reset_reason));
        state_reset();
        state.battery_present = battery_probe(); /* restart only; see battery_probe() */
        __SLEEP_MS(STARTUP_DELAY_MS);            /* cold boot only, so it costs nothing per cycle */
    }

    if (!app_cycle())
        ESP_LOGE(__tag_app, "cycle %" PRIu32 " incomplete", state.cycles);
    state.cycles++;

    app_sleep(time_start);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
