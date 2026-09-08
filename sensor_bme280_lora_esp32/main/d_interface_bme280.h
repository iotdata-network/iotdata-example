
// -----------------------------------------------------------------------------------------------------------------------------------------
// BME280 — Temperature / Pressure / Humidity (I2C, forced mode)
// https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
// -----------------------------------------------------------------------------------------------------------------------------------------

static const char *__tag_bme280 = "bme280";

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

// -----------------------------------------------------------------------------------------------------------------------------------------

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

// -----------------------------------------------------------------------------------------------------------------------------------------

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
