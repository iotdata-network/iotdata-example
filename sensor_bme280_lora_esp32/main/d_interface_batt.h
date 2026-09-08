
// -----------------------------------------------------------------------------------------------------------------------------------------
// Battery voltage divider (optional)
// -----------------------------------------------------------------------------------------------------------------------------------------

static const char *__tag_batt = "battery";

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
        ESP_LOGE(__tag_batt, "gpio_config(en): %s", esp_err_to_name(err));
        return false;
    }
    (void)gpio_set_level(PIN_BATTERY_EN, 0);

    adc_unit_t unit;
    if ((err = adc_oneshot_io_to_channel(PIN_BATTERY_ADC, &unit, &battery_channel)) != ESP_OK) {
        ESP_LOGE(__tag_batt, "gpio %d is not an ADC pin: %s", (int)PIN_BATTERY_ADC, esp_err_to_name(err));
        return false;
    }
    if (unit != ADC_UNIT_1) {
        /* ADC2 shares its hardware with the radio on some parts; this app only uses ADC1. */
        ESP_LOGE(__tag_batt, "gpio %d is on ADC%d, expected ADC1", (int)PIN_BATTERY_ADC, (int)unit + 1);
        return false;
    }

    const adc_oneshot_unit_init_cfg_t unit_config = { .unit_id = ADC_UNIT_1 };
    if ((err = adc_oneshot_new_unit(&unit_config, &battery_adc)) != ESP_OK) {
        ESP_LOGE(__tag_batt, "adc_oneshot_new_unit: %s", esp_err_to_name(err));
        battery_adc = NULL;
        return false;
    }
    const adc_oneshot_chan_cfg_t chan_config = { .atten = BATTERY_ADC_ATTEN, .bitwidth = BATTERY_ADC_BITWIDTH };
    if ((err = adc_oneshot_config_channel(battery_adc, battery_channel, &chan_config)) != ESP_OK) {
        ESP_LOGE(__tag_batt, "adc_oneshot_config_channel: %s", esp_err_to_name(err));
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
        ESP_LOGW(__tag_batt, "no eFuse calibration: voltages are approximate");

    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

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
        ESP_LOGE(__tag_batt, "adc_oneshot_read: %s", esp_err_to_name(err));
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

// -----------------------------------------------------------------------------------------------------------------------------------------

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

// -----------------------------------------------------------------------------------------------------------------------------------------

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

// -----------------------------------------------------------------------------------------------------------------------------------------

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
    ESP_LOGI(__tag_batt, "measured: %sV (%u%%)%s", centi_str(sb, sizeof(sb), mv / 10), (unsigned)out->percent, out->charging ? " charging" : "");
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
        ESP_LOGW(__tag_batt, "probe: ADC unreadable, assuming no divider");
        return false;
    }

    ESP_LOGI(__tag_batt, "probe: raw off=%d on=%d off=%d (%dmV at the pack)", raw_off1, raw_on, raw_off2, mv);

    const char *why = NULL;
    if (raw_off1 > BATTERY_PROBE_OFF_MAX_RAW || raw_off2 > BATTERY_PROBE_OFF_MAX_RAW)
        why = "pin not pulled to ground when off (R2 missing, or the switch is stuck on)";
    else if (raw_on - raw_off1 < BATTERY_PROBE_DELTA_MIN_RAW)
        why = "pin does not follow the enable line (no divider fitted, no cell connected, or the switch is stuck off)";
    else if (mv < BATTERY_PROBE_MV_MIN || mv > BATTERY_PROBE_MV_MAX)
        why = "voltage outside any plausible pack (wrong divider ratio, or no battery)";

    if (why != NULL) {
        ESP_LOGW(__tag_batt, "probe: no usable divider — %s", why);
        return false;
    }

    ESP_LOGI(__tag_batt, "probe: divider present");
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
