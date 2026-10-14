
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

bool intervalable(const uint16_t interval_s, uint32_t *const last_ms, const uint32_t now_ms) {
    if (*last_ms == 0u) {
        *last_ms = now_ms;
        return false; /* the first call only primes: the first pass is one interval from now */
    }
    if ((now_ms - *last_ms) > (uint32_t)interval_s * 1000u) {
        *last_ms = now_ms;
        return true;
    }
    return false;
}

bool intervalable_and_initial(const uint16_t interval_s, uint32_t *const last_ms, const uint32_t now_ms) {
    if (*last_ms == 0u) {
        *last_ms = now_ms;
        return true; /* and this one fires on it, so a caller that must act at once does */
    }
    if ((now_ms - *last_ms) > (uint32_t)interval_s * 1000u) {
        *last_ms = now_ms;
        return true;
    }
    return false;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

void debug_hexdump(const char *prefix, const uint8_t *data, size_t length) {
    char line[FORMAT_HEXDUMP_LINE_MAX];
    for (size_t off = 0; format_hexdump_line(line, sizeof(line), data, length, off); off += FORMAT_HEXDUMP_COLUMNS)
        PRINTF_INFO("%s%s\n", prefix ? prefix : "", line);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* Time-weighted EMA: alpha = 1 - exp(-dt / tau). First sample initialises. */
void ema_update_timed(int16_t value, int16_t *value_ema, uint32_t *value_cnt, time_t *value_last_time, time_t now, float tau_secs) {
    if ((*value_cnt)++ == 0 || *value_last_time == 0) {
        *value_ema = value;
        *value_last_time = now;
    } else {
        const float alpha = 1.0f - expf(-((float)(now - *value_last_time)) / tau_secs);
        *value_ema = (int16_t)((alpha * (float)value + (1.0f - alpha) * (float)(*value_ema)) + (value < 0 ? -0.5f : 0.5f));
        *value_last_time = now;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define GW_TOPIC_STATION_MAX 160
static inline const char *gateway_topic_for_station(char *const out, const size_t size, const char *const base, const uint16_t station) {
    return snprintf_inline(out, size, "%s/%03" PRIX16, base, station);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static inline void debug_inject_random_latency_ms(unsigned ms) {
    if (ms > 0)
        hw_delay_ms_yieldable((uint32_t)rand() % (ms + 1)); /* the seam, not a raw nanosleep: yieldable, because this waits for hundreds of ms */
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
