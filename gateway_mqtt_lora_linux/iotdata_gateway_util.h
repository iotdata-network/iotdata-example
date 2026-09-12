
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

time_t intervalable(const time_t interval, time_t *last) {
    time_t now = time(NULL);
    if (*last == 0) {
        *last = now;
        return 0;
    }
    if ((now - *last) > interval) {
        const time_t diff = now - *last;
        *last = now;
        return diff;
    }
    return 0;
}

time_t intervalable_and_initial(const time_t interval, time_t *last) {
    time_t now = time(NULL);
    if (*last == 0) {
        *last = now;
        return 1;
    }
    if ((now - *last) > interval) {
        const time_t diff = now - *last;
        *last = now;
        return diff;
    }
    return 0;
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
void ema_update_timed(uint8_t value, uint8_t *value_ema, uint32_t *value_cnt, time_t *value_last_time, time_t now, float tau_secs) {
    if ((*value_cnt)++ == 0 || *value_last_time == 0) {
        *value_ema = value;
        *value_last_time = now;
    } else {
        const float alpha = 1.0f - expf(-((float)(now - *value_last_time)) / tau_secs);
        *value_ema = (uint8_t)((alpha * (float)value + (1.0f - alpha) * (float)(*value_ema)) + 0.5f);
        *value_last_time = now;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
