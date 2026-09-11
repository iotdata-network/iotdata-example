
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

// XXX MOVE
#define IOTDATA_GATEWAY_TOPIC_REQ  "/manage/req"
#define IOTDATA_GATEWAY_TOPIC_RESP "/manage/resp"

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

__attribute__((format(printf, 3, 4))) static inline const char *snprintf_inline(char *buf, size_t size, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    (void)vsnprintf(buf, size, fmt, args);
    va_end(args);
    return buf;
}

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
    static const char hexdigit[] = "0123456789ABCDEF";
    for (size_t offset = 0; offset < length; offset += 16) {
        char hex[(16 * 3) + 2], asc[16 + 2];
        size_t h = 0, a = 0;
        for (size_t i = 0; i < 16; i++) {
            if (i == 8) { 
                hex[h++] = ' ';
                asc[a++] = ' ';
            }
            if (offset + i < length) {
                const uint8_t b = data[offset + i];
                hex[h++] = hexdigit[b >> 4];
                hex[h++] = hexdigit[b & 0x0F];
                hex[h++] = ' ';
                asc[a++] = isprint(b) ? (char)b : '.';
            } else {
                hex[h++] = ' ';
                hex[h++] = ' ';
                hex[h++] = ' ';
                asc[a++] = ' ';
            }
        }
        hex[h] = '\0';
        asc[a] = '\0';
        PRINTF_INFO("%s[%04X] %s %s\n", prefix ? prefix : "", (unsigned)offset, hex, asc);
    }
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
