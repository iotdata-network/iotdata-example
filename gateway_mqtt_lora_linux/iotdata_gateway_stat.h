
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include <cjson/cJSON.h>

// -----------------------------------------------------------------------------------------------------------------------------------------

/* Time-weighted EMA: alpha = 1 - exp(-dt / tau). First sample initialises. */
#define EMA_TIMED_TAU_SECS_DEFAULT 300.0f
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

/* These bound the largest allocation in the gateway by a wide margin: a stat_ring_t is
   STAT_RING_SIZE timestamps, and there is one per station plus one per variant, so stat_state_t is
   ~95% of .bss. Overridable because a constrained host wants them far smaller -- 128 stations x a
   256-deep ring is a fleet-sized budget on a machine with memory to spare. */
#ifndef STAT_MAX_STATIONS
#define STAT_MAX_STATIONS 128
#endif
#ifndef STAT_RING_SIZE
#define STAT_RING_SIZE 256
#endif
#ifndef STAT_MESH_PEERS_MAX
#define STAT_MESH_PEERS_MAX 16
#endif
#ifndef STAT_TOPIC_STR_MAX
#define STAT_TOPIC_STR_MAX 256
#endif
#ifndef STAT_STRING_MAX
#define STAT_STRING_MAX 2048 /* the assembled one-line stat summary */
#endif
#define STAT_MQTT_TOPIC_DEFAULT "iotdata/stats"

#define STAT_WINDOW_COUNT       8
static const time_t stat_windows_secs[STAT_WINDOW_COUNT] = { 5 * 60, 15 * 60, 60 * 60, 3 * 3600, 12 * 3600, 24 * 3600, 3 * 86400, 7 * 86400 };
static const char *const stat_windows_names[STAT_WINDOW_COUNT] = { "5m", "15m", "1h", "3h", "12h", "24h", "3d", "7d" };

// -----------------------------------------------------------------------------------------------------------------------------------------

typedef struct {
    time_t time[STAT_RING_SIZE];
    uint16_t head, fill;
    uint32_t total;
} stat_ring_t;

static inline void stat_ring_add(stat_ring_t *r, time_t now) {
    r->time[r->head] = now;
    r->head = (uint16_t)((r->head + 1U) % STAT_RING_SIZE);
    if (r->fill < STAT_RING_SIZE)
        r->fill++;
    r->total++;
}

static inline void stat_ring_count_windows(const stat_ring_t *r, time_t now, uint32_t counts[STAT_WINDOW_COUNT]) {
    for (int i = 0; i < STAT_WINDOW_COUNT; i++)
        counts[i] = 0;
    for (uint16_t i = 0; i < r->fill; i++) {
        const time_t age = now - r->time[((unsigned)r->head + (unsigned)STAT_RING_SIZE - 1U - (unsigned)i) % (unsigned)STAT_RING_SIZE];
        for (int w = 0; w < STAT_WINDOW_COUNT; w++)
            if (age <= stat_windows_secs[w])
                counts[w]++;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

typedef struct {
    bool valid;
    uint16_t station_id;
    time_t first_seen;
    time_t last_seen;
    int8_t last_link_rssi;
    bool last_link_valid;
    int32_t link_rssi_sum;
    uint32_t link_rssi_cnt;
    uint8_t last_battery_level;
    bool last_battery_charging;
    bool last_battery_valid;
    uint32_t packet_count;
    uint64_t bytes_rx;
    uint32_t decode_errors;
    uint32_t process_errors;
    uint16_t last_sequence;
    bool last_sequence_valid;
    uint32_t stat_missed;
    uint32_t stat_mesh_unexpected;
    uint32_t variant_count[IOTDATA_VARIANT_MAPS_COUNT];
    time_t variant_last[IOTDATA_VARIANT_MAPS_COUNT];
    stat_ring_t ring;
} stat_station_t;

typedef struct {
    uint32_t packet_count;
    uint64_t bytes_rx;
    uint32_t decode_errors;
    uint32_t process_errors;
    time_t last_seen;
    stat_ring_t ring;
} stat_variant_t;

typedef struct {
    const char *name;
    const char *type;
    /* config snapshot */
    uint16_t address;
    uint8_t network;
    uint8_t channel;
    uint32_t frequency_khz;
    uint8_t packet_size_idx;
    uint8_t packet_rate_idx;
    uint8_t transmit_power_idx;
    /* runtime counters */
    uint32_t rx_packets;
    uint64_t rx_bytes;
    uint32_t rx_errors;
    uint32_t rx_drops;           /* drops that cannot be attributed to a station/variant */
    uint32_t rx_mesh_unexpected; /* mesh-variant packets received while mesh is disabled */
    uint16_t rx_size_min;
    uint16_t rx_size_max;
    /* rssi (raw e22 0-255, converted via get_rssi_dbm on read) */
    uint8_t rssi_packet_ema;
    uint32_t rssi_packet_cnt;
    uint32_t rssi_packet_err;
    time_t rssi_packet_last_time;
    uint8_t rssi_channel_ema;
    uint32_t rssi_channel_cnt;
    uint32_t rssi_channel_err;
    time_t rssi_channel_last_time;
    /* ring for window rates */
    stat_ring_t rx_ring;
} stat_link_t;

typedef struct {
    uint32_t rx_ok;   /* decoded and published successfully */
    uint32_t rx_drop; /* decode errors + process errors + link drops */
} stat_totals_t;

typedef struct {
    bool valid;
    uint16_t gateway_id;
    uint16_t generation;
    uint8_t cost;
    uint8_t flags;
    time_t last_seen;
} stat_peer_t;

/* The counters the summary line reports as deltas. It prints "since the last line", so it has to
   remember what it last saw; that snapshot is state and lives with the rest of it. */
typedef struct {
    time_t when;          /* when the last line was emitted, for the per-second rates */
    stat_totals_t totals; /* rx_ok / rx_drop at that moment */
    uint32_t forwards_rx, forwards_unwrapped, duplicates, beacons_tx, acks_tx, ctrl_rx, ctrl_err;
    uint32_t send_cycles, send_entries, recv_cycles, recv_entries, injected;
} stat_delta_t;

typedef struct {
    char mqtt_topic[STAT_TOPIC_STR_MAX];
    const char *version;
    uint16_t gateway_id;
    time_t start_time;
    stat_link_t link;
    stat_station_t stations[STAT_MAX_STATIONS];
    int stations_count;
    stat_variant_t variants[IOTDATA_VARIANT_MAPS_COUNT];
    stat_peer_t peers[STAT_MESH_PEERS_MAX];
    int peers_count;
    char _buffer_stat[STAT_STRING_MAX];
    stat_delta_t _delta_last; /* previous counter values, for the "since last line" deltas */
} stat_state_t;

// -----------------------------------------------------------------------------------------------------------------------------------------

bool stat_begin(stat_state_t *s, const char *version, uint16_t gateway_id, const e22900t22_config_t *lora_config) {

    s->version = version;
    s->gateway_id = gateway_id;
    s->start_time = time(NULL);
    s->link.name = "e22-900t22";
    s->link.type = "lora";
    s->link.address = lora_config->address;
    s->link.network = lora_config->network;
    s->link.channel = lora_config->channel;
    s->link.frequency_khz = 850125U + (uint32_t)lora_config->channel * 1000U;
    s->link.packet_size_idx = lora_config->packet_size;
    s->link.packet_rate_idx = lora_config->packet_rate;
    s->link.transmit_power_idx = lora_config->transmit_power;

    PRINTF_INFO("stats: started (gateway=%04" PRIX16 ", link=%s, channel=%" PRIu8 ", freq=%" PRIu32 " kHz)\n", s->gateway_id, s->link.name, s->link.channel, s->link.frequency_khz);

    return true;
}

void stat_end(stat_state_t *s) {
    (void)s;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* The station/peer tables are scanned per received packet, so the scans below stop at the last valid
   entry: `c` counts valid entries passed, and the search can end once every one has been seen. For a
   find-or-create that means `c == count` AND a free slot already in hand — the hole lies at or after
   the last valid entry, so the test cannot move into the loop condition. Entries are never
   invalidated (a full table evicts by overwrite), so there is no removal case. `i < MAX` stays in
   every loop: it keeps a count desync a wasted scan rather than a walk off the end of the array. */

static inline stat_station_t *stat_station_find_or_create(stat_state_t *s, uint16_t station_id) {
    int free_slot = -1;
    for (int i = 0, c = 0; i < STAT_MAX_STATIONS; i++) { /* UPSERT */
        if (s->stations[i].valid) {
            c++;
            if (s->stations[i].station_id == station_id)
                return &s->stations[i];
        } else if (free_slot < 0)
            free_slot = i;
        if (c == s->stations_count && free_slot >= 0)
            break; /* every valid entry seen (no match ahead) and a free slot in hand */
    }
    int slot;
    if (free_slot >= 0) {
        slot = free_slot;
        s->stations_count++;
    } else { /* table full -> evict the stalest (every slot is valid) */
        slot = 0;
        for (int i = 1; i < STAT_MAX_STATIONS; i++)
            if (s->stations[i].last_seen < s->stations[slot].last_seen)
                slot = i;
    }
    stat_station_t *st = &s->stations[slot];
    memset(st, 0, sizeof(*st));
    st->valid = true;
    st->station_id = station_id;
    st->first_seen = time(NULL);
    return st;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void stat_on_link_rx_packet(stat_state_t *s, uint16_t length) {
    const time_t now = time(NULL);
    s->link.rx_packets++;
    s->link.rx_bytes += (uint64_t)length;
    if (s->link.rx_size_min == 0 || length < s->link.rx_size_min)
        s->link.rx_size_min = length;
    if (length > s->link.rx_size_max)
        s->link.rx_size_max = length;
    stat_ring_add(&s->link.rx_ring, now);
}
void stat_on_link_rx_error(stat_state_t *s) {
    s->link.rx_errors++;
}
void stat_on_link_rx_drop(stat_state_t *s) {
    s->link.rx_drops++;
}
void stat_on_link_rx_mesh_unexpected(stat_state_t *s, uint16_t station_id) {
    s->link.rx_mesh_unexpected++;
    stat_station_t *st = stat_station_find_or_create(s, station_id);
    st->last_seen = time(NULL);
    st->stat_mesh_unexpected++;
}
void stat_on_link_rssi_packet(stat_state_t *s, uint8_t raw) {
    ema_update_timed(raw, &s->link.rssi_packet_ema, &s->link.rssi_packet_cnt, &s->link.rssi_packet_last_time, time(NULL), EMA_TIMED_TAU_SECS_DEFAULT);
}
void stat_on_link_rssi_packet_error(stat_state_t *s) {
    s->link.rssi_packet_err++;
}
void stat_on_link_rssi_channel(stat_state_t *s, uint8_t raw) {
    ema_update_timed(raw, &s->link.rssi_channel_ema, &s->link.rssi_channel_cnt, &s->link.rssi_channel_last_time, time(NULL), EMA_TIMED_TAU_SECS_DEFAULT);
}
void stat_on_link_rssi_channel_error(stat_state_t *s) {
    s->link.rssi_channel_err++;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void stat_get_totals(const stat_state_t *s, stat_totals_t *out) {
    out->rx_ok = 0;
    out->rx_drop = s->link.rx_drops;
    for (int v = 0; v < IOTDATA_VARIANT_MAPS_COUNT; v++) {
        const uint32_t pc = s->variants[v].packet_count, pe = s->variants[v].process_errors, de = s->variants[v].decode_errors;
        out->rx_ok += (pc >= pe) ? (pc - pe) : 0;
        out->rx_drop += de + pe;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void stat_on_packet_decoded(stat_state_t *s, uint16_t station_id, uint16_t sequence, uint8_t variant_id, uint16_t length, const iotdata_decoder_t *dec) {
    const time_t now = time(NULL);
    stat_station_t *st = stat_station_find_or_create(s, station_id);
    st->last_seen = now;
    st->packet_count++;
    st->bytes_rx += (uint64_t)length;
    if (st->last_sequence_valid) {
        const unsigned int diff = ((unsigned int)sequence - (unsigned int)st->last_sequence) & 0xFFFFU;
        if (diff > 1U && diff < 1024U) /* cap absurd gaps (restart/wrap noise) */
            st->stat_missed += (diff - 1U);
    }
    st->last_sequence = sequence;
    st->last_sequence_valid = true;
    if (variant_id < IOTDATA_VARIANT_MAPS_COUNT) {
        st->variant_count[variant_id]++;
        st->variant_last[variant_id] = now;
        s->variants[variant_id].packet_count++;
        s->variants[variant_id].bytes_rx += (uint64_t)length;
        s->variants[variant_id].last_seen = now;
        stat_ring_add(&s->variants[variant_id].ring, now);
    }
    stat_ring_add(&st->ring, now);
#if defined(IOTDATA_ENABLE_BATTERY)
    if (dec && IOTDATA_FIELD_PRESENT(dec->fields, IOTDATA_FIELD_BATTERY)) {
        st->last_battery_level = dec->battery_level;
        st->last_battery_charging = dec->battery_charging;
        st->last_battery_valid = true;
    }
#endif
#if defined(IOTDATA_ENABLE_LINK)
    if (dec && IOTDATA_FIELD_PRESENT(dec->fields, IOTDATA_FIELD_LINK)) {
        st->last_link_rssi = dec->link_rssi;
        st->last_link_valid = true;
        st->link_rssi_sum += (int32_t)dec->link_rssi;
        st->link_rssi_cnt++;
    }
#endif
    (void)dec;
}

void stat_on_packet_decode_error(stat_state_t *s, uint16_t station_id, uint8_t variant_id) {
    stat_station_t *st = stat_station_find_or_create(s, station_id);
    st->last_seen = time(NULL);
    st->decode_errors++;
    if (variant_id < IOTDATA_VARIANT_MAPS_COUNT)
        s->variants[variant_id].decode_errors++;
}

void stat_on_packet_process_error(stat_state_t *s, uint16_t station_id, uint8_t variant_id) {
    stat_station_t *st = stat_station_find_or_create(s, station_id);
    st->last_seen = time(NULL);
    st->process_errors++;
    if (variant_id < IOTDATA_VARIANT_MAPS_COUNT)
        s->variants[variant_id].process_errors++;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void stat_on_peer(stat_state_t *s, uint16_t gateway_id, uint16_t generation, uint8_t cost, uint8_t flags) {
    const time_t now = time(NULL);
    int slot = -1;
    for (int i = 0, c = 0; i < STAT_MESH_PEERS_MAX && c < s->peers_count && slot < 0; i++) /* LOOKUP */
        if (s->peers[i].valid) {
            c++;
            if (s->peers[i].gateway_id == gateway_id)
                slot = i;
        }
    if (slot < 0) {
        if (s->peers_count < STAT_MESH_PEERS_MAX) { /* a free slot exists -> take the first */
            for (int i = 0; i < STAT_MESH_PEERS_MAX; i++)
                if (!s->peers[i].valid) {
                    slot = i;
                    break;
                }
            s->peers_count++;
        } else { /* full -> evict the stalest (every slot is valid) */
            slot = 0;
            for (int i = 1; i < STAT_MESH_PEERS_MAX; i++)
                if (s->peers[i].last_seen < s->peers[slot].last_seen)
                    slot = i;
        }
    }
    s->peers[slot].valid = true;
    s->peers[slot].gateway_id = gateway_id;
    s->peers[slot].generation = generation;
    s->peers[slot].cost = cost;
    s->peers[slot].flags = flags;
    s->peers[slot].last_seen = now;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static cJSON *stat_json_windows(const stat_ring_t *r, time_t now) {
    uint32_t counts[STAT_WINDOW_COUNT];
    stat_ring_count_windows(r, now, counts);
    cJSON *o = cJSON_CreateObject();
    for (int i = 0; i < STAT_WINDOW_COUNT; i++)
        cJSON_AddNumberToObject(o, stat_windows_names[i], (double)counts[i]);
    return o;
}

cJSON *stat_build_links_json(const stat_state_t *s, const mesh_state_t *mesh) {
    const time_t now = time(NULL);
    cJSON *root = cJSON_CreateObject();
    cJSON *links = cJSON_AddArrayToObject(root, "links");
    {
        cJSON *link = cJSON_CreateObject();
        cJSON_AddStringToObject(link, "name", s->link.name ? s->link.name : "");
        cJSON_AddStringToObject(link, "type", s->link.type ? s->link.type : "");
        cJSON *config = cJSON_AddObjectToObject(link, "config");
        cJSON_AddNumberToObject(config, "address", (double)s->link.address);
        cJSON_AddNumberToObject(config, "network", (double)s->link.network);
        cJSON_AddNumberToObject(config, "channel", (double)s->link.channel);
        cJSON_AddNumberToObject(config, "frequency_khz", (double)s->link.frequency_khz);
        cJSON_AddNumberToObject(config, "packet_size_idx", (double)s->link.packet_size_idx);
        cJSON_AddNumberToObject(config, "packet_rate_idx", (double)s->link.packet_rate_idx);
        cJSON_AddNumberToObject(config, "transmit_power_idx", (double)s->link.transmit_power_idx);
        cJSON_AddStringToObject(config, "transmit_power", get_transmit_power(s->link.transmit_power_idx));
        cJSON *rssi = cJSON_AddObjectToObject(link, "rssi");
        cJSON_AddNumberToObject(rssi, "packet_dbm", (double)get_rssi_dbm(s->link.rssi_packet_ema));
        cJSON_AddNumberToObject(rssi, "packet_samples", (double)s->link.rssi_packet_cnt);
        cJSON_AddNumberToObject(rssi, "packet_errors", (double)s->link.rssi_packet_err);
        cJSON_AddNumberToObject(rssi, "channel_dbm", (double)get_rssi_dbm(s->link.rssi_channel_ema));
        cJSON_AddNumberToObject(rssi, "channel_samples", (double)s->link.rssi_channel_cnt);
        cJSON_AddNumberToObject(rssi, "channel_errors", (double)s->link.rssi_channel_err);
        cJSON *rx = cJSON_AddObjectToObject(link, "rx");
        cJSON_AddNumberToObject(rx, "packets", (double)s->link.rx_packets);
        cJSON_AddNumberToObject(rx, "bytes", (double)s->link.rx_bytes);
        cJSON_AddNumberToObject(rx, "errors", (double)s->link.rx_errors);
        cJSON_AddNumberToObject(rx, "drops", (double)s->link.rx_drops);
        cJSON_AddNumberToObject(rx, "mesh_unexpected", (double)s->link.rx_mesh_unexpected);
        cJSON_AddNumberToObject(rx, "size_min", (double)s->link.rx_size_min);
        cJSON_AddNumberToObject(rx, "size_max", (double)s->link.rx_size_max);
        cJSON_AddNumberToObject(rx, "size_mean", s->link.rx_packets > 0 ? (double)s->link.rx_bytes / (double)s->link.rx_packets : 0.0);
        cJSON_AddItemToObject(rx, "windows", stat_json_windows(&s->link.rx_ring, now));
        cJSON *tx = cJSON_AddObjectToObject(link, "tx");
        cJSON_AddNumberToObject(tx, "packets", (double)(mesh ? (mesh->stat_beacons_tx + mesh->stat_acks_tx) : 0));
        cJSON_AddNumberToObject(tx, "bytes", (double)(mesh ? mesh->stat_bytes_tx : 0));
        cJSON_AddNumberToObject(tx, "errors", (double)(mesh ? mesh->stat_errors_tx : 0));
        cJSON_AddItemToArray(links, link);
    }
    return root;
}

cJSON *stat_build_stations_json(const stat_state_t *s, const mesh_state_t *mesh, const ddup_state_t *dedup) {
    const time_t now = time(NULL);
    cJSON *root = cJSON_CreateObject();
    char buf[16];
    cJSON_AddNumberToObject(root, "count", (double)s->stations_count);
    cJSON *arr = cJSON_AddArrayToObject(root, "stations");
    for (int i = 0, c = 0; i < STAT_MAX_STATIONS && c < s->stations_count; i++) /* LOOKUP */
        if (s->stations[i].valid) {
            c++;
            const stat_station_t *st = &s->stations[i];
            cJSON *o = cJSON_CreateObject();
            cJSON_AddStringToObject(o, "id", snprintf_inline(buf, sizeof(buf), "%04" PRIX16, st->station_id));
            cJSON_AddNumberToObject(o, "first_seen", (double)st->first_seen);
            cJSON_AddNumberToObject(o, "last_seen", (double)st->last_seen);
            cJSON_AddNumberToObject(o, "age_secs", (double)(now - st->last_seen));
            cJSON_AddNumberToObject(o, "packets", (double)st->packet_count);
            cJSON_AddNumberToObject(o, "bytes", (double)st->bytes_rx);
            cJSON_AddNumberToObject(o, "missed", (double)st->stat_missed);
            cJSON_AddNumberToObject(o, "mesh_unexpected", (double)st->stat_mesh_unexpected);
            cJSON_AddNumberToObject(o, "decode_errors", (double)st->decode_errors);
            cJSON_AddNumberToObject(o, "process_errors", (double)st->process_errors);
            if (st->last_link_valid)
                cJSON_AddNumberToObject(o, "link_rssi", (double)st->last_link_rssi);
            if (st->link_rssi_cnt > 0)
                cJSON_AddNumberToObject(o, "link_rssi_avg", (double)st->link_rssi_sum / (double)st->link_rssi_cnt);
            if (st->last_battery_valid) {
                cJSON *bat = cJSON_AddObjectToObject(o, "battery");
                cJSON_AddNumberToObject(bat, "level", (double)st->last_battery_level);
                cJSON_AddBoolToObject(bat, "charging", st->last_battery_charging);
            }
            cJSON *vs = cJSON_AddObjectToObject(o, "variants");
            for (int v = 0; v < IOTDATA_VARIANT_MAPS_COUNT; v++)
                if (st->variant_count[v] > 0) {
                    const iotdata_variant_def_t *vdef = iotdata_get_variant((uint8_t)v);
                    cJSON *vo = cJSON_AddObjectToObject(vs, vdef ? vdef->name : "?");
                    cJSON_AddNumberToObject(vo, "count", (double)st->variant_count[v]);
                    cJSON_AddNumberToObject(vo, "last_seen", (double)st->variant_last[v]);
                }
            cJSON_AddItemToObject(o, "windows", stat_json_windows(&st->ring, now));
            cJSON_AddItemToArray(arr, o);
        }
    if (mesh) {
        cJSON *m = cJSON_AddObjectToObject(root, "mesh");
        cJSON_AddBoolToObject(m, "enabled", mesh->enabled);
        cJSON_AddStringToObject(m, "station_id", snprintf_inline(buf, sizeof(buf), "%04" PRIX16, mesh->station_id));
        cJSON_AddNumberToObject(m, "beacons_tx", (double)mesh->stat_beacons_tx);
        /* per frame type: only types actually seen are emitted, so the object stays small on a
           quiet link but gains a key the first time a new type appears */
        cJSON *const frames = cJSON_AddObjectToObject(m, "frames");
        if (frames != NULL)
            for (unsigned t = 0; t < MESH_CTRL_COUNT; t++) {
                const mesh_ctrl_stat_t *const c = &mesh->ctrl[t];
                if (c->rx == 0 && c->err == 0)
                    continue;
                cJSON *const ft = cJSON_AddObjectToObject(frames, iotdata_mesh_ctrl_name((uint8_t)t));
                if (ft == NULL)
                    continue;
                cJSON_AddNumberToObject(ft, "rx", (double)c->rx);
                cJSON_AddNumberToObject(ft, "err", (double)c->err);
                cJSON_AddNumberToObject(ft, "bytes", (double)c->bytes);
            }
        cJSON_AddNumberToObject(m, "rx", (double)mesh_stat_total(mesh, false));
        cJSON_AddNumberToObject(m, "rx_errors", (double)mesh_stat_total(mesh, true));
        cJSON_AddNumberToObject(m, "rx_runts", (double)mesh->stat_ctrl_runt);
        cJSON_AddNumberToObject(m, "forwards_unwrapped", (double)mesh->stat_forwards_unwrapped);
        cJSON_AddNumberToObject(m, "duplicates", (double)mesh->stat_duplicates);
        cJSON_AddNumberToObject(m, "beacons_tx", (double)mesh->stat_beacons_tx);
        cJSON_AddNumberToObject(m, "acks_tx", (double)mesh->stat_acks_tx);
        cJSON_AddNumberToObject(m, "tx_errors", (double)mesh->stat_errors_tx);
        cJSON_AddNumberToObject(m, "tx_bytes", (double)mesh->stat_bytes_tx);
        cJSON *peers = cJSON_AddArrayToObject(m, "peers");
        for (int i = 0, c = 0; i < STAT_MESH_PEERS_MAX && c < s->peers_count; i++) /* LOOKUP */
            if (s->peers[i].valid) {
                c++;
                cJSON *p = cJSON_CreateObject();
                cJSON_AddStringToObject(p, "gateway_id", snprintf_inline(buf, sizeof(buf), "%04" PRIX16, s->peers[i].gateway_id));
                cJSON_AddNumberToObject(p, "generation", (double)s->peers[i].generation);
                cJSON_AddNumberToObject(p, "cost", (double)s->peers[i].cost);
                cJSON_AddNumberToObject(p, "flags", (double)s->peers[i].flags);
                cJSON_AddNumberToObject(p, "last_seen", (double)s->peers[i].last_seen);
                cJSON_AddNumberToObject(p, "age_secs", (double)(now - s->peers[i].last_seen));
                cJSON_AddItemToArray(peers, p);
            }
    }
    if (dedup) {
        cJSON *d = cJSON_AddObjectToObject(root, "dedup");
        cJSON_AddBoolToObject(d, "enabled", dedup->enabled);
        cJSON_AddNumberToObject(d, "peers", (double)dedup->peers_count);
        cJSON_AddNumberToObject(d, "peers_resolved", (double)dedup->stat_peers_resolved);
        cJSON_AddNumberToObject(d, "peers_unresolved", (double)dedup->stat_peers_unresolved);
        cJSON_AddNumberToObject(d, "send_cycles", (double)dedup->stat_send_cycles);
        cJSON_AddNumberToObject(d, "send_entries", (double)dedup->stat_send_entries);
        cJSON_AddNumberToObject(d, "send_errors", (double)dedup->stat_send_errors);
        cJSON_AddNumberToObject(d, "recv_cycles", (double)dedup->stat_recv_cycles);
        cJSON_AddNumberToObject(d, "recv_entries", (double)dedup->stat_recv_entries);
        cJSON_AddNumberToObject(d, "recv_errors", (double)dedup->stat_recv_errors);
        cJSON_AddNumberToObject(d, "injected", (double)dedup->stat_injected);
        cJSON_AddNumberToObject(d, "pending_overflow", (double)dedup->stat_pending_overflow);
    }
    return root;
}

cJSON *stat_build_variants_json(const stat_state_t *s) {
    const time_t now = time(NULL);
    cJSON *root = cJSON_CreateObject();
    cJSON *arr = cJSON_AddArrayToObject(root, "variants");
    for (int v = 0; v < IOTDATA_VARIANT_MAPS_COUNT; v++)
        if (s->variants[v].packet_count > 0 || s->variants[v].decode_errors > 0 || s->variants[v].process_errors > 0) {
            const iotdata_variant_def_t *vdef = iotdata_get_variant((uint8_t)v);
            cJSON *o = cJSON_CreateObject();
            cJSON_AddNumberToObject(o, "id", (double)v);
            cJSON_AddStringToObject(o, "name", vdef ? vdef->name : "?");
            cJSON_AddNumberToObject(o, "packets", (double)s->variants[v].packet_count);
            cJSON_AddNumberToObject(o, "bytes", (double)s->variants[v].bytes_rx);
            cJSON_AddNumberToObject(o, "decode_errors", (double)s->variants[v].decode_errors);
            cJSON_AddNumberToObject(o, "process_errors", (double)s->variants[v].process_errors);
            cJSON_AddNumberToObject(o, "last_seen", (double)s->variants[v].last_seen);
            cJSON_AddNumberToObject(o, "age_secs", (double)(s->variants[v].last_seen ? (now - s->variants[v].last_seen) : 0));
            cJSON_AddItemToObject(o, "windows", stat_json_windows(&s->variants[v].ring, now));
            cJSON_AddItemToArray(arr, o);
        }
    return root;
}

cJSON *stat_build_mqtt_json(void) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "connected_state", mqtt_is_connected());
    cJSON_AddNumberToObject(root, "connected_time", (double)mqtt_stat_last_connect_time);
    cJSON_AddNumberToObject(root, "connected", (double)mqtt_stat_connects);
    cJSON_AddNumberToObject(root, "disconnected", (double)mqtt_stat_disconnects);
    cJSON_AddNumberToObject(root, "reconnected", (double)mqtt_stat_reconnects);
    cJSON_AddNumberToObject(root, "published", (double)mqtt_stat_publishes);
    cJSON_AddNumberToObject(root, "published_bytes", (double)mqtt_stat_publish_bytes);
    cJSON_AddNumberToObject(root, "published_errors", (double)mqtt_stat_publish_errors);
    return root;
}

cJSON *stat_build_stat_json(const stat_state_t *s, const mesh_state_t *mesh, const ddup_state_t *dedup) {
    const time_t now = time(NULL);
    cJSON *root = cJSON_CreateObject();
    char buf[16];
    cJSON_AddStringToObject(root, "gateway_id", snprintf_inline(buf, sizeof(buf), "%04" PRIX16, s->gateway_id));
    cJSON_AddStringToObject(root, "version", s->version ? s->version : "");
    cJSON_AddNumberToObject(root, "time", (double)now);
    cJSON_AddNumberToObject(root, "uptime_secs", (double)(now - s->start_time));
    cJSON_AddItemToObject(root, "links", stat_build_links_json(s, mesh));
    cJSON_AddItemToObject(root, "stations", stat_build_stations_json(s, mesh, dedup));
    cJSON_AddItemToObject(root, "variants", stat_build_variants_json(s));
    cJSON_AddItemToObject(root, "mqtt", stat_build_mqtt_json());
    return root;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* Formats into a CALLER-supplied buffer and returns it. It used to build into a local and hand back
   a malloc'd copy, which bought nothing: the only caller printed it and freed it immediately. */
/* NOT const: the delta snapshot it keeps now lives in the state, so this updates it. */
const char *stat_build_stat_string(char *const buf, const size_t size, stat_state_t *const s, const mesh_state_t *mesh, const ddup_state_t *dedup) {
    size_t off = 0;
    int n;
#define STAT_APPEND(...) \
    do { \
        n = snprintf(buf + off, size - off, __VA_ARGS__); \
        if (n > 0) { \
            off += (size_t)n; \
            if (off >= size) \
                off = size - 1; \
        } \
    } while (0)

    const time_t now = time(NULL);
    stat_delta_t *const last = &s->_delta_last;
    stat_totals_t cur_totals;
    stat_get_totals(s, &cur_totals);
    const uint32_t period_stat = (last->when > 0 && now > last->when) ? (uint32_t)(now - last->when) : 1U;
    const uint32_t delta_okay = (last->when > 0) ? (cur_totals.rx_ok - last->totals.rx_ok) : 0U;
    const uint32_t delta_drop = (last->when > 0) ? (cur_totals.rx_drop - last->totals.rx_drop) : 0U;
    last->totals = cur_totals;
    last->when = now;
    const uint32_t rate_okay = (delta_okay * 6000U) / period_stat, rate_drop = (delta_drop * 6000U) / period_stat;
    STAT_APPEND("packets{okay=%" PRIu32 " (%" PRIu32 ".%02" PRIu32 "/min), drop=%" PRIu32 " (%" PRIu32 ".%02" PRIu32 "/min)}", delta_okay, rate_okay / 100, rate_okay % 100, delta_drop, rate_drop / 100, rate_drop % 100);
    if (s->link.rssi_channel_cnt > 0 || s->link.rssi_packet_cnt > 0) {
        STAT_APPEND(", rssi{");
        if (s->link.rssi_channel_cnt > 0)
            STAT_APPEND("channel=%d dBm (%" PRIu32 ")", get_rssi_dbm(s->link.rssi_channel_ema), s->link.rssi_channel_cnt);
        if (s->link.rssi_channel_cnt > 0 && s->link.rssi_packet_cnt > 0)
            STAT_APPEND(", ");
        if (s->link.rssi_packet_cnt > 0)
            STAT_APPEND("packet=%d dBm (%" PRIu32 ")", get_rssi_dbm(s->link.rssi_packet_ema), s->link.rssi_packet_cnt);
        STAT_APPEND("}");
    }
    if (mesh && mesh->enabled) {
        const uint32_t ctrl_rx = mesh_stat_total(mesh, false), ctrl_err = mesh_stat_total(mesh, true);
        STAT_APPEND(", mesh{fwd=%" PRIu32 ", unwrap=%" PRIu32 ", dedup=%" PRIu32 ", beacons=%" PRIu32 ", acks=%" PRIu32 ", ctrl=%" PRIu32 ", err=%" PRIu32 "}", mesh->ctrl[IOTDATA_MESH_CTRL_FORWARD].rx - last->forwards_rx,
                    mesh->stat_forwards_unwrapped - last->forwards_unwrapped, mesh->stat_duplicates - last->duplicates, mesh->stat_beacons_tx - last->beacons_tx, mesh->stat_acks_tx - last->acks_tx, ctrl_rx - last->ctrl_rx,
                    ctrl_err - last->ctrl_err);
        last->forwards_rx = mesh->ctrl[IOTDATA_MESH_CTRL_FORWARD].rx;
        last->ctrl_err = ctrl_err;
        last->forwards_unwrapped = mesh->stat_forwards_unwrapped;
        last->duplicates = mesh->stat_duplicates;
        last->beacons_tx = mesh->stat_beacons_tx;
        last->acks_tx = mesh->stat_acks_tx;
        last->ctrl_rx = ctrl_rx;
    }
    if (dedup && dedup->enabled) {
        STAT_APPEND(", dedup{sends=%" PRIu32 "/%" PRIu32 ", recvs=%" PRIu32 "/%" PRIu32 ", injected=%" PRIu32 "}", dedup->stat_send_cycles - last->send_cycles, dedup->stat_send_entries - last->send_entries,
                    dedup->stat_recv_cycles - last->recv_cycles, dedup->stat_recv_entries - last->recv_entries, dedup->stat_injected - last->injected);
        last->send_cycles = dedup->stat_send_cycles;
        last->send_entries = dedup->stat_send_entries;
        last->recv_cycles = dedup->stat_recv_cycles;
        last->recv_entries = dedup->stat_recv_entries;
        last->injected = dedup->stat_injected;
    }
    STAT_APPEND(", mqtt{%s, disconnects=%" PRIu32 "}", mqtt_is_connected() ? "up" : "down", mqtt_stat_disconnects);
#undef STAT_APPEND
    return buf;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

void stat_publish(const stat_state_t *s, const mesh_state_t *mesh, const ddup_state_t *dedup) {
    if (!mqtt_is_connected())
        return;
    cJSON *root = stat_build_stat_json(s, mesh, dedup);
    if (root) {
        char *json = cJSON_PrintUnformatted(root);
        cJSON_Delete(root);
        if (json) {
            char topic[STAT_TOPIC_STR_MAX + 8];
            (void)mqtt_send(snprintf_inline(topic, sizeof(topic), "%s/%04" PRIX16, s->mqtt_topic, s->gateway_id), json, (int)strlen(json));
            free(json);
        }
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void stat_display(stat_state_t *s, const mesh_state_t *mesh, const ddup_state_t *dedup) {
    PRINTF_INFO("%s\n", stat_build_stat_string(s->_buffer_stat, sizeof(s->_buffer_stat), s, mesh, dedup));
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
