
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include <cjson/cJSON.h>

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

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
#ifndef STAT_EMA_TIMED_TAU_SECS_DEFAULT
#define STAT_EMA_TIMED_TAU_SECS_DEFAULT 300.0f
#endif
#define STAT_MQTT_TOPIC_DEFAULT "iotdata/stats"
#define STAT_WINDOW_COUNT       8
static const time_t stat_windows_secs[STAT_WINDOW_COUNT] = { 5 * 60, 15 * 60, 60 * 60, 3 * 3600, 12 * 3600, 24 * 3600, 3 * 86400, 7 * 86400 };
static const char *const stat_windows_names[STAT_WINDOW_COUNT] = { "5m", "15m", "1h", "3h", "12h", "24h", "3d", "7d" };

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

typedef struct {
    time_t time[STAT_RING_SIZE];
    uint16_t head, fill;
    uint32_t total;
} stat_ring_t;

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
    uint16_t station_id;
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
    uint16_t station_id;
    time_t start_time;
    stat_link_t link;
    stat_station_t stations[STAT_MAX_STATIONS];
    int stations_count;
    stat_variant_t variants[IOTDATA_VARIANT_MAPS_COUNT];
    stat_peer_t peers[STAT_MESH_PEERS_MAX];
    int peers_count;
    char _buffer_topic[STAT_TOPIC_STR_MAX + 8];
    char _buffer_stat[STAT_STRING_MAX];
    stat_delta_t _delta_last; /* previous counter values, for the "since last line" deltas */
} stat_state_t;

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static inline void stat_ring_add(stat_ring_t *r, time_t now) {
    r->time[r->head] = now;
    r->head = (uint16_t)((r->head + 1U) % (sizeof(r->time) / sizeof(r->time[0])));
    if (r->fill < (sizeof(r->time) / sizeof(r->time[0])))
        r->fill++;
    r->total++;
}

static inline void stat_ring_count(const stat_ring_t *r, time_t now, uint32_t counts[]) {
    for (int i = 0; i < (int)(sizeof(stat_windows_secs) / sizeof(stat_windows_secs[0])); i++)
        counts[i] = 0;
    for (uint16_t i = 0; i < r->fill; i++) {
        const time_t age = now - r->time[((unsigned)r->head + (unsigned)(sizeof(r->time) / sizeof(r->time[0])) - 1U - (unsigned)i) % (unsigned)(sizeof(r->time) / sizeof(r->time[0]))];
        for (int w = 0; w < (int)(sizeof(stat_windows_secs) / sizeof(stat_windows_secs[0])); w++)
            if (age <= stat_windows_secs[w])
                counts[w]++;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static inline stat_station_t *stat_station_find_or_create(stat_state_t *s, uint16_t station_id, const time_t now) {
    int slot_free = -1, slot_oldest = -1;
    for (int i = 0, c = 0; i < (int)(sizeof(s->stations) / sizeof(s->stations[0])) && !(c == s->stations_count && slot_free >= 0); i++) { /* UPSERT */
        stat_station_t *const e = &s->stations[i];
        if (e->valid) {
            c++;
            if (e->station_id == station_id) {
                e->last_seen = now;
                return e;
            } else if (s->stations_count == (int)(sizeof(s->stations) / sizeof(s->stations[0])) && (slot_oldest < 0 || e->last_seen < s->stations[slot_oldest].last_seen))
                slot_oldest = i;
        } else if (slot_free < 0)
            slot_free = i;
    }
    int slot;
    if (slot_free >= 0) {
        slot = slot_free;
        s->stations_count++;
    } else
        slot = (slot_oldest >= 0) ? slot_oldest : 0;
    stat_station_t *e = &s->stations[slot];
    memset(e, 0, sizeof(*e));
    e->valid = true;
    e->station_id = station_id;
    e->first_seen = now;
    return e;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
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
    stat_station_t *st = stat_station_find_or_create(s, station_id, time(NULL));
    st->stat_mesh_unexpected++;
}
void stat_on_link_rssi_packet(stat_state_t *s, uint8_t raw) {
    ema_update_timed(raw, &s->link.rssi_packet_ema, &s->link.rssi_packet_cnt, &s->link.rssi_packet_last_time, time(NULL), STAT_EMA_TIMED_TAU_SECS_DEFAULT);
}
void stat_on_link_rssi_packet_error(stat_state_t *s) {
    s->link.rssi_packet_err++;
}
void stat_on_link_rssi_channel(stat_state_t *s, uint8_t raw) {
    ema_update_timed(raw, &s->link.rssi_channel_ema, &s->link.rssi_channel_cnt, &s->link.rssi_channel_last_time, time(NULL), STAT_EMA_TIMED_TAU_SECS_DEFAULT);
}
void stat_on_link_rssi_channel_error(stat_state_t *s) {
    s->link.rssi_channel_err++;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void stat_get_totals(const stat_state_t *s, stat_totals_t *out) {
    out->rx_ok = 0;
    out->rx_drop = s->link.rx_drops;
    for (int v = 0; v < (int)(sizeof(s->variants) / sizeof(s->variants[0])); v++) {
        const stat_variant_t *const e = &s->variants[v];
        out->rx_ok += (e->packet_count >= e->process_errors) ? (e->packet_count - e->process_errors) : 0;
        out->rx_drop += e->decode_errors + e->process_errors;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void stat_on_packet_decoded(stat_state_t *s, uint16_t station_id, uint16_t sequence, uint8_t variant_id, uint16_t length, const iotdata_decoder_t *dec) {
    const time_t now = time(NULL);
    stat_station_t *st = stat_station_find_or_create(s, station_id, now);
    st->packet_count++;
    st->bytes_rx += (uint64_t)length;
    if (st->last_sequence_valid) {
        const unsigned int diff = ((unsigned int)sequence - (unsigned int)st->last_sequence) & 0xFFFFU;
        if (diff > 1U && diff < 1024U) /* cap absurd gaps (restart/wrap noise) */
            st->stat_missed += (diff - 1U);
    }
    st->last_sequence = sequence;
    st->last_sequence_valid = true;
    if (variant_id < (int)(sizeof(s->variants) / sizeof(s->variants[0]))) {
        st->variant_count[variant_id]++;
        st->variant_last[variant_id] = now;
        stat_variant_t *const e = &s->variants[variant_id];
        e->packet_count++;
        e->bytes_rx += (uint64_t)length;
        e->last_seen = now;
        stat_ring_add(&e->ring, now);
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
    stat_station_t *st = stat_station_find_or_create(s, station_id, time(NULL));
    st->decode_errors++;
    if (variant_id < (int)(sizeof(s->variants) / sizeof(s->variants[0]))) {
        stat_variant_t *const e = &s->variants[variant_id];
        e->decode_errors++;
    }
}

void stat_on_packet_process_error(stat_state_t *s, uint16_t station_id, uint8_t variant_id) {
    stat_station_t *st = stat_station_find_or_create(s, station_id, time(NULL));
    st->process_errors++;
    if (variant_id < (int)(sizeof(s->variants) / sizeof(s->variants[0]))) {
        stat_variant_t *const e = &s->variants[variant_id];
        e->process_errors++;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void stat_on_peer(stat_state_t *s, uint16_t station_id, uint16_t generation, uint8_t cost, uint8_t flags) {
    int slot = -1, slot_free = -1, slot_oldest = -1;
    for (int i = 0, c = 0; i < (int)(sizeof(s->peers) / sizeof(s->peers[0])) && !(c == s->peers_count && slot_free >= 0); i++) { /* UPSERT: match, free slot and stalest in one pass */
        const stat_peer_t *const e = &s->peers[i];
        if (e->valid) {
            c++;
            if (e->station_id == station_id) {
                slot = i;
                break;
            } else if (s->peers_count == (int)(sizeof(s->peers) / sizeof(s->peers[0])) && (slot_oldest < 0 || e->last_seen < s->peers[slot_oldest].last_seen))
                slot_oldest = i;
        } else if (slot_free < 0)
            slot_free = i;
    }
    if (slot < 0) {
        if (slot_free >= 0) {
            slot = slot_free;
            s->peers_count++;
        } else
            slot = (slot_oldest >= 0) ? slot_oldest : 0; /* eviction reuses a valid slot: count unchanged */
    }
    stat_peer_t *const e = &s->peers[slot];
    e->valid = true;
    e->station_id = station_id;
    e->generation = generation;
    e->cost = cost;
    e->flags = flags;
    e->last_seen = time(NULL);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static cJSON *stat_json_windows(const stat_ring_t *const r, const time_t now) {
    uint32_t counts[STAT_WINDOW_COUNT];
    stat_ring_count(r, now, counts);
    cJSON *o = cJSON_CreateObject();
    for (int i = 0; i < (int)(sizeof(counts) / sizeof(counts[0])); i++)
        cJSON_AddNumberToObject(o, stat_windows_names[i], (double)counts[i]);
    return o;
}

cJSON *stat_build_links_json(const stat_state_t *const s, const time_t now, const mesh_state_t *const mesh) {
    cJSON *root = cJSON_CreateObject(), *links = cJSON_AddArrayToObject(root, "links");
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

cJSON *stat_build_stations_json(const stat_state_t *const s, const time_t now, const mesh_state_t *const mesh, const ddup_state_t *const ddup) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "count", (double)s->stations_count);
    cJSON *arr = cJSON_AddArrayToObject(root, "stations");
    for (int i = 0, c = 0; i < (int)(sizeof(s->stations) / sizeof(s->stations[0])) && c < s->stations_count; i++) { /* LOOKUP */
        const stat_station_t *st = &s->stations[i];
        if (st->valid) {
            c++;
            cJSON *o = cJSON_CreateObject();
            char buf[4 + 1];
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
            for (int v = 0; v < (int)(sizeof(st->variant_count) / sizeof(st->variant_count[0])); v++)
                if (st->variant_count[v] > 0) {
                    const iotdata_variant_def_t *vdef = iotdata_get_variant((uint8_t)v);
                    cJSON *vo = cJSON_AddObjectToObject(vs, vdef ? vdef->name : "?");
                    cJSON_AddNumberToObject(vo, "count", (double)st->variant_count[v]);
                    cJSON_AddNumberToObject(vo, "last_seen", (double)st->variant_last[v]);
                }
            cJSON_AddItemToObject(o, "windows", stat_json_windows(&st->ring, now));
            cJSON_AddItemToArray(arr, o);
        }
    }
    if (mesh) {
        cJSON *m = cJSON_AddObjectToObject(root, "mesh");
        cJSON_AddBoolToObject(m, "enabled", mesh->enabled);
        char buf[4 + 1];
        cJSON_AddStringToObject(m, "station_id", snprintf_inline(buf, sizeof(buf), "%04" PRIX16, mesh->station_id));
        cJSON_AddNumberToObject(m, "beacons_tx", (double)mesh->stat_beacons_tx);
        /* per frame type: only types actually seen are emitted, so the object stays small on a
           quiet link but gains a key the first time a new type appears */
        cJSON *const frames = cJSON_AddObjectToObject(m, "frames");
        if (frames != NULL)
            for (unsigned t = 0; t < (int)(sizeof(mesh->ctrl) / sizeof(mesh->ctrl[0])); t++) {
                const mesh_ctrl_stat_t *const c = &mesh->ctrl[t];
                if (c->rx > 0 || c->err > 0) {
                    cJSON *const ft = cJSON_AddObjectToObject(frames, iotdata_mesh_ctrl_name((uint8_t)t));
                    if (ft != NULL) {
                        cJSON_AddNumberToObject(ft, "rx", (double)c->rx);
                        cJSON_AddNumberToObject(ft, "err", (double)c->err);
                        cJSON_AddNumberToObject(ft, "bytes", (double)c->bytes);
                    }
                }
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
        for (int i = 0, c = 0; i < (int)(sizeof(s->peers) / sizeof(s->peers[0])) && c < s->peers_count; i++) { /* LOOKUP */
            const stat_peer_t *const e = &s->peers[i];
            if (e->valid) {
                c++;
                cJSON *p = cJSON_CreateObject();
                cJSON_AddStringToObject(p, "station_id", snprintf_inline(buf, sizeof(buf), "%04" PRIX16, e->station_id));
                cJSON_AddNumberToObject(p, "generation", (double)e->generation);
                cJSON_AddNumberToObject(p, "cost", (double)e->cost);
                cJSON_AddNumberToObject(p, "flags", (double)e->flags);
                cJSON_AddNumberToObject(p, "last_seen", (double)e->last_seen);
                cJSON_AddNumberToObject(p, "age_secs", (double)(now - e->last_seen));
                cJSON_AddItemToArray(peers, p);
            }
        }
    }
    if (ddup) {
        cJSON *d = cJSON_AddObjectToObject(root, "ddup");
        cJSON_AddBoolToObject(d, "enabled", ddup->enabled);
        cJSON_AddNumberToObject(d, "peers", (double)ddup->peers_count);
        cJSON_AddNumberToObject(d, "peers_resolved", (double)ddup->stat_peers_resolved);
        cJSON_AddNumberToObject(d, "peers_unresolved", (double)ddup->stat_peers_unresolved);
        cJSON_AddNumberToObject(d, "send_cycles", (double)ddup->stat_send_cycles);
        cJSON_AddNumberToObject(d, "send_entries", (double)ddup->stat_send_entries);
        cJSON_AddNumberToObject(d, "send_errors", (double)ddup->stat_send_errors);
        cJSON_AddNumberToObject(d, "recv_cycles", (double)ddup->stat_recv_cycles);
        cJSON_AddNumberToObject(d, "recv_entries", (double)ddup->stat_recv_entries);
        cJSON_AddNumberToObject(d, "recv_errors", (double)ddup->stat_recv_errors);
        cJSON_AddNumberToObject(d, "injected", (double)ddup->stat_injected);
        cJSON_AddNumberToObject(d, "pending_overflow", (double)ddup->stat_pending_overflow);
    }
    return root;
}

cJSON *stat_build_variants_json(const stat_state_t *const s, const time_t now) {
    cJSON *root = cJSON_CreateObject(), *arr = cJSON_AddArrayToObject(root, "variants");
    for (int v = 0; v < (int)(sizeof(s->variants) / sizeof(s->variants[0])); v++) {
        const stat_variant_t *const e = &s->variants[v];
        if (e->packet_count > 0 || e->decode_errors > 0 || e->process_errors > 0) {
            const iotdata_variant_def_t *vdef = iotdata_get_variant((uint8_t)v);
            cJSON *o = cJSON_CreateObject();
            cJSON_AddNumberToObject(o, "id", (double)v);
            cJSON_AddStringToObject(o, "name", vdef ? vdef->name : "");
            cJSON_AddNumberToObject(o, "packets", (double)e->packet_count);
            cJSON_AddNumberToObject(o, "bytes", (double)e->bytes_rx);
            cJSON_AddNumberToObject(o, "decode_errors", (double)e->decode_errors);
            cJSON_AddNumberToObject(o, "process_errors", (double)e->process_errors);
            cJSON_AddNumberToObject(o, "last_seen", (double)e->last_seen);
            cJSON_AddNumberToObject(o, "age_secs", (double)(e->last_seen ? (now - e->last_seen) : 0));
            cJSON_AddItemToObject(o, "windows", stat_json_windows(&e->ring, now));
            cJSON_AddItemToArray(arr, o);
        }
    }
    return root;
}

cJSON *stat_build_mqtt_json(__attribute__((unused)) const stat_state_t *const s, __attribute__((unused)) const time_t now) {
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

cJSON *stat_build_stat_json(const stat_state_t *const s, const mesh_state_t *const mesh, const ddup_state_t *const ddup) {
    const time_t now = time(NULL);
    cJSON *root = cJSON_CreateObject();
    char buf[4 + 1];
    cJSON_AddStringToObject(root, "station_id", snprintf_inline(buf, sizeof(buf), "%04" PRIX16, s->station_id));
    cJSON_AddStringToObject(root, "version", s->version ? s->version : "");
    cJSON_AddNumberToObject(root, "time", (double)now);
    cJSON_AddNumberToObject(root, "uptime_secs", (double)(now - s->start_time));
    cJSON_AddItemToObject(root, "links", stat_build_links_json(s, now, mesh));
    cJSON_AddItemToObject(root, "stations", stat_build_stations_json(s, now, mesh, ddup));
    cJSON_AddItemToObject(root, "variants", stat_build_variants_json(s, now));
    cJSON_AddItemToObject(root, "mqtt", stat_build_mqtt_json(s, now));
    return root;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

const char *stat_build_stat_string(char *const buf, const size_t size, stat_state_t *const s, const mesh_state_t *const mesh, const ddup_state_t *const ddup) {
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
        STAT_APPEND(", mesh{fwd=%" PRIu32 ", unwrap=%" PRIu32 ", ddup=%" PRIu32 ", beacons=%" PRIu32 ", acks=%" PRIu32 ", ctrl=%" PRIu32 ", err=%" PRIu32 "}", mesh->ctrl[IOTDATA_MESH_CTRL_FORWARD].rx - last->forwards_rx,
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
    if (ddup && ddup->enabled) {
        STAT_APPEND(", ddup{sends=%" PRIu32 "/%" PRIu32 ", recvs=%" PRIu32 "/%" PRIu32 ", injected=%" PRIu32 "}", ddup->stat_send_cycles - last->send_cycles, ddup->stat_send_entries - last->send_entries,
                    ddup->stat_recv_cycles - last->recv_cycles, ddup->stat_recv_entries - last->recv_entries, ddup->stat_injected - last->injected);
        last->send_cycles = ddup->stat_send_cycles;
        last->send_entries = ddup->stat_send_entries;
        last->recv_cycles = ddup->stat_recv_cycles;
        last->recv_entries = ddup->stat_recv_entries;
        last->injected = ddup->stat_injected;
    }
    STAT_APPEND(", mqtt{%s, disconnects=%" PRIu32 "}", mqtt_is_connected() ? "up" : "down", mqtt_stat_disconnects);
#undef STAT_APPEND
    return buf;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

void stat_publish(stat_state_t *const s, const mesh_state_t *const mesh, const ddup_state_t *const ddup) {
    if (!mqtt_is_connected())
        return;
    cJSON *root = stat_build_stat_json(s, mesh, ddup);
    if (root) {
        char *json = cJSON_PrintUnformatted(root);
        cJSON_Delete(root);
        if (json) {
            (void)mqtt_send(snprintf_inline(s->_buffer_topic, sizeof(s->_buffer_topic), "%s/%04" PRIX16, s->mqtt_topic, s->station_id), json, (int)strlen(json));
            free(json);
        }
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void stat_display(stat_state_t *const s, const mesh_state_t *const mesh, const ddup_state_t *const ddup) {
    PRINTF_INFO("stat: %s\n", stat_build_stat_string(s->_buffer_stat, sizeof(s->_buffer_stat), s, mesh, ddup));
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

bool stat_begin(stat_state_t *const s, __attribute__((unused)) const char *topic_prefix, uint16_t station_id, const char *const version, const e22900t22_config_t *const lora_config) {
    assert(topic_prefix && version && lora_config);

    s->version = version;
    s->station_id = station_id;
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

    PRINTF_INFO("stat: started (gateway=%04" PRIX16 ", link=%s, channel=%" PRIu8 ", freq=%" PRIu32 " kHz)\n", s->station_id, s->link.name, s->link.channel, s->link.frequency_khz);

    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void stat_end(__attribute__((unused)) stat_state_t *s) {
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
