
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#ifndef NETWORK_MAX
#define NETWORK_MAX 48
#endif

// -----------------------------------------------------------------------------------------------------------------------------------------

typedef enum { NET_KIND_UNKNOWN = 0, NET_KIND_GATEWAY, NET_KIND_RELAY, NET_KIND_SENSOR } net_kind_t;

typedef enum { NET_PATH_DIRECT = 0, NET_PATH_MESH } net_path_t;

// Per-path reception stats for a sensor: how many packets arrived on this path, how many were
// duplicates (already had them via some path), and how many sequence numbers went missing.
typedef struct {
    uint32_t rx;       /* receptions on this path (duplicates included) */
    uint32_t dup;      /* of those, ones we already had (suppressed) */
    uint32_t gaps;     /* missed sequence numbers in this path's stream */
    uint16_t seq_last; /* last sequence, for gap detection */
    bool seq_valid;
} net_pathstat_t;

#ifndef NET_VIA_MAX
#define NET_VIA_MAX 8 /* distinct relays tracked per sensor (report shows the top 5) */
#endif

typedef struct {
    uint16_t relay;
    uint32_t count; /* forwards this relay carried for the sensor */
} net_via_t;

#ifndef NET_HEARS_MAX
#define NET_HEARS_MAX 16 /* stations we record a relay hearing (from its NEIGHBOUR_REPORT) */
#endif

typedef struct {
    uint16_t station;
    uint8_t cost; /* the heard station's own cost (0xFF = a sensor, per IOTDATA_MESH_NBR_COST_SENSOR) */
    int rssi;     /* dBm the relay hears it at */
} net_heard_t;

typedef struct {
    bool valid;
    uint16_t station;
    net_kind_t kind;
    uint8_t variant;   /* last variant seen */
    int rssi;          /* dBm of the last direct reception (0 = not known) */
    time_t first_seen; /* when we first heard this station (for lifetime-average rates) */
    time_t last_seen;
    uint32_t rx_count;
    /* sensor-data reception stats: per path + the deduped (published) stream */
    net_pathstat_t direct;
    net_pathstat_t mesh;
    net_pathstat_t uniq;        /* the unique/published stream; .dup unused */
    net_via_t via[NET_VIA_MAX]; /* sensor: per-relay forward counts (which relays reach it) */
    int via_count;
    /* mesh node (relay / gateway) liveliness + topology */
    net_pathstat_t tx;  /* the node's WHOLE transmission stream: sender_seq is one counter shared by
                           beacons + forwards + route_errors, so tx.gaps (frames we MISSED) is the
                           only gap we can see — it is relay->gateway link loss, not any one type */
    uint32_t beacon_rx; /* of those frames, how many were beacons (a plain count) */
    bool accepting;     /* advertises ACCEPTING (a usable parent) */
    uint32_t fwd_sent;  /* of those frames, how many were FORWARDs (its forwarding load) */
    uint8_t cost;
    uint16_t generation;
    uint16_t gateway;
    /* relay-end RSSI (from NEIGHBOUR_REPORTs): how well the BEST relay hears this station -> the
       "both ends of the stick" vs our own rssi. 0 = none reported. */
    int relay_rssi;
    uint16_t relay_rssi_from;
    /* relay: the stations THIS relay reports hearing (its NEIGHBOUR_REPORT), for the `hears=[...]`
       row. Replaced wholesale on each report (a relay's current view, not accumulated). */
    net_heard_t hears[NET_HEARS_MAX];
    int hears_count;
} net_station_t;

typedef struct {
    net_station_t s[NETWORK_MAX];
} network_t;

// -----------------------------------------------------------------------------------------------------------------------------------------

const char *net_kind_name(net_kind_t k) {
    switch (k) {
    case NET_KIND_GATEWAY:
        return "GATE";
    case NET_KIND_RELAY:
        return "RLAY";
    case NET_KIND_SENSOR:
        return "SENS";
    default:
        return "?";
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void network_init(network_t *n) {
    memset(n, 0, sizeof(*n));
}

// -----------------------------------------------------------------------------------------------------------------------------------------

int network_count(const network_t *n) {
    int c = 0;
    for (int i = 0; i < NETWORK_MAX; i++)
        if (n->s[i].valid)
            c++;
    return c;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

int network_locate(const network_t *n, uint16_t station) {
    for (int i = 0; i < NETWORK_MAX; i++)
        if (n->s[i].valid && n->s[i].station == station)
            return i;
    return -1;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

net_station_t *network_upsert(network_t *n, uint16_t station, time_t now) {
    int slot = -1, empty = -1, oldest = -1;
    for (int i = 0; i < NETWORK_MAX; i++) {
        if (!n->s[i].valid) {
            if (empty < 0)
                empty = i; /* first free slot; keep scanning for an existing entry */
        } else if (n->s[i].station == station) {
            slot = i; /* existing entry -> update in place */
            break;
        } else if (oldest < 0 || n->s[i].last_seen < n->s[oldest].last_seen)
            oldest = i; /* stalest occupied slot, for eviction if the table is full */
    }
    if (slot < 0) { /* new station: take a free slot if any, else evict the stalest */
        slot = (empty >= 0) ? empty : oldest;
        memset(&n->s[slot], 0, sizeof(n->s[slot]));
        n->s[slot].valid = true;
        n->s[slot].station = station;
        n->s[slot].first_seen = now;
    }
    n->s[slot].last_seen = now;
    n->s[slot].rx_count++;
    return &n->s[slot];
}

// -----------------------------------------------------------------------------------------------------------------------------------------

#ifndef NET_SEQ_MAX_GAP
#define NET_SEQ_MAX_GAP 256 /* a forward jump beyond this is treated as a sensor reset/resync, not loss */
#endif

// Advance a path's sequence tracker, counting how many sequence numbers were skipped. A large
// forward jump, or any backward step (16-bit wrap / reorder / sensor reboot), resyncs without
// counting, so a reboot does not register as thousands of phantom losses.
static inline void net_seq_track(net_pathstat_t *p, uint16_t seq) {
    if (!p->seq_valid) {
        p->seq_last = seq;
        p->seq_valid = true;
    } else {
        const uint16_t delta = (uint16_t)(seq - p->seq_last);
        if (delta != 0) {
            if (delta <= NET_SEQ_MAX_GAP)
                p->gaps += (uint32_t)(delta - 1u);
            p->seq_last = seq; /* advance (or resync, if the jump was a discontinuity) */
        }
    }
}

// Record one sensor-data reception (direct or via a mesh forward). `is_new` comes from the dedup
// ring: true = first time we have seen {station,seq} (it was published), false = duplicate.
void network_note_receive(network_t *n, uint16_t station, uint8_t variant, uint16_t sequence, net_path_t path, bool is_new, int rssi_dbm, uint16_t via_relay, time_t now) {
    net_station_t *e = network_upsert(n, station, now);
    e->variant = variant;
    e->kind = NET_KIND_SENSOR;
    if (rssi_dbm != 0) /* direct rssi only; a forwarded packet's rssi is the relay hop, not the sensor */
        e->rssi = rssi_dbm;
    if (via_relay != 0) { /* attribute this forward to the carrying relay (per-relay counts = topology) */
        int k = 0;
        while (k < e->via_count && e->via[k].relay != via_relay)
            k++;
        if (k == e->via_count && e->via_count < NET_VIA_MAX) {
            e->via[k].relay = via_relay;
            e->via[k].count = 0;
            e->via_count++;
        }
        if (k < e->via_count) /* found or just added; a new relay past the cap is dropped */
            e->via[k].count++;
    }
    net_pathstat_t *const p = (path == NET_PATH_MESH) ? &e->mesh : &e->direct;
    p->rx++;
    if (!is_new)
        p->dup++;
    net_seq_track(p, sequence);
    if (is_new) { /* the deduped stream = what we actually published; its gaps are true end-to-end loss */
        e->uniq.rx++;
        net_seq_track(&e->uniq, sequence);
    }
}

// Every mesh frame a node sends carries sender_seq from one shared counter, so this is the ONLY
// gap we can measure: frames of ANY type we missed = relay->gateway link loss. Call once per
// received mesh frame (station_id / sequence from the header).
void network_note_transmit(network_t *n, uint16_t station, uint16_t sender_seq, time_t now) {
    net_station_t *e = network_upsert(n, station, now);
    e->tx.rx++;
    net_seq_track(&e->tx, sender_seq);
}

// Beacon specifics: liveliness count, the ACCEPTING flag, and cost/generation/gateway topology.
// The sequence gap is tracked by network_note_tx across all frame types, not here.
void network_note_beacon(network_t *n, uint16_t station, uint8_t flags, uint8_t cost, uint16_t generation, uint16_t gateway, int rssi_dbm, time_t now) {
    net_station_t *e = network_upsert(n, station, now);
    e->kind = (cost == 0) ? NET_KIND_GATEWAY : NET_KIND_RELAY;
    e->variant = IOTDATA_MESH_VARIANT;
    if (rssi_dbm != 0)
        e->rssi = rssi_dbm;
    e->cost = cost;
    e->generation = generation;
    e->gateway = gateway;
    e->accepting = (flags & IOTDATA_MESH_FLAG_ACCEPTING) != 0;
    e->beacon_rx++;
}

// A FORWARD arrived from this relay: bump how many it has forwarded to us (its forwarding load).
void network_note_forward(network_t *n, uint16_t relay_station, time_t now) {
    net_station_t *e = network_upsert(n, relay_station, now);
    e->fwd_sent++;
}

// Ingest a relay's NEIGHBOUR_REPORT: replace its `hears=[...]` list with what it now reports, and
// fold each heard station's relay-end RSSI into the both-ends view (keeping the strongest). The
// heard list is collected locally FIRST, so the reporting relay's own entry can be written last --
// after any network_upsert evictions the per-neighbour loop may trigger -- with no dangling pointer.
void network_note_neighbour_report(network_t *n, const uint8_t *buf, int len, time_t now) {
    iotdata_mesh_neighbour_report_t r;
    if (iotdata_mesh_unpack_neighbour_report(buf, len, &r)) {
        net_heard_t heard[NET_HEARS_MAX];
        int hc = 0;
        for (int k = 0; k < (int)r.num_neighbours; k++) {
            iotdata_mesh_nbr_entry_t ent;
            if (!iotdata_mesh_neighbour_report_entry(buf, len, k, &ent))
                break;
            const int rssi = iotdata_mesh_rssi_from_q4(ent.rssi_q4);
            if (hc < NET_HEARS_MAX)
                heard[hc++] = (net_heard_t){ .station = ent.station, .cost = ent.cost, .rssi = rssi };
            net_station_t *s = network_upsert(n, ent.station, now); /* used immediately, not held */
            if (s->kind == NET_KIND_UNKNOWN)
                s->kind = (ent.cost == IOTDATA_MESH_NBR_COST_SENSOR) ? NET_KIND_SENSOR : (ent.cost == 0 ? NET_KIND_GATEWAY : NET_KIND_RELAY);
            if (s->relay_rssi == 0 || rssi > s->relay_rssi) {
                s->relay_rssi = rssi;
                s->relay_rssi_from = r.sender_station;
            }
        }
        net_station_t *rel = network_upsert(n, r.sender_station, now); /* written last: no eviction risk */
        if (rel->kind == NET_KIND_UNKNOWN)
            rel->kind = (r.my_cost == 0) ? NET_KIND_GATEWAY : NET_KIND_RELAY;
        if (r.my_cost != IOTDATA_MESH_NBR_COST_SENSOR)
            rel->cost = r.my_cost;
        rel->gateway = r.gateway_id;
        for (int k = 0; k < hc; k++)
            rel->hears[k] = heard[k];
        rel->hears_count = hc;
    }
}

// Lifetime-average rate (events/minute) since first heard. Stable for slow, fixed-interval
// senders where a per-report-interval delta would just read 0 or 1.
static inline double net_rate_per_min(uint32_t count, time_t first_seen, time_t now) {
    const double secs = (double)(now - first_seen);
    return (secs >= 1.0) ? ((double)count * 60.0 / secs) : 0.0;
}

// Format a sensor's forwarding relays as "[0xAAAA=n,0xBBBB=m]" (empty if none), the top 5 by
// count, "…" if more. Bounded and truncation-safe.
static inline const char *net_via_format(const net_station_t *e, char *out, size_t outlen) {
    if (outlen > 0) {
        out[0] = '\0';
        if (e->via_count > 0) {
            bool used[NET_VIA_MAX] = { false };
            int pos = snprintf(out, outlen, "[");
            if (pos < 0)
                pos = 0;
            const int limit = (e->via_count < 5) ? e->via_count : 5;
            for (int s = 0; s < limit && (size_t)pos < outlen; s++) {
                int best = -1;
                for (int k = 0; k < e->via_count; k++)
                    if (!used[k] && (best < 0 || e->via[k].count > e->via[best].count))
                        best = k;
                if (best < 0)
                    break;
                used[best] = true;
                const int w = snprintf(out + pos, outlen - (size_t)pos, "%s%04" PRIX16 "=%" PRIu32, s ? "," : "", e->via[best].relay, e->via[best].count);
                if (w > 0)
                    pos += w;
            }
            if ((size_t)pos < outlen)
                (void)snprintf(out + pos, outlen - (size_t)pos, "%s]", e->via_count > 5 ? ",..." : "");
        }
    }
    return out;
}

// Format a relay's heard stations as " hears=[0F1B/1/-45dBm,0AFA/255/-55dBm]" (empty if none) --
// station/cost/rssi, cost 255 = a sensor. Includes its own leading " hears=" so callers append it
// unconditionally. Bounded + truncation-safe.
static inline const char *net_hears_format(const net_station_t *e, char *out, size_t outlen) {
    if (outlen > 0) {
        out[0] = '\0';
        if (e->hears_count > 0) {
            int pos = snprintf(out, outlen, " hears=[");
            if (pos < 0)
                pos = 0;
            for (int k = 0; k < e->hears_count && (size_t)pos < outlen; k++) {
                const int w = snprintf(out + pos, outlen - (size_t)pos, "%s%04" PRIX16 "/%u/%ddBm", k ? "," : "", e->hears[k].station, (unsigned)e->hears[k].cost, e->hears[k].rssi);
                if (w > 0)
                    pos += w;
            }
            if ((size_t)pos < outlen)
                (void)snprintf(out + pos, outlen - (size_t)pos, "]");
        }
    }
    return out;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

// End-to-end loss as a percentage: gaps / (received + gaps). 0 when nothing is expected yet.
static inline double net_loss_pct(uint32_t gaps, uint32_t rx) {
    const uint32_t total = gaps + rx;
    return total ? ((double)gaps * 100.0 / (double)total) : 0.0;
}

#ifndef NET_STALE_SEC
#define NET_STALE_SEC 3600 /* a sensor unheard this long is hidden from the snapshot + summary (record kept) */
#endif
static inline bool net_is_stale(const net_station_t *e, time_t now) {
    return (now - e->last_seen) > NET_STALE_SEC;
}

void network_report(const network_t *n, uint16_t gateway_id) {
    const time_t now = time(NULL);
    int n_relay = 0, n_sensor = 0, n_gw = 0, n_stale = 0;
    uint32_t loss = 0, rx_total = 0;
    for (int i = 0; i < NETWORK_MAX; i++) {
        const net_station_t *e = &n->s[i];
        if (e->valid) {
            if (e->kind == NET_KIND_GATEWAY)
                n_gw++;
            else if (e->kind == NET_KIND_RELAY)
                n_relay++;
            else if (net_is_stale(e, now)) /* ghost sensor: keep the record, drop it from the live view */
                n_stale++;
            else {
                n_sensor++;
                loss += e->uniq.gaps;
                rx_total += e->uniq.rx;
            }
        }
    }
    char stale[32] = "";
    if (n_stale > 0)
        (void)snprintf(stale, sizeof(stale), ", %d stale hidden", n_stale);
    printf("network: gw=%04" PRIX16 " | %d relay(s), %d sensor(s)%s%s | end-to-end loss=%" PRIu32 " seq (%.1f%%):\n", gateway_id, n_relay, n_sensor, n_gw ? " (+peer gw)" : "", stale, loss, net_loss_pct(loss, rx_total));
    for (int i = 0; i < NETWORK_MAX; i++) { /* mesh nodes: gateways + relays */
        const net_station_t *e = &n->s[i];
        if (e->valid && (e->kind == NET_KIND_RELAY || e->kind == NET_KIND_GATEWAY)) {
            char hears[NET_HEARS_MAX * 20];
            printf("  %04" PRIX16 " %-4s rssi=%ddBm age=%lds tx=%" PRIu32 "(gap=%" PRIu32 ",%.1f/min) beacons=%" PRIu32 " fwd=%" PRIu32 "(%.1f/min) acc=%c cost=%u gen=%u gw=%04" PRIX16 "%s\n", e->station, net_kind_name(e->kind), e->rssi,
                   (long)(now - e->last_seen), e->tx.rx, e->tx.gaps, net_rate_per_min(e->tx.rx, e->first_seen, now), e->beacon_rx, e->fwd_sent, net_rate_per_min(e->fwd_sent, e->first_seen, now), e->accepting ? 'Y' : 'N', (unsigned)e->cost,
                   (unsigned)e->generation, e->gateway, net_hears_format(e, hears, sizeof(hears)));
        }
    }
    for (int i = 0; i < NETWORK_MAX; i++) { /* sensors (and anything not yet classified as a mesh node) */
        const net_station_t *e = &n->s[i];
        if (e->valid && !(e->kind == NET_KIND_RELAY || e->kind == NET_KIND_GATEWAY) && !net_is_stale(e, now)) {
            char via[80], rly[40] = "";
            if (e->relay_rssi != 0) /* both ends of the stick: gateway rssi (above) vs the best relay's */
                (void)snprintf(rly, sizeof(rly), " relay=%ddBm(%04" PRIX16 ")", e->relay_rssi, e->relay_rssi_from);
            printf("  %04" PRIX16 " %-4s rssi=%ddBm%s age=%lds var=%u uniq=%" PRIu32 "(gap=%" PRIu32 ",%.1f%%,%.1f/min) recv=%" PRIu32 "(dup=%" PRIu32 ",gap=%" PRIu32 ") mesh=%" PRIu32 "(dup=%" PRIu32 ",gap=%" PRIu32 ")%s\n", e->station,
                   net_kind_name(e->kind), e->rssi, rly, (long)(now - e->last_seen), (unsigned)e->variant, e->uniq.rx, e->uniq.gaps, net_loss_pct(e->uniq.gaps, e->uniq.rx), net_rate_per_min(e->uniq.rx, e->first_seen, now), e->direct.rx,
                   e->direct.dup, e->direct.gaps, e->mesh.rx, e->mesh.dup, e->mesh.gaps, net_via_format(e, via, sizeof(via)));
        }
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
