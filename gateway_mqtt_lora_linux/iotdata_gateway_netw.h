
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#ifndef NETWORK_MAX
#define NETWORK_MAX 48
#endif

// -----------------------------------------------------------------------------------------------------------------------------------------

typedef enum { NET_KIND_UNKNOWN = 0, NET_KIND_GATEWAY, NET_KIND_RELAY, NET_KIND_SENSOR } net_kind_t;

typedef struct {
    bool valid;
    uint16_t station;
    net_kind_t kind;
    uint8_t variant; /* last variant seen */
    int rssi;        /* dBm of the last direct reception (0 = not known) */
    time_t last_seen;
    uint32_t rx_count;
    /* mesh topology hints (relays / gateways), from beacons */
    uint8_t cost;
    uint16_t generation;
    uint16_t gateway;
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
    }
    n->s[slot].last_seen = now;
    n->s[slot].rx_count++;
    return &n->s[slot];
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void network_seen(network_t *n, uint16_t station, net_kind_t kind, uint8_t variant, int rssi_dbm, time_t now) {
    net_station_t *e = network_upsert(n, station, now);
    e->variant = variant;
    if (rssi_dbm != 0)
        e->rssi = rssi_dbm;
    if (kind != NET_KIND_UNKNOWN)
        e->kind = kind;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void network_note_mesh(network_t *n, uint16_t station, uint8_t cost, uint16_t generation, uint16_t gateway) {
    const int i = network_locate(n, station);
    if (i < 0)
        return;
    n->s[i].cost = cost;
    n->s[i].generation = generation;
    n->s[i].gateway = gateway;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void network_report(const network_t *n, uint16_t gateway_id) {
    const time_t now = time(NULL);
    printf("network: gateway=0x%04" PRIX16 ", %d station(s) heard:\n", gateway_id, network_count(n));
    for (int i = 0; i < NETWORK_MAX; i++) {
        const net_station_t *e = &n->s[i];
        if (!e->valid)
            continue;
        if (e->kind == NET_KIND_RELAY || e->kind == NET_KIND_GATEWAY)
            printf("  0x%04" PRIX16 " %-4s var=%-2u rssi=%ddBm age=%lds rx=%" PRIu32 " cost=%u gen=%u gw=0x%04" PRIX16 "\n", e->station, net_kind_name(e->kind), (unsigned)e->variant, e->rssi, (long)(now - e->last_seen), e->rx_count,
                   (unsigned)e->cost, (unsigned)e->generation, e->gateway);
        else
            printf("  0x%04" PRIX16 " %-4s var=%-2u rssi=%ddBm age=%lds rx=%" PRIu32 "\n", e->station, net_kind_name(e->kind), (unsigned)e->variant, e->rssi, (long)(now - e->last_seen), e->rx_count);
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
