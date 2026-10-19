
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#ifndef DDUP_PEERS_MAX
#define DDUP_PEERS_MAX 16
#endif
#ifndef DDUP_PENDING_MAX
#define DDUP_PENDING_MAX 256
#endif
#ifndef DDUP_BATCH_MAX
#define DDUP_BATCH_MAX 32
#endif

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

// UDP has no app-level ACK, so a single dropped datagram tells us nothing. But the LOCAL stack
// still surfaces a gone peer via sendto() errors — ARP timeout on a LAN peer, or a returned ICMP
// no-route/unreachable. A RUN of such failures (not a one-off) drops the peer to `unresolved`; the
// tick below then periodically retries (re-resolve / re-ARP) so it recovers when the host returns.
#define DDUP_PEER_FAIL_STREAK_MAX    8  /* consecutive sendto() failures before a peer -> unresolved */
#define DDUP_PEER_RESOLVE_RETRY_SECS 30 /* re-resolve unresolved peers this often (boot DNS not ready, or recovery) */
#define DDUP_CLASH_WARN_SECS         30 /* throttle the shared-gateway-id warning to one per this (every packet would clash) */

// AUTO-PEER LIVENESS: the presence message (iotdata/ddup/<gwid>) is RETAINED with an empty-payload LWT,
// so a clean death (process exit / local-broker disconnect) prunes the peer instantly. But a WHOLE-BOX or
// bridge death leaves the last retained presence frozen on the master — the LWT fires on the dead box's
// LOCAL broker and never reaches us. So the presence also carries the publisher's `ts`; we prune any auto
// peer whose ts hasn't advanced within the TTL. Using the PAYLOAD ts (not receive time) is deliberate: a
// retained re-delivery on a bridge reconnect carries the OLD ts, so it can't falsely reset the timer.
#define DDUP_PEER_TTL_SECS           90 /* prune an auto peer not re-announced within this (3x DDUP_PEERS_REPUBLISH_SECS) */

#define DDUP_PKT_HEADER_SIZE         3
#define DDUP_PKT_SIZE                (DDUP_PKT_HEADER_SIZE + DDUP_BATCH_MAX * 4) /* 131 bytes */

// HOLD-AND-ELECT: when ddup runs with peers, a freshly-received reading isn't published immediately
// — it's HELD for `hold_ms` while we broadcast our claim and listen for peers'. The lowest gateway-id
// that received it wins (publishes on hold expiry); everyone else drops. This is what actually kills
// the concurrent-receive duplicate (both gateways hearing the same broadcast ~simultaneously).
#define DDUP_HOLD_MAX                256  /* max readings held in flight */
#define DDUP_HOLD_PKT_MAX            256  /* bytes copied per held reading (>= e22 max packet) */
#define DDUP_HOLD_MS_DEFAULT         1000 /* ddup-hold-ms: the SAFETY CEILING — the dynamic hold never exceeds it */

// The same election decides which gateway sends a direct ACK (iotdata_gateway_mesh.h): one waiting on
// its slot asks ddup_claimed() first, and stands down if a lower-id peer has the frame. A slot is a
// fraction of a second, so a short ring of the latest claims covers it many times over.
#define DDUP_CLAIMS_MAX              16

// DYNAMIC HOLD: instead of always waiting the full ceiling, tune the hold DOWN toward the measured
// peer-claim latency (max + margin), bounded by [floor, ceiling]. Leak-safe: never below an observed
// claim latency, never above the configured ceiling. Lowest-gateway publishes immediately (no hold).
#define DDUP_HOLD_FLOOR_MS           50 /* never hold less than this */
#define DDUP_LATENCY_DECAY_SECS      30 /* bleed the measured-max down 1/8 every this often (re-tighten as it proves fast) */
#define DDUP_LATENCY_MIN_SAMPLES     20 /* stay at the ceiling until we've measured at least this many claim latencies */

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

// verdict from ddup_admit(): the single decision point for a received sensor packet.
typedef enum {
    DDUP_REJECT,  /* already decided (retransmit / mesh-multipath / late forward) — drop now */
    DDUP_PUBLISH, /* publish immediately (ddup off, or no peers to contest) */
    DDUP_HOLD,    /* held for the election — released later via the callback */
} ddup_verdict_t;

// release callback: ddup hands a held reading back to the publish path (process_sensor_packet) on expiry.
/* RSSI TRAVELS WITH THE HELD READING. It is a property of THIS hop's reception -- the moment the
   packet arrived -- so it cannot be recovered when the hold expires a second later. Without it
   every CONTESTED reading would publish with no signal strength, which is precisely the set of
   readings an overlapping-coverage site most wants it for. */
typedef void (*ddup_release_fn)(void *ctx, const uint8_t *packet, int packet_len, uint8_t variant, uint16_t station, uint16_t sequence, const char *topic_prefix, const char *via, int rssi_dbm);

typedef struct {
    bool used;
    uint16_t station, sequence;
    uint8_t variant;
    int packet_len;
    uint8_t packet[DDUP_HOLD_PKT_MAX]; /* copied: the rx buffer is reused next loop */
    const char *topic_prefix, *via;    /* stable pointers, carried through to the release */
    int rssi_dbm;                      /* this hop's reception, unrecoverable once the moment passes */
    uint32_t received_ms;              /* the tick we received it on — for the claim-latency measurement on a drop */
    uint32_t deadline_ms;              /* received + dynamic hold; publish (we won) when reached */
} ddup_hold_entry_t;

// -----------------------------------------------------------------------------------------------------------------------------------------

typedef struct {
    bool used;               /* slot occupied (sparse array — add/remove set/clear, never compact, so lockless readers stay consistent) */
    bool is_auto;            /* discovered via MQTT presence (keyed by announced_gwid) vs static config (keyed by host:port) */
    uint16_t announced_gwid; /* the presence gateway-id for an auto peer (the upsert key) */
    char host[128];
    uint16_t port;
    struct sockaddr_in addr;
    bool resolved;
    bool broken;               /* peer is announcing OUR gateway-id (shared-id misconfig): drop its claims, but keep sending so recovery is observable. cleared when it sends a distinct id again */
    uint32_t send_fail_streak; /* consecutive sendto() failures (ARP timeout / returned ICMP); a run drops the peer to unresolved */
    time_t announced_ts;       /* AUTO peers only: the publisher's epoch ts from its last presence; ages the peer out when its box/bridge died and the LWT never reached us. 0 = un-stamped (legacy peer) -> never TTL-expired */
} ddup_peer_t;

typedef struct {
    bool enabled;
    uint16_t port;
    uint32_t delay_ms;
    ddup_peer_t peers[DDUP_PEERS_MAX];
    int peers_count;              /* number of USED slots (sparse array; iterate slots by .used, not this) */
    bool auto_peer;               /* ddup-peers contained an "mqtt[:qualifier]" token -> auto-discover via MQTT presence */
    char advertise_qualifier[64]; /* what to advertise as our address: "" = auto-detect, "eth0"/"wlan0" = interface, or a literal IP */
    pthread_mutex_t mutex;
    pthread_t thread;
    iotdata_mesh_dedup_entry_t pending[DDUP_PENDING_MAX];
    int pending_count;
    uint32_t pending_first_ms; /* tick the batch opened on; only read while pending_count > 0, so it needs no unset sentinel */
    uint16_t gateway_id;
    iotdata_mesh_dedup_ring_t *dedup_ring;
    volatile bool *running;
    bool debug;
    unsigned debug_inject_latency_ms;
    /* hold-and-elect queue (publish-point dedup) */
    uint32_t hold_ms;                     /* ddup-hold-ms: the SAFETY CEILING; the dynamic hold never exceeds it (0 = publish immediately) */
    uint32_t hold_ms_dynamic;             /* the auto-tuned hold actually used (<= hold_ms); for reporting */
    uint32_t claim_latency_max_ms;        /* decaying high-water of measured peer-claim latency -> drives the dynamic hold */
    uint64_t claim_latency_sum_ms;        /* for the reported average */
    uint32_t claim_latency_count;         /* number of claim-latency samples */
    uint32_t claim_latency_decay_last_ms; /* tick we last bled the max down on (0 = never) */
    ddup_hold_entry_t hold[DDUP_HOLD_MAX];
    iotdata_mesh_dedup_entry_t claims[DDUP_CLAIMS_MAX]; /* lower-id peers' latest claims, for ddup_claimed() */
    int claims_count, claims_next;
    /* statistics */
    uint32_t stat_held;          /* readings entered into the hold queue */
    uint32_t stat_released;      /* held readings that won the election and published */
    uint32_t stat_dropped_peer;  /* held readings dropped because a lower-id peer claimed them */
    uint32_t stat_hold_overflow; /* hold queue full -> reading published un-held (degrade, don't lose) */
    uint32_t stat_duplicates;    /* sensor packets suppressed as duplicates at the publish point (direct / orthogonal ddup) */
    uint32_t stat_send_cycles;
    uint32_t stat_send_entries;
    uint32_t stat_send_errors;
    uint32_t stat_recv_cycles;
    uint32_t stat_recv_entries;
    uint32_t stat_recv_errors;
    uint32_t stat_injected;
    uint32_t stat_pending_overflow;
    uint32_t stat_peers_resolved;
    uint32_t stat_peers_unresolved;
    uint32_t stat_peers_expired; /* auto peers pruned by the presence-ts TTL (box/bridge death the LWT couldn't cover) */
    uint32_t stat_id_clash;      /* peer packets seen carrying OUR gateway-id (misconfig) */
    uint32_t stat_recv_unknown;  /* packets dropped: source IP is not a configured peer */
    uint32_t last_clash_warn_ms; /* throttle the clash warning (0 = never warned) */
} ddup_state_t;

#define DDUP_MIN(a, b) ((a) < (b) ? (a) : (b))

// -----------------------------------------------------------------------------------------------------------------------------------------

typedef uint8_t ddup_packet_t[DDUP_PKT_SIZE];

#define ddup_packet_get_length(pkt)                      (size_t)(3 + (size_t)pkt[2] * 4)

#define ddup_packet_get_gateway_id(pkt)                  (((uint16_t)pkt[0] << 8) | (uint16_t)pkt[1])
#define ddup_packet_get_entry_count(pkt)                 (DDUP_MIN(pkt[2], DDUP_BATCH_MAX))
#define ddup_packet_get_entry_station(pkt, entry_index)  (((uint16_t)pkt[(3 + entry_index * 4) + 0] << 8) | (uint16_t)pkt[(3 + entry_index * 4) + 1])
#define ddup_packet_get_entry_sequence(pkt, entry_index) (((uint16_t)pkt[(3 + entry_index * 4) + 2] << 8) | (uint16_t)pkt[(3 + entry_index * 4) + 3])

#define ddup_packet_set_gateway_id(pkt, gateway_id) \
    do { \
        pkt[0] = (uint8_t)((gateway_id) >> 8); \
        pkt[1] = (uint8_t)((gateway_id) & 0xFF); \
    } while (0)
#define ddup_packet_set_entry_count(pkt, entry_count) \
    do { \
        pkt[2] = (uint8_t)(entry_count); \
    } while (0)
#define ddup_packet_set_entry_station(pkt, entry_index, station) \
    do { \
        pkt[(3 + entry_index * 4) + 0] = (uint8_t)((station) >> 8); \
        pkt[(3 + entry_index * 4) + 1] = (uint8_t)((station) & 0xFF); \
    } while (0)
#define ddup_packet_set_entry_sequence(pkt, entry_index, sequence) \
    do { \
        pkt[(3 + entry_index * 4) + 2] = (uint8_t)((sequence) >> 8); \
        pkt[(3 + entry_index * 4) + 3] = (uint8_t)((sequence) & 0xFF); \
    } while (0)

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

void ddup_peers_parse(ddup_state_t *state, const char *peers_str) {
    if (!peers_str || !*peers_str)
        return;
    char buf[1024];
    char *save = NULL, *tok = strtok_r(snprintf_inline(buf, sizeof(buf), "%s", peers_str), ",", &save);
    while (tok) {
        while (*tok == ' ')
            tok++;
        // "mqtt" or "mqtt:<iface-or-ip>" enables MQTT auto-discovery; the qualifier is what WE
        // advertise as our own address (iface name / literal IP / empty = auto-detect). Not a peer.
        if (strncmp(tok, "mqtt", 4) == 0 && (tok[4] == '\0' || tok[4] == ':')) {
            state->auto_peer = true;
            snprintf(state->advertise_qualifier, sizeof(state->advertise_qualifier), "%s", tok[4] == ':' ? tok + 5 : "");
        } else {
            // static peer host[:port] -> lowest free slot (sparse)
            int slot = -1;
            for (int i = 0; i < (int)(sizeof(state->peers) / sizeof(state->peers[0])); i++) {
                ddup_peer_t *p = &state->peers[i];
                if (!p->used) {
                    slot = i;
                    break;
                }
            }
            if (slot < 0)
                break;
            char *colon = strrchr(tok, ':');
            uint16_t port = state->port;
            if (colon) {
                *colon = '\0';
                port = (uint16_t)atoi(colon + 1);
            }
            ddup_peer_t *p = &state->peers[slot];
            *p = (ddup_peer_t){ .used = true };
            snprintf(p->host, sizeof(p->host), "%s", tok);
            p->port = port;
            state->peers_count++;
        }
        tok = strtok_r(NULL, ",", &save);
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

bool ddup_peer_resolve_one(ddup_peer_t *peer) {
    char port_str[8];
    struct addrinfo *res;
    const int err = getaddrinfo(peer->host, snprintf_inline(port_str, sizeof(port_str), "%" PRIu16, peer->port), &(const struct addrinfo){ .ai_family = AF_INET, .ai_socktype = SOCK_DGRAM }, &res);
    if (err == 0) {
        memcpy(&peer->addr, res->ai_addr, sizeof(peer->addr));
        peer->resolved = true;
        freeaddrinfo(res);
        return true;
    }
    peer->resolved = false;
    PRINTF_ERROR("ddup: peer %s:%" PRIu16 " resolution failed: %s\n", peer->host, peer->port, gai_strerror(err));
    return false;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void ddup_peers_resolve(ddup_state_t *state) {
    state->stat_peers_resolved = 0;
    state->stat_peers_unresolved = 0;
    for (int i = 0; i < (int)(sizeof(state->peers) / sizeof(state->peers[0])); i++) {
        ddup_peer_t *p = &state->peers[i];
        if (p->used) {
            if (ddup_peer_resolve_one(p)) {
                state->stat_peers_resolved++;
                PRINTF_INFO("ddup: peer[%d] %s:%" PRIu16 " resolved\n", i, p->host, p->port);
            } else
                state->stat_peers_unresolved++;
        }
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void ddup_reconfigure(ddup_state_t *live, const char *peers_str, uint32_t delay_ms, bool debug, unsigned debug_inject_latency_ms) {
    ddup_state_t scratch = { .port = live->port }; // ddup_peers_parse uses it as the default per-peer port
    ddup_peers_parse(&scratch, peers_str);
    for (int i = 0; i < (int)(sizeof(scratch.peers) / sizeof(scratch.peers[0])); i++) {
        ddup_peer_t *p = &scratch.peers[i];
        if (p->used)
            (void)ddup_peer_resolve_one(p); // getaddrinfo OUTSIDE the lock
    }

    pthread_mutex_lock(&live->mutex);
    // rebuild ONLY the static peers — auto-discovered (is_auto) peers are owned by the MQTT presence
    // path and must survive a config reconcile (e.g. a ddup-delay change must not wipe live peers).
    for (int i = 0; i < (int)(sizeof(live->peers) / sizeof(live->peers[0])); i++) {
        ddup_peer_t *p = &live->peers[i];
        if (p->used && !p->is_auto) {
            p->used = false;
            if (live->peers_count > 0)
                live->peers_count--;
        }
    }
    for (int i = 0; i < (int)(sizeof(scratch.peers) / sizeof(scratch.peers[0])); i++) {
        ddup_peer_t *p = &scratch.peers[i];
        if (p->used) {
            int slot = -1;
            for (int j = 0; j < (int)(sizeof(live->peers) / sizeof(live->peers[0])); j++) {
                ddup_peer_t *p2 = &live->peers[j];
                if (!p2->used) {
                    slot = j;
                    break;
                }
            }
            if (slot < 0)
                break;
            ddup_peer_t *p2 = &live->peers[slot];
            *p2 = *p; // already resolved
            live->peers_count++;
        }
    }
    live->auto_peer = scratch.auto_peer;
    snprintf(live->advertise_qualifier, sizeof(live->advertise_qualifier), "%s", scratch.advertise_qualifier);
    live->delay_ms = delay_ms;
    live->debug = debug;
    live->debug_inject_latency_ms = debug_inject_latency_ms;
    pthread_mutex_unlock(&live->mutex);
    PRINTF_INFO("ddup: live reconfigure -> %d peers (auto=%c), delay=%" PRIu32 "ms, debug=%s\n", live->peers_count, live->auto_peer ? 'y' : 'n', live->delay_ms, debug ? "on" : "off");
}

// -----------------------------------------------------------------------------------------------------------------------------------------

bool ddup_peer_insert(ddup_state_t *state, uint16_t gwid, const char *host, uint16_t port, time_t announced_ts) {
    pthread_mutex_lock(&state->mutex);
    int slot = -1, slot_free = -1;
    for (int i = 0; i < (int)(sizeof(state->peers) / sizeof(state->peers[0])); i++) {
        ddup_peer_t *p = &state->peers[i];
        if (!p->used) {
            if (slot_free < 0)
                slot_free = i;
        } else if (p->is_auto && p->announced_gwid == gwid) {
            slot = i;
            break;
        }
    }
    const bool fresh = (slot < 0);
    if (fresh) {
        if (slot_free < 0) {
            pthread_mutex_unlock(&state->mutex);
            return false; // table full
        }
        slot = slot_free;
    }
    ddup_peer_t *p = &state->peers[slot];
    const bool changed = !fresh && (p->port != port || strcmp(p->host, host) != 0);
    if (fresh) {
        *p = (ddup_peer_t){ 0 };
        state->peers_count++;
    }
    p->used = true;
    p->is_auto = true;
    p->announced_gwid = gwid;
    snprintf(p->host, sizeof(p->host), "%s", host);
    p->port = port;
    if (fresh || changed) {
        // (re)resolve; clear broken/streak so a moved peer re-elects cleanly
        p->resolved = false;
        p->broken = false;
        p->send_fail_streak = 0;
        struct in_addr ina;
        if (inet_pton(AF_INET, host, &ina) == 1) {
            p->addr.sin_family = AF_INET;
            p->addr.sin_addr = ina;
            p->addr.sin_port = htons(port);
            p->resolved = true;
        }
    }
    p->announced_ts = announced_ts; /* refresh liveness on every presence, even an unchanged-address republish */
    pthread_mutex_unlock(&state->mutex);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void ddup_peer_remove(ddup_state_t *state, uint16_t gwid) {
    pthread_mutex_lock(&state->mutex);
    for (int i = 0; i < (int)(sizeof(state->peers) / sizeof(state->peers[0])); i++) {
        ddup_peer_t *p = &state->peers[i];
        if (p->used && p->is_auto && p->announced_gwid == gwid) {
            p->used = false;
            if (state->peers_count > 0)
                state->peers_count--;
            break;
        }
    }
    pthread_mutex_unlock(&state->mutex);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

int ddup_peers_expire(ddup_state_t *state, time_t now) {
    int removed = 0;
    pthread_mutex_lock(&state->mutex);
    for (int i = 0; i < (int)(sizeof(state->peers) / sizeof(state->peers[0])); i++) {
        ddup_peer_t *p = &state->peers[i];
        if (p->used && p->is_auto && p->announced_ts != 0) {
            if ((long long)(now - p->announced_ts) > DDUP_PEER_TTL_SECS) {
                PRINTF_WARN("ddup: auto peer gateway=0x%04" PRIX16 " presence stale (%llds, no LWT) -> removed\n", p->announced_gwid, (long long)(now - p->announced_ts));
                p->used = false;
                if (state->peers_count > 0)
                    state->peers_count--;
                state->stat_peers_expired++;
                removed++;
            }
        }
    }
    pthread_mutex_unlock(&state->mutex);
    return removed;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

int ddup_recv_setup(ddup_state_t *state) {
    const int recv_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (recv_fd < 0) {
        PRINTF_ERROR("ddup: recv socket: %s\n", strerror(errno));
        return -1;
    }
    int optval = 1;
    setsockopt(recv_fd, SOL_SOCKET, SO_REUSEADDR, &optval, (socklen_t)sizeof(optval));
    if (bind(recv_fd, (const struct sockaddr *)&(struct sockaddr_in){ .sin_family = AF_INET, .sin_port = htons(state->port), .sin_addr.s_addr = htonl(INADDR_ANY) }, (socklen_t)sizeof(struct sockaddr_in)) < 0) {
        PRINTF_ERROR("ddup: bind port %" PRIu16 ": %s\n", state->port, strerror(errno));
        close(recv_fd);
        return -1;
    }
    return recv_fd;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void ddup_hold_mark_dropped(ddup_state_t *state, uint16_t station, uint16_t sequence); /* defined below; used by the election hook here */
void ddup_claim_note(ddup_state_t *state, uint16_t station, uint16_t sequence);        /* likewise */

void ddup_recv_from_peers(ddup_state_t *state, int recv_fd) {
    struct pollfd pfd = { .fd = recv_fd, .events = POLLIN, .revents = 0 };
    if (poll(&pfd, 1, 5) > 0 && (pfd.revents & POLLIN)) {
        ddup_packet_t pkt;
        struct sockaddr_in from;
        socklen_t from_len = (socklen_t)sizeof(from);
        const ssize_t recv_len = recvfrom(recv_fd, pkt, sizeof(pkt), 0, (struct sockaddr *)&from, &from_len);
        if (recv_len < DDUP_PKT_HEADER_SIZE) {
            state->stat_recv_errors++;
            return;
        }
        const int entry_count = ddup_packet_get_entry_count(pkt);
        if (recv_len < (ssize_t)ddup_packet_get_length(pkt)) {
            state->stat_recv_errors++;
            return;
        }
        const uint16_t sender = ddup_packet_get_gateway_id(pkt);
        char peerdesc[160] = "";
        bool drop = false, warn = false, recovered = false;
        pthread_mutex_lock(&state->mutex);
        // map the source IP to a configured peer (an initial sanity layer — not real security yet)
        int pi = -1;
        for (int i = 0; i < (int)(sizeof(state->peers) / sizeof(state->peers[0])); i++) {
            ddup_peer_t *p = &state->peers[i];
            if (p->used && p->resolved && p->addr.sin_addr.s_addr == from.sin_addr.s_addr) {
                pi = i;
                break;
            }
        }
        if (pi < 0) {
            drop = true; /* unknown source — not one of ours */
            state->stat_recv_unknown++;
        } else if (sender == state->gateway_id) {
            ddup_peer_t *p = &state->peers[pi];
            // Shared gateway-id: the election can't work against this peer (lowest-id wins is a tie).
            // Flag it BROKEN and drop its claims — but DON'T stop sending to it (going silent would
            // deadlock recovery, which is observed by RECEIVING a distinct id). admit() publishes
            // immediately while no usable peer remains; self-heals when the peer's id becomes distinct.
            drop = true;
            state->stat_id_clash++;
            p->broken = true;
            snprintf(peerdesc, sizeof(peerdesc), "%s:%" PRIu16, p->host, p->port);
            const uint32_t now_ms = hw_time_ms();
            if (state->last_clash_warn_ms == 0 || (now_ms - state->last_clash_warn_ms) >= DDUP_CLASH_WARN_SECS * 1000u) {
                state->last_clash_warn_ms = now_ms;
                warn = true;
            }
        } else {
            ddup_peer_t *p = &state->peers[pi];
            if (p->broken) { /* distinct id again -> recovered */
                p->broken = false;
                recovered = true;
                snprintf(peerdesc, sizeof(peerdesc), "%s:%" PRIu16, p->host, p->port);
            }
            for (int entry_index = 0; entry_index < entry_count; entry_index++) {
                const uint16_t st = ddup_packet_get_entry_station(pkt, entry_index), sq = ddup_packet_get_entry_sequence(pkt, entry_index);
                iotdata_mesh_dedup_insert(state->dedup_ring, st, sq); /* decided-set: suppress my later receive */
                if (sender < state->gateway_id) {                     /* election: lower-id peer claims it -> drop my hold, and my direct ACK */
                    ddup_hold_mark_dropped(state, st, sq);
                    ddup_claim_note(state, st, sq);
                }
                state->stat_injected++;
            }
        }
        pthread_mutex_unlock(&state->mutex);

        if (warn)
            PRINTF_WARN("ddup: WARNING peer %s announced OUR gateway-id 0x%04" PRIX16 " — shared id, ddup can't elect; marked broken (still sending, dropping its claims). Fix the gateway-id config.\n", peerdesc, sender);
        if (recovered)
            PRINTF_INFO("ddup: peer %s recovered (distinct gateway-id 0x%04" PRIX16 ") — resuming election\n", peerdesc, sender);
        if (drop)
            return;
        state->stat_recv_cycles++;
        state->stat_recv_entries += (uint32_t)entry_count;
        if (state->debug)
            PRINTF_INFO("ddup: rx from gateway=0x%04" PRIX16 ", entries=%d\n", sender, entry_count);
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

int ddup_send_setup(__attribute__((unused)) ddup_state_t *state) {
    const int send_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (send_fd < 0) {
        PRINTF_WARN("ddup: send socket: %s\n", strerror(errno));
        return -1;
    }
    return send_fd;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

int ddup_send_collect(ddup_state_t *state, iotdata_mesh_dedup_entry_t *send_entries) {
    int send_count = 0;
    const uint32_t now_ms = hw_time_ms();
    pthread_mutex_lock(&state->mutex);
    if (state->pending_count > 0 && (now_ms - state->pending_first_ms) >= state->delay_ms) {
        send_count = state->pending_count;
        memcpy(send_entries, state->pending, (size_t)send_count * sizeof(iotdata_mesh_dedup_entry_t));
        state->pending_count = 0;
    }
    pthread_mutex_unlock(&state->mutex);
    return send_count;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void ddup_send_to_peers(ddup_state_t *state, int send_fd, iotdata_mesh_dedup_entry_t *send_entries, int send_count) {
    debug_inject_random_latency_ms(state->debug_inject_latency_ms); // TEST: simulate peer-sync network latency (0=off)
    int send_offset = 0;
    while (send_offset < send_count) {
        const int entry_count = DDUP_MIN(send_count - send_offset, DDUP_BATCH_MAX);
        ddup_packet_t pkt;
        ddup_packet_set_gateway_id(pkt, state->gateway_id);
        ddup_packet_set_entry_count(pkt, entry_count);
        for (int entry_index = 0; entry_index < entry_count; entry_index++) {
            ddup_packet_set_entry_station(pkt, entry_index, send_entries[send_offset + entry_index].station);
            ddup_packet_set_entry_sequence(pkt, entry_index, send_entries[send_offset + entry_index].sequence);
        }
        const size_t pkt_len = ddup_packet_get_length(pkt);
        for (int i = 0; i < (int)(sizeof(state->peers) / sizeof(state->peers[0])); i++) {
            ddup_peer_t *p = &state->peers[i];
            if (p->used && p->resolved) {
                if (sendto(send_fd, pkt, pkt_len, 0, (const struct sockaddr *)&p->addr, (socklen_t)sizeof(p->addr)) < 0) {
                    state->stat_send_errors++;
                    if (++p->send_fail_streak >= DDUP_PEER_FAIL_STREAK_MAX) {
                        pthread_mutex_lock(&state->mutex);
                        p->resolved = false;
                        p->send_fail_streak = 0;
                        if (state->stat_peers_resolved > 0)
                            state->stat_peers_resolved--;
                        state->stat_peers_unresolved++;
                        pthread_mutex_unlock(&state->mutex);
                        PRINTF_WARN("ddup: peer %s:%" PRIu16 " unreachable (%d send failures) -> unresolved, will retry\n", p->host, p->port, DDUP_PEER_FAIL_STREAK_MAX);
                    }
                } else
                    p->send_fail_streak = 0;
            }
        }
        state->stat_send_cycles++;
        state->stat_send_entries += (uint32_t)entry_count;
        send_offset += entry_count;
    }
    if (state->debug)
        PRINTF_INFO("ddup: tx %d entries to %d peers\n", send_count, state->peers_count);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void ddup_peers_reresolve(ddup_state_t *state) {
    pthread_mutex_lock(&state->mutex);
    ddup_peer_t snap[DDUP_PEERS_MAX];
    memcpy(snap, state->peers, sizeof(snap));
    pthread_mutex_unlock(&state->mutex);
    int resolved_any = 0;
    for (int i = 0; i < (int)(sizeof(snap) / sizeof(snap[0])); i++) {
        ddup_peer_t *p = &snap[i];
        if (p->used && !p->resolved && ddup_peer_resolve_one(p)) /* getaddrinfo OUTSIDE the lock */
            resolved_any++;
    }
    if (resolved_any > 0) {
        pthread_mutex_lock(&state->mutex);
        for (int i = 0; i < (int)(sizeof(snap) / sizeof(snap[0])); i++) {
            ddup_peer_t *p = &snap[i];
            if (p->used && p->resolved) {
                for (int j = 0; j < (int)(sizeof(state->peers) / sizeof(state->peers[0])); j++) {
                    ddup_peer_t *p2 = &state->peers[j];
                    if (p2->used && !p2->resolved && p2->port == p->port && strcmp(p2->host, p->host) == 0) {
                        p2->addr = p2->addr;
                        p2->resolved = true;
                        p2->send_fail_streak = 0;
                        if (state->stat_peers_unresolved > 0)
                            state->stat_peers_unresolved--;
                        state->stat_peers_resolved++;
                        PRINTF_INFO("ddup: peer %s:%" PRIu16 " re-resolved\n", p2->host, p2->port);
                        break;
                    }
                }
            }
        }
        pthread_mutex_unlock(&state->mutex);
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void *ddup_thread(void *arg) {
    ddup_state_t *state = (ddup_state_t *)arg;
    int recv_fd, send_fd;
    if ((recv_fd = ddup_recv_setup(state)) < 0)
        return NULL;
    if ((send_fd = ddup_send_setup(state)) < 0) {
        close(recv_fd);
        return NULL;
    }
    iotdata_mesh_dedup_entry_t send_entries[DDUP_PENDING_MAX];
    uint32_t last_resolve_ms = hw_time_ms();
    while (*state->running) {
        ddup_recv_from_peers(state, recv_fd);
        // Always drain `pending` (it batches new ring entries), even with no peers — otherwise a
        // mesh gateway that forces ddup on but has no concentration peers would fill `pending` and
        // overflow forever. Only transmit when peers exist; ddup-peers is live, so they can appear later.
        const int send_count = ddup_send_collect(state, send_entries);
        if (send_count > 0 && state->peers_count > 0)
            ddup_send_to_peers(state, send_fd, send_entries, send_count);
        // periodically retry unresolved peers (boot DNS not ready, or a dropped peer that may be back)
        const uint32_t now_ms = hw_time_ms();
        if ((now_ms - last_resolve_ms) >= DDUP_PEER_RESOLVE_RETRY_SECS * 1000u) {
            last_resolve_ms = now_ms;
            ddup_peers_reresolve(state);
        }
    }
    close(send_fd);
    close(recv_fd);
    return NULL;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

bool ddup_check_and_add(ddup_state_t *state, uint16_t station, uint16_t sequence) {
    if (!state->enabled)
        return iotdata_mesh_dedup_insert(state->dedup_ring, station, sequence);
    else {
        pthread_mutex_lock(&state->mutex);
        const bool is_new = iotdata_mesh_dedup_insert(state->dedup_ring, station, sequence);
        if (is_new) {
            if (state->pending_count < DDUP_PENDING_MAX) {
                state->pending[state->pending_count].station = station;
                state->pending[state->pending_count].sequence = sequence;
                if (state->pending_count++ == 0)
                    state->pending_first_ms = hw_time_ms();
            } else
                state->stat_pending_overflow++;
        }
        pthread_mutex_unlock(&state->mutex);
        return is_new;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// HOLD-AND-ELECT — the single admit decision + the hold queue
// -----------------------------------------------------------------------------------------------------------------------------------------

// The one decision point for a received sensor packet (direct or unwrapped-from-mesh). ddup_check_and_add
// does the ring (decided-set) check AND queues our claim to peers; we just choose the verdict around it:
//   not new            -> REJECT  (retransmit / mesh-multipath / late forward — handled regardless of peers)
//   new, ddup off       -> PUBLISH (no dedup)
//   new, no peers        -> PUBLISH (nobody to contest; ring already covers time-separated dups)
//   new, peers present   -> HOLD    (run the election; released later)
// usable = resolved AND not broken (shared-id). Read locklessly (like the send path) — a stale read
// just affects one reading's hold decision and self-corrects. No usable peer -> nothing to elect
// against -> publish immediately (don't pay the hold for a futile election).
static int ddup_usable_peers(const ddup_state_t *state) {
    int n = 0;
    for (int i = 0; i < (int)(sizeof(state->peers) / sizeof(state->peers[0])); i++) {
        const ddup_peer_t *p = &state->peers[i];
        if (p->used && p->resolved && !p->broken)
            n++;
    }
    return n;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

// Are we CONFIDENTLY the lowest gateway-id among usable peers? Then we always win the election, so we can
// publish IMMEDIATELY (no hold) — we still broadcast our claim so peers drop. Requires KNOWING each usable
// peer's gateway-id: auto-discovered peers carry it (announced_gwid via MQTT presence); a static peer's id
// isn't known until it sends a claim, so ANY usable static peer -> not confident -> hold (the safe path).
static bool ddup_is_lowest(const ddup_state_t *state) {
    bool any = false;
    for (int i = 0; i < (int)(sizeof(state->peers) / sizeof(state->peers[0])); i++) {
        const ddup_peer_t *p = &state->peers[i];
        if (p->used && p->resolved && !p->broken) {
            if (!p->is_auto || p->announced_gwid <= state->gateway_id)
                return false; /* static (id unknown) or a peer at/below our id -> not confidently lowest */
            any = true;
        }
    }
    return any; /* true only when there's >=1 usable peer we're strictly below */
}

// -----------------------------------------------------------------------------------------------------------------------------------------

ddup_verdict_t ddup_admit(ddup_state_t *state, uint16_t station, uint16_t sequence) {
    /* THE RING IS LOCAL DEDUP AND ALWAYS APPLIES. A sensor heard both directly and through a relay
       is one reading, whether or not this gateway is contesting with another one -- what `enabled`
       governs is the DISTRIBUTED half: the claim, the hold, the election. Returning early above
       this skipped the ring entirely, so a direct reception never registered and the forwarded copy
       that followed it looked new: every sensor within earshot of both published twice. */
    if (!ddup_check_and_add(state, station, sequence)) { /* rings + broadcasts our claim */
        state->stat_duplicates++;                        /* already decided: retransmit / mesh-multipath / late forward */
        return DDUP_REJECT;
    }
    if (!state->enabled)
        return DDUP_PUBLISH;
    if (state->hold_ms == 0 || ddup_usable_peers(state) == 0)
        return DDUP_PUBLISH; /* nobody to contest -> publish now */
    if (ddup_is_lowest(state))
        return DDUP_PUBLISH; /* we always win -> publish now (claim still sent); no hold */
    return DDUP_HOLD;        /* not (confidently) lowest -> hold for the election */
}

// -----------------------------------------------------------------------------------------------------------------------------------------

// Record one measured peer-claim latency (now - received) into the dynamic-hold tracker. CALLER HOLDS the lock.
static void ddup_claim_latency_sample(ddup_state_t *state, uint32_t now_ms, uint32_t received_ms) {
    const uint32_t d = now_ms - received_ms; /* unsigned: wrap-correct, and a monotonic tick never runs backwards */
    if (d > state->claim_latency_max_ms)
        state->claim_latency_max_ms = d;
    state->claim_latency_sum_ms += d;
    state->claim_latency_count++;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

// The hold we ACTUALLY use: tighten BELOW the ceiling toward the measured claim latency (max*1.5 + margin),
// but only once we have enough samples; bleed the measured-max down over time so it re-tightens if the
// deployment gets faster. Always in [floor, ceiling]. CALLER HOLDS the lock. Also stores hold_ms_dynamic.
static uint32_t ddup_hold_ms_effective(ddup_state_t *state, uint32_t now_ms) {
    if (state->claim_latency_decay_last_ms == 0)
        state->claim_latency_decay_last_ms = now_ms;
    uint32_t periods = (now_ms - state->claim_latency_decay_last_ms) / (DDUP_LATENCY_DECAY_SECS * 1000u);
    if (periods > 0) {
        while (periods-- > 0 && state->claim_latency_max_ms > 0)
            state->claim_latency_max_ms -= state->claim_latency_max_ms / 8; /* -12.5% per period */
        state->claim_latency_decay_last_ms = now_ms;
    }
    uint32_t dyn;
    if (state->claim_latency_count < DDUP_LATENCY_MIN_SAMPLES || state->claim_latency_max_ms == 0)
        dyn = state->hold_ms; /* not enough data yet -> stay conservative at the ceiling */
    else {
        dyn = state->claim_latency_max_ms + state->claim_latency_max_ms / 2 + 50; /* max*1.5 + 50ms margin */
        if (dyn < DDUP_HOLD_FLOOR_MS)
            dyn = DDUP_HOLD_FLOOR_MS;
        if (dyn > state->hold_ms)
            dyn = state->hold_ms; /* never exceed the configured ceiling */
    }
    state->hold_ms_dynamic = dyn;
    return dyn;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

// Copy a reading into the hold queue with deadline = now + hold_ms. Called from the rx/exec thread.
// If the queue is full, returns false so the caller publishes it un-held (degrade, never lose data).
bool ddup_hold_add(ddup_state_t *state, const uint8_t *packet, int packet_len, uint8_t variant, uint16_t station, uint16_t sequence, const char *topic_prefix, const char *via, int rssi_dbm) {
    if (packet_len > DDUP_HOLD_PKT_MAX)
        packet_len = DDUP_HOLD_PKT_MAX;
    const uint32_t now_ms = hw_time_ms();
    pthread_mutex_lock(&state->mutex);
    int slot = -1;
    for (int i = 0; i < (int)(sizeof(state->hold) / sizeof(state->hold[0])); i++) {
        ddup_hold_entry_t *h = &state->hold[i];
        if (!h->used) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        state->stat_hold_overflow++;
        pthread_mutex_unlock(&state->mutex);
        return false;
    }
    const uint32_t eff = ddup_hold_ms_effective(state, now_ms); /* dynamic hold (<= ceiling), under lock */
    ddup_hold_entry_t *h = &state->hold[slot];
    *h = (ddup_hold_entry_t){
        .used = true,
        .station = station,
        .sequence = sequence,
        .variant = variant,
        .topic_prefix = topic_prefix,
        .via = via,
        .rssi_dbm = rssi_dbm,
        .received_ms = now_ms,
        .deadline_ms = now_ms + eff, /* a tick deadline is one add; the compare below is wrap-safe */
    };
    h->packet_len = packet_len;
    memcpy(h->packet, packet, (size_t)packet_len);
    state->stat_held++;
    pthread_mutex_unlock(&state->mutex);
    if (state->debug)
        PRINTF_INFO("ddup: HOLD station=0x%04" PRIX16 " seq=%" PRIu16 " (+%" PRIu32 "ms dynamic)\n", station, sequence, eff);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

// A lower gateway-id peer claimed this reading — we LOST. Measure the claim latency (feeds the dynamic
// hold) and EARLY-DROP: free the slot immediately rather than letting it sit until expiry. CALLER MUST
// HOLD state->mutex (invoked from the recv loop, already under the lock).
void ddup_hold_mark_dropped(ddup_state_t *state, uint16_t station, uint16_t sequence) {
    const uint32_t now_ms = hw_time_ms();
    for (int i = 0, found = 0; i < (int)(sizeof(state->hold) / sizeof(state->hold[0])) && !found; i++) {
        ddup_hold_entry_t *h = &state->hold[i];
        if (h->used && h->station == station && h->sequence == sequence) {
            ddup_claim_latency_sample(state, now_ms, h->received_ms); /* learn how long the claim took */
            h->used = false;                                          /* early-drop: we lost, free the slot now */
            state->stat_dropped_peer++;
            if (state->debug)
                PRINTF_INFO("ddup: DROP station=0x%04" PRIX16 " seq=%" PRIu16 " (lower-id peer claimed; freed)\n", station, sequence);
            found = 1;
        }
    }
}

/* Lock held (recv path), as for ddup_hold_mark_dropped. */
void ddup_claim_note(ddup_state_t *state, uint16_t station, uint16_t sequence) {
    state->claims[state->claims_next] = (iotdata_mesh_dedup_entry_t){ .station = station, .sequence = sequence };
    state->claims_next = (state->claims_next + 1) % DDUP_CLAIMS_MAX;
    if (state->claims_count < DDUP_CLAIMS_MAX)
        state->claims_count++;
}

/* From the main loop, so it takes the lock; with ddup off there is no lock, and no peer to claim. */
bool ddup_claimed(ddup_state_t *state, uint16_t station, uint16_t sequence) {
    if (!state->enabled)
        return false;
    bool claimed = false;
    pthread_mutex_lock(&state->mutex);
    for (int i = 0; i < state->claims_count && !claimed; i++)
        claimed = state->claims[i].station == station && state->claims[i].sequence == sequence;
    pthread_mutex_unlock(&state->mutex);
    return claimed;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

// Release any held reading whose hold has expired — losers were already early-dropped, so every entry
// still here is a WINNER: publish it. Snapshots one entry at a time UNDER the lock, fires the callback
// OUTSIDE the lock (so mqtt_send never runs holding the mutex). Called each iteration of the rx/exec loop.
void ddup_hold_release(ddup_state_t *state, ddup_release_fn cb, void *ctx) {
    /* READS THE CLOCK ITSELF rather than taking the exec pass's now_ms, unlike every other tick.
       The pass is stamped BEFORE a lora_read() that blocks for up to LORA_READ_TIMEOUT_MS, so by
       the time the loop reaches here that stamp can be seconds old -- which against a 50-1000ms
       hold would defer every winner by a whole further iteration. Second-scale cadences don't
       care; a sub-second deadline does. */
    const uint32_t now_ms = hw_time_ms();
    for (;;) {
        ddup_hold_entry_t e;
        bool found = false;
        pthread_mutex_lock(&state->mutex);
        for (int i = 0; i < (int)(sizeof(state->hold) / sizeof(state->hold[0])) && !found; i++) {
            ddup_hold_entry_t *h = &state->hold[i];
            if (h->used && (int32_t)(now_ms - h->deadline_ms) >= 0) {
                e = *h; /* copy out before freeing the slot */
                h->used = false;
                state->stat_released++;
                found = true;
            }
        }
        pthread_mutex_unlock(&state->mutex);
        if (!found)
            break;
        if (state->debug)
            PRINTF_INFO("ddup: RELEASE (won) station=0x%04" PRIX16 " seq=%" PRIu16 "\n", e.station, e.sequence);
        cb(ctx, e.packet, e.packet_len, e.variant, e.station, e.sequence, e.topic_prefix, e.via, e.rssi_dbm);
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

bool ddup_begin(ddup_state_t *state, uint16_t gateway_id, iotdata_mesh_dedup_ring_t *dedup_ring, volatile bool *running) {
    /* bind the caller's shared ring even when ddup is disabled — mesh-relay and direct-packet
     * dedup read it through ddup_check_and_add (ring-only when disabled, +peer-sync when on). */
    state->dedup_ring = dedup_ring;
    if (!state->enabled) {
        PRINTF_INFO("ddup: disabled, not starting\n");
        return true;
    }
    state->running = running;
    state->gateway_id = gateway_id;
    PRINTF_INFO("ddup: enabled, port=%" PRIu16 ", peers=%d, gateway_id=%04" PRIX16 ", delay=%" PRIu32 "ms, debug=%c\n", state->port, state->peers_count, state->gateway_id, state->delay_ms, state->debug ? 'y' : 'n');
    ddup_peers_resolve(state);
    pthread_mutex_init(&state->mutex, NULL);
    if (pthread_create(&state->thread, NULL, ddup_thread, state) != 0) {
        state->enabled = false;
        PRINTF_ERROR("ddup: thread create failed: %s\n", strerror(errno));
        pthread_mutex_destroy(&state->mutex);
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void ddup_end(ddup_state_t *state) {
    if (!state->enabled)
        return;
    pthread_join(state->thread, NULL);
    pthread_mutex_destroy(&state->mutex);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// iotdata_gateway_peer.h - ddup AUTO-PEERING via MQTT presence.
//
// THE PROBLEM IT SOLVES: a static `ddup-peers` list is a list of addresses somebody has to keep
// correct, on every gateway, as a fleet grows or a box changes address. Presence turns that round:
// each gateway says where IT is, and everybody else listens. `ddup-peers = mqtt` is the whole
// configuration.
//
// exec/main owns MQTT here; ddup only exposes the lockable peer add/remove. Presence is published
// retained to iotdata/ddup/<gwid> = {"ip":..,"port":..,"time":<epoch>}; the LWT is an empty retained
// message (offline). The topic is fleet-bridged (iotdata/ddup/#, both directions), so every gateway
// sees every other's address. On receipt we ddup_peer_insert/remove (keyed by gateway-id); our own
// presence (the bridge echo) is ignored. The `ts` lets a peer be aged out when its box/bridge dies and
// the LWT can't reach us (the LWT fires on the dead box's LOCAL broker) — see DDUP_PEER_TTL_SECS. A
// stale retained presence re-delivered on a bridge reconnect carries the OLD ts, so it can't refresh a
// dead peer. Public/private-IP/NAT is a sysadmin concern (VPN/tunnel).
// -----------------------------------------------------------------------------------------------------------------------------------------

#include <ifaddrs.h>
#include <net/if.h>
#include <cjson/cJSON.h>

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#ifndef DDUP_PEERS_REPUBLISH_SECS
#define DDUP_PEERS_REPUBLISH_SECS 30 // re-publish at least this often (recover from a broker restart)
#endif

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

typedef struct {
    ddup_state_t *ddup;
    uint16_t gwid;
    char topic[64];   // iotdata/ddup/<gwid>
    char last_ip[64]; // last advertised IP (re-publish when it changes)
    time_t last_pub;
} ddup_peers_t;

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static char *ddup_peers_topic(uint16_t gwid, char *out, size_t outsz) {
    return snprintf_inline(out, outsz, "iotdata/ddup/%03" PRIX16, gwid);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

// Resolve our advertise IPv4 from the qualifier: a literal IP -> as-is; an interface name -> its
// IPv4; "" -> the first non-loopback IPv4. Returns true + writes ip.
static bool ddup_peer_resolve_advertise_ip(const char *qualifier, char *ip, size_t ipsz) {
    bool found = false;
    struct in_addr lit;
    if (qualifier[0] && inet_pton(AF_INET, qualifier, &lit) == 1) {
        snprintf(ip, ipsz, "%s", qualifier);
        found = true;
    } else {
        struct ifaddrs *ifap = NULL;
        if (getifaddrs(&ifap) == 0) {
            for (struct ifaddrs *p = ifap; p && !found; p = p->ifa_next) {
                if (!p->ifa_addr || p->ifa_addr->sa_family != AF_INET || (p->ifa_flags & IFF_LOOPBACK))
                    continue;
                if (qualifier[0] && strcmp(p->ifa_name, qualifier) != 0)
                    continue;
                char buf[INET_ADDRSTRLEN];
                if (inet_ntop(AF_INET, &((const struct sockaddr_in *)(const void *)p->ifa_addr)->sin_addr, buf, sizeof(buf))) {
                    snprintf(ip, ipsz, "%s", buf);
                    found = true;
                }
            }
            freeifaddrs(ifap);
        }
    }
    return found;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void ddup_peers_on_message(void *ctx, const char *topic, const void *payload, int len) {
    ddup_peers_t *pp = (ddup_peers_t *)ctx;
    const char *slash = strrchr(topic, '/');
    if (!slash || !slash[1])
        return;
    const uint16_t gwid = (uint16_t)strtol(slash + 1, NULL, 16);
    if (gwid == pp->gwid)
        return;     // our own presence (bridge echo)
    if (len <= 0) { // empty retained = offline / LWT
        ddup_peer_remove(pp->ddup, gwid);
        return;
    }
    cJSON *j = cJSON_ParseWithLength(payload, (size_t)len);
    if (j) {
        const cJSON *jaddr = cJSON_GetObjectItem(j, "ip");
        const cJSON *jport = cJSON_GetObjectItem(j, "port");
        const cJSON *jtime = cJSON_GetObjectItem(j, "time");
        if (cJSON_IsString(jaddr) && jaddr->valuestring[0]) {
            const uint16_t port = cJSON_IsNumber(jport) ? (uint16_t)jport->valueint : pp->ddup->port;
            const time_t ts = cJSON_IsNumber(jtime) ? (time_t)jtime->valuedouble : 0; // 0 = un-stamped (legacy peer)
            // A STALE retained presence (the peer's box/bridge died, the LWT fired only on its local broker)
            // carries an old ts — treat it as offline instead of (re)inserting a ghost; otherwise insert/refresh.
            if (ts != 0 && (long long)(hw_clock() - ts) > DDUP_PEER_TTL_SECS)
                ddup_peer_remove(pp->ddup, gwid);
            else
                ddup_peer_insert(pp->ddup, gwid, jaddr->valuestring, port, ts);
        }
        cJSON_Delete(j);
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void ddup_peers_tick(ddup_peers_t *pp, time_t now) {
    if (!pp || !pp->ddup || !pp->ddup->auto_peer || !pp->ddup->enabled) // uninitialised (static peers) / ddup-enable master switch
        return;
    ddup_peers_expire(pp->ddup, now); // backstop: prune peers whose presence ts went stale (box/bridge death the LWT can't cover)
    char ip[64];
    if (ddup_peer_resolve_advertise_ip(pp->ddup->advertise_qualifier, ip, sizeof(ip))) {
        const bool changed = strcmp(ip, pp->last_ip) != 0;
        if (changed || pp->last_pub == 0 || (now - pp->last_pub) >= DDUP_PEERS_REPUBLISH_SECS) {
            char payload[128];
            const int n = snprintf(payload, sizeof(payload), "{\"ip\":\"%s\",\"port\":%u,\"time\":%lld}", ip, (unsigned)pp->ddup->port, (long long)now);
            if (mqtt_send_retained(pp->topic, payload, n)) {
                if (changed)
                    PRINTF_INFO("ddup-peer: presence %s -> %s\n", pp->topic, payload);
                snprintf(pp->last_ip, sizeof(pp->last_ip), "%s", ip);
                pp->last_pub = now;
            }
        }
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

// Set ONLY the last-will. MUST be called BEFORE mqtt_begin() (the will is registered at
// connect). The subscription is deferred to ddup_peers_begin() because the handler calls into
// ddup, whose mutex isn't initialised until ddup_begin() (after the config connect/settle).
__attribute__((unused)) static void ddup_peers_init(ddup_peers_t *pp, ddup_state_t *ddup, uint16_t gwid) {
    *pp = (ddup_peers_t){ .ddup = ddup, .gwid = gwid };
    mqtt_set_will(ddup_peers_topic(gwid, pp->topic, sizeof(pp->topic))); // LWT: empty retained = offline. BEFORE mqtt_begin(), or it is never registered.
    PRINTF_INFO("ddup-peer: auto-discovery via MQTT presence on %s (advertise=%s)\n", pp->topic, ddup->advertise_qualifier[0] ? ddup->advertise_qualifier : "auto");
}

// -----------------------------------------------------------------------------------------------------------------------------------------

__attribute__((unused)) static void ddup_peers_begin(ddup_peers_t *pp) {
    (void)mqtt_subscribe("iotdata/ddup/#", MQTT_PUBLISH_QOS, ddup_peers_on_message, pp);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

__attribute__((unused)) static void ddup_peers_end(ddup_peers_t *pp) {
    if (pp->ddup && pp->ddup->auto_peer)
        (void)mqtt_send_retained(pp->topic, NULL, 0);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
