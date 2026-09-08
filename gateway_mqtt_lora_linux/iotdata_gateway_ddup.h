
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#ifndef DDUP_PORT_DEFAULT
#define DDUP_PORT_DEFAULT 9876
#endif
#ifndef DDUP_DELAY_MS_DEFAULT
#define DDUP_DELAY_MS_DEFAULT 20
#endif
#ifndef DDUP_PEERS_MAX
#define DDUP_PEERS_MAX 16
#endif
#ifndef DDUP_PENDING_MAX
#define DDUP_PENDING_MAX 256
#endif

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define DDUP_PKT_IDENTIFIER_SIZE 4 // station_id - uint16_t
#define DDUP_PKT_BATCH_SIZE      32
#define DDUP_PKT_HEADER_SIZE     3
#define DDUP_PKT_SIZE            (DDUP_PKT_HEADER_SIZE + (DDUP_PKT_BATCH_SIZE * DDUP_PKT_IDENTIFIER_SIZE)) /* 131 bytes */

#define DDUP_MIN(a, b)           ((a) < (b) ? (a) : (b))

typedef uint8_t ddup_packet_t[DDUP_PKT_SIZE];

#define ddup_packet_get_length(pkt)                      (size_t)(3 + (size_t)pkt[2] * 4)

#define ddup_packet_get_gateway_id(pkt)                  (((uint16_t)pkt[0] << 8) | (uint16_t)pkt[1])
#define ddup_packet_get_entry_count(pkt)                 (DDUP_MIN(pkt[2], DDUP_PKT_BATCH_SIZE))
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
#define ddup_packet_set_entry_station(pkt, entry_index, station_id) \
    do { \
        pkt[(3 + entry_index * 4) + 0] = (uint8_t)((station_id) >> 8); \
        pkt[(3 + entry_index * 4) + 1] = (uint8_t)((station_id) & 0xFF); \
    } while (0)
#define ddup_packet_set_entry_sequence(pkt, entry_index, sequence) \
    do { \
        pkt[(3 + entry_index * 4) + 2] = (uint8_t)((sequence) >> 8); \
        pkt[(3 + entry_index * 4) + 3] = (uint8_t)((sequence) & 0xFF); \
    } while (0)

// -----------------------------------------------------------------------------------------------------------------------------------------

typedef struct {
    char host[128];
    uint16_t port;
    struct sockaddr_in addr;
    bool resolved;
} ddup_peer_t;

typedef struct {
    bool enabled;
    uint16_t port;
    uint32_t delay_ms;
    ddup_peer_t peers[DDUP_PEERS_MAX];
    int peers_count;
    pthread_mutex_t mutex;
    pthread_t thread;
    iotdata_mesh_dedup_entry_t pending[DDUP_PENDING_MAX];
    int pending_count;
    struct timespec pending_first;
    uint16_t gateway_id;
    iotdata_mesh_dedup_ring_t *ddup_ring;
    volatile bool *running;
    bool debug;
    char _buffer_config[1024];
    ddup_packet_t _buffer_packet;
    /* statistics */
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
} ddup_state_t;

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

void ddup_peers_parse(ddup_state_t *st, const char *peers_str) {
    if (!peers_str || !*peers_str)
        return;
    strncpy(st->_buffer_config, peers_str, sizeof(st->_buffer_config) - 1);
    st->_buffer_config[sizeof(st->_buffer_config) - 1] = '\0';
    char *save = NULL, *tok = strtok_r(st->_buffer_config, ",", &save);
    while (tok && st->peers_count < (int)(sizeof(st->peers) / sizeof(st->peers[0]))) {
        ddup_peer_t *const peer = &st->peers[st->peers_count];
        while (*tok == ' ')
            tok++;
        char *colon = strrchr(tok, ':');
        uint16_t pport = st->port;
        if (colon) {
            *colon = '\0';
            pport = (uint16_t)atoi(colon + 1);
        }
        strncpy(peer->host, tok, sizeof(peer->host) - 1);
        peer->host[sizeof(peer->host) - 1] = '\0';
        peer->port = pport;
        st->peers_count++;
        tok = strtok_r(NULL, ",", &save);
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void ddup_peers_resolve(ddup_state_t *st) {
    st->stat_peers_resolved = 0;
    st->stat_peers_unresolved = 0;
    for (int i = 0; i < st->peers_count; i++) {
        ddup_peer_t *const peer = &st->peers[i];
        char port_str[8];
        struct addrinfo *res;
        const int err = getaddrinfo(peer->host, snprintf_inline(port_str, sizeof(port_str), "%" PRIu16, peer->port), &(struct addrinfo){ .ai_family = AF_INET, .ai_socktype = SOCK_DGRAM }, &res);
        if (err == 0) {
            memcpy(&peer->addr, res->ai_addr, sizeof(peer->addr));
            peer->resolved = true;
            st->stat_peers_resolved++;
            freeaddrinfo(res);
            PRINTF_INFO("ddup: peer[%d] %s:%" PRIu16 " resolved\n", i, peer->host, peer->port);
        } else {
            peer->resolved = false;
            st->stat_peers_unresolved++;
            PRINTF_ERROR("ddup: peer[%d] %s:%" PRIu16 " resolution failed: %s\n", i, peer->host, peer->port, gai_strerror(err));
        }
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

int ddup_recv_setup(ddup_state_t *st) {
    const int recv_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (recv_fd < 0) {
        PRINTF_ERROR("ddup: recv socket: %s\n", strerror(errno));
        return -1;
    }
    int optval = 1;
    setsockopt(recv_fd, SOL_SOCKET, SO_REUSEADDR, &optval, (socklen_t)sizeof(optval));
    if (bind(recv_fd, (const struct sockaddr *)&(const struct sockaddr_in){ .sin_family = AF_INET, .sin_port = htons(st->port), .sin_addr.s_addr = htonl(INADDR_ANY) }, (socklen_t)sizeof(const struct sockaddr_in)) < 0) {
        PRINTF_ERROR("ddup: bind port %" PRIu16 ": %s\n", st->port, strerror(errno));
        close(recv_fd);
        return -1;
    }
    return recv_fd;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void ddup_peers_recv(ddup_state_t *st, int recv_fd) {
    struct pollfd pfd = { .fd = recv_fd, .events = POLLIN, .revents = 0 };
    if (poll(&pfd, 1, 5) > 0 && (pfd.revents & POLLIN)) {
        struct sockaddr_in from;
        socklen_t from_len = (socklen_t)sizeof(from);
        const ssize_t recv_len = recvfrom(recv_fd, st->_buffer_packet, sizeof(st->_buffer_packet), 0, (struct sockaddr *)&from, &from_len);
        if (recv_len < DDUP_PKT_HEADER_SIZE)
            st->stat_recv_errors++;
        else {
            const int entry_count = ddup_packet_get_entry_count(st->_buffer_packet);
            if (recv_len < (ssize_t)ddup_packet_get_length(st->_buffer_packet))
                st->stat_recv_errors++;
            else {
                pthread_mutex_lock(&st->mutex);
                for (int entry_index = 0; entry_index < entry_count; entry_index++) {
                    iotdata_mesh_dedup_insert(st->ddup_ring, ddup_packet_get_entry_station(st->_buffer_packet, entry_index), ddup_packet_get_entry_sequence(st->_buffer_packet, entry_index));
                    st->stat_injected++;
                }
                pthread_mutex_unlock(&st->mutex);
                st->stat_recv_cycles++;
                st->stat_recv_entries += (uint32_t)entry_count;
                if (st->debug)
                    PRINTF_INFO("ddup: rx from gateway=%04" PRIX16 ", entries=%d\n", ddup_packet_get_gateway_id(st->_buffer_packet), entry_count);
            }
        }
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

int ddup_send_setup(__attribute__((unused)) ddup_state_t *st) {
    const int send_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (send_fd < 0) {
        PRINTF_ERROR("ddup: send socket: %s\n", strerror(errno));
        return -1;
    }
    return send_fd;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

int ddup_send_collect(ddup_state_t *st, iotdata_mesh_dedup_entry_t *send_entries) {
    int send_count = 0;
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    pthread_mutex_lock(&st->mutex);
    if (st->pending_count > 0) {
        const int32_t elapsed_ms = (int32_t)((now.tv_sec - st->pending_first.tv_sec) * 1000L) + (int32_t)((now.tv_nsec - st->pending_first.tv_nsec) / 1000000L);
        if (elapsed_ms > 0 && (uint32_t)elapsed_ms >= st->delay_ms) {
            send_count = st->pending_count;
            memcpy(send_entries, st->pending, (size_t)send_count * sizeof(iotdata_mesh_dedup_entry_t));
            st->pending_count = 0;
        }
    }
    pthread_mutex_unlock(&st->mutex);
    return send_count;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void ddup_peers_send(ddup_state_t *st, int send_fd, iotdata_mesh_dedup_entry_t *send_entries, int send_count) {
    int send_offset = 0;
    while (send_offset < send_count) {
        const int entry_count = DDUP_MIN(send_count - send_offset, DDUP_PKT_BATCH_SIZE);
        ddup_packet_set_gateway_id(st->_buffer_packet, st->gateway_id);
        ddup_packet_set_entry_count(st->_buffer_packet, entry_count);
        for (int entry_index = 0; entry_index < entry_count; entry_index++) {
            ddup_packet_set_entry_station(st->_buffer_packet, entry_index, send_entries[send_offset + entry_index].station_id);
            ddup_packet_set_entry_sequence(st->_buffer_packet, entry_index, send_entries[send_offset + entry_index].sequence);
        }
        const size_t pkt_len = ddup_packet_get_length(st->_buffer_packet);
        for (int peer = 0; peer < st->peers_count; peer++) {
            const ddup_peer_t *const e = &st->peers[peer];
            if (e->resolved)
                if (sendto(send_fd, st->_buffer_packet, pkt_len, 0, (const struct sockaddr *)&e->addr, (socklen_t)sizeof(e->addr)) < 0)
                    st->stat_send_errors++;
        }
        st->stat_send_cycles++;
        st->stat_send_entries += (uint32_t)entry_count;
        send_offset += entry_count;
    }
    if (st->debug)
        PRINTF_INFO("ddup: tx %d entries to %d peers\n", send_count, st->peers_count);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void *ddup_thread_func(void *arg) {
    ddup_state_t *st = (ddup_state_t *)arg;
    int recv_fd, send_fd;
    if ((recv_fd = ddup_recv_setup(st)) < 0)
        goto ddup_end_all;
    if ((send_fd = ddup_send_setup(st)) < 0)
        goto ddup_end_send;
    iotdata_mesh_dedup_entry_t send_entries[DDUP_PENDING_MAX]; // multiple
    while (st->running && *st->running) {
        ddup_peers_recv(st, recv_fd);
        if (st->peers_count > 0) {
            const int send_count = ddup_send_collect(st, send_entries);
            if (send_count > 0)
                ddup_peers_send(st, send_fd, send_entries, send_count);
        }
    }
    close(send_fd);
ddup_end_send:
    close(recv_fd);
ddup_end_all:
    return NULL;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

bool ddup_insert(ddup_state_t *st, uint16_t station_id, uint16_t sequence) {
    if (!st->enabled)
        return iotdata_mesh_dedup_insert(st->ddup_ring, station_id, sequence);
    pthread_mutex_lock(&st->mutex);
    const bool is_new = iotdata_mesh_dedup_insert(st->ddup_ring, station_id, sequence);
    if (is_new) {
        if (st->pending_count < (int)(sizeof(st->pending) / sizeof(st->pending[0]))) {
            st->pending[st->pending_count].station_id = station_id;
            st->pending[st->pending_count].sequence = sequence;
            if (st->pending_count++ == 0)
                clock_gettime(CLOCK_MONOTONIC, &st->pending_first);
        } else
            st->stat_pending_overflow++;
    }
    pthread_mutex_unlock(&st->mutex);
    return is_new;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

bool ddup_begin(ddup_state_t *st, uint16_t gateway_id, iotdata_mesh_dedup_ring_t *ddup_ring, volatile bool *running) {
    assert(st && ddup_ring);
    // XXX, mesh code uses the dedup ring as well.
    st->ddup_ring = ddup_ring;
    if (!st->enabled) {
        PRINTF_INFO("ddup: disabled, not starting\n");
        return true;
    }
    st->running = running;
    st->gateway_id = gateway_id;
    PRINTF_INFO("ddup: enabled, port=%" PRIu16 ", peers=%d, gateway_id=%04" PRIX16 ", delay=%" PRIu32 "ms\n", st->port, st->peers_count, st->gateway_id, st->delay_ms);
    ddup_peers_resolve(st);
    pthread_mutex_init(&st->mutex, NULL);
    if (pthread_create(&st->thread, NULL, ddup_thread_func, st) != 0) {
        st->enabled = false;
        PRINTF_ERROR("ddup: thread create failed: %s\n", strerror(errno));
        pthread_mutex_destroy(&st->mutex);
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void ddup_end(ddup_state_t *st) {
    if (!st->enabled)
        return;
    pthread_join(st->thread, NULL);
    pthread_mutex_destroy(&st->mutex);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
