
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include <errno.h>
#include <inttypes.h>
#include <poll.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "iotdata_mesh.h"

#include "iotdata_gateway_mesh.h"
#include "iotdata_gateway_ddup.h"

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static int tests_run = 0, tests_passed = 0;

#define ASSERT(cond) \
    do { \
        if (!(cond)) { \
            printf("FAIL (line %d: %s)\n", __LINE__, #cond); \
            return false; \
        } \
    } while (0)

#define ASSERT_EQ_INT(a, b) ASSERT((a) == (b))

#define RUN_TEST(name) \
    do { \
        printf("  %-55s ", #name); \
        tests_run++; \
        if (test_##name()) { \
            printf("PASS\n"); \
            tests_passed++; \
        } \
    } while (0)

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#define CAPTURED_MAX 16
static uint8_t captured_packets[CAPTURED_MAX][256];
static int captured_lengths[CAPTURED_MAX];
static int packet_handler_calls = 0;

static bool test_packet_handler(const uint8_t *packet, const int length) {
    if (packet_handler_calls < CAPTURED_MAX) {
        memcpy(captured_packets[packet_handler_calls], packet, (size_t)length);
        captured_lengths[packet_handler_calls] = length;
    }
    packet_handler_calls++;
    return true;
}

static bool test_packet_handler_fail(const uint8_t *packet, const int length) {
    (void)packet;
    (void)length;
    packet_handler_calls++;
    return false;
}

static int dedup_handler_calls = 0;
static bool dedup_handler_result = true;

static bool test_dedup_handler(void *ctx, uint16_t station_id, uint16_t sequence) {
    (void)station_id;
    (void)sequence;
    (void)ctx;
    dedup_handler_calls++;
    return dedup_handler_result;
}

static void reset_test_helpers(void) {
    memset(captured_packets, 0, sizeof(captured_packets));
    memset(captured_lengths, 0, sizeof(captured_lengths));
    packet_handler_calls = 0;
    dedup_handler_calls = 0;
    dedup_handler_result = true;
}

// =========================================================================================================================================
// MESH TESTS
// =========================================================================================================================================

// -----------------------------------------------------------------------------------------------------------------------------------------
// mesh_begin
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool test_mesh_begin_disabled(void) {
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.enabled = false;
    ASSERT(mesh_begin(&ms, test_packet_handler, test_dedup_handler, NULL));
    ASSERT(ms.packet_handler == NULL);
    ASSERT(ms.dedup_handler == NULL);
    return true;
}

static bool test_mesh_begin_enabled(void) {
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.enabled = true;
    ms.station_id = 0x0042;
    ms.beacon_interval = 60;
    ASSERT(mesh_begin(&ms, test_packet_handler, test_dedup_handler, NULL));
    ASSERT(ms.packet_handler == test_packet_handler);
    ASSERT(ms.dedup_handler == test_dedup_handler);
    return true;
}

static bool test_mesh_begin_no_packet_handler(void) {
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.enabled = true;
    ASSERT(mesh_begin(&ms, NULL, test_dedup_handler, NULL));
    ASSERT(ms.packet_handler == NULL);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// mesh_beacon_send
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool test_mesh_beacon_send_ok(void) {
    reset_test_helpers();
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.enabled = true;
    ms.station_id = 0x0042;
    ms.packet_handler = test_packet_handler;

    mesh_beacon_send(&ms);

    ASSERT_EQ_INT(packet_handler_calls, 1);
    ASSERT_EQ_INT(captured_lengths[0], IOTDATA_MESH_BEACON_SIZE);
    ASSERT_EQ_INT(ms.stat_beacons_tx, 1u);
    ASSERT_EQ_INT(ms.mesh_seq, 1u);

    iotdata_mesh_beacon_t b;
    ASSERT(iotdata_mesh_unpack_beacon(captured_packets[0], captured_lengths[0], &b));
    ASSERT_EQ_INT(b.sender_station, 0x0042u);
    ASSERT_EQ_INT(b.gateway_id, 0x0042u);
    ASSERT_EQ_INT(b.cost, 0u);
    ASSERT(b.flags & IOTDATA_MESH_FLAG_ACCEPTING);
    return true;
}

static bool test_mesh_beacon_send_fail(void) {
    reset_test_helpers();
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.enabled = true;
    ms.station_id = 0x0042;
    ms.packet_handler = test_packet_handler_fail;

    mesh_beacon_send(&ms);

    ASSERT_EQ_INT(ms.stat_beacons_tx, 0u);
    return true;
}

static bool test_mesh_beacon_generation_wraps(void) {
    reset_test_helpers();
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.enabled = true;
    ms.station_id = 0x0001;
    ms.packet_handler = test_packet_handler;
    ms.beacon_generation = (uint16_t)(IOTDATA_MESH_GENERATION_MOD - 1);

    mesh_beacon_send(&ms);

    ASSERT_EQ_INT(ms.beacon_generation, 0u);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// mesh_ack_send
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool test_mesh_ack_send_ok(void) {
    reset_test_helpers();
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.enabled = true;
    ms.station_id = 0x0001;
    ms.packet_handler = test_packet_handler;

    mesh_ack_send(&ms, 0x0002, 100);

    ASSERT_EQ_INT(packet_handler_calls, 1);
    ASSERT_EQ_INT(captured_lengths[0], IOTDATA_MESH_ACK_SIZE);
    ASSERT_EQ_INT(ms.stat_acks_tx, 1u);

    iotdata_mesh_ack_t ack;
    ASSERT(iotdata_mesh_unpack_ack(captured_packets[0], captured_lengths[0], &ack));
    ASSERT_EQ_INT(ack.sender_station, 0x0001u);
    ASSERT_EQ_INT(ack.fwd_station, 0x0002u);
    ASSERT_EQ_INT(ack.fwd_seq, 100u);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// mesh_handle_forward
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool test_mesh_handle_forward_new(void) {
    reset_test_helpers();
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.enabled = true;
    ms.station_id = 0x0001;
    ms.packet_handler = test_packet_handler;
    ms.dedup_handler = test_dedup_handler;
    dedup_handler_result = true;

    /* build forward: sender=0x005, seq=50, ttl=3, inner = fake 8-byte iotdata packet */
    uint8_t inner[8] = { 0x10, 0x42, 0x00, 0x01, 0xAA, 0xBB, 0xCC, 0xDD };
    uint8_t fwd_buf[IOTDATA_MESH_FORWARD_HDR_SIZE + 8];
    iotdata_mesh_pack_forward(fwd_buf, 0x0005, 50, 3, inner, 8);

    const uint8_t *inner_out;
    int inner_len;
    ASSERT(mesh_handle_forward(&ms, fwd_buf, IOTDATA_MESH_FORWARD_HDR_SIZE + 8, &inner_out, &inner_len));

    ASSERT_EQ_INT(ms.stat_forwards_rx, 1u);
    ASSERT_EQ_INT(ms.stat_forwards_unwrapped, 1u);
    ASSERT_EQ_INT(inner_len, 8);
    ASSERT_EQ_INT(dedup_handler_calls, 1);
    /* ACK should have been sent */
    ASSERT_EQ_INT(packet_handler_calls, 1);
    ASSERT_EQ_INT(ms.stat_acks_tx, 1u);
    return true;
}

static bool test_mesh_handle_forward_duplicate(void) {
    reset_test_helpers();
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.enabled = true;
    ms.station_id = 0x0001;
    ms.packet_handler = test_packet_handler;
    ms.dedup_handler = test_dedup_handler;
    dedup_handler_result = false; /* duplicate */

    uint8_t inner[8] = { 0x10, 0x42, 0x00, 0x01, 0xAA, 0xBB, 0xCC, 0xDD };
    uint8_t fwd_buf[IOTDATA_MESH_FORWARD_HDR_SIZE + 8];
    iotdata_mesh_pack_forward(fwd_buf, 0x0005, 50, 3, inner, 8);

    const uint8_t *inner_out;
    int inner_len;
    ASSERT(!mesh_handle_forward(&ms, fwd_buf, IOTDATA_MESH_FORWARD_HDR_SIZE + 8, &inner_out, &inner_len));

    ASSERT_EQ_INT(ms.stat_forwards_rx, 1u);
    ASSERT_EQ_INT(ms.stat_forwards_unwrapped, 0u);
    ASSERT_EQ_INT(ms.stat_duplicates, 1u);
    /* ACK still sent even for duplicates */
    ASSERT_EQ_INT(packet_handler_calls, 1);
    ASSERT_EQ_INT(ms.stat_acks_tx, 1u);
    return true;
}

static bool test_mesh_handle_forward_no_dedup(void) {
    reset_test_helpers();
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.enabled = true;
    ms.station_id = 0x0001;
    ms.packet_handler = test_packet_handler;
    ms.dedup_handler = NULL;

    uint8_t inner[8] = { 0x10, 0x42, 0x00, 0x01, 0xAA, 0xBB, 0xCC, 0xDD };
    uint8_t fwd_buf[IOTDATA_MESH_FORWARD_HDR_SIZE + 8];
    iotdata_mesh_pack_forward(fwd_buf, 0x0005, 50, 3, inner, 8);

    const uint8_t *inner_out;
    int inner_len;
    ASSERT(mesh_handle_forward(&ms, fwd_buf, IOTDATA_MESH_FORWARD_HDR_SIZE + 8, &inner_out, &inner_len));

    ASSERT_EQ_INT(ms.stat_forwards_rx, 1u);
    ASSERT_EQ_INT(ms.stat_forwards_unwrapped, 1u);
    return true;
}

static bool test_mesh_handle_forward_too_short(void) {
    reset_test_helpers();
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.enabled = true;
    ms.station_id = 0x0001;
    ms.packet_handler = test_packet_handler;

    uint8_t short_buf[4] = { 0xF0, 0x05, 0x00, 0x32 };
    const uint8_t *inner_out;
    int inner_len;
    ASSERT(!mesh_handle_forward(&ms, short_buf, 4, &inner_out, &inner_len));
    return true;
}

static bool test_mesh_handle_forward_disabled_no_ack(void) {
    reset_test_helpers();
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.enabled = false; /* mesh disabled -- no ACKs */
    ms.station_id = 0x0001;
    ms.packet_handler = test_packet_handler;
    ms.dedup_handler = test_dedup_handler;
    dedup_handler_result = true;

    uint8_t inner[8] = { 0x10, 0x42, 0x00, 0x01, 0xAA, 0xBB, 0xCC, 0xDD };
    uint8_t fwd_buf[IOTDATA_MESH_FORWARD_HDR_SIZE + 8];
    iotdata_mesh_pack_forward(fwd_buf, 0x0005, 50, 3, inner, 8);

    const uint8_t *inner_out;
    int inner_len;
    ASSERT(mesh_handle_forward(&ms, fwd_buf, IOTDATA_MESH_FORWARD_HDR_SIZE + 8, &inner_out, &inner_len));

    /* no ACK when disabled */
    ASSERT_EQ_INT(packet_handler_calls, 0);
    ASSERT_EQ_INT(ms.stat_acks_tx, 0u);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// mesh_handle_beacon / route_error / pong
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool test_mesh_handle_beacon_rx(void) {
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.debug = false;

    uint8_t buf[IOTDATA_MESH_BEACON_SIZE];
    const iotdata_mesh_beacon_t b = {
        .sender_station = 0x0099,
        .sender_seq = 42,
        .gateway_id = 0x0099,
        .cost = 2,
        .flags = IOTDATA_MESH_FLAG_ACCEPTING,
        .generation = 100,
    };
    iotdata_mesh_pack_beacon(buf, &b);

    mesh_handle_beacon(&ms, buf, IOTDATA_MESH_BEACON_SIZE);
    /* no crash = pass */
    return true;
}

static bool test_mesh_handle_route_error_rx(void) {
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));

    uint8_t buf[IOTDATA_MESH_ROUTE_ERROR_SIZE];
    iotdata_mesh_pack_header(buf, 0x0010, 5);
    buf[4] = (uint8_t)((IOTDATA_MESH_CTRL_ROUTE_ERROR << 4) | IOTDATA_MESH_REASON_PARENT_LOST);

    mesh_handle_route_error(&ms, buf, IOTDATA_MESH_ROUTE_ERROR_SIZE);
    return true;
}

static bool test_mesh_handle_pong_rx(void) {
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));

    uint8_t buf[IOTDATA_MESH_PONG_SIZE];
    iotdata_mesh_pack_header(buf, 0x0020, 7);
    buf[4] = (uint8_t)(IOTDATA_MESH_CTRL_PONG << 4);
    memset(&buf[5], 0, (size_t)(IOTDATA_MESH_PONG_SIZE - 5));

    mesh_handle_pong(&ms, buf, IOTDATA_MESH_PONG_SIZE);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// mesh sequence counter
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool test_mesh_seq_increments(void) {
    reset_test_helpers();
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.enabled = true;
    ms.station_id = 0x0001;
    ms.packet_handler = test_packet_handler;

    mesh_beacon_send(&ms);
    ASSERT_EQ_INT(ms.mesh_seq, 1u);

    mesh_ack_send(&ms, 0x0002, 1);
    ASSERT_EQ_INT(ms.mesh_seq, 2u);

    mesh_beacon_send(&ms);
    ASSERT_EQ_INT(ms.mesh_seq, 3u);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// mesh_end
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool test_mesh_end_noop(void) {
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    mesh_end(&ms);
    return true;
}

// =========================================================================================================================================
// DEDUP TESTS
// =========================================================================================================================================

// -----------------------------------------------------------------------------------------------------------------------------------------
// packet encoding macros
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool test_ddup_packet_encoding(void) {
    ddup_packet_t pkt;
    memset(pkt, 0, sizeof(pkt));

    ddup_packet_set_gateway_id(pkt, 0x1234);
    ASSERT_EQ_INT(ddup_packet_get_gateway_id(pkt), 0x1234u);

    ddup_packet_set_entry_count(pkt, 5);
    ASSERT_EQ_INT(ddup_packet_get_entry_count(pkt), 5);

    ddup_packet_set_entry_station(pkt, 0, 0xABCD);
    ASSERT_EQ_INT(ddup_packet_get_entry_station(pkt, 0), 0xABCDu);

    ddup_packet_set_entry_sequence(pkt, 0, 0x5678);
    ASSERT_EQ_INT(ddup_packet_get_entry_sequence(pkt, 0), 0x5678u);

    /* multiple entries */
    ddup_packet_set_entry_station(pkt, 1, 0x1111);
    ddup_packet_set_entry_sequence(pkt, 1, 0x2222);
    ASSERT_EQ_INT(ddup_packet_get_entry_station(pkt, 1), 0x1111u);
    ASSERT_EQ_INT(ddup_packet_get_entry_sequence(pkt, 1), 0x2222u);

    /* first entry unchanged */
    ASSERT_EQ_INT(ddup_packet_get_entry_station(pkt, 0), 0xABCDu);
    ASSERT_EQ_INT(ddup_packet_get_entry_sequence(pkt, 0), 0x5678u);
    return true;
}

static bool test_ddup_packet_length(void) {
    ddup_packet_t pkt;
    memset(pkt, 0, sizeof(pkt));

    ddup_packet_set_entry_count(pkt, 0);
    ASSERT_EQ_INT(ddup_packet_get_length(pkt), 3u);

    ddup_packet_set_entry_count(pkt, 1);
    ASSERT_EQ_INT(ddup_packet_get_length(pkt), 7u);

    ddup_packet_set_entry_count(pkt, 32);
    ASSERT_EQ_INT(ddup_packet_get_length(pkt), (size_t)(3 + 32 * 4));
    return true;
}

static bool test_ddup_packet_entry_count_clamped(void) {
    ddup_packet_t pkt;
    memset(pkt, 0, sizeof(pkt));
    pkt[2] = 100; /* raw count exceeds DDUP_BATCH_MAX */
    ASSERT_EQ_INT(ddup_packet_get_entry_count(pkt), DDUP_BATCH_MAX);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// ddup_peers_parse
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool test_ddup_peers_parse_basic(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    ds.port = 9876;

    ddup_peers_parse(&ds, "host1,host2,host3");

    ASSERT_EQ_INT(ds.peers_count, 3);
    ASSERT(strcmp(ds.peers[0].host, "host1") == 0);
    ASSERT_EQ_INT(ds.peers[0].port, 9876u);
    ASSERT(strcmp(ds.peers[1].host, "host2") == 0);
    ASSERT(strcmp(ds.peers[2].host, "host3") == 0);
    return true;
}

static bool test_ddup_peers_parse_with_ports(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    ds.port = 9876;

    ddup_peers_parse(&ds, "host1:1234,host2:5678");

    ASSERT_EQ_INT(ds.peers_count, 2);
    ASSERT(strcmp(ds.peers[0].host, "host1") == 0);
    ASSERT_EQ_INT(ds.peers[0].port, 1234u);
    ASSERT(strcmp(ds.peers[1].host, "host2") == 0);
    ASSERT_EQ_INT(ds.peers[1].port, 5678u);
    return true;
}

static bool test_ddup_peers_parse_empty(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));

    ddup_peers_parse(&ds, "");
    ASSERT_EQ_INT(ds.peers_count, 0);

    ddup_peers_parse(&ds, NULL);
    ASSERT_EQ_INT(ds.peers_count, 0);
    return true;
}

static bool test_ddup_peers_parse_whitespace(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    ds.port = 9876;

    ddup_peers_parse(&ds, " host1, host2");

    ASSERT_EQ_INT(ds.peers_count, 2);
    ASSERT(strcmp(ds.peers[0].host, "host1") == 0);
    ASSERT(strcmp(ds.peers[1].host, "host2") == 0);
    return true;
}

static bool test_ddup_peers_parse_max(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    ds.port = 9876;

    char buf[512];
    buf[0] = '\0';
    for (int i = 0; i < DDUP_PEERS_MAX + 5; i++) {
        if (i > 0)
            strcat(buf, ",");
        char h[16];
        snprintf(h, sizeof(h), "h%d", i);
        strcat(buf, h);
    }

    ddup_peers_parse(&ds, buf);
    ASSERT_EQ_INT(ds.peers_count, DDUP_PEERS_MAX);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// ddup_check_and_add
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool test_ddup_check_and_add_new(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    ds.enabled = false;
    iotdata_mesh_dedup_ring_t ring;
    iotdata_mesh_dedup_init(&ring);
    ds.dedup_ring = &ring;

    ASSERT(ddup_check_and_add(&ds, 0x0042, 1));
    return true;
}

static bool test_ddup_check_and_add_duplicate(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    ds.enabled = false;
    iotdata_mesh_dedup_ring_t ring;
    iotdata_mesh_dedup_init(&ring);
    ds.dedup_ring = &ring;

    ASSERT(ddup_check_and_add(&ds, 0x0042, 1));
    ASSERT(!ddup_check_and_add(&ds, 0x0042, 1));
    return true;
}

static bool test_ddup_check_and_add_different_station(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    ds.enabled = false;
    iotdata_mesh_dedup_ring_t ring;
    iotdata_mesh_dedup_init(&ring);
    ds.dedup_ring = &ring;

    ASSERT(ddup_check_and_add(&ds, 0x0042, 1));
    ASSERT(ddup_check_and_add(&ds, 0x0043, 1)); /* different station, same seq */
    return true;
}

static bool test_ddup_check_and_add_different_seq(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    ds.enabled = false;
    iotdata_mesh_dedup_ring_t ring;
    iotdata_mesh_dedup_init(&ring);
    ds.dedup_ring = &ring;

    ASSERT(ddup_check_and_add(&ds, 0x0042, 1));
    ASSERT(ddup_check_and_add(&ds, 0x0042, 2)); /* same station, different seq */
    return true;
}

static bool test_ddup_check_and_add_with_pending(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    ds.enabled = true;
    iotdata_mesh_dedup_ring_t ring;
    iotdata_mesh_dedup_init(&ring);
    ds.dedup_ring = &ring;
    pthread_mutex_init(&ds.mutex, NULL);

    ASSERT(ddup_check_and_add(&ds, 0x0042, 1));
    ASSERT_EQ_INT(ds.pending_count, 1);
    ASSERT_EQ_INT(ds.pending[0].station_id, 0x0042u);
    ASSERT_EQ_INT(ds.pending[0].sequence, 1u);

    /* duplicate not added to pending */
    ASSERT(!ddup_check_and_add(&ds, 0x0042, 1));
    ASSERT_EQ_INT(ds.pending_count, 1);

    ASSERT(ddup_check_and_add(&ds, 0x0043, 2));
    ASSERT_EQ_INT(ds.pending_count, 2);

    pthread_mutex_destroy(&ds.mutex);
    return true;
}

static bool test_dedup_ring_overflow(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    ds.enabled = false;
    iotdata_mesh_dedup_ring_t ring;
    iotdata_mesh_dedup_init(&ring);
    ds.dedup_ring = &ring;

    /* fill beyond capacity */
    for (int i = 0; i < IOTDATA_MESH_DEDUP_RING_SIZE + 10; i++)
        ddup_check_and_add(&ds, (uint16_t)i, 1);

    /* first entries evicted -- should appear new again */
    ASSERT(ddup_check_and_add(&ds, 0, 1));

    /* recent entries still present */
    ASSERT(!ddup_check_and_add(&ds, (uint16_t)(IOTDATA_MESH_DEDUP_RING_SIZE + 9), 1));
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// ddup_send_collect
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool test_ddup_send_collect_empty(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    pthread_mutex_init(&ds.mutex, NULL);

    iotdata_mesh_dedup_entry_t entries[DDUP_PENDING_MAX];
    ASSERT_EQ_INT(ddup_send_collect(&ds, entries), 0);

    pthread_mutex_destroy(&ds.mutex);
    return true;
}

static bool test_ddup_send_collect_with_delay(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    ds.enabled = true;
    ds.delay_ms = 10;
    iotdata_mesh_dedup_ring_t ring;
    iotdata_mesh_dedup_init(&ring);
    ds.dedup_ring = &ring;
    pthread_mutex_init(&ds.mutex, NULL);

    ddup_check_and_add(&ds, 0x0042, 1);
    ddup_check_and_add(&ds, 0x0043, 2);
    ASSERT_EQ_INT(ds.pending_count, 2);

    /* collect immediately -- delay not elapsed */
    iotdata_mesh_dedup_entry_t entries[DDUP_PENDING_MAX];
    ASSERT_EQ_INT(ddup_send_collect(&ds, entries), 0);

    /* wait for delay */
    usleep(15000);

    int count = ddup_send_collect(&ds, entries);
    ASSERT_EQ_INT(count, 2);
    ASSERT_EQ_INT(entries[0].station_id, 0x0042u);
    ASSERT_EQ_INT(entries[0].sequence, 1u);
    ASSERT_EQ_INT(entries[1].station_id, 0x0043u);
    ASSERT_EQ_INT(entries[1].sequence, 2u);
    ASSERT_EQ_INT(ds.pending_count, 0);

    pthread_mutex_destroy(&ds.mutex);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// dedup socket setup/teardown
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool test_ddup_recv_setup_and_teardown(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    ds.port = 19876;

    int fd = ddup_recv_setup(&ds);
    ASSERT(fd >= 0);
    close(fd);
    return true;
}

static bool test_ddup_send_setup_and_teardown(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));

    int fd = ddup_send_setup(&ds);
    ASSERT(fd >= 0);
    close(fd);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// ddup_begin disabled
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool test_ddup_begin_disabled(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    ds.enabled = false;
    iotdata_mesh_dedup_ring_t ring;

    ASSERT(ddup_begin(&ds, 0x0001, &ring, NULL));
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// ddup_send_to_peers batching
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool test_ddup_send_to_peers_batching(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    ds.enabled = true;
    ds.port = 19010;
    ds.gateway_id = 0x0001;
    iotdata_mesh_dedup_ring_t ring;
    iotdata_mesh_dedup_init(&ring);
    ds.dedup_ring = &ring;

    ddup_peers_parse(&ds, "127.0.0.1:19011");
    ddup_peers_resolve(&ds);
    pthread_mutex_init(&ds.mutex, NULL);

    /* set up receiving socket */
    int recv_fd = socket(AF_INET, SOCK_DGRAM, 0);
    ASSERT(recv_fd >= 0);
    int optval = 1;
    setsockopt(recv_fd, SOL_SOCKET, SO_REUSEADDR, &optval, (socklen_t)sizeof(optval));
    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(19011);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    ASSERT(bind(recv_fd, (struct sockaddr *)&bind_addr, (socklen_t)sizeof(bind_addr)) == 0);

    /* create entries exceeding DDUP_BATCH_MAX */
    int count = DDUP_BATCH_MAX + 10;
    iotdata_mesh_dedup_entry_t entries[DDUP_BATCH_MAX + 10];
    for (int i = 0; i < count; i++) {
        entries[i].station_id = (uint16_t)(i + 1);
        entries[i].sequence = (uint16_t)(i * 10);
    }

    int send_fd = ddup_send_setup(&ds);
    ASSERT(send_fd >= 0);

    ddup_send_to_peers(&ds, send_fd, entries, count);

    /* should have sent 2 batches */
    ASSERT_EQ_INT(ds.stat_send_cycles, 2u);
    ASSERT_EQ_INT(ds.stat_send_entries, (uint32_t)count);

    /* receive and verify first batch */
    struct pollfd pfd = { .fd = recv_fd, .events = POLLIN, .revents = 0 };
    ddup_packet_t pkt;

    ASSERT(poll(&pfd, 1, 100) > 0);
    ssize_t n = recv(recv_fd, pkt, sizeof(pkt), 0);
    ASSERT(n > 0);
    ASSERT_EQ_INT(ddup_packet_get_gateway_id(pkt), 0x0001u);
    ASSERT_EQ_INT(ddup_packet_get_entry_count(pkt), DDUP_BATCH_MAX);

    /* second batch */
    ASSERT(poll(&pfd, 1, 100) > 0);
    n = recv(recv_fd, pkt, sizeof(pkt), 0);
    ASSERT(n > 0);
    ASSERT_EQ_INT(ddup_packet_get_entry_count(pkt), 10);

    close(send_fd);
    close(recv_fd);
    pthread_mutex_destroy(&ds.mutex);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// dedup peer-to-peer communication
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool test_ddup_peer_communication(void) {
    volatile bool test_running = true;

    /* instance A on port 19001 */
    ddup_state_t sa;
    memset(&sa, 0, sizeof(sa));
    sa.enabled = true;
    sa.port = 19001;
    sa.delay_ms = 5;
    sa.gateway_id = 0x0001;
    sa.running = &test_running;
    iotdata_mesh_dedup_ring_t ring_a;
    iotdata_mesh_dedup_init(&ring_a);
    sa.dedup_ring = &ring_a;
    ddup_peers_parse(&sa, "127.0.0.1:19002");
    ddup_peers_resolve(&sa);
    ASSERT_EQ_INT(sa.peers_count, 1);
    ASSERT(sa.peers[0].resolved);

    /* instance B on port 19002 */
    ddup_state_t sb;
    memset(&sb, 0, sizeof(sb));
    sb.enabled = true;
    sb.port = 19002;
    sb.delay_ms = 5;
    sb.gateway_id = 0x0002;
    sb.running = &test_running;
    iotdata_mesh_dedup_ring_t ring_b;
    iotdata_mesh_dedup_init(&ring_b);
    sb.dedup_ring = &ring_b;
    ddup_peers_parse(&sb, "127.0.0.1:19001");
    ddup_peers_resolve(&sb);
    ASSERT_EQ_INT(sb.peers_count, 1);
    ASSERT(sb.peers[0].resolved);

    /* start threads */
    pthread_mutex_init(&sa.mutex, NULL);
    pthread_mutex_init(&sb.mutex, NULL);

    pthread_t ta, tb;
    ASSERT(pthread_create(&ta, NULL, ddup_thread_func, &sa) == 0);
    ASSERT(pthread_create(&tb, NULL, ddup_thread_func, &sb) == 0);

    /* A sees a packet */
    ddup_check_and_add(&sa, 0x0042, 100);

    /* wait for propagation */
    usleep(100000);

    /* B should now have this entry via UDP injection */
    pthread_mutex_lock(&sb.mutex);
    bool is_new = iotdata_mesh_dedup_check_and_add(sb.dedup_ring, 0x0042, 100);
    pthread_mutex_unlock(&sb.mutex);

    ASSERT(!is_new);
    ASSERT(sb.stat_injected > 0);

    test_running = false;
    pthread_join(ta, NULL);
    pthread_join(tb, NULL);

    pthread_mutex_destroy(&sa.mutex);
    pthread_mutex_destroy(&sb.mutex);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// dedup bidirectional sync
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool test_ddup_bidirectional_sync(void) {
    volatile bool test_running = true;

    ddup_state_t sa;
    memset(&sa, 0, sizeof(sa));
    sa.enabled = true;
    sa.port = 19003;
    sa.delay_ms = 5;
    sa.gateway_id = 0x0001;
    sa.running = &test_running;
    iotdata_mesh_dedup_ring_t ring_a;
    iotdata_mesh_dedup_init(&ring_a);
    sa.dedup_ring = &ring_a;
    ddup_peers_parse(&sa, "127.0.0.1:19004");
    ddup_peers_resolve(&sa);

    ddup_state_t sb;
    memset(&sb, 0, sizeof(sb));
    sb.enabled = true;
    sb.port = 19004;
    sb.delay_ms = 5;
    sb.gateway_id = 0x0002;
    sb.running = &test_running;
    iotdata_mesh_dedup_ring_t ring_b;
    iotdata_mesh_dedup_init(&ring_b);
    sb.dedup_ring = &ring_b;
    ddup_peers_parse(&sb, "127.0.0.1:19003");
    ddup_peers_resolve(&sb);

    pthread_mutex_init(&sa.mutex, NULL);
    pthread_mutex_init(&sb.mutex, NULL);

    pthread_t ta, tb;
    ASSERT(pthread_create(&ta, NULL, ddup_thread_func, &sa) == 0);
    ASSERT(pthread_create(&tb, NULL, ddup_thread_func, &sb) == 0);

    /* A sees packet X, B sees packet Y */
    ddup_check_and_add(&sa, 0x0042, 100);
    ddup_check_and_add(&sb, 0x0043, 200);

    usleep(100000);

    /* A should have B's entry */
    pthread_mutex_lock(&sa.mutex);
    bool a_has_b = !iotdata_mesh_dedup_check_and_add(sa.dedup_ring, 0x0043, 200);
    pthread_mutex_unlock(&sa.mutex);

    /* B should have A's entry */
    pthread_mutex_lock(&sb.mutex);
    bool b_has_a = !iotdata_mesh_dedup_check_and_add(sb.dedup_ring, 0x0042, 100);
    pthread_mutex_unlock(&sb.mutex);

    ASSERT(a_has_b);
    ASSERT(b_has_a);

    test_running = false;
    pthread_join(ta, NULL);
    pthread_join(tb, NULL);

    pthread_mutex_destroy(&sa.mutex);
    pthread_mutex_destroy(&sb.mutex);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// dedup three-gateway sync
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool test_ddup_three_gateway_sync(void) {
    volatile bool test_running = true;
    const uint16_t ports[3] = { 19005, 19006, 19007 };

    ddup_state_t states[3];
    iotdata_mesh_dedup_ring_t rings[3];
    pthread_t threads[3];

    for (int i = 0; i < 3; i++) {
        memset(&states[i], 0, sizeof(states[i]));
        states[i].enabled = true;
        states[i].port = ports[i];
        states[i].delay_ms = 5;
        states[i].gateway_id = (uint16_t)(i + 1);
        states[i].running = &test_running;
        iotdata_mesh_dedup_init(&rings[i]);
        states[i].dedup_ring = &rings[i];

        /* peer with the other two */
        char peers[128];
        int pi = 0;
        for (int j = 0; j < 3; j++) {
            if (j == i)
                continue;
            if (pi > 0)
                peers[pi++] = ',';
            pi += snprintf(peers + pi, sizeof(peers) - (size_t)pi, "127.0.0.1:%d", ports[j]);
        }
        peers[pi] = '\0';
        ddup_peers_parse(&states[i], peers);
        ddup_peers_resolve(&states[i]);

        pthread_mutex_init(&states[i].mutex, NULL);
        ASSERT(pthread_create(&threads[i], NULL, ddup_thread_func, &states[i]) == 0);
    }

    /* gateway 0 sees a packet */
    ddup_check_and_add(&states[0], 0x0042, 500);

    usleep(150000);

    /* gateways 1 and 2 should have the entry */
    for (int i = 1; i < 3; i++) {
        pthread_mutex_lock(&states[i].mutex);
        bool is_new = iotdata_mesh_dedup_check_and_add(states[i].dedup_ring, 0x0042, 500);
        pthread_mutex_unlock(&states[i].mutex);
        ASSERT(!is_new);
    }

    test_running = false;
    for (int i = 0; i < 3; i++) {
        pthread_join(threads[i], NULL);
        pthread_mutex_destroy(&states[i].mutex);
    }
    return true;
}

// =========================================================================================================================================
// Main
// =========================================================================================================================================

int main(void) {
    setbuf(stdout, NULL);

    printf("\n=== Mesh Tests ===\n\n");

    RUN_TEST(mesh_begin_disabled);
    RUN_TEST(mesh_begin_enabled);
    RUN_TEST(mesh_begin_no_packet_handler);
    RUN_TEST(mesh_beacon_send_ok);
    RUN_TEST(mesh_beacon_send_fail);
    RUN_TEST(mesh_beacon_generation_wraps);
    RUN_TEST(mesh_ack_send_ok);
    RUN_TEST(mesh_handle_forward_new);
    RUN_TEST(mesh_handle_forward_duplicate);
    RUN_TEST(mesh_handle_forward_no_dedup);
    RUN_TEST(mesh_handle_forward_too_short);
    RUN_TEST(mesh_handle_forward_disabled_no_ack);
    RUN_TEST(mesh_handle_beacon_rx);
    RUN_TEST(mesh_handle_route_error_rx);
    RUN_TEST(mesh_handle_pong_rx);
    RUN_TEST(mesh_seq_increments);
    RUN_TEST(mesh_end_noop);

    printf("\n=== Dedup Tests ===\n\n");

    RUN_TEST(ddup_packet_encoding);
    RUN_TEST(ddup_packet_length);
    RUN_TEST(ddup_packet_entry_count_clamped);
    RUN_TEST(ddup_peers_parse_basic);
    RUN_TEST(ddup_peers_parse_with_ports);
    RUN_TEST(ddup_peers_parse_empty);
    RUN_TEST(ddup_peers_parse_whitespace);
    RUN_TEST(ddup_peers_parse_max);
    RUN_TEST(ddup_check_and_add_new);
    RUN_TEST(ddup_check_and_add_duplicate);
    RUN_TEST(ddup_check_and_add_different_station);
    RUN_TEST(ddup_check_and_add_different_seq);
    RUN_TEST(ddup_check_and_add_with_pending);
    RUN_TEST(dedup_ring_overflow);
    RUN_TEST(ddup_send_collect_empty);
    RUN_TEST(ddup_send_collect_with_delay);
    RUN_TEST(ddup_recv_setup_and_teardown);
    RUN_TEST(ddup_send_setup_and_teardown);
    RUN_TEST(ddup_begin_disabled);
    RUN_TEST(ddup_send_to_peers_batching);
    RUN_TEST(ddup_peer_communication);
    RUN_TEST(ddup_bidirectional_sync);
    RUN_TEST(ddup_three_gateway_sync);

    printf("\n=== Results ===\n\n");
    printf("  Total: %d, Passed: %d, Failed: %d\n\n", tests_run, tests_passed, tests_run - tests_passed);

    return tests_run > tests_passed ? EXIT_FAILURE : EXIT_SUCCESS;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
