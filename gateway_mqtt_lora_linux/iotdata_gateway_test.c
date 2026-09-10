
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
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <assert.h>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>

/* The stat table lives behind the gateway's include chain (e22 config type, mqtt state, cJSON,
   variant map count), so the prelude below mirrors iotdata_gateway.c's — with the e22 print/sleep
   hooks stubbed, since nothing here touches the radio. */
#include <stdarg.h>
#include <cjson/cJSON.h>

static void test_e22_printf_stub(const char *format, ...) {
    (void)format;
}
#define PRINTF_DEBUG test_e22_printf_stub
#define PRINTF_ERROR test_e22_printf_stub
#define PRINTF_INFO  test_e22_printf_stub
#define PRINTF_WARN  test_e22_printf_stub
/* Same stack as the gateway: the common E22 driver on the Linux platform shim. Only needed here
   for the types the headers under test use (lora_config_t, LORA_PACKET_SIZE_MAX). */
#define EMU_LINUX
#include "d_platform_linux.h"
#include "d_common.h"
#define PIN_DEVICE_UART_TX  GPIO_NUM_NC
#define PIN_DEVICE_UART_RX  GPIO_NUM_NC
#define PIN_DEVICE_LORA_AUX GPIO_NUM_NC
#define PIN_DEVICE_LORA_M0  GPIO_NUM_NC
#define PIN_DEVICE_LORA_M1  GPIO_NUM_NC
#include "d_interface_e22900t22.h"

static inline int get_rssi_dbm(const uint8_t rssi) {
    return -(256 - (int)rssi);
}
static inline uint8_t rssi_raw_from_dbm(const int dbm) {
    return (uint8_t)(dbm + 256);
}
/* __sleep_ms was the depend core's delay callback; the common driver uses hw_delay_ms_yieldable
   from d_common.h, which the platform shim backs with nanosleep. */

#define MQTT_CONNECT_TIMEOUT 60
#define MQTT_PUBLISH_QOS     0
#define MQTT_PUBLISH_RETAIN  false
#include "mqtt_linux.h"

#include "iotdata_gateway_util.h"

#include "iotdata_config.h"
#include "iotdata_variant.h"
#include "iotdata.c"
#include "iotdata_mesh.h"
#include "iotdata_down.h"
#include "iotdata_node.h"

#define BLACKBOX_PERSIST BLACKBOX_PERSIST_NONE
#define IOTDATA_BLACKBOX_IMPLEMENTATION
#include "iotdata_blackbox.h"

typedef struct {
    blackbox_handle_t handle;
    blackbox_config_t config;
    char pool[IOTDATA_BLACKBOX_POOL_SZ];
    char path[512];
} bbox_state_t;

#include "iotdata_gateway_mesh.h"
#include "iotdata_gateway_ddup.h"
#include "iotdata_gateway_stat.h"
#include "iotdata_gateway_netw.h"
/* node/ctrl/exec are pulled in for the process_mesh_packet tests; the harness exercises only part
   of them, so their lifecycle entry points are legitimately unused here. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#include "iotdata_gateway_node.h"
#include "iotdata_gateway_ctrl.h"
#include "iotdata_gateway_exec.h"
#pragma GCC diagnostic pop

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
// mesh_transmit_beacon
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool test_mesh_transmit_beacon_ok(void) {
    reset_test_helpers();
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.enabled = true;
    ms.station_id = 0x0042;
    ms.packet_handler = test_packet_handler;

    mesh_transmit_beacon(&ms);

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

static bool test_mesh_transmit_beacon_fail(void) {
    reset_test_helpers();
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.enabled = true;
    ms.station_id = 0x0042;
    ms.packet_handler = test_packet_handler_fail;

    mesh_transmit_beacon(&ms);

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

    mesh_transmit_beacon(&ms);

    ASSERT_EQ_INT(ms.beacon_generation, 0u);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// mesh_transmit_ack
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool test_mesh_transmit_ack_ok(void) {
    reset_test_helpers();
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.enabled = true;
    ms.station_id = 0x0001;
    ms.packet_handler = test_packet_handler;

    mesh_transmit_ack(&ms, 0x0002, 100);

    ASSERT_EQ_INT(packet_handler_calls, 1);
    ASSERT_EQ_INT(captured_lengths[0], IOTDATA_MESH_ACK_SIZE);
    ASSERT_EQ_INT(ms.stat_acks_tx, 1u);

    iotdata_mesh_ack_t ack;
    ASSERT(iotdata_mesh_unpack_ack(captured_packets[0], captured_lengths[0], &ack));
    ASSERT_EQ_INT(ack.sender_station, 0x0001u);
    ASSERT_EQ_INT(ack.origin_station, 0x0002u);
    ASSERT_EQ_INT(ack.origin_sequence, 100u);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// mesh_receive_forward
// -----------------------------------------------------------------------------------------------------------------------------------------

/* mesh_receive_forward_r only READS the frame -- unpack and accounting. The dedup decision and the
   ACK that follows it belong to the caller, so they are covered by the process_mesh_packet tests
   below rather than here. */
static bool test_mesh_receive_forward_new(void) {
    reset_test_helpers();
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.enabled = true;
    ms.station_id = 0x0001;
    ms.packet_handler = test_packet_handler;

    uint8_t inner[8] = { 0x10, 0x42, 0x00, 0x01, 0xAA, 0xBB, 0xCC, 0xDD };
    uint8_t fwd_buf[IOTDATA_MESH_FORWARD_HDR_SIZE + 8];
    iotdata_mesh_pack_forward(fwd_buf, sizeof(fwd_buf), 0x0005, 50, 3, inner, 8);

    iotdata_mesh_forward_t fw;
    ASSERT(mesh_receive_forward_r(&ms, fwd_buf, IOTDATA_MESH_FORWARD_HDR_SIZE + 8, &fw));
    ASSERT_EQ_INT(ms.ctrl[IOTDATA_MESH_CTRL_FORWARD].rx, 1u);
    ASSERT_EQ_INT(fw.inner_len, 8);
    ASSERT_EQ_INT(fw.sender_station, 0x0005);
    /* it must NOT have transmitted, and must NOT have consulted dedup */
    ASSERT_EQ_INT(packet_handler_calls, 0);
    ASSERT_EQ_INT(ms.stat_acks_tx, 0u);
    ASSERT_EQ_INT(dedup_handler_calls, 0);
    return true;
}

static bool test_mesh_receive_forward_too_short(void) {
    reset_test_helpers();
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.enabled = true;
    ms.station_id = 0x0001;
    ms.packet_handler = test_packet_handler;

    uint8_t short_buf[4] = { 0xF0, 0x05, 0x00, 0x32 };
    iotdata_mesh_forward_t fw;
    ASSERT(!mesh_receive_forward_r(&ms, short_buf, 4, &fw));
    ASSERT_EQ_INT(ms.ctrl[IOTDATA_MESH_CTRL_FORWARD].err, 1u);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// process_mesh_packet FORWARD: dedup + ACK policy, which moved out of the receive handler.
//
// The inner packet is deliberately too short to peek, so process_sensor_packet is never reached
// and nothing is published -- leaving the dedup/ACK decision as the only thing under test.
// -----------------------------------------------------------------------------------------------------------------------------------------

static void fwd_test_state(process_state_t *ps, mesh_state_t *ms, stat_state_t *ss, bool enabled, bool dedup_says_new) {
    memset(ps, 0, sizeof(*ps));
    memset(ms, 0, sizeof(*ms));
    memset(ss, 0, sizeof(*ss));
    ms->enabled = enabled;
    ms->station_id = 0x0001;
    ms->packet_handler = test_packet_handler;
    ms->dedup_handler = test_dedup_handler;
    dedup_handler_result = dedup_says_new;
    netw_begin(&ps->network);
    ps->state_mesh = ms;
    ps->state_stat = ss;
    ps->mqtt_topic_prefix = "test";
}

/* two bytes of inner: unpackable as a FORWARD, not peekable as an iotdata packet */
#define FWD_TEST_BUILD(buf) \
    uint8_t buf[IOTDATA_MESH_FORWARD_HDR_SIZE + 4]; \
    do { \
        const uint8_t _inner[4] = { 0xC0, 0x42, 0x00, 0x01 }; \
        iotdata_mesh_pack_forward(buf, sizeof(buf), 0x0005, 50, 3, _inner, 4); \
    } while (0)

static bool test_process_forward_new_acks(void) {
    reset_test_helpers();
    static process_state_t ps;
    mesh_state_t ms;
    static stat_state_t ss; /* ~300KB: never on the stack */
    fwd_test_state(&ps, &ms, &ss, true, true /* new */);
    FWD_TEST_BUILD(fwd_buf);

    process_mesh_packet(&ps, fwd_buf, (int)sizeof(fwd_buf), 15, 0x0005, 50, "test", 0);
    ASSERT_EQ_INT(ms.ctrl[IOTDATA_MESH_CTRL_FORWARD].rx, 1u);
    ASSERT_EQ_INT(ms.stat_forwards_unwrapped, 1u);
    ASSERT_EQ_INT(ms.stat_duplicates, 0u);
    ASSERT_EQ_INT(dedup_handler_calls, 1);
    ASSERT_EQ_INT(ms.stat_acks_tx, 1u); /* ACKed */
    return true;
}

static bool test_process_forward_duplicate_still_acks(void) {
    reset_test_helpers();
    static process_state_t ps;
    mesh_state_t ms;
    static stat_state_t ss; /* ~300KB: never on the stack */
    fwd_test_state(&ps, &ms, &ss, true, false /* duplicate */);
    FWD_TEST_BUILD(fwd_buf);

    process_mesh_packet(&ps, fwd_buf, (int)sizeof(fwd_buf), 15, 0x0005, 50, "test", 0);

    ASSERT_EQ_INT(ms.stat_duplicates, 1u);
    ASSERT_EQ_INT(ms.stat_forwards_unwrapped, 0u);
    /* still ACKed: a duplicate means our earlier ACK went missing and the forwarder is retrying */
    ASSERT_EQ_INT(ms.stat_acks_tx, 1u);
    return true;
}

static bool test_process_forward_no_dedup_handler(void) {
    reset_test_helpers();
    static process_state_t ps;
    mesh_state_t ms;
    static stat_state_t ss; /* ~300KB: never on the stack */
    fwd_test_state(&ps, &ms, &ss, true, true);
    ms.dedup_handler = NULL; /* no dedup configured -> everything is new */
    FWD_TEST_BUILD(fwd_buf);

    process_mesh_packet(&ps, fwd_buf, (int)sizeof(fwd_buf), 15, 0x0005, 50, "test", 0);

    ASSERT_EQ_INT(ms.stat_forwards_unwrapped, 1u);
    ASSERT_EQ_INT(ms.stat_duplicates, 0u);
    return true;
}

static bool test_process_forward_disabled_no_ack(void) {
    reset_test_helpers();
    static process_state_t ps;
    mesh_state_t ms;
    static stat_state_t ss; /* ~300KB: never on the stack */
    fwd_test_state(&ps, &ms, &ss, false /* mesh disabled */, true);
    FWD_TEST_BUILD(fwd_buf);

    process_mesh_packet(&ps, fwd_buf, (int)sizeof(fwd_buf), 15, 0x0005, 50, "test", 0);

    ASSERT_EQ_INT(ms.stat_forwards_unwrapped, 1u); /* still accounted for */
    ASSERT_EQ_INT(ms.stat_acks_tx, 0u);            /* but silent */
    ASSERT_EQ_INT(packet_handler_calls, 0);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// mesh_receive_beacon / route_error / pong
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool test_mesh_receive_beacon_rx(void) {
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
    const int len = iotdata_mesh_pack_beacon(buf, sizeof(buf), &b);
    ASSERT_EQ_INT(len, IOTDATA_MESH_BEACON_SIZE);

    iotdata_mesh_beacon_t got;
    ASSERT(mesh_receive_beacon_r(&ms, buf, IOTDATA_MESH_BEACON_SIZE, &got));
    ASSERT_EQ_INT(got.gateway_id, b.gateway_id);
    /* no crash = pass */
    return true;
}

static bool test_mesh_receive_route_error_rx(void) {
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));

    uint8_t buf[IOTDATA_MESH_ROUTE_ERROR_SIZE];
    iotdata_mesh_pack_header(buf, 0x0010, 5);
    buf[4] = (uint8_t)((IOTDATA_MESH_CTRL_ROUTE_ERROR << 4) | IOTDATA_MESH_REASON_PARENT_LOST);

    mesh_receive_route_error(&ms, buf, IOTDATA_MESH_ROUTE_ERROR_SIZE);
    return true;
}

static bool test_mesh_receive_pong_rx(void) {
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));

    uint8_t buf[IOTDATA_MESH_PONG_SIZE];
    iotdata_mesh_pack_header(buf, 0x0020, 7);
    buf[4] = (uint8_t)(IOTDATA_MESH_CTRL_PONG << 4);
    memset(&buf[5], 0, (size_t)(IOTDATA_MESH_PONG_SIZE - 5));

    mesh_receive_pong(&ms, buf, IOTDATA_MESH_PONG_SIZE);
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

    mesh_transmit_beacon(&ms);
    ASSERT_EQ_INT(ms.mesh_seq, 1u);

    mesh_transmit_ack(&ms, 0x0002, 1);
    ASSERT_EQ_INT(ms.mesh_seq, 2u);

    mesh_transmit_beacon(&ms);
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
    pkt[2] = 100; /* raw count exceeds DDUP_PKT_BATCH_SIZE */
    ASSERT_EQ_INT(ddup_packet_get_entry_count(pkt), DDUP_PKT_BATCH_SIZE);
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
// ddup_insert
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool test_ddup_insert_new(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    ds.enabled = false;
    iotdata_mesh_dedup_ring_t ring;
    iotdata_mesh_dedup_init(&ring);
    ds.ddup_ring = &ring;

    ASSERT(ddup_insert(&ds, 0x0042, 1));
    return true;
}

static bool test_ddup_insert_duplicate(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    ds.enabled = false;
    iotdata_mesh_dedup_ring_t ring;
    iotdata_mesh_dedup_init(&ring);
    ds.ddup_ring = &ring;

    ASSERT(ddup_insert(&ds, 0x0042, 1));
    ASSERT(!ddup_insert(&ds, 0x0042, 1));
    return true;
}

static bool test_ddup_insert_different_station(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    ds.enabled = false;
    iotdata_mesh_dedup_ring_t ring;
    iotdata_mesh_dedup_init(&ring);
    ds.ddup_ring = &ring;

    ASSERT(ddup_insert(&ds, 0x0042, 1));
    ASSERT(ddup_insert(&ds, 0x0043, 1)); /* different station, same seq */
    return true;
}

static bool test_ddup_insert_different_seq(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    ds.enabled = false;
    iotdata_mesh_dedup_ring_t ring;
    iotdata_mesh_dedup_init(&ring);
    ds.ddup_ring = &ring;

    ASSERT(ddup_insert(&ds, 0x0042, 1));
    ASSERT(ddup_insert(&ds, 0x0042, 2)); /* same station, different seq */
    return true;
}

static bool test_ddup_insert_with_pending(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    ds.enabled = true;
    iotdata_mesh_dedup_ring_t ring;
    iotdata_mesh_dedup_init(&ring);
    ds.ddup_ring = &ring;
    pthread_mutex_init(&ds.mutex, NULL);

    ASSERT(ddup_insert(&ds, 0x0042, 1));
    ASSERT_EQ_INT(ds.pending_count, 1);
    ASSERT_EQ_INT(ds.pending[0].station_id, 0x0042u);
    ASSERT_EQ_INT(ds.pending[0].sequence, 1u);

    /* duplicate not added to pending */
    ASSERT(!ddup_insert(&ds, 0x0042, 1));
    ASSERT_EQ_INT(ds.pending_count, 1);

    ASSERT(ddup_insert(&ds, 0x0043, 2));
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
    ds.ddup_ring = &ring;

    /* fill beyond capacity */
    for (int i = 0; i < IOTDATA_MESH_DEDUP_RING_SIZE + 10; i++)
        ddup_insert(&ds, (uint16_t)i, 1);

    /* first entries evicted -- should appear new again */
    ASSERT(ddup_insert(&ds, 0, 1));

    /* recent entries still present */
    ASSERT(!ddup_insert(&ds, (uint16_t)(IOTDATA_MESH_DEDUP_RING_SIZE + 9), 1));
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
    ds.ddup_ring = &ring;
    pthread_mutex_init(&ds.mutex, NULL);

    ddup_insert(&ds, 0x0042, 1);
    ddup_insert(&ds, 0x0043, 2);
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
// ddup_peers_send batching
// -----------------------------------------------------------------------------------------------------------------------------------------

/*
 * A sensor heard BOTH directly and relayed must publish once.
 *
 * Regression test. The direct path (ddup_check_sensor_packet) used to additionally require
 * state_ddup->enabled, which defaults to false -- so a direct reception was never recorded in the
 * dedup ring, the FORWARD path's lookup missed, and the same reading went to MQTT twice ("via
 * direct" then "via mesh"). The local ring must work with cross-gateway ddup DISABLED, which is
 * the normal single-gateway configuration; ddup_insert() consults `enabled` only to decide whether
 * to also broadcast the entry to peers.
 *
 * Both orderings are checked: whichever path sees {station, sequence} first wins and the other is
 * suppressed.
 */
static bool test_ddup_direct_and_mesh_dedup_with_ddup_disabled(void) {
    iotdata_mesh_dedup_ring_t ring;
    iotdata_mesh_dedup_init(&ring);

    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    ds.enabled = false; /* the default, and the whole point of the test */
    ds.ddup_ring = &ring;
    pthread_mutex_init(&ds.mutex, NULL);

    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.enabled = true;
    ms.dedup_handler = ddup_insert_handler;

    process_state_t ps;
    memset(&ps, 0, sizeof(ps));
    ps.state_mesh = &ms;
    ps.state_ddup = &ds;
    ms.dedup_handler_ctx = &ps;

    /* direct first, then the relay's forward of the same origin */
    ASSERT(ddup_check_sensor_packet(&ps, 0x06ED, 3));           /* published */
    ASSERT(!ms.dedup_handler(ms.dedup_handler_ctx, 0x06ED, 3)); /* suppressed */

    /* and the other way round: forward first, then a direct copy */
    ASSERT(ms.dedup_handler(ms.dedup_handler_ctx, 0x0537, 12)); /* published */
    ASSERT(!ddup_check_sensor_packet(&ps, 0x0537, 12));         /* suppressed */

    /* a different sequence from the same station is not a duplicate */
    ASSERT(ddup_check_sensor_packet(&ps, 0x06ED, 4));

    /* One, not two: only ddup_check_sensor_packet (the direct path) counts its own duplicate. The
       forward path's stat_duplicates++ sits in the exec loop around the dedup handler, not in the
       handler itself, and this test drives the handler directly. */
    ASSERT_EQ_INT(ms.stat_duplicates, 1);

    pthread_mutex_destroy(&ds.mutex);
    return true;
}

static bool test_ddup_peers_send_batching(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    ds.enabled = true;
    ds.port = 19010;
    ds.gateway_id = 0x0001;
    iotdata_mesh_dedup_ring_t ring;
    iotdata_mesh_dedup_init(&ring);
    ds.ddup_ring = &ring;

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

    /* create entries exceeding DDUP_PKT_BATCH_SIZE */
    int count = DDUP_PKT_BATCH_SIZE + 10;
    iotdata_mesh_dedup_entry_t entries[DDUP_PKT_BATCH_SIZE + 10];
    for (int i = 0; i < count; i++) {
        entries[i].station_id = (uint16_t)(i + 1);
        entries[i].sequence = (uint16_t)(i * 10);
    }

    int send_fd = ddup_send_setup(&ds);
    ASSERT(send_fd >= 0);

    ddup_peers_send(&ds, send_fd, entries, count);

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
    ASSERT_EQ_INT(ddup_packet_get_entry_count(pkt), DDUP_PKT_BATCH_SIZE);

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
    sa.ddup_ring = &ring_a;
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
    sb.ddup_ring = &ring_b;
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
    ddup_insert(&sa, 0x0042, 100);

    /* wait for propagation */
    usleep(100000);

    /* B should now have this entry via UDP injection */
    pthread_mutex_lock(&sb.mutex);
    bool is_new = iotdata_mesh_dedup_insert(sb.ddup_ring, 0x0042, 100);
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
    sa.ddup_ring = &ring_a;
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
    sb.ddup_ring = &ring_b;
    ddup_peers_parse(&sb, "127.0.0.1:19003");
    ddup_peers_resolve(&sb);

    pthread_mutex_init(&sa.mutex, NULL);
    pthread_mutex_init(&sb.mutex, NULL);

    pthread_t ta, tb;
    ASSERT(pthread_create(&ta, NULL, ddup_thread_func, &sa) == 0);
    ASSERT(pthread_create(&tb, NULL, ddup_thread_func, &sb) == 0);

    /* A sees packet X, B sees packet Y */
    ddup_insert(&sa, 0x0042, 100);
    ddup_insert(&sb, 0x0043, 200);

    usleep(100000);

    /* A should have B's entry */
    pthread_mutex_lock(&sa.mutex);
    bool a_has_b = !iotdata_mesh_dedup_insert(sa.ddup_ring, 0x0043, 200);
    pthread_mutex_unlock(&sa.mutex);

    /* B should have A's entry */
    pthread_mutex_lock(&sb.mutex);
    bool b_has_a = !iotdata_mesh_dedup_insert(sb.ddup_ring, 0x0042, 100);
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
        states[i].ddup_ring = &rings[i];

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
    ddup_insert(&states[0], 0x0042, 500);

    usleep(150000);

    /* gateways 1 and 2 should have the entry */
    for (int i = 1; i < 3; i++) {
        pthread_mutex_lock(&states[i].mutex);
        bool is_new = iotdata_mesh_dedup_insert(states[i].ddup_ring, 0x0042, 500);
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

// -----------------------------------------------------------------------------------------------------------------------------------------
// netw + stat table tests.
//
// Both tables keep a hand-maintained `count` of their valid slots that the scan loops use to stop at
// the last one. That invariant is not compiler-checkable and is updated wherever an entry is created,
// so each test re-derives the true number of valid slots the slow way and compares (ASSERT_COUNT_*).
// Entries in these tables are never invalidated -- a full table evicts by overwrite -- so the counts
// only ever rise to the cap. The tables are large, hence the statics.
// -----------------------------------------------------------------------------------------------------------------------------------------

static int netw_valid_slots(const netw_t *n) {
    int c = 0;
    for (int i = 0; i < NETW_MAX; i++)
        if (n->s[i].valid)
            c++;
    return c;
}
#define ASSERT_COUNT_NETW(n) ASSERT((n)->count == netw_valid_slots(n))

static bool test_netw_upsert_on_empty(void) {
    static netw_t net;
    netw_begin(&net);
    ASSERT(netw_count(&net) == 0);
    ASSERT(netw_locate(&net, 0x111) == -1); /* lookup on an empty table */
    const netw_station_t *const e = netw_upsert(&net, 0x111, 1000);
    ASSERT(e != NULL && e->station == 0x111); /* must allocate, not refuse */
    ASSERT_EQ_INT(netw_count(&net), 1);
    ASSERT_COUNT_NETW(&net);
    ASSERT_EQ_INT(netw_locate(&net, 0x111), 0);
    return true;
}

static bool test_netw_upsert_in_place(void) {
    static netw_t net;
    netw_begin(&net);
    const netw_station_t *const a = netw_upsert(&net, 0x111, 1000);
    const netw_station_t *const b = netw_upsert(&net, 0x111, 1001);
    ASSERT(a == b);                     /* same entry */
    ASSERT_EQ_INT(netw_count(&net), 1); /* must NOT double-count */
    ASSERT_EQ_INT((int)b->rx_count, 2);
    ASSERT_COUNT_NETW(&net);
    return true;
}

static bool test_netw_sequential_upserts(void) {
    static netw_t net;
    netw_begin(&net);
    (void)netw_upsert(&net, 0x111, 1000);
    (void)netw_upsert(&net, 0x222, 1001); /* the free slot lies past the valid entries */
    (void)netw_upsert(&net, 0x333, 1002);
    ASSERT_EQ_INT(netw_count(&net), 3);
    ASSERT_COUNT_NETW(&net);
    ASSERT_EQ_INT(netw_locate(&net, 0x222), 1);
    ASSERT_EQ_INT(netw_locate(&net, 0x333), 2);
    ASSERT_EQ_INT(netw_locate(&net, 0x999), -1); /* absent, table non-empty */
    return true;
}

static bool test_netw_fill_and_evict_stalest(void) {
    static netw_t net;
    netw_begin(&net);
    (void)netw_upsert(&net, 0x111, 1000); /* the stalest, once the table is full */
    for (int i = 1; i < NETW_MAX; i++)
        (void)netw_upsert(&net, (uint16_t)(0x400 + i), 2000 + i);
    ASSERT_EQ_INT(netw_count(&net), NETW_MAX);
    ASSERT_COUNT_NETW(&net);
    (void)netw_upsert(&net, 0xABC, 9000);
    ASSERT_EQ_INT(netw_count(&net), NETW_MAX); /* eviction reuses a slot: count saturates */
    ASSERT_COUNT_NETW(&net);
    ASSERT(netw_locate(&net, 0xABC) >= 0);
    ASSERT_EQ_INT(netw_locate(&net, 0x111), -1); /* the stalest was the one evicted */
    return true;
}

static bool test_netw_all_locatable(void) {
    static netw_t net;
    netw_begin(&net);
    for (int i = 0; i < NETW_MAX; i++)
        (void)netw_upsert(&net, (uint16_t)(0x500 + i), 1000 + i);
    int found = 0; /* a scan bound that cut short would lose the later entries */
    for (int i = 0; i < NETW_MAX; i++)
        if (net.s[i].valid && netw_locate(&net, net.s[i].station) == i)
            found++;
    ASSERT_EQ_INT(found, NETW_MAX);
    ASSERT_COUNT_NETW(&net);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* find_or_create() creates on miss, so it can never answer "is this present?" -- scan instead */
static bool stat_station_present(const stat_state_t *s, uint16_t station_id) {
    for (int i = 0; i < (int)(sizeof(s->stations) / sizeof(s->stations[0])); i++)
        if (s->stations[i].valid && s->stations[i].station_id == station_id)
            return true;
    return false;
}

static bool stat_peer_present(const stat_state_t *s, uint16_t station_id) {
    for (int i = 0; i < (int)(sizeof(s->peers) / sizeof(s->peers[0])); i++)
        if (s->peers[i].valid && s->peers[i].station_id == station_id)
            return true;
    return false;
}

static int stat_valid_stations(const stat_state_t *s) {
    int c = 0;
    for (int i = 0; i < STAT_MAX_STATIONS; i++)
        if (s->stations[i].valid)
            c++;
    return c;
}
static int stat_valid_peers(const stat_state_t *s) {
    int c = 0;
    for (int i = 0; i < STAT_MESH_PEERS_MAX; i++)
        if (s->peers[i].valid)
            c++;
    return c;
}
#define ASSERT_COUNT_STAT(s) \
    do { \
        ASSERT((s)->stations_count == stat_valid_stations(s)); \
        ASSERT((s)->peers_count == stat_valid_peers(s)); \
    } while (0)

static bool test_stat_station_create_and_find(void) {
    static stat_state_t s;
    memset(&s, 0, sizeof(s));
    stat_station_t *const a = stat_station_find_or_create(&s, 0x111, time(NULL));
    ASSERT(a != NULL && a->station_id == 0x111); /* create on an EMPTY table */
    ASSERT_EQ_INT(s.stations_count, 1);
    ASSERT(stat_station_find_or_create(&s, 0x111, time(NULL)) == a); /* find, not re-create */
    ASSERT_EQ_INT(s.stations_count, 1);                              /* must NOT double-count */
    (void)stat_station_find_or_create(&s, 0x222, time(NULL));        /* free slot lies past the valid entries */
    (void)stat_station_find_or_create(&s, 0x333, time(NULL));
    ASSERT_EQ_INT(s.stations_count, 3);
    ASSERT_COUNT_STAT(&s);
    ASSERT(stat_station_find_or_create(&s, 0x222, time(NULL))->station_id == 0x222); /* still findable */
    ASSERT_EQ_INT(s.stations_count, 3);
    return true;
}

static bool test_stat_station_fill_and_evict(void) {
    static stat_state_t s;
    memset(&s, 0, sizeof(s));
    for (int i = 0; i < STAT_MAX_STATIONS; i++) {
        stat_station_t *const e = stat_station_find_or_create(&s, (uint16_t)(0x400 + i), time(NULL));
        e->last_seen = 9000 - i; /* DEscending: the stalest is the LAST slot, not the first, so an
                                    "always evict index 0" bug cannot pass by coincidence */
    }
    ASSERT_EQ_INT(s.stations_count, STAT_MAX_STATIONS);
    ASSERT_COUNT_STAT(&s);

    (void)stat_station_find_or_create(&s, 0xABC, time(NULL)); /* full -> evicts the stalest */
    ASSERT_EQ_INT(s.stations_count, STAT_MAX_STATIONS);       /* count saturates */
    ASSERT_COUNT_STAT(&s);
    ASSERT(stat_station_present(&s, 0xABC));
    ASSERT(!stat_station_present(&s, 0x400 + STAT_MAX_STATIONS - 1)); /* the stalest went */
    ASSERT(stat_station_present(&s, 0x400));                          /* the freshest stayed */
    ASSERT(stat_station_present(&s, 0x400 + STAT_MAX_STATIONS - 2));  /* and only one went */
    return true;
}

static bool test_stat_station_all_findable(void) {
    static stat_state_t s;
    memset(&s, 0, sizeof(s));
    for (int i = 0; i < STAT_MAX_STATIONS; i++)
        (void)stat_station_find_or_create(&s, (uint16_t)(0x600 + i), time(NULL));
    int found = 0; /* a scan bound that cut short would re-create instead of finding */
    for (int i = 0; i < STAT_MAX_STATIONS; i++)
        if (s.stations[i].valid && stat_station_find_or_create(&s, s.stations[i].station_id, time(NULL)) == &s.stations[i])
            found++;
    ASSERT_EQ_INT(found, STAT_MAX_STATIONS);
    ASSERT_EQ_INT(s.stations_count, STAT_MAX_STATIONS);
    ASSERT_COUNT_STAT(&s);
    return true;
}

static bool test_stat_mesh_peer_create_and_update(void) {
    static stat_state_t s;
    memset(&s, 0, sizeof(s));
    stat_on_peer(&s, 0xAAA, 1, 1, 0);
    ASSERT_EQ_INT(s.peers_count, 1);
    ASSERT_COUNT_STAT(&s);
    stat_on_peer(&s, 0xAAA, 2, 1, 0); /* same gateway -> update in place */
    ASSERT_EQ_INT(s.peers_count, 1);
    ASSERT_COUNT_STAT(&s);
    stat_on_peer(&s, 0xBBB, 1, 1, 0); /* second peer takes the next free slot */
    ASSERT_EQ_INT(s.peers_count, 2);
    ASSERT_COUNT_STAT(&s);
    return true;
}

static bool test_stat_mesh_peer_fill_and_evict(void) {
    static stat_state_t s;
    memset(&s, 0, sizeof(s));
    for (int i = 0; i < STAT_MESH_PEERS_MAX; i++)
        stat_on_peer(&s, (uint16_t)(0xB00 + i), 1, 1, 0);
    ASSERT_EQ_INT(s.peers_count, STAT_MESH_PEERS_MAX);
    ASSERT_COUNT_STAT(&s);
    /* stat_on_peer stamps last_seen with time(NULL), so every entry ties within the same second.
       Restamp DESCENDING so the stalest is the LAST slot: an "always evict index 0" bug then
       cannot pass by coincidence. Fill order means peers[i] is 0xB00+i. */
    for (int i = 0; i < STAT_MESH_PEERS_MAX; i++)
        s.peers[i].last_seen = 9000 - i;

    stat_on_peer(&s, 0xCCC, 1, 1, 0); /* full -> evict the stalest, count saturates */
    ASSERT_EQ_INT(s.peers_count, STAT_MESH_PEERS_MAX);
    ASSERT_COUNT_STAT(&s);
    ASSERT(stat_peer_present(&s, 0xCCC));
    ASSERT(!stat_peer_present(&s, 0xB00 + STAT_MESH_PEERS_MAX - 1)); /* the stalest went */
    ASSERT(stat_peer_present(&s, 0xB00));                            /* the freshest stayed */
    ASSERT(stat_peer_present(&s, 0xB00 + STAT_MESH_PEERS_MAX - 2));  /* and only one went */

    stat_on_peer(&s, 0xB00, 7, 2, 3); /* existing -> update in place, no growth */
    ASSERT_EQ_INT(s.peers_count, STAT_MESH_PEERS_MAX);
    ASSERT_COUNT_STAT(&s);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

int main(void) {
    setbuf(stdout, NULL);

    printf("\n=== Mesh Tests ===\n\n");

    RUN_TEST(mesh_begin_disabled);
    RUN_TEST(mesh_begin_enabled);
    RUN_TEST(mesh_begin_no_packet_handler);
    RUN_TEST(mesh_transmit_beacon_ok);
    RUN_TEST(mesh_transmit_beacon_fail);
    RUN_TEST(mesh_beacon_generation_wraps);
    RUN_TEST(mesh_transmit_ack_ok);
    RUN_TEST(mesh_receive_forward_new);
    RUN_TEST(mesh_receive_forward_too_short);
    RUN_TEST(process_forward_new_acks);
    RUN_TEST(process_forward_duplicate_still_acks);
    RUN_TEST(process_forward_no_dedup_handler);
    RUN_TEST(process_forward_disabled_no_ack);
    RUN_TEST(mesh_receive_beacon_rx);
    RUN_TEST(mesh_receive_route_error_rx);
    RUN_TEST(mesh_receive_pong_rx);
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
    RUN_TEST(ddup_insert_new);
    RUN_TEST(ddup_insert_duplicate);
    RUN_TEST(ddup_insert_different_station);
    RUN_TEST(ddup_insert_different_seq);
    RUN_TEST(ddup_insert_with_pending);
    RUN_TEST(dedup_ring_overflow);
    RUN_TEST(ddup_send_collect_empty);
    RUN_TEST(ddup_send_collect_with_delay);
    RUN_TEST(ddup_recv_setup_and_teardown);
    RUN_TEST(ddup_send_setup_and_teardown);
    RUN_TEST(ddup_begin_disabled);
    RUN_TEST(ddup_peers_send_batching);
    RUN_TEST(ddup_peer_communication);
    RUN_TEST(ddup_direct_and_mesh_dedup_with_ddup_disabled);
    RUN_TEST(ddup_bidirectional_sync);
    RUN_TEST(ddup_three_gateway_sync);

    printf("\n=== Netw Table Tests ===\n\n");

    RUN_TEST(netw_upsert_on_empty);
    RUN_TEST(netw_upsert_in_place);
    RUN_TEST(netw_sequential_upserts);
    RUN_TEST(netw_fill_and_evict_stalest);
    RUN_TEST(netw_all_locatable);

    printf("\n=== Stat Table Tests ===\n\n");

    RUN_TEST(stat_station_create_and_find);
    RUN_TEST(stat_station_fill_and_evict);
    RUN_TEST(stat_station_all_findable);
    RUN_TEST(stat_mesh_peer_create_and_update);
    RUN_TEST(stat_mesh_peer_fill_and_evict);

    printf("\n=== Results ===\n\n");
    printf("  Total: %d, Passed: %d, Failed: %d\n\n", tests_run, tests_passed, tests_run - tests_passed);

    return tests_run > tests_passed ? EXIT_FAILURE : EXIT_SUCCESS;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
