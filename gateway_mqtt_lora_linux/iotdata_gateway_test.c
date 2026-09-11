
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
#include "d_format.h"

/* The pool as the gateway really builds it: locked, because the gateway is not single-threaded.
   The harness itself is, but compiling the same shape is the point -- an unlocked test pool would
   not be testing what ships. */
#define BUFFER_LOCK_TYPE       pthread_mutex_t
#define BUFFER_LOCK_INIT(l)    pthread_mutex_init((l), NULL)
#define BUFFER_LOCK_ACQUIRE(l) pthread_mutex_lock(l)
#define BUFFER_LOCK_RELEASE(l) pthread_mutex_unlock(l)
#include "d_module_buffers.h"

#define TEST_FRAME_MAX 240
/* Set here, before the pool is declared, exactly as the app sets it before including the store:
   the down table's size is what the pool has to carry on top of the traffic. Kept small for the
   harness -- the gateway itself dimensions for a whole network. */
#define IOTDATA_DOWN_SLOTS 8
#define TEST_POOL_COUNT    (IOTDATA_DOWN_SLOTS + 8)
BUFFER_POOL_DECLARE(t_pool, TEST_POOL_COUNT, TEST_FRAME_MAX + 8);
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

#include "iotdata_station_filter.h"
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
    ms.pool = &t_pool;
    ms.enabled = false;
    ASSERT(mesh_begin(&ms, &t_pool, test_packet_handler, test_dedup_handler, NULL));
    ASSERT(ms.packet_handler == NULL);
    ASSERT(ms.dedup_handler == NULL);
    return true;
}

static bool test_mesh_begin_enabled(void) {
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.pool = &t_pool;
    ms.enabled = true;
    ms.station_id = 0x0042;
    ms.beacon_interval = 60;
    ASSERT(mesh_begin(&ms, &t_pool, test_packet_handler, test_dedup_handler, NULL));
    ASSERT(ms.packet_handler == test_packet_handler);
    ASSERT(ms.dedup_handler == test_dedup_handler);
    return true;
}

static bool test_mesh_begin_no_packet_handler(void) {
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.pool = &t_pool;
    ms.enabled = true;
    ASSERT(mesh_begin(&ms, &t_pool, NULL, test_dedup_handler, NULL));
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
    ms.pool = &t_pool;
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
    ms.pool = &t_pool;
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
    ms.pool = &t_pool;
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
    ms.pool = &t_pool;
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
    ms.pool = &t_pool;
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
    ms.pool = &t_pool;
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

/* The forward path hands an inner packet to the node layer, so a node state is not optional --
   process_run() always sets one. The fixtures below used an inner that fails iotdata_peek(), so
   this went unnoticed until a test used a real one. */
static node_state_t fwd_test_node;

static void fwd_test_state(process_state_t *ps, mesh_state_t *ms, stat_state_t *ss, bool enabled, bool dedup_says_new) {
    memset(ps, 0, sizeof(*ps));
    memset(&fwd_test_node, 0, sizeof(fwd_test_node));
    fwd_test_node.stat = ss;
    fwd_test_node.pool = &t_pool;
    fwd_test_node.tx = test_packet_handler;
    iotdata_down_init(&fwd_test_node.down, &t_pool);
    ps->state_node = &fwd_test_node;
    ps->pool = &t_pool;
    memset(ms, 0, sizeof(*ms));
    memset(ss, 0, sizeof(*ss));
    ms->pool = &t_pool;
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
    ms.pool = &t_pool;
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
    ms.pool = &t_pool;

    uint8_t buf[IOTDATA_MESH_ROUTE_ERROR_SIZE];
    iotdata_mesh_pack_header(buf, 0x0010, 5);
    buf[4] = (uint8_t)((IOTDATA_MESH_CTRL_ROUTE_ERROR << 4) | IOTDATA_MESH_REASON_PARENT_LOST);

    mesh_receive_route_error(&ms, buf, IOTDATA_MESH_ROUTE_ERROR_SIZE);
    return true;
}

static bool test_mesh_receive_pong_rx(void) {
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.pool = &t_pool;

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
    ms.pool = &t_pool;
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
    ms.pool = &t_pool;
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
    ms.pool = &t_pool;
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

// -----------------------------------------------------------------------------------------------------------------------------------------
// Ctrl (MQTT -> node CONTROL) tests
//
// ctrl_on_message() runs on the mosquitto thread and only STAGES a payload; ctrl_tick() drains it
// on the main loop. These drive the real handler and inspect the staged bytes, which is where the
// mesh-management commands used to become MANAGE frames and now become CONTROL kvr entries.
// A kvr entry on the wire is { key, vlen, value... }.
// -----------------------------------------------------------------------------------------------------------------------------------------

#define CTRL_TEST_STATION 0x001 /* the gateway's own id: tests target 0x123 so the diag local path stays out of it */

static void ctrl_test_state(ctrl_state_t *const st) {
    memset(st, 0, sizeof(*st));
    st->station_id = CTRL_TEST_STATION;
    st->pool = &t_pool;
    buffer_queue_init(&st->queue, st->queue_slot, CTRL_QUEUE_MAX, &t_pool);
    g_ctrl = st;
}

/* These tests look at the head of the staging queue rather than draining it with ctrl_tick(), so
   each one hands the buffers back itself -- the pool leak check at the end of main is watching.
   peek() is the right tool here precisely because this queue has one thread in the test: the
   handle cannot go stale under us the way it could on a queue another thread takes from. */
static bool ctrl_staged(const ctrl_state_t *const st) {
    return buffer_queue_peek(&st->queue, (uint32_t)__ticks_ms(), NULL, NULL) != BUFFER_NONE;
}

static uint16_t ctrl_staged_target(const ctrl_state_t *const st) {
    uint32_t key = 0;
    return buffer_queue_peek(&st->queue, (uint32_t)__ticks_ms(), NULL, &key) != BUFFER_NONE ? (uint16_t)key : 0;
}

static uint16_t ctrl_staged_len(const ctrl_state_t *const st) {
    const buffer_handle_t h = buffer_queue_peek(&st->queue, (uint32_t)__ticks_ms(), NULL, NULL);
    return h != BUFFER_NONE ? buffer_len(st->pool, h) : 0;
}

/* drain one the way ctrl_tick does, so a test can check what comes out and in what order */
static bool ctrl_test_take(ctrl_state_t *const st, uint8_t *const key0, uint16_t *const target) {
    uint32_t k = 0;
    const buffer_handle_t h = buffer_queue_take(&st->queue, (uint32_t)__ticks_ms(), NULL, &k);
    if (h == BUFFER_NONE)
        return false;
    const uint8_t *const data = buffer_data(st->pool, h);
    if (key0 != NULL)
        *key0 = data != NULL ? data[0] : 0;
    if (target != NULL)
        *target = (uint16_t)k;
    buffer_unref(st->pool, h);
    return true;
}

static const uint8_t *ctrl_staged_bytes(const ctrl_state_t *const st) {
    static const uint8_t none[8] = { 0 }; /* so a FAIL print on an empty queue is not a crash */
    const buffer_handle_t h = buffer_queue_peek(&st->queue, (uint32_t)__ticks_ms(), NULL, NULL);
    const uint8_t *const p = h != BUFFER_NONE ? buffer_data(st->pool, h) : NULL;
    return p ? p : none;
}

static void ctrl_test_drop(ctrl_state_t *const st) {
    buffer_queue_clear(&st->queue);
}

static void ctrl_test_feed(ctrl_state_t *const st, const char *const json) {
    g_ctrl = st;
    ctrl_on_message("test/manage/req", (const unsigned char *)json, (int)strlen(json));
}

static bool test_ctrl_commands_to_control(void) {
    static const struct {
        const char *json;
        uint8_t expect[8];
        int expect_len;
        uint16_t target;
    } cases[] = {
        /* mesh management: the old MANAGE vocabulary, now CONTROL keys off MESH_BASE */
        { "{\"cmd\":\"status\",\"target\":\"0x123\"}", { 0x20, 0x01, 0x02 }, 3, 0x123 },
        /* The old table names printed to the node's console, and still do -- which is now the DUMP
           key, not the REQUEST key, because a request is answered by a report. The behaviour is
           what was preserved; the key moved underneath it. */
        { "{\"cmd\":\"stations\",\"target\":\"0x123\"}", { 0x41, 0x00 }, 2, 0x123 },
        { "{\"cmd\":\"peers\",\"target\":\"0x123\"}", { 0x4B, 0x00 }, 2, 0x123 },
        { "{\"cmd\":\"peers-remove\",\"station\":\"0x456\",\"target\":\"0x123\"}", { 0x49, 0x03, 0x04, 0x56, 0x00 }, 5, 0x123 },
        { "{\"cmd\":\"peers-clear\",\"target\":\"0x123\"}", { 0x4A, 0x00 }, 2, 0x123 },
        { "{\"cmd\":\"flush\",\"target\":\"0x123\"}", { 0x4A, 0x00 }, 2, 0x123 },
        { "{\"cmd\":\"filters\",\"target\":\"0x123\"}", { 0x53, 0x00 }, 2, 0x123 },
        { "{\"cmd\":\"block\",\"station\":\"0x456\",\"target\":\"0x123\"}", { 0x51, 0x03, 0x04, 0x56, 0x01 }, 5, 0x123 },
        { "{\"cmd\":\"allow\",\"station\":\"0x456\",\"target\":\"0x123\"}", { 0x51, 0x03, 0x04, 0x56, 0x02 }, 5, 0x123 },
        { "{\"cmd\":\"unfilter\",\"station\":\"0x456\",\"target\":\"0x123\"}", { 0x51, 0x03, 0x04, 0x56, 0x00 }, 5, 0x123 },
        { "{\"cmd\":\"filter-clear\",\"target\":\"0x123\"}", { 0x52, 0x01, 0x00 }, 3, 0x123 },
        { "{\"cmd\":\"filter-clear\",\"scope\":\"manual\",\"target\":\"0x123\"}", { 0x52, 0x01, 0x01 }, 3, 0x123 },
        { "{\"cmd\":\"filter-clear\",\"scope\":\"auto\",\"target\":\"0x123\"}", { 0x52, 0x01, 0x02 }, 3, 0x123 },
        /* diagnostics: generic node commands, not mesh ones -- one vocabulary for every node */
        { "{\"cmd\":\"diag\",\"target\":\"0x123\"}", { 0x30, 0x00 }, 2, 0x123 },
        { "{\"cmd\":\"diag-enable\",\"target\":\"0x123\"}", { 0x31, 0x01, 0x01 }, 3, 0x123 },
        { "{\"cmd\":\"diag-disable\",\"target\":\"0x123\"}", { 0x31, 0x01, 0x00 }, 3, 0x123 },
        { "{\"cmd\":\"diag-clear\",\"target\":\"0x123\"}", { 0x32, 0x00 }, 2, 0x123 },
        { "{\"cmd\":\"diag-dump\",\"target\":\"0x123\"}", { 0x33, 0x00 }, 2, 0x123 },
        /* node: the "node-<tlv>" path, which built CONTROL payloads all along */
        { "{\"cmd\":\"node-status\",\"target\":\"0x123\"}", { 0x20, 0x00 }, 2, 0x123 },
        { "{\"cmd\":\"node-version\",\"target\":\"0x123\"}", { 0x08, 0x00 }, 2, 0x123 },
        { "{\"cmd\":\"node-diagnostics\",\"target\":\"0x123\"}", { 0x30, 0x00 }, 2, 0x123 },
    };

    ctrl_state_t st;
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        ctrl_test_state(&st);
        ctrl_test_feed(&st, cases[i].json);
        if (!ctrl_staged(&st) || ctrl_staged_len(&st) != cases[i].expect_len || memcmp(ctrl_staged_bytes(&st), cases[i].expect, (size_t)cases[i].expect_len) != 0 || ctrl_staged_target(&st) != cases[i].target) {
            printf("FAIL (case %zu: %s -> ", i, cases[i].json);
            for (int k = 0; k < ctrl_staged_len(&st); k++)
                printf("%02X ", ctrl_staged_bytes(&st)[k]);
            printf("target=%04X)\n", (unsigned)ctrl_staged_target(&st));
            ctrl_test_drop(&st);
            return false;
        }
        ctrl_test_drop(&st);
    }
    return true;
}

/* "status" asks for the mesh group only; "node-status" (no value) means every group. The two are
   deliberately different -- the old MANAGE status was a mesh-management command. */
static bool test_ctrl_status_scope_differs_from_node_status(void) {
    ctrl_state_t st;

    ctrl_test_state(&st);
    ctrl_test_feed(&st, "{\"cmd\":\"status\",\"target\":\"0x123\"}");
    ASSERT(ctrl_staged(&st));
    ASSERT_EQ_INT(ctrl_staged_bytes(&st)[0], IOTDATA_NODE_CONTROL_STATUS_REQUEST);
    ASSERT_EQ_INT(ctrl_staged_bytes(&st)[1], 1);
    ASSERT_EQ_INT(ctrl_staged_bytes(&st)[2], IOTDATA_NODE_STATUS_SCOPE_MESH);
    ctrl_test_drop(&st); /* ctrl_test_state below memsets the slot: give this one back first */

    ctrl_test_state(&st);
    ctrl_test_feed(&st, "{\"cmd\":\"node-status\",\"target\":\"0x123\"}");
    ASSERT(ctrl_staged(&st));
    ASSERT_EQ_INT(ctrl_staged_bytes(&st)[0], IOTDATA_NODE_CONTROL_STATUS_REQUEST);
    ASSERT_EQ_INT(ctrl_staged_bytes(&st)[1], 0); /* no scope value = all groups */
    ctrl_test_drop(&st);
    return true;
}

/* A station-taking command carries { u16 station, u8 action } -- a list, so one request can change
   several stations; this gateway sends a list of one. */
static bool test_ctrl_mesh_update_entry_shape(void) {
    ctrl_state_t st;
    ctrl_test_state(&st);
    ctrl_test_feed(&st, "{\"cmd\":\"block\",\"station\":\"0xABC\",\"target\":\"0x123\"}");
    ASSERT(ctrl_staged(&st));
    ASSERT_EQ_INT(ctrl_staged_len(&st), 2 + IOTDATA_NODE_CONTROL_MESH_UPDATE_ENTRY_SIZE);
    ASSERT_EQ_INT(ctrl_staged_bytes(&st)[0], IOTDATA_NODE_CONTROL_MESH_FILTERS_UPDATE);
    ASSERT_EQ_INT(ctrl_staged_bytes(&st)[1], IOTDATA_NODE_CONTROL_MESH_UPDATE_ENTRY_SIZE);
    ASSERT_EQ_INT((uint16_t)((ctrl_staged_bytes(&st)[2] << 8) | ctrl_staged_bytes(&st)[3]), 0x0ABC);
    ASSERT_EQ_INT(ctrl_staged_bytes(&st)[4], IOTDATA_NODE_CONTROL_MESH_FILTERS_BLOCK);
    ctrl_test_drop(&st);
    return true;
}

/* The whole point of one response topic: every producer must land on it. node_begin and
   ctrl_begin build the string independently, so this is the invariant that would quietly regress
   if either grew its own suffix again. */
static bool test_ctrl_response_topic_is_shared(void) {
    node_state_t ns;
    memset(&ns, 0, sizeof(ns));
    stat_state_t ss;
    memset(&ss, 0, sizeof(ss));
    bbox_state_t bb;
    memset(&bb, 0, sizeof(bb));
    ASSERT(node_begin(&ns, 0x0001, "test", &ss, &bb, &t_pool, test_packet_handler, NULL, NULL, NULL, NULL, NULL, 0, "pre"));

    ctrl_state_t st;
    ctrl_test_state(&st);
    /* ctrl_begin sets the topics before it subscribes, so the (expected) subscribe failure with no
       broker does not stop us checking them */
    (void)ctrl_begin(&st, "pre", 0x0001, &bb, &t_pool);

    ASSERT(strcmp(st.topic_req, "pre/manage/req") == 0);
    ASSERT(strcmp(st.topic_resp, "pre/manage/resp") == 0);
    ASSERT(strcmp(ns.topic_resp, st.topic_resp) == 0);
    return true;
}

static bool test_ctrl_unknown_command_stages_nothing(void) {
    ctrl_state_t st;
    ctrl_test_state(&st);
    ctrl_test_feed(&st, "{\"cmd\":\"no-such-command\",\"target\":\"0x123\"}");
    ASSERT(!ctrl_staged(&st));
    ASSERT_EQ_INT(st.stat_req_bad, 1u);
    ASSERT_EQ_INT(st.stat_req_rx, 1u);
    return true;
}

static bool test_ctrl_bad_json_stages_nothing(void) {
    ctrl_state_t st;
    ctrl_test_state(&st);
    ctrl_test_feed(&st, "{not json");
    ASSERT(!ctrl_staged(&st));
    ASSERT_EQ_INT(st.stat_req_bad, 1u);
    return true;
}

/* target absent means broadcast, and it reaches the staged payload -- ctrl_tick() needs it to
   decide between executing locally and airing a DOWN frame. */
static bool test_ctrl_target_defaults_to_broadcast(void) {
    ctrl_state_t st;
    ctrl_test_state(&st);
    ctrl_test_feed(&st, "{\"cmd\":\"peers\"}");
    ASSERT(ctrl_staged(&st));
    ASSERT_EQ_INT(ctrl_staged_target(&st), IOTDATA_STATION_BROADCAST);
    ctrl_test_drop(&st);
    return true;
}

/* Requests queue rather than superseding each other, and come back out in the order they arrived.
   The single slot this replaced delivered only the LAST one, silently losing every command an
   operator sent between two ticks of the loop -- and they are not interchangeable, because each
   one can name a different station. */
static bool test_ctrl_requests_queue_in_order(void) {
    ctrl_state_t st;
    ctrl_test_state(&st);
    ctrl_test_feed(&st, "{\"cmd\":\"peers\",\"target\":\"0x123\"}");
    ctrl_test_feed(&st, "{\"cmd\":\"filters\",\"target\":\"0x456\"}");
    ASSERT_EQ_INT(buffer_queue_count(&st.queue), 2u);
    ASSERT_EQ_INT(buffer_queue_rejected(&st.queue), 0u);

    uint8_t key = 0;
    uint16_t target = 0;
    ASSERT(ctrl_test_take(&st, &key, &target));
    ASSERT_EQ_INT(key, IOTDATA_NODE_CONTROL_MESH_PEERS_DUMP /* `peers` dumps */);
    ASSERT_EQ_INT(target, 0x123); /* the target travels with each one, not just with the last */
    ASSERT(ctrl_test_take(&st, &key, &target));
    ASSERT_EQ_INT(key, IOTDATA_NODE_CONTROL_MESH_FILTERS_DUMP /* `filters` dumps */);
    ASSERT_EQ_INT(target, 0x456);
    ASSERT(!ctrl_test_take(&st, NULL, NULL));
    return true;
}

/* The queue is bounded, so a burst cannot eat the pool the receive path draws from. Past the
   bound the operator is TOLD ("busy"), which a superseding slot could never do. */
static bool test_ctrl_queue_full_is_refused(void) {
    ctrl_state_t st;
    ctrl_test_state(&st);
    for (int i = 0; i < CTRL_QUEUE_MAX + 2; i++)
        ctrl_test_feed(&st, "{\"cmd\":\"peers\",\"target\":\"0x123\"}");
    ASSERT_EQ_INT(buffer_queue_count(&st.queue), CTRL_QUEUE_MAX);
    ASSERT_EQ_INT(buffer_queue_rejected(&st.queue), 2u);
    ASSERT_EQ_INT(st.stat_req_bad, 2u); /* each refusal answered, not dropped on the floor */
    ctrl_test_drop(&st);
    return true;
}

/* A command staged while nothing could send it must go stale rather than go out minutes late: a
   mesh command is relative to state the node has since moved on from. */
static bool test_ctrl_staged_command_expires(void) {
    ctrl_state_t st;
    ctrl_test_state(&st);
    ctrl_test_feed(&st, "{\"cmd\":\"peers\",\"target\":\"0x123\"}");
    ASSERT_EQ_INT(buffer_queue_count(&st.queue), 1u);
    const uint32_t now_ms = (uint32_t)__ticks_ms();
    ASSERT_EQ_INT(buffer_queue_expire(&st.queue, now_ms + CTRL_QUEUE_TTL_MS - 1000), 0u); /* not yet */
    ASSERT_EQ_INT(buffer_queue_expire(&st.queue, now_ms + CTRL_QUEUE_TTL_MS + 1000), 1u);
    ASSERT_EQ_INT(buffer_queue_count(&st.queue), 0u);
    ASSERT_EQ_INT(buffer_queue_expired(&st.queue), 1u);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// The gateway as a mesh node: it answers the same mesh questions a relay does, and acts on the
// same commands. These drive the real hooks (exec_node_control / exec_node_status_mesh), which
// reach their state through g_exec exactly as the live gateway's do.
// -----------------------------------------------------------------------------------------------------------------------------------------

static void mesh_node_test_state(process_state_t *ps, mesh_state_t *ms, stat_state_t *ss) {
    fwd_test_state(ps, ms, ss, true, true);
    filter_init(&ps->filter);
    g_exec = ps;
}

/* A gateway reports the mesh group, and reports itself as the ROOT: state GATEWAY, cost 0, no
   parent. That is the asymmetry the wire format is designed to carry, rather than the gateway
   simply having no answer. */
static bool test_mesh_node_gateway_status_is_root(void) {
    process_state_t ps;
    mesh_state_t ms;
    stat_state_t ss;
    mesh_node_test_state(&ps, &ms, &ss);
    ms.beacon_generation = 7;
    ms.stat_beacons_tx = 11;
    ms.ctrl[IOTDATA_MESH_CTRL_BEACON].rx = 5;
    stat_on_peer(&ss, 0x05BF, 1, 1, 0);
    stat_on_peer(&ss, 0x0F1B, 1, 1, 0);

    iotdata_node_status_mesh_t m;
    memset(&m, 0, sizeof(m));
    exec_node_status_mesh(&m);
    ASSERT(m.present);
    ASSERT_EQ_INT(m.state, IOTDATA_NODE_STATUS_MESH_STATE_GATEWAY);
    ASSERT_EQ_INT(m.cost, 0);
    ASSERT_EQ_INT(m.parent, 0);
    ASSERT_EQ_INT(m.generation, 7);
    ASSERT_EQ_INT(m.beacon_tx, 11u);
    ASSERT_EQ_INT(m.beacon_rx, 5u);
    ASSERT_EQ_INT(m.peers, 2);
    ASSERT_EQ_INT(m.reparent, 0u); /* a root never reparents: zero is the answer, not a gap */
    ASSERT_EQ_INT(m.failover, 0u);
    ASSERT_EQ_INT(m.orphan, 0u);
    g_exec = NULL;
    return true;
}

/* With the mesh off there is no group to report -- absent, not zeroed. */
static bool test_mesh_node_status_absent_when_mesh_off(void) {
    process_state_t ps;
    mesh_state_t ms;
    stat_state_t ss;
    mesh_node_test_state(&ps, &ms, &ss);
    ms.enabled = false;

    iotdata_node_status_mesh_t m;
    memset(&m, 0, sizeof(m));
    exec_node_status_mesh(&m);
    ASSERT(!m.present);
    g_exec = NULL;
    return true;
}

/* The command a relay answers, answered the same way here -- and it must actually take effect on
   the receive path, which is the whole point of the gateway having a filter at all. */
static bool test_mesh_node_filter_update_blocks_rx(void) {
    process_state_t ps;
    mesh_state_t ms;
    stat_state_t ss;
    mesh_node_test_state(&ps, &ms, &ss);

    ASSERT(filter_allows(&ps.filter, 0x0537)); /* nothing filtered yet */

    const uint8_t block[IOTDATA_NODE_CONTROL_MESH_UPDATE_ENTRY_SIZE] = { 0x05, 0x37, IOTDATA_NODE_CONTROL_MESH_FILTERS_BLOCK };
    ASSERT(exec_node_control(IOTDATA_NODE_CONTROL_MESH_FILTERS_UPDATE, block, (uint8_t)sizeof(block)));
    ASSERT(!filter_allows(&ps.filter, 0x0537)); /* blocked */
    ASSERT(filter_allows(&ps.filter, 0x0538));  /* and only that one */
    ASSERT_EQ_INT(filter_count(&ps.filter), 1);

    /* NONE removes it: there is no separate remove command, by design */
    const uint8_t none[IOTDATA_NODE_CONTROL_MESH_UPDATE_ENTRY_SIZE] = { 0x05, 0x37, IOTDATA_NODE_CONTROL_MESH_FILTERS_NONE };
    ASSERT(exec_node_control(IOTDATA_NODE_CONTROL_MESH_FILTERS_UPDATE, none, (uint8_t)sizeof(none)));
    ASSERT(filter_allows(&ps.filter, 0x0537));
    ASSERT_EQ_INT(filter_count(&ps.filter), 0);
    g_exec = NULL;
    return true;
}

/* Blocking a station also drops what we already believe about it, so the block takes effect on
   the current topology rather than only on future frames. */
static bool test_mesh_node_block_drops_peer(void) {
    process_state_t ps;
    mesh_state_t ms;
    stat_state_t ss;
    mesh_node_test_state(&ps, &ms, &ss);
    stat_on_peer(&ss, 0x05BF, 1, 1, 0);
    ASSERT_EQ_INT(ss.peers_count, 1);

    const uint8_t block[IOTDATA_NODE_CONTROL_MESH_UPDATE_ENTRY_SIZE] = { 0x05, 0xBF, IOTDATA_NODE_CONTROL_MESH_FILTERS_BLOCK };
    ASSERT(exec_node_control(IOTDATA_NODE_CONTROL_MESH_FILTERS_UPDATE, block, (uint8_t)sizeof(block)));
    ASSERT_EQ_INT(ss.peers_count, 0);
    g_exec = NULL;
    return true;
}

/* peers-update/NONE and peers-clear, and their idempotence: a second identical command is a
   no-op, which is what makes them safe to send over a link with no acknowledgement. */
static bool test_mesh_node_peers_update_and_clear(void) {
    process_state_t ps;
    mesh_state_t ms;
    stat_state_t ss;
    mesh_node_test_state(&ps, &ms, &ss);
    stat_on_peer(&ss, 0x05BF, 1, 1, 0);
    stat_on_peer(&ss, 0x0F1B, 1, 1, 0);
    ASSERT_EQ_INT(ss.peers_count, 2);

    const uint8_t forget[IOTDATA_NODE_CONTROL_MESH_UPDATE_ENTRY_SIZE] = { 0x05, 0xBF, IOTDATA_NODE_CONTROL_MESH_PEER_NONE };
    ASSERT(exec_node_control(IOTDATA_NODE_CONTROL_MESH_PEERS_UPDATE, forget, (uint8_t)sizeof(forget)));
    ASSERT_EQ_INT(ss.peers_count, 1);
    ASSERT(exec_node_control(IOTDATA_NODE_CONTROL_MESH_PEERS_UPDATE, forget, (uint8_t)sizeof(forget))); /* again */
    ASSERT_EQ_INT(ss.peers_count, 1);                                                                   /* still 1: idempotent */

    /* the hole a removal leaves must be reusable, or the table leaks slots */
    stat_on_peer(&ss, 0x0AF9, 1, 1, 0);
    ASSERT_EQ_INT(ss.peers_count, 2);

    ASSERT(exec_node_control(IOTDATA_NODE_CONTROL_MESH_PEERS_CLEAR, NULL, 0));
    ASSERT_EQ_INT(ss.peers_count, 0);
    ASSERT(exec_node_control(IOTDATA_NODE_CONTROL_MESH_PEERS_CLEAR, NULL, 0)); /* again */
    ASSERT_EQ_INT(ss.peers_count, 0);
    g_exec = NULL;
    return true;
}

/* A list can carry several stations, because the wire format takes one -- the gateway's own MQTT
   interface names one at a time, but a peer manager need not. */
static bool test_mesh_node_filter_update_takes_a_list(void) {
    process_state_t ps;
    mesh_state_t ms;
    stat_state_t ss;
    mesh_node_test_state(&ps, &ms, &ss);

    const uint8_t three[3 * IOTDATA_NODE_CONTROL_MESH_UPDATE_ENTRY_SIZE] = {
        0x05, 0x37, IOTDATA_NODE_CONTROL_MESH_FILTERS_BLOCK, 0x05, 0x38, IOTDATA_NODE_CONTROL_MESH_FILTERS_BLOCK, 0x0A, 0xF9, IOTDATA_NODE_CONTROL_MESH_FILTERS_ALLOW,
    };
    ASSERT(exec_node_control(IOTDATA_NODE_CONTROL_MESH_FILTERS_UPDATE, three, (uint8_t)sizeof(three)));
    ASSERT_EQ_INT(filter_count(&ps.filter), 3);
    ASSERT(!filter_allows(&ps.filter, 0x0537));
    ASSERT(!filter_allows(&ps.filter, 0x0538));
    ASSERT(filter_allows(&ps.filter, 0x0AF9));
    /* an allow entry exists, so a station with no entry at all is now excluded (whitelist) */
    ASSERT(!filter_allows(&ps.filter, 0x06ED));
    g_exec = NULL;
    return true;
}

/* A key the gateway does not implement must be refused, not silently swallowed -- that is how the
   node layer knows to count it unknown, and how a manager learns the node cannot do it. */
static bool test_mesh_node_unimplemented_key_refused(void) {
    process_state_t ps;
    mesh_state_t ms;
    stat_state_t ss;
    mesh_node_test_state(&ps, &ms, &ss);
    ASSERT(!exec_node_control(0x7F, NULL, 0));
    ASSERT(!exec_node_control(IOTDATA_NODE_CONTROL_REBOOT, NULL, 0)); /* the node layer's, not ours */
    g_exec = NULL;
    return true;
}

/* Whatever the hook implements has to appear in the CONTROL report, or a manager cannot discover
   it. The list is declared once and used for both, so this pins them together. */
static bool test_mesh_node_control_keys_are_all_handled(void) {
    process_state_t ps;
    mesh_state_t ms;
    stat_state_t ss;
    mesh_node_test_state(&ps, &ms, &ss);
    for (size_t i = 0; i < sizeof(exec_node_control_keys) / sizeof(exec_node_control_keys[0]); i++) {
        const uint8_t key = exec_node_control_keys[i];
        const uint8_t entry[IOTDATA_NODE_CONTROL_MESH_UPDATE_ENTRY_SIZE] = { 0x05, 0x37, 0x00 };
        if (!exec_node_control(key, entry, (uint8_t)sizeof(entry))) {
            printf("FAIL (advertised key 0x%02X is not handled)\n", key);
            g_exec = NULL;
            return false;
        }
    }
    g_exec = NULL;
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* The exact frames from a real split reception (2026-09-11): a 56-byte node VERSION report from
   relay 05BF, torn in half by the UART read gap on a USB dongle. Kept verbatim because a
   synthesised truncation would not have proved the interesting part -- that the FIRST piece
   decodes as far as its header and so looks like a real packet from a real station. */
static const uint8_t split_head[] = { 0x05, 0xBF, 0x00, 0x05, 0x40, 0x02, 0x35, 0x00, 0x0D, 0x64, 0x61, 0x66, 0x63, 0x31, 0x36, 0x38, 0x2D, 0x64, 0x69, 0x72, 0x74, 0x79, 0x02, 0x07, 0x65, 0x73, 0x70, 0x33, 0x32, 0x63, 0x33 };
static const uint8_t split_tail[] = { 0x0D, 0x69, 0x6F, 0x74, 0x64, 0x61, 0x74, 0x61, 0x5F, 0x72, 0x65, 0x6C, 0x61, 0x79, 0x06, 0x0C, 0x37, 0x30, 0x41, 0x46, 0x30, 0x39, 0x31, 0x35, 0x35, 0x32, 0x46, 0x30 };

/* Real telemetry, captured off the air 2026-09-11: a water_level packet from 0538 and a
   wind_station packet from 0AF9. Verbatim, because the point is what a REAL sensor sends -- a
   synthesised frame is exactly what failed to catch this. */
static const uint8_t telemetry_0538[] = { 0x35, 0x38, 0x00, 0x01, 0x2C, 0xC1, 0x68, 0x97, 0x3E };
static const uint8_t telemetry_0AF9[] = { 0x6A, 0xF9, 0x00, 0x1F, 0x2C, 0xD8, 0xEA, 0xBA, 0x73, 0x60, 0x04 };

/*
 * A packet may carry variant FIELDS, TLVs, or both -- so neither count is a validity test.
 *
 * The presence byte has a TLV flag precisely so the two can coexist: these captured sensor frames
 * carry fields and no TLV; a sleeping sensor's frame carries fields AND a RECEIVE TLV; the TSA
 * carries a proprietary TLV and no fields. All three are well-formed.
 *
 * A gate that required tlv_count > 0 therefore rejected every direct reception of the first shape,
 * while the identical bytes published normally when they arrived inside a relay's FORWARD, since
 * that path does not consult the gate. Every test at the time built its frames with
 * iotdata_encode_tlv(), so every test frame had a TLV and none of them could notice.
 */
static bool test_telemetry_without_tlvs_decodes(void) {
    process_state_t ps;
    memset(&ps, 0, sizeof(ps));

    iotdata_decoder_t d;
    memset(&d, 0, sizeof(d));
    ASSERT(iotdata_decode(telemetry_0538, sizeof(telemetry_0538), &d) == IOTDATA_OK);
    ASSERT_EQ_INT(d.tlv_count, 0); /* legitimate, and the property that broke the gate */
    ASSERT_EQ_INT(d.variant, 3);
    ASSERT_EQ_INT(d.station, 0x0538);
    ASSERT(process_packet_decodes(&ps, telemetry_0538, (int)sizeof(telemetry_0538)));

    memset(&d, 0, sizeof(d));
    ASSERT(iotdata_decode(telemetry_0AF9, sizeof(telemetry_0AF9), &d) == IOTDATA_OK);
    ASSERT_EQ_INT(d.tlv_count, 0);
    ASSERT_EQ_INT(d.variant, 6);
    ASSERT(process_packet_decodes(&ps, telemetry_0AF9, (int)sizeof(telemetry_0AF9)));
    return true;
}

/* The mirror shape, which the TSA sends: a variant and a proprietary TLV, no fields. It must pass
   the same gate -- a rule about either count would have rejected one shape or the other. */
static bool test_tlv_only_packet_decodes(void) {
    process_state_t ps;
    memset(&ps, 0, sizeof(ps));

    /* built exactly as tsa_encoding.h does: begin(variant) -> encode_tlv -> end */
    const uint8_t blob[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02 };
    uint8_t frame[TEST_FRAME_MAX];
    iotdata_encoder_t enc;
    size_t len = 0;
    ASSERT(iotdata_encode_begin(&enc, frame, sizeof(frame), 0 /* weather_station, as TSA uses */, 0x0A01, 7) == IOTDATA_OK);
    ASSERT(iotdata_encode_tlv(&enc, 0x20u /* TSA_IOTDATA_TLV_TYPE: application-defined */, blob, (uint8_t)sizeof(blob)) == IOTDATA_OK);
    ASSERT(iotdata_encode_end(&enc, &len) == IOTDATA_OK);

    iotdata_decoder_t d;
    memset(&d, 0, sizeof(d));
    ASSERT(iotdata_decode(frame, len, &d) == IOTDATA_OK);
    ASSERT_EQ_INT(d.tlv_count, 1);
    ASSERT(process_packet_decodes(&ps, frame, (int)len));
    return true;
}

/* And it must still publish: the gate is only worth having if what passes it is usable. */
static bool test_telemetry_decodes_to_json(void) {
    static iotdata_decode_to_json_scratch_t scratch;
    char *json = NULL;
    ASSERT(iotdata_decode_to_json(telemetry_0538, sizeof(telemetry_0538), &json, &scratch) == IOTDATA_OK);
    ASSERT(json != NULL);
    ASSERT(strstr(json, "\"variant\":3") != NULL);
    free(json);
    return true;
}

/* Neither half may pass the gate that guards the dedup ring. The head is the dangerous one: it
   peeks as station 05BF sequence 5, and before this gate existed it claimed that slot and made
   the relay's forward of the same report look like a duplicate -- so nothing was published at
   all, which is the opposite of what having a mesh is for. */
static bool test_split_frame_is_not_a_packet(void) {
    process_state_t ps;
    memset(&ps, 0, sizeof(ps));

    uint8_t variant = 0;
    uint16_t station = 0, sequence = 0;
    ASSERT(iotdata_peek(split_head, sizeof(split_head), &variant, &station, &sequence) == IOTDATA_OK);
    ASSERT_EQ_INT(station, 0x05BF); /* a real station: the header survived the tear */
    ASSERT_EQ_INT(sequence, 5);
    ASSERT(!process_packet_decodes(&ps, split_head, (int)sizeof(split_head)));

    /* the tail peeks as a nonsense station (its first bytes are a length prefix and 'i') */
    ASSERT(!process_packet_decodes(&ps, split_tail, (int)sizeof(split_tail)));
    return true;
}

/* The gate must not reject anything real. A well-formed packet of the same shape -- a system TLV
   carrying strings -- has to pass, or the fix would simply stop the gateway publishing. */
static bool test_whole_frame_still_decodes(void) {
    process_state_t ps;
    memset(&ps, 0, sizeof(ps));

    uint8_t kv[64];
    iotdata_kvr_t b;
    iotdata_kvr_init(&b, kv, sizeof(kv));
    iotdata_kvr_add_str(&b, IOTDATA_NODE_VERSION_FIRMWARE, "dafc168-dirty");
    iotdata_kvr_add_str(&b, IOTDATA_NODE_VERSION_PLATFORM, "esp32c3");
    iotdata_kvr_add_str(&b, IOTDATA_NODE_VERSION_APPLICATION, "iotdata_relay");
    ASSERT(!b.overflow);

    uint8_t frame[TEST_FRAME_MAX];
    iotdata_encoder_t enc;
    size_t len = 0;
    ASSERT(iotdata_encode_begin(&enc, frame, sizeof(frame), 0, 0x05BF, 5) == IOTDATA_OK);
    ASSERT(iotdata_encode_tlv(&enc, IOTDATA_NODE_TLV_VERSION, kv, (uint8_t)b.len) == IOTDATA_OK);
    ASSERT(iotdata_encode_end(&enc, &len) == IOTDATA_OK);
    ASSERT(len > sizeof(split_head)); /* the whole thing is longer than the piece that tore off */

    ASSERT(process_packet_decodes(&ps, frame, (int)len));

    /* and truncating that same good frame anywhere past the header must be caught */
    for (size_t cut = 5; cut < len; cut++)
        if (process_packet_decodes(&ps, frame, (int)cut)) {
            printf("FAIL (a %zu-byte truncation of a %zu-byte frame passed the gate)\n", cut, len);
            return false;
        }
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* The AUX drain wait is derived from the configured air rate, because air time is. Observed on a
   relay under load: forward retries filled the module, and a fixed 1000ms wait gave up and
   reported an error for ordinary back-pressure. Only DIP modules wait at all -- USB has no AUX --
   but the arithmetic is worth pinning wherever it can be run. */
static bool test_transmit_wait_scales_with_air_rate(void) {
    const uint16_t saved = _lora_rtc.config.air_data_rate;
    static const struct {
        uint16_t bps;
        uint32_t expect;
        const char *why;
    } cases[] = {
        { 2400, 1600, "the rate the mesh runs at: 800ms of air time, doubled" },
        { 1200, 3200, "half the rate, twice the wait" },
        { 300, 12800, "the slowest rate: where a fixed 1000ms was wrong by 6x" },
        { 9600, _LORA_TRANSMIT_WAIT_MIN_MS, "fast enough that the floor takes over" },
        { 62500, _LORA_TRANSMIT_WAIT_MIN_MS, "likewise" },
        { 0, _LORA_TRANSMIT_WAIT_MIN_MS, "unset: the floor, not a division by zero" },
        { 100, _LORA_TRANSMIT_WAIT_MAX_MS, "absurdly slow: the cap, not an unbounded block" },
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        _lora_rtc.config.air_data_rate = cases[i].bps;
        const uint32_t got = _lora_transmit_wait_ms();
        if (got != cases[i].expect) {
            printf("FAIL (%ubps -> %" PRIu32 "ms, expected %" PRIu32 ": %s)\n", (unsigned)cases[i].bps, got, cases[i].expect, cases[i].why);
            _lora_rtc.config.air_data_rate = saved;
            return false;
        }
    }
    /* monotonic: a slower rate must never wait less than a faster one */
    uint32_t prev = 0;
    static const uint16_t rates[] = { 62500, 38400, 19200, 9600, 4800, 2400, 1200, 300 };
    for (size_t i = 0; i < sizeof(rates) / sizeof(rates[0]); i++) {
        _lora_rtc.config.air_data_rate = rates[i];
        const uint32_t got = _lora_transmit_wait_ms();
        if (got < prev) {
            printf("FAIL (%ubps waits %" PRIu32 "ms, less than the faster rate's %" PRIu32 "ms)\n", (unsigned)rates[i], got, prev);
            _lora_rtc.config.air_data_rate = saved;
            return false;
        }
        prev = got;
    }
    _lora_rtc.config.air_data_rate = saved;
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// A node report has to reach the node topic by EITHER path.
//
// A station one hop away is heard directly; a station two hops away is only ever heard inside a
// relay's FORWARD. If the two paths do not both reach the node layer, a node's reports work or do
// not work depending on where it happens to sit in the tree -- which is how `node-control` came to
// work for 05BF (direct) and not for 0F1B (via 05BF).
// -----------------------------------------------------------------------------------------------------------------------------------------

/* Build a normal (non-DOWN) packet from `station` carrying one system TLV. */
static int build_report(uint8_t *out, size_t size, uint16_t station, uint16_t sequence, uint8_t type) {
    uint8_t kv[32];
    iotdata_kvr_t b;
    iotdata_kvr_init(&b, kv, sizeof(kv));
    iotdata_kvr_add_flag(&b, IOTDATA_NODE_CONTROL_VERSION_REQUEST);
    iotdata_kvr_add_flag(&b, IOTDATA_NODE_CONTROL_MESH_PEERS_CLEAR);
    iotdata_encoder_t enc;
    size_t len = 0;
    if (iotdata_encode_begin(&enc, out, size, 0, station, sequence) != IOTDATA_OK)
        return -1;
    if (iotdata_encode_tlv(&enc, type, kv, (uint8_t)b.len) != IOTDATA_OK)
        return -1;
    if (iotdata_encode_end(&enc, &len) != IOTDATA_OK)
        return -1;
    return (int)len;
}

/* The CONTROL report -- a node saying which commands it accepts -- must be published. It used to
   be skipped outright, so this is the case that could never work for any station. */
static bool test_control_report_is_published(void) {
    static node_state_t ns;
    static stat_state_t ss;
    memset(&ns, 0, sizeof(ns));
    memset(&ss, 0, sizeof(ss));
    ns.stat = &ss;
    ns.tx = test_packet_handler;
    iotdata_down_init(&ns.down, &t_pool);

    uint8_t frame[TEST_FRAME_MAX];
    const int n = build_report(frame, sizeof(frame), 0x0F1B, 5, IOTDATA_NODE_TLV_CONTROL);
    ASSERT(n > 0);
    ASSERT(node_on_packet(&ns, frame, (size_t)n, 0x0F1B));
    ASSERT_EQ_INT(ns.stat_rx, 1u);
    return true;
}

/* But our OWN command coming back off a relay's rebroadcast must not be: it is a DOWN frame, and
   republishing it would claim a node had reported something it was merely told. */
static bool test_down_echo_is_not_published(void) {
    static node_state_t ns;
    static stat_state_t ss;
    memset(&ns, 0, sizeof(ns));
    memset(&ss, 0, sizeof(ss));
    ns.stat = &ss;
    ns.tx = test_packet_handler;
    iotdata_down_init(&ns.down, &t_pool);

    uint8_t frame[TEST_FRAME_MAX];
    const int n = build_report(frame, sizeof(frame), 0x0F1B, IOTDATA_SEQUENCE_DOWN, IOTDATA_NODE_TLV_CONTROL);
    ASSERT(n > 0);
    ASSERT(!node_on_packet(&ns, frame, (size_t)n, 0x0F1B));
    ASSERT_EQ_INT(ns.stat_rx, 0u); /* nothing published */
    return true;
}

/* And the two-hop case: the same report arriving wrapped in a FORWARD. */
static bool test_forwarded_report_reaches_node_handler(void) {
    reset_test_helpers();
    static process_state_t ps;
    mesh_state_t ms;
    static stat_state_t ss;
    fwd_test_state(&ps, &ms, &ss, true, true /* new */);

    uint8_t inner[TEST_FRAME_MAX];
    const int inner_len = build_report(inner, sizeof(inner), 0x0F1B, 5, IOTDATA_NODE_TLV_CONTROL);
    ASSERT(inner_len > 0);
    uint8_t fwd[IOTDATA_MESH_FORWARD_HDR_SIZE + TEST_FRAME_MAX];
    const int fwd_len = iotdata_mesh_pack_forward(fwd, sizeof(fwd), 0x0F1B, 5, 7, inner, inner_len);
    ASSERT(fwd_len > 0);

    process_mesh_packet(&ps, fwd, fwd_len, 15, 0x05BF, 32, "test", 0);
    ASSERT_EQ_INT(ms.stat_forwards_unwrapped, 1u);
    ASSERT_EQ_INT(fwd_test_node.stat_rx, 1u); /* the inner report reached the node layer */
    return true;
}

/* A duplicate forward must not republish -- the report goes out once however many relays carry it. */
static bool test_forwarded_report_duplicate_not_republished(void) {
    reset_test_helpers();
    static process_state_t ps;
    mesh_state_t ms;
    static stat_state_t ss;
    fwd_test_state(&ps, &ms, &ss, true, false /* duplicate */);

    uint8_t inner[TEST_FRAME_MAX];
    const int inner_len = build_report(inner, sizeof(inner), 0x0F1B, 5, IOTDATA_NODE_TLV_CONTROL);
    uint8_t fwd[IOTDATA_MESH_FORWARD_HDR_SIZE + TEST_FRAME_MAX];
    const int fwd_len = iotdata_mesh_pack_forward(fwd, sizeof(fwd), 0x0F1B, 5, 7, inner, inner_len);

    process_mesh_packet(&ps, fwd, fwd_len, 15, 0x05BF, 32, "test", 0);
    ASSERT_EQ_INT(ms.stat_duplicates, 1u);
    ASSERT_EQ_INT(fwd_test_node.stat_rx, 0u);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* The four-letter subject names, which match the device's own USB CLI so one vocabulary serves
   both interfaces. The old names remain as aliases and are covered by the table test above. */
static bool test_ctrl_short_names(void) {
    static const struct {
        const char *cmd;
        uint8_t expect[8];
        int expect_len;
    } cases[] = {
        { "{\"cmd\":\"vers\",\"target\":\"0x123\"}", { 0x08, 0x00 }, 2 },
        { "{\"cmd\":\"vari\",\"target\":\"0x123\"}", { 0x10, 0x00 }, 2 },
        { "{\"cmd\":\"ctrl\",\"target\":\"0x123\"}", { 0x18, 0x00 }, 2 },
        { "{\"cmd\":\"stat\",\"target\":\"0x123\"}", { 0x20, 0x00 }, 2 },                          /* no scope = all */
        { "{\"cmd\":\"stat\",\"scope\":\"mesh\",\"target\":\"0x123\"}", { 0x20, 0x01, 0x02 }, 3 }, /* wait: scope parses filter scopes */
        { "{\"cmd\":\"conf\",\"target\":\"0x123\"}", { 0x28, 0x00 }, 2 },
        { "{\"cmd\":\"diag\",\"target\":\"0x123\"}", { 0x30, 0x00 }, 2 },
        { "{\"cmd\":\"cont\",\"target\":\"0x123\"}", { 0x38, 0x00 }, 2 },
        { "{\"cmd\":\"boot\",\"target\":\"0x123\"}", { 0x00, 0x00 }, 2 },
        { "{\"cmd\":\"mesh-stations\",\"target\":\"0x123\"}", { 0x40, 0x00 }, 2 },
        { "{\"cmd\":\"mesh-peers\",\"target\":\"0x123\"}", { 0x48, 0x00 }, 2 },
        { "{\"cmd\":\"mesh-peers-clear\",\"target\":\"0x123\"}", { 0x4A, 0x00 }, 2 },
        { "{\"cmd\":\"mesh-filters\",\"target\":\"0x123\"}", { 0x50, 0x00 }, 2 },       /* a request: the report answers */
        { "{\"cmd\":\"mesh-stations-dump\",\"target\":\"0x123\"}", { 0x41, 0x00 }, 2 }, /* a dump: the node prints */
        { "{\"cmd\":\"mesh-peers-dump\",\"target\":\"0x123\"}", { 0x4B, 0x00 }, 2 },
        { "{\"cmd\":\"mesh-filters-dump\",\"target\":\"0x123\"}", { 0x53, 0x00 }, 2 },
        { "{\"cmd\":\"mesh-filters-update\",\"station\":\"0x456\",\"action\":\"block\",\"target\":\"0x123\"}", { 0x51, 0x03, 0x04, 0x56, 0x01 }, 5 },
        { "{\"cmd\":\"mesh-filters-update\",\"station\":\"0x456\",\"action\":\"allow\",\"target\":\"0x123\"}", { 0x51, 0x03, 0x04, 0x56, 0x02 }, 5 },
        { "{\"cmd\":\"mesh-filters-update\",\"station\":\"0x456\",\"action\":\"none\",\"target\":\"0x123\"}", { 0x51, 0x03, 0x04, 0x56, 0x00 }, 5 },
        { "{\"cmd\":\"mesh-peers-update\",\"station\":\"0x456\",\"action\":\"remove\",\"target\":\"0x123\"}", { 0x49, 0x03, 0x04, 0x56, 0x00 }, 5 },
        { "{\"cmd\":\"mesh-filter-block\",\"station\":\"0x456\",\"target\":\"0x123\"}", { 0x51, 0x03, 0x04, 0x56, 0x01 }, 5 },
        { "{\"cmd\":\"mesh-filter-none\",\"station\":\"0x456\",\"target\":\"0x123\"}", { 0x51, 0x03, 0x04, 0x56, 0x00 }, 5 },
        { "{\"cmd\":\"mesh-peers-remove\",\"station\":\"0x456\",\"target\":\"0x123\"}", { 0x49, 0x03, 0x04, 0x56, 0x00 }, 5 },
        { "{\"cmd\":\"mesh-filter-clear\",\"target\":\"0x123\"}", { 0x52, 0x01, 0x00 }, 3 }, /* width 1: the byte is sent */
    };
    ctrl_state_t st;
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        ctrl_test_state(&st);
        ctrl_test_feed(&st, cases[i].cmd);
        if (!ctrl_staged(&st) || ctrl_staged_len(&st) != cases[i].expect_len || memcmp(ctrl_staged_bytes(&st), cases[i].expect, (size_t)cases[i].expect_len) != 0) {
            printf("FAIL (%s -> ", cases[i].cmd);
            for (int k = 0; k < ctrl_staged_len(&st); k++)
                printf("%02X ", ctrl_staged_bytes(&st)[k]);
            printf(")\n");
            ctrl_test_drop(&st);
            return false;
        }
        ctrl_test_drop(&st);
    }
    return true;
}

/* Every row in the table has to produce a payload the key table agrees with: a key declaring a
   fixed width must carry exactly that many bytes, and a key declaring VARIABLE may carry none.
   This is the check that would have caught sending MESH_FILTER_CLEAR as a bare flag. */
static bool test_ctrl_table_matches_key_widths(void) {
    ctrl_state_t st;
    for (size_t i = 0; i < sizeof(ctrl_commands) / sizeof(ctrl_commands[0]); i++) {
        const ctrl_command_t *const c = &ctrl_commands[i];
        if (c->arg == CTRL_ARG_REPORTS)
            continue; /* not one key */
        ctrl_test_state(&st);
        char json[128];
        snprintf(json, sizeof(json), "{\"cmd\":\"%s\",\"station\":\"0x456\",\"target\":\"0x123\"}", c->name);
        ctrl_test_feed(&st, json);
        if (!ctrl_staged(&st)) {
            printf("FAIL (%s staged nothing)\n", c->name);
            return false;
        }
        if (ctrl_staged_bytes(&st)[0] != c->key) {
            printf("FAIL (%s emitted key %02X, table says %02X)\n", c->name, ctrl_staged_bytes(&st)[0], c->key);
            ctrl_test_drop(&st);
            return false;
        }
        const uint8_t vlen = ctrl_staged_bytes(&st)[1];
        const uint8_t declared = iotdata_node_tlv_key_width(IOTDATA_NODE_TLV_CONTROL, c->key);
        if (declared != IOTDATA_NODE_WIDTH_VARIABLE && vlen != declared) {
            printf("FAIL (%s: %u byte value, key %02X declares %u)\n", c->name, vlen, c->key, declared);
            ctrl_test_drop(&st);
            return false;
        }
        /* a key that declares a name must have one: an unnamed key is one nobody can decode */
        if (iotdata_node_tlv_key_name(IOTDATA_NODE_TLV_CONTROL, c->key) == NULL) {
            printf("FAIL (%s: key %02X is not in the key table at all)\n", c->name, c->key);
            ctrl_test_drop(&st);
            return false;
        }
        ctrl_test_drop(&st);
    }
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* A gateway is a mesh node and owes the same table answers a relay does. These drive its real
   table sources and then read the report back through the same accessors a reader would. */
static bool test_mesh_node_table_report(void) {
    process_state_t ps;
    mesh_state_t ms;
    static stat_state_t ss;
    mesh_node_test_state(&ps, &ms, &ss);
    netw_note_receive(&ps.network, 0x0537, 4, 1, NETW_PATH_DIRECT, true, -48, 0, time(NULL));
    netw_note_receive(&ps.network, 0x06ED, 0, 1, NETW_PATH_DIRECT, true, -55, 0, time(NULL));
    stat_on_peer(&ss, 0x05BF, 7, 1, IOTDATA_MESH_FLAG_ACCEPTING);
    ASSERT(exec_node_control(IOTDATA_NODE_CONTROL_MESH_FILTERS_UPDATE, (const uint8_t[]){ 0x0A, 0xF9, IOTDATA_NODE_CONTROL_MESH_FILTERS_BLOCK }, IOTDATA_NODE_CONTROL_MESH_UPDATE_ENTRY_SIZE));

    ASSERT_EQ_INT(exec_node_table_count(IOTDATA_NODE_TLV_MESH_STATIONS), 2);
    ASSERT_EQ_INT(exec_node_table_count(IOTDATA_NODE_TLV_MESH_PEERS), 1);
    ASSERT_EQ_INT(exec_node_table_count(IOTDATA_NODE_TLV_MESH_FILTERS), 1);

    /* a peer row, read back exactly as a reader would */
    uint8_t row[IOTDATA_NODE_TABLE_ROW_MAX];
    memset(row, 0, sizeof(row));
    ASSERT(exec_node_table_row(IOTDATA_NODE_TLV_MESH_PEERS, 0, row));
    ASSERT_EQ_INT(iotdata_node_table_get_u16(row, 0), 0x05BF);
    ASSERT_EQ_INT(iotdata_node_table_get_u16(row, 2), 0x0001); /* the gateway is its own gateway */
    ASSERT_EQ_INT(iotdata_node_table_get_u16(row, 5), 7);      /* generation */
    ASSERT_EQ_INT(row[10] & IOTDATA_NODE_TABLE_PEER_ACCEPTING, IOTDATA_NODE_TABLE_PEER_ACCEPTING);
    ASSERT_EQ_INT(row[10] & IOTDATA_NODE_TABLE_PEER_PARENT, 0); /* a root has no parent */

    /* a filter row */
    memset(row, 0, sizeof(row));
    ASSERT(exec_node_table_row(IOTDATA_NODE_TLV_MESH_FILTERS, 0, row));
    ASSERT_EQ_INT(iotdata_node_table_get_u16(row, 0), 0x0AF9);
    ASSERT_EQ_INT(row[2], IOTDATA_NODE_CONTROL_MESH_FILTERS_BLOCK);
    ASSERT_EQ_INT(row[3], IOTDATA_NODE_CONTROL_MESH_FILTERS_SCOPE_MANUAL);

    /* an index past the end is refused rather than returning a stale row */
    ASSERT(!exec_node_table_row(IOTDATA_NODE_TLV_MESH_PEERS, 5, row));
    ASSERT(!exec_node_table_row(IOTDATA_NODE_TLV_STATUS, 0, row)); /* not a table */
    g_exec = NULL;
    return true;
}

/* Removal leaves a hole, and a dense report index must skip it -- otherwise the Nth row is the Nth
   slot and a removed entry makes the report short or repeats one. */
static bool test_mesh_node_table_skips_holes(void) {
    process_state_t ps;
    mesh_state_t ms;
    static stat_state_t ss;
    mesh_node_test_state(&ps, &ms, &ss);
    stat_on_peer(&ss, 0x0501, 1, 1, 0);
    stat_on_peer(&ss, 0x0502, 1, 1, 0);
    stat_on_peer(&ss, 0x0503, 1, 1, 0);
    ASSERT(stat_mesh_peer_remove(&ss, 0x0502)); /* a hole in the middle */
    ASSERT_EQ_INT(exec_node_table_count(IOTDATA_NODE_TLV_MESH_PEERS), 2);

    uint8_t row[IOTDATA_NODE_TABLE_ROW_MAX];
    ASSERT(exec_node_table_row(IOTDATA_NODE_TLV_MESH_PEERS, 0, row));
    ASSERT_EQ_INT(iotdata_node_table_get_u16(row, 0), 0x0501);
    ASSERT(exec_node_table_row(IOTDATA_NODE_TLV_MESH_PEERS, 1, row));
    ASSERT_EQ_INT(iotdata_node_table_get_u16(row, 0), 0x0503); /* not the hole */
    ASSERT(!exec_node_table_row(IOTDATA_NODE_TLV_MESH_PEERS, 2, row));
    g_exec = NULL;
    return true;
}

/* A table report must render as decoded rows, not as strings. The generic renderer hands a value
   to iotdata_kvr_str, which stops at the first NUL -- and a row is full of them, so a row rendered
   that way loses almost everything. This is the end-to-end shape a reader actually receives. */
static bool test_table_report_json(void) {
    static node_state_t ns;
    static stat_state_t ss;
    memset(&ns, 0, sizeof(ns));
    memset(&ss, 0, sizeof(ss));
    ns.stat = &ss;
    ns.tx = test_packet_handler;
    iotdata_down_init(&ns.down, &t_pool);

    uint8_t kv[64];
    iotdata_kvr_t b;
    iotdata_kvr_init(&b, kv, sizeof(kv));
    uint8_t row[IOTDATA_NODE_TABLE_PEERS_ROW_SIZE];
    memset(row, 0, sizeof(row));
    iotdata_node_table_put_u16(row, 0, 0x0F1B);
    iotdata_node_table_put_u16(row, 2, 0x0001); /* a NUL byte, mid-row, on purpose */
    row[4] = 1;
    iotdata_node_table_put_u16(row, 5, 9);
    row[7] = (uint8_t)(int8_t)-42;
    iotdata_node_table_put_u16(row, 8, 3);
    row[10] = IOTDATA_NODE_TABLE_PEER_ACCEPTING;
    iotdata_kvr_add_u8(&b, IOTDATA_NODE_TABLE_COUNT, 1);
    iotdata_kvr_add(&b, IOTDATA_NODE_TABLE_ROW, row, (uint8_t)sizeof(row));

    ASSERT(iotdata_node_tlv_is_table(IOTDATA_NODE_TLV_MESH_PEERS));
    cJSON *const root = cJSON_CreateObject();
    ASSERT(root != NULL);
    cJSON *const data = cJSON_AddObjectToObject(root, "data");
    node_json_table(data, IOTDATA_NODE_TLV_MESH_PEERS, kv, b.len);
    char *const out = cJSON_PrintUnformatted(root);
    ASSERT(out != NULL);

    /* the whole table's count, and a row decoded into named fields */
    ASSERT(strstr(out, "\"count\":1") != NULL);
    ASSERT(strstr(out, "\"rows\":[") != NULL);
    ASSERT(strstr(out, "\"station\":\"0F1B\"") != NULL);
    ASSERT(strstr(out, "\"gateway\":\"0001\"") != NULL); /* survived the NUL */
    ASSERT(strstr(out, "\"cost\":1") != NULL);
    ASSERT(strstr(out, "\"rssi\":-42") != NULL);
    ASSERT(strstr(out, "\"accepting\":true") != NULL);
    ASSERT(strstr(out, "\"parent\":false") != NULL);
    free(out);
    cJSON_Delete(root);

    /* a row of the wrong width is dropped, not misread as another table's row */
    iotdata_kvr_init(&b, kv, sizeof(kv));
    iotdata_kvr_add_u8(&b, IOTDATA_NODE_TABLE_COUNT, 1);
    iotdata_kvr_add(&b, IOTDATA_NODE_TABLE_ROW, row, 4); /* a FILTERS-width row in a PEERS report */
    cJSON *const r2 = cJSON_CreateObject();
    cJSON *const d2 = cJSON_AddObjectToObject(r2, "data");
    node_json_table(d2, IOTDATA_NODE_TLV_MESH_PEERS, kv, b.len);
    char *const o2 = cJSON_PrintUnformatted(r2);
    ASSERT(strstr(o2, "\"rows\":[]") != NULL);
    free(o2);
    cJSON_Delete(r2);
    return true;
}

/* The hex fallback must not swallow the legitimate text values -- VERSION strings and blackbox
   records are the reason that branch exists. */
static bool test_text_values_still_render_as_strings(void) {
    static node_state_t ns;
    static stat_state_t ss;
    memset(&ns, 0, sizeof(ns));
    memset(&ss, 0, sizeof(ss));
    ns.stat = &ss;

    uint8_t kv[64];
    iotdata_kvr_t b;
    iotdata_kvr_init(&b, kv, sizeof(kv));
    iotdata_kvr_add_str(&b, IOTDATA_NODE_VERSION_FIRMWARE, "dafc168-dirty");
    iotdata_kvr_add_str(&b, IOTDATA_NODE_VERSION_PLATFORM, "esp32c3");

    cJSON *const root = cJSON_CreateObject();
    node_json_kvr(&ns, root, IOTDATA_NODE_TLV_VERSION, kv, b.len);
    char *const out = cJSON_PrintUnformatted(root);
    ASSERT(out != NULL);
    ASSERT(strstr(out, "\"firmware\":\"dafc168-dirty\"") != NULL);
    ASSERT(strstr(out, "\"platform\":\"esp32c3\"") != NULL);
    free(out);
    cJSON_Delete(root);
    return true;
}

int main(void) {
    setbuf(stdout, NULL);

    BUFFER_POOL_INIT(t_pool, TEST_POOL_COUNT, TEST_FRAME_MAX + 8, 0);

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

    printf("\n=== Node report routing Tests ===\n\n");

    RUN_TEST(control_report_is_published);
    RUN_TEST(down_echo_is_not_published);
    RUN_TEST(forwarded_report_reaches_node_handler);
    RUN_TEST(forwarded_report_duplicate_not_republished);

    printf("\n=== Radio timing Tests ===\n\n");

    RUN_TEST(transmit_wait_scales_with_air_rate);

    printf("\n=== Split-frame / dedup-poisoning Tests ===\n\n");

    RUN_TEST(telemetry_without_tlvs_decodes);
    RUN_TEST(tlv_only_packet_decodes);
    RUN_TEST(telemetry_decodes_to_json);
    RUN_TEST(split_frame_is_not_a_packet);
    RUN_TEST(whole_frame_still_decodes);

    printf("\n=== Gateway-as-mesh-node Tests ===\n\n");

    RUN_TEST(mesh_node_gateway_status_is_root);
    RUN_TEST(table_report_json);
    RUN_TEST(text_values_still_render_as_strings);
    RUN_TEST(mesh_node_table_report);
    RUN_TEST(mesh_node_table_skips_holes);
    RUN_TEST(mesh_node_status_absent_when_mesh_off);
    RUN_TEST(mesh_node_filter_update_blocks_rx);
    RUN_TEST(mesh_node_block_drops_peer);
    RUN_TEST(mesh_node_peers_update_and_clear);
    RUN_TEST(mesh_node_filter_update_takes_a_list);
    RUN_TEST(mesh_node_unimplemented_key_refused);
    RUN_TEST(mesh_node_control_keys_are_all_handled);

    printf("\n=== Ctrl Tests ===\n\n");

    RUN_TEST(ctrl_commands_to_control);
    RUN_TEST(ctrl_short_names);
    RUN_TEST(ctrl_table_matches_key_widths);
    RUN_TEST(ctrl_status_scope_differs_from_node_status);
    RUN_TEST(ctrl_mesh_update_entry_shape);
    RUN_TEST(ctrl_response_topic_is_shared);
    RUN_TEST(ctrl_unknown_command_stages_nothing);
    RUN_TEST(ctrl_bad_json_stages_nothing);
    RUN_TEST(ctrl_target_defaults_to_broadcast);
    RUN_TEST(ctrl_requests_queue_in_order);
    RUN_TEST(ctrl_queue_full_is_refused);
    RUN_TEST(ctrl_staged_command_expires);

    /* Every frame the suite built came from the pool, so every one must have come back. A leaked
       reference is invisible in a passing test and fatal in a gateway that has been up for days --
       this is the only place it gets caught cheaply. */
    printf("\n=== Buffers ===\n\n");
    printf("  pool: used=%u/%u, high_water=%u, acquires=%" PRIu32 ", failures=%" PRIu32 "\n", (unsigned)buffer_pool_used(&t_pool), (unsigned)buffer_pool_total(&t_pool), (unsigned)buffer_pool_high_water(&t_pool),
           buffer_pool_acquires(&t_pool), buffer_pool_fails(&t_pool));
    tests_run++;
    if (buffer_pool_used(&t_pool) == 0 && buffer_pool_fails(&t_pool) == 0) {
        tests_passed++;
        printf("  buffer_pool_no_leaks                                    PASS\n");
    } else
        printf("  buffer_pool_no_leaks                                    FAIL (%u still held, %" PRIu32 " failures)\n", (unsigned)buffer_pool_used(&t_pool), buffer_pool_fails(&t_pool));

    printf("\n=== Results ===\n\n");
    printf("  Total: %d, Passed: %d, Failed: %d\n\n", tests_run, tests_passed, tests_run - tests_passed);

    return tests_run > tests_passed ? EXIT_FAILURE : EXIT_SUCCESS;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
