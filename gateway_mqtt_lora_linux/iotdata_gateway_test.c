
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

#define TEST_FRAME_MAX          240
/* Set here, before the pool is declared, exactly as the app sets it before including the store:
   the down table's size is what the pool has to carry on top of the traffic. Kept small for the
   harness -- the gateway itself dimensions for a whole network. */
#define IOTDATA_NODE_DOWN_SLOTS 8
#define TEST_POOL_COUNT         (IOTDATA_NODE_DOWN_SLOTS + 8)
BUFFER_POOL_DECLARE(t_pool, TEST_POOL_COUNT, TEST_FRAME_MAX + 8);
#define PIN_DEVICE_UART_TX  GPIO_NUM_NC
#define PIN_DEVICE_UART_RX  GPIO_NUM_NC
#define PIN_DEVICE_LORA_AUX GPIO_NUM_NC
#define PIN_DEVICE_LORA_M0  GPIO_NUM_NC
#define PIN_DEVICE_LORA_M1  GPIO_NUM_NC
#include "d_interface_e22900t22.h"

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
#include "iotdata_node.h"
#include "iotdata_node_down.h" /* after iotdata_node.h: it reads RECEIVE advertisements */
#include "iotdata_node_utils.h"
#include "iotdata_node_version.h"
#include "iotdata_node_status.h"
#include "iotdata_node_config.h" /* the record helpers ctrl.h uses to build a config write */
#include "iotdata_node_control.h"

/* THE RECORDER, IN RAM. The gateway's own is FILE-backed out of its config; a suite must not touch
   a filesystem, so this one is the RAM ring -- which is what IOTDATA_NODE_DIAGNOSTICS 1 selects,
   and BLACKBOX_PERSIST says the same thing explicitly rather than by implication. */
#define IOTDATA_NODE_DIAGNOSTICS 1
#define BLACKBOX_PERSIST         BLACKBOX_PERSIST_NONE
#define IOTDATA_NODE_BLACKBOX_IMPLEMENTATION
#include "iotdata_node_diagnostics.h"
#include "iotdata_node_partial.h"
#include "iotdata_node_variant.h"

typedef struct {
    blackbox_handle_t handle;
    blackbox_config_t config;
    char pool[IOTDATA_NODE_BLACKBOX_POOL_SZ];
    char path[512];
} bbox_state_t;

/* The real state store, not a stub: mesh_begin registers a block in it, so a stub would be
   testing a different mesh_begin than the one that ships. STATE_MESH_TAG has to match what
   iotdata_gateway.c declares. No datastore behind it -- a store with no backing still accepts
   inserts and touches, which is all the mesh does with it. */
#include "device/d_module_datastore_linux.h"
#include "iotdata_node_state.h"
#include "iotdata_node_settings.h"
#include "iotdata_node_firmware.h" /* the node subject resolves ROLLBACK through it */
#include "iotdata_node_runtime.h"
#define STATE_NODE_TAG 0xB1E28004UL
/* The station's identity, shared by the mesh exactly as iotdata_gateway.c shares it. No datastore
   behind it: an unpersisted node still counts, it just starts from 0 after a restart. */
static iotdata_node_t t_inode;
/* The cross-gateway cases stand up several simulated gateways, but ddup identifies them by a plain
   gateway id rather than by a node -- it copies the id at begin(), because the election compares it
   on the receive thread. So there is no array of nodes here any more. */

#include "iotdata_node_stations.h"

/* THE CONFIG TABLE FIRST, exactly as iotdata_gateway.c orders it -- and the order is load-bearing,
   not cosmetic. The module headers below read settings through the typed accessors, and those do
   not exist until the table has been expanded: a header included before this point sees
   iotdata_node_config_u16() as an implicit declaration and the row names as undeclared identifiers.
   This block used to sit after them and only compiled while nothing above reached for a row. */
#define IOTDATA_NODE_CONSOLE_IMPLEMENTATION
#include "iotdata_node_console.h" /* before the table: its `conf` command compiles only if this is in */
#include "iotdata_gateway_conf.h"
#include "iotdata_node_config.h"
#include "iotdata_gateway_conf.h"

#include "iotdata_gateway_mesh.h"
#include "iotdata_gateway_ddup.h"
#include "iotdata_gateway_stat.h"
#include "iotdata_gateway_netw.h"
/* node/ctrl/exec are pulled in for the process_mesh_packet tests; the harness exercises only part
   of them, so their lifecycle entry points are legitimately unused here. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
/* the exec loop ticks presence, so the harness needs its declarations -- but only tick() is
   reached from here, and the lifecycle entry points belong to main */
#include "iotdata_gateway_peer.h"
#include "iotdata_gateway_node.h"
#include "iotdata_gateway_survey.h"
#include "iotdata_gateway_trace.h"
#include "iotdata_gateway_state.h"
/* EXPLICITLY, because no iotdata_node_* header includes another: the main TU gets these from
   iotdata_node_platform.h, and this suite does not include platform.h, so it names them itself.
   The LINUX filestore is here for its TYPES only -- iotdata_gateway_content.h is written against
   the directory backend -- while the store this suite actually runs is the RAM one below. */
#include "d_module_filestore.h" /* the seam first: iotdata_node_content.h is written against it */
#include "d_module_filestore_linux.h"
/* the OTA seams: iotdata_node_otaupdate.h assembles these, and every node that uses the
   framework has an update path, so every node includes them. None depends on an iotdata
   header, so they sit here in the d_ block with the rest. */
#include "d_module_otaslot_linux.h"
#include "d_module_inflate_linux.h"
#include "d_module_hpatch.h"
#include "iotdata_node_content.h"
#include "iotdata_node_content_ota.h"
#include "iotdata_node_content_control.h"
#include "iotdata_gateway_content.h"
#include "d_module_filestore_ram.h" /* the suite's content store: RAM, never a directory */
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

/* WIRING IS UNCONDITIONAL; `enabled` GATES USE, NOT SETUP. mesh_begin() attaches the handlers, the
   pool and the dedup ring whether or not the mesh is on -- which is what lets `mesh-enable` be
   flipped at runtime without a restart, because there is nothing left to set up when it flips.
   (It used to refuse to wire a disabled mesh, and that is what made the row a REBOOT row.) */
static bool test_mesh_begin_disabled(void) {
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.pool = &t_pool;
    ms.enabled = false;
    ASSERT(mesh_begin(&ms, &t_inode, &t_pool, test_packet_handler, test_dedup_handler, NULL));
    ASSERT(ms.packet_handler == test_packet_handler);
    ASSERT(ms.dedup_handler == test_dedup_handler);
    ASSERT(!ms.enabled); /* and wiring it did not turn it on */
    return true;
}

static bool test_mesh_begin_enabled(void) {
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.pool = &t_pool;
    ms.enabled = true;
    iotdata_node_init(&t_inode, 0x0042, NULL, 0u); /* fresh station + sequence for this case */
    ms.inode = &t_inode;
    ms.beacon_interval_s = 60;
    ASSERT(mesh_begin(&ms, &t_inode, &t_pool, test_packet_handler, test_dedup_handler, NULL));
    ASSERT(ms.packet_handler == test_packet_handler);
    ASSERT(ms.dedup_handler == test_dedup_handler);
    return true;
}

/* A MISSING PACKET HANDLER IS NOW A PROGRAMMING ERROR, not a runtime case: mesh_begin() asserts on
   it. There is no behaviour left to test -- a test that could observe it would have to survive an
   abort -- so the case that used to check for a graceful false is gone rather than disabled. */

// -----------------------------------------------------------------------------------------------------------------------------------------
// mesh_transmit_beacon
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool test_mesh_transmit_beacon_ok(void) {
    reset_test_helpers();
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.pool = &t_pool;
    ms.enabled = true;
    ms.beacon_interval_s = 90;                     /* a root HAS a cadence, and states it -- see I.1 */
    iotdata_node_init(&t_inode, 0x0042, NULL, 0u); /* fresh station + sequence for this case */
    ms.inode = &t_inode;
    ms.packet_handler = test_packet_handler;

    mesh_transmit_beacon(&ms);

    ASSERT_EQ_INT(packet_handler_calls, 1);
    ASSERT_EQ_INT(captured_lengths[0], IOTDATA_MESH_BEACON_SIZE);
    { /* and it is the root's OWN interval on the wire, where a relay can read it */
        iotdata_mesh_beacon_t sent;
        ASSERT(iotdata_mesh_unpack_beacon(captured_packets[0], captured_lengths[0], &sent));
        ASSERT_EQ_INT(sent.interval_s, 90);
    }
    ASSERT_EQ_INT(ms.stat_beacons_tx, 1u);
    ASSERT_EQ_INT(iotdata_node_sequence(&t_inode), 1u);

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
    iotdata_node_init(&t_inode, 0x0042, NULL, 0u); /* fresh station + sequence for this case */
    ms.inode = &t_inode;
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
    iotdata_node_init(&t_inode, 0x0001, NULL, 0u); /* fresh station + sequence for this case */
    ms.inode = &t_inode;
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
    iotdata_node_init(&t_inode, 0x0001, NULL, 0u); /* fresh station + sequence for this case */
    ms.inode = &t_inode;
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
    iotdata_node_init(&t_inode, 0x0001, NULL, 0u); /* fresh station + sequence for this case */
    ms.inode = &t_inode;
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
    iotdata_node_init(&t_inode, 0x0001, NULL, 0u); /* fresh station + sequence for this case */
    ms.inode = &t_inode;
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

/* The unwrapped inner reading goes through the admit point now, which asks ddup for a verdict --
   so a ddup state is not optional either, exactly as the node state above was not. Disabled, which
   makes ddup_admit() answer PUBLISH without touching a socket or a peer table. */
static ddup_state_t fwd_test_ddup;
static iotdata_mesh_dedup_ring_t fwd_test_ring;

static void fwd_test_state(process_state_t *ps, mesh_state_t *ms, stat_state_t *ss, bool enabled, bool dedup_says_new) {
    memset(ps, 0, sizeof(*ps));
    memset(&fwd_test_ddup, 0, sizeof(fwd_test_ddup));
    memset(&fwd_test_ring, 0, sizeof(fwd_test_ring));
    fwd_test_ddup.dedup_ring = &fwd_test_ring;
    ps->state_ddup = &fwd_test_ddup;
    memset(&fwd_test_node, 0, sizeof(fwd_test_node));
    fwd_test_node.stat = ss;
    fwd_test_node.pool = &t_pool;
    fwd_test_node.tx = test_packet_handler;
    iotdata_node_down_init(&fwd_test_node.down, &t_pool);
    ps->state_node = &fwd_test_node;
    ps->pool = &t_pool;
    memset(ms, 0, sizeof(*ms));
    memset(ss, 0, sizeof(*ss));
    ms->pool = &t_pool;
    ms->enabled = enabled;
    iotdata_node_init(&t_inode, 0x0001, NULL, 0u);
    ms->inode = &t_inode;
    ms->packet_handler = test_packet_handler;
    ms->dedup_handler = test_dedup_handler;
    dedup_handler_result = dedup_says_new;
    netw_begin(&ps->network);
    ps->state_mesh = ms;
    ps->state_stat = ss;
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
// mesh_receive_beacon / route_error
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
    ASSERT(mesh_receive_beacon_r(&ms, buf, len, &got));
    ASSERT_EQ_INT(got.gateway_id, b.gateway_id);
    /* this sender stated nothing, so the field carries 0 -- which means "I do not know the cadence",
       not "there is no field". A relay that has joined but not yet heard an interval of its own
       sends exactly this, and a reader has to tell it apart from a real cadence. */
    ASSERT_EQ_INT(got.interval_s, 0);

    /* THE ROOT'S CLOCK, STATED -- and MANDATORY. Every beacon is 11 bytes and a frame short of that
       is not a beacon, so "what cadence is this tree on" has an answer in every frame rather than
       one that depends on which build the sender happens to be. */
    {
        uint8_t wide[IOTDATA_MESH_BEACON_SIZE];
        iotdata_mesh_beacon_t withi = b;
        withi.interval_s = 600;
        ASSERT_EQ_INT(iotdata_mesh_pack_beacon(wide, sizeof(wide), &withi), IOTDATA_MESH_BEACON_SIZE);
        iotdata_mesh_beacon_t back;
        ASSERT(iotdata_mesh_unpack_beacon(wide, IOTDATA_MESH_BEACON_SIZE, &back));
        ASSERT_EQ_INT(back.interval_s, 600);
        ASSERT_EQ_INT(back.generation, b.generation); /* and nothing before it moved */
        ASSERT_EQ_INT(back.cost, b.cost);

        /* a frame one byte short is REFUSED rather than read with a truncated interval, and the
           packer will not emit a partial beacon into a buffer that cannot hold the whole thing */
        ASSERT(!iotdata_mesh_unpack_beacon(wide, IOTDATA_MESH_BEACON_SIZE - 1, &back));
        uint8_t narrow[IOTDATA_MESH_BEACON_SIZE - 1];
        ASSERT_EQ_INT(iotdata_mesh_pack_beacon(narrow, sizeof(narrow), &withi), 0);
    }
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

// -----------------------------------------------------------------------------------------------------------------------------------------
// mesh sequence counter
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool test_mesh_seq_increments(void) {
    reset_test_helpers();
    mesh_state_t ms;
    memset(&ms, 0, sizeof(ms));
    ms.pool = &t_pool;
    ms.enabled = true;
    iotdata_node_init(&t_inode, 0x0001, NULL, 0u); /* fresh station + sequence for this case */
    ms.inode = &t_inode;
    ms.packet_handler = test_packet_handler;

    mesh_transmit_beacon(&ms);
    ASSERT_EQ_INT(iotdata_node_sequence(&t_inode), 1u);

    mesh_transmit_ack(&ms, 0x0002, 1);
    ASSERT_EQ_INT(iotdata_node_sequence(&t_inode), 2u);

    mesh_transmit_beacon(&ms);
    ASSERT_EQ_INT(iotdata_node_sequence(&t_inode), 3u);
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

static bool test_ddup_peers_parse_mqtt_token(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    ds.port = 9876;
    ddup_peers_parse(&ds, "mqtt:eth0");
    ASSERT(ds.auto_peer);
    ASSERT(strcmp(ds.advertise_qualifier, "eth0") == 0);
    ASSERT_EQ_INT(ds.peers_count, 0); /* the mqtt token is not a peer */

    memset(&ds, 0, sizeof(ds));
    ds.port = 9876;
    ddup_peers_parse(&ds, "1.2.3.4:9876,mqtt"); /* hybrid: one static + auto */
    ASSERT(ds.auto_peer);
    ASSERT(strcmp(ds.advertise_qualifier, "") == 0);
    ASSERT_EQ_INT(ds.peers_count, 1);
    return true;
}

static int test_find_gwid(const ddup_state_t *ds, uint16_t gwid) {
    for (int i = 0; i < DDUP_PEERS_MAX; i++)
        if (ds->peers[i].used && ds->peers[i].is_auto && ds->peers[i].announced_gwid == gwid)
            return i;
    return -1;
}

static bool test_ddup_peer_insert_upsert(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    ds.port = 9876;
    pthread_mutex_init(&ds.mutex, NULL);

    ASSERT(ddup_peer_insert(&ds, 0x0002, "192.168.0.2", 9876, time(NULL)));
    ASSERT_EQ_INT(ds.peers_count, 1);
    int i2 = test_find_gwid(&ds, 0x0002);
    ASSERT(i2 >= 0 && ds.peers[i2].resolved && strcmp(ds.peers[i2].host, "192.168.0.2") == 0); /* literal IP resolves instantly */

    ASSERT(ddup_peer_insert(&ds, 0x0002, "192.168.0.99", 9876, time(NULL))); /* same gwid, new IP -> upsert in place */
    ASSERT_EQ_INT(ds.peers_count, 1);
    ASSERT(strcmp(ds.peers[test_find_gwid(&ds, 0x0002)].host, "192.168.0.99") == 0);

    ASSERT(ddup_peer_insert(&ds, 0x0003, "192.168.0.3", 9876, time(NULL))); /* different gwid -> new slot */
    ASSERT_EQ_INT(ds.peers_count, 2);

    ddup_peer_remove(&ds, 0x0002);
    ASSERT_EQ_INT(ds.peers_count, 1);
    ASSERT(test_find_gwid(&ds, 0x0002) < 0 && test_find_gwid(&ds, 0x0003) >= 0);

    pthread_mutex_destroy(&ds.mutex);
    return true;
}

// presence-ts TTL prunes a peer whose box/bridge died (LWT never arrived); legacy un-stamped peers don't.
static bool test_ddup_peer_expire(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    ds.port = 9876;
    pthread_mutex_init(&ds.mutex, NULL);

    const time_t now = time(NULL);
    ASSERT(ddup_peer_insert(&ds, 0x0002, "192.168.0.2", 9876, now));                          /* fresh */
    ASSERT(ddup_peer_insert(&ds, 0x0003, "192.168.0.3", 9876, now - DDUP_PEER_TTL_SECS - 5)); /* already stale */
    ASSERT(ddup_peer_insert(&ds, 0x0004, "192.168.0.4", 9876, 0));                            /* legacy, un-stamped */
    ASSERT_EQ_INT(ds.peers_count, 3);

    /* the stale one is pruned; the fresh and the legacy survive */
    ASSERT_EQ_INT(ddup_peers_expire(&ds, now), 1);
    ASSERT_EQ_INT(ds.peers_count, 2);
    ASSERT(test_find_gwid(&ds, 0x0003) < 0);
    ASSERT_EQ_INT((int)ds.stat_peers_expired, 1);

    /* far in the future the once-fresh peer ages out, but the legacy (ts==0) is NEVER expired */
    ASSERT_EQ_INT(ddup_peers_expire(&ds, now + 100000), 1);
    ASSERT(test_find_gwid(&ds, 0x0002) < 0 && test_find_gwid(&ds, 0x0004) >= 0);

    pthread_mutex_destroy(&ds.mutex);
    return true;
}

static bool test_ddup_reconfigure_preserves_auto(void) {
    ddup_state_t ds;
    memset(&ds, 0, sizeof(ds));
    ds.port = 9876;
    pthread_mutex_init(&ds.mutex, NULL);

    ddup_peers_parse(&ds, "10.0.0.1:9876");                         /* static */
    ddup_peer_insert(&ds, 0x0005, "192.168.0.5", 9876, time(NULL)); /* auto */
    ASSERT_EQ_INT(ds.peers_count, 2);

    ddup_reconfigure(&ds, "10.0.0.2:9876", 30, false, 0); /* new static set; auto must survive */
    ASSERT_EQ_INT(ds.peers_count, 2);

    int auto5 = 0, static2 = 0, static1 = 0;
    for (int i = 0; i < DDUP_PEERS_MAX; i++) {
        if (!ds.peers[i].used)
            continue;
        if (ds.peers[i].is_auto && ds.peers[i].announced_gwid == 0x0005)
            auto5 = 1;
        if (!ds.peers[i].is_auto && strcmp(ds.peers[i].host, "10.0.0.2") == 0)
            static2 = 1;
        if (!ds.peers[i].is_auto && strcmp(ds.peers[i].host, "10.0.0.1") == 0)
            static1 = 1;
    }
    ASSERT(auto5);    /* auto peer preserved */
    ASSERT(static2);  /* new static installed */
    ASSERT(!static1); /* old static removed */

    pthread_mutex_destroy(&ds.mutex);
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

// -----------------------------------------------------------------------------------------------------------------------------------------
// ddup_admit verdicts: lowest-publishes / static-peer-holds / dynamic-hold / early-drop
// -----------------------------------------------------------------------------------------------------------------------------------------

static void td_ddup_init(ddup_state_t *ds, iotdata_mesh_dedup_ring_t *ring, uint16_t gwid) {
    memset(ds, 0, sizeof(*ds));
    ds->enabled = true;
    ds->hold_ms = 1000; /* ceiling */
    ds->gateway_id = gwid;
    iotdata_mesh_dedup_init(ring);
    ds->dedup_ring = ring;
    pthread_mutex_init(&ds->mutex, NULL);
}
static void td_ddup_add_auto_peer(ddup_state_t *ds, int slot, uint16_t announced_gwid) {
    ds->peers[slot].used = true;
    ds->peers[slot].resolved = true;
    ds->peers[slot].is_auto = true;
    ds->peers[slot].announced_gwid = announced_gwid;
    ds->peers_count++;
}

static bool test_ddup_admit_lowest_publishes(void) {
    ddup_state_t ds;
    iotdata_mesh_dedup_ring_t ring;
    td_ddup_init(&ds, &ring, 0x001);      /* we are 0x001 */
    td_ddup_add_auto_peer(&ds, 0, 0x002); /* peer 0x002 is higher -> we are lowest */
    ASSERT(ddup_is_lowest(&ds));
    ASSERT(ddup_admit(&ds, 0x0042, 1) == DDUP_PUBLISH); /* lowest -> publish now, no hold */
    pthread_mutex_destroy(&ds.mutex);
    return true;
}

static bool test_ddup_admit_not_lowest_holds(void) {
    ddup_state_t ds;
    iotdata_mesh_dedup_ring_t ring;
    td_ddup_init(&ds, &ring, 0x002);      /* we are 0x002 */
    td_ddup_add_auto_peer(&ds, 0, 0x001); /* peer 0x001 is lower -> we are NOT lowest */
    ASSERT(!ddup_is_lowest(&ds));
    ASSERT(ddup_admit(&ds, 0x0042, 1) == DDUP_HOLD); /* not lowest -> hold */
    pthread_mutex_destroy(&ds.mutex);
    return true;
}

static bool test_ddup_admit_static_peer_holds(void) {
    ddup_state_t ds;
    iotdata_mesh_dedup_ring_t ring;
    td_ddup_init(&ds, &ring, 0x001); /* we'd be lowest, but... */
    ds.peers[0].used = true;
    ds.peers[0].resolved = true;
    ds.peers[0].is_auto = false; /* static peer: gateway-id unknown -> can't be confident */
    ds.peers_count++;
    ASSERT(!ddup_is_lowest(&ds));
    ASSERT(ddup_admit(&ds, 0x0042, 1) == DDUP_HOLD); /* unknown-id peer -> hold (safe) */
    pthread_mutex_destroy(&ds.mutex);
    return true;
}

static bool test_ddup_hold_ms_effective(void) {
    ddup_state_t ds;
    iotdata_mesh_dedup_ring_t ring;
    td_ddup_init(&ds, &ring, 0x002);
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    ds.claim_latency_count = 5; /* < MIN_SAMPLES -> stay at the ceiling */
    ds.claim_latency_max_ms = 100;
    ASSERT_EQ_INT(ddup_hold_ms_effective(&ds, &now), 1000u);
    ds.claim_latency_count = 50; /* enough, small max -> tighten to max*1.5+50 = 200 */
    ASSERT_EQ_INT(ddup_hold_ms_effective(&ds, &now), 200u);
    ds.claim_latency_max_ms = 5000; /* huge -> clamped to the ceiling */
    ASSERT_EQ_INT(ddup_hold_ms_effective(&ds, &now), 1000u);
    pthread_mutex_destroy(&ds.mutex);
    return true;
}

static bool test_ddup_hold_early_drop(void) {
    ddup_state_t ds;
    iotdata_mesh_dedup_ring_t ring;
    td_ddup_init(&ds, &ring, 0x002);
    const uint8_t pkt[4] = { 1, 2, 3, 4 };
    ASSERT(ddup_hold_add(&ds, pkt, 4, 1, 0x0042, 7, "iotdata/data", "direct", -73 /* this hop's rssi, carried through the hold */));
    int held = -1;
    for (int i = 0; i < DDUP_HOLD_MAX; i++)
        if (ds.hold[i].used) {
            held = i;
            break;
        }
    ASSERT(held >= 0);
    ASSERT_EQ_INT(ds.claim_latency_count, 0u);
    pthread_mutex_lock(&ds.mutex); /* mark_dropped requires the lock held (recv path) */
    ddup_hold_mark_dropped(&ds, 0x0042, 7);
    pthread_mutex_unlock(&ds.mutex);
    ASSERT(!ds.hold[held].used); /* early-drop: slot freed immediately */
    ASSERT_EQ_INT(ds.stat_dropped_peer, 1u);
    ASSERT_EQ_INT(ds.claim_latency_count, 1u); /* latency measured */
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
    bool is_new = iotdata_mesh_dedup_insert(sb.dedup_ring, 0x0042, 100);
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
    bool a_has_b = !iotdata_mesh_dedup_insert(sa.dedup_ring, 0x0043, 200);
    pthread_mutex_unlock(&sa.mutex);

    /* B should have A's entry */
    pthread_mutex_lock(&sb.mutex);
    bool b_has_a = !iotdata_mesh_dedup_insert(sb.dedup_ring, 0x0042, 100);
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
        bool is_new = iotdata_mesh_dedup_insert(states[i].dedup_ring, 0x0042, 500);
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
// Both tables are DENSE: entries live at [0..count-1] with no holes and carry no valid flag, so
// `count` is the scan bound everywhere. That invariant is not compiler-checkable -- count is
// hand-maintained wherever an entry is appended, recycled or removed -- so each test re-derives it
// the slow way (ASSERT_COUNT_*): every live entry names a station, no station appears twice, and
// nothing survives past count. The last of those is what catches a swap-remove that forgot to
// clear the tail it moved from. Station 0 is never assignable, so a zero id means "empty".
// A full table here evicts by overwrite, so the counts only ever rise to the cap; the one removal
// is stat_mesh_peer_remove(). The tables are large, hence the statics.
// -----------------------------------------------------------------------------------------------------------------------------------------

static bool netw_is_dense(const netw_t *n) {
    if (n->count < 0 || n->count > NETW_MAX)
        return false;
    for (int i = 0; i < n->count; i++) {
        if (n->s[i].station == 0)
            return false;
        for (int j = i + 1; j < n->count; j++)
            if (n->s[i].station == n->s[j].station)
                return false;
    }
    for (int i = n->count; i < NETW_MAX; i++)
        if (n->s[i].station != 0)
            return false;
    return true;
}
#define ASSERT_COUNT_NETW(n) ASSERT(netw_is_dense(n))

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
    (void)netw_upsert(&net, 0x222, 1001); /* appended after the first */
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
    for (int i = 0; i < net.count; i++)
        if (netw_locate(&net, net.s[i].station) == i)
            found++;
    ASSERT_EQ_INT(found, NETW_MAX);
    ASSERT_COUNT_NETW(&net);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* find_or_create() creates on miss, so it can never answer "is this present?" -- scan instead */
static bool stat_station_present(const stat_state_t *s, uint16_t station_id) {
    for (int i = 0; i < s->stations_count; i++)
        if (s->stations[i].station_id == station_id)
            return true;
    return false;
}

static bool stat_peer_present(const stat_state_t *s, uint16_t station_id) {
    for (int i = 0; i < s->peers_count; i++)
        if (s->peers[i].station_id == station_id)
            return true;
    return false;
}

/* The same dense audit as netw_is_dense(), for both of stat's tables -- but only up to the count:
   stat_mesh_peer_remove() and _peers_clear() deliberately do NOT clear what they vacate, since
   nothing ever reads past the count. What that gives up is the ability to spot a stale slot, so the
   duplicate check below is doing the work of catching a removal that forgot to decrement. */
static bool stat_is_dense(const stat_state_t *s) {
    if (s->stations_count < 0 || s->stations_count > STAT_STATIONS_MAX || s->peers_count < 0 || s->peers_count > STAT_MESH_PEERS_MAX)
        return false;
    for (int i = 0; i < s->stations_count; i++) {
        if (s->stations[i].station_id == 0)
            return false;
        for (int j = i + 1; j < s->stations_count; j++)
            if (s->stations[i].station_id == s->stations[j].station_id)
                return false;
    }
    for (int i = 0; i < s->peers_count; i++) {
        if (s->peers[i].station_id == 0)
            return false;
        for (int j = i + 1; j < s->peers_count; j++)
            if (s->peers[i].station_id == s->peers[j].station_id)
                return false;
    }
    return true;
}
#define ASSERT_COUNT_STAT(s) ASSERT(stat_is_dense(s))

/* THE SHAPE OF THE STATS DOCUMENT, which a generic reader walks by key: every collection is a
   bare array under a key that names it, and `mesh`/`ddup` are siblings of `stations` rather than
   nested inside it. The previous {count, entries} wrapper made all three collections identical,
   so anything flattening the document printed `entries` three times and said nothing. */
static bool test_stat_json_shape(void) {
    static stat_state_t s;
    memset(&s, 0, sizeof(s));
    iotdata_node_init(&t_inode, 0x0F1B, NULL, 0u);
    s.inode = &t_inode;
    s.version = "9.9.9";
    s.start_time = time(NULL) - 60;
    (void)stat_station_find_or_create(&s, 0x111, time(NULL));

    cJSON *const root = stat_build_stat_json(&s, NULL, NULL);
    ASSERT(root != NULL);
    char *const out = cJSON_PrintUnformatted(root);
    ASSERT(out != NULL);

    ASSERT(strstr(out, "\"station\":3867") != NULL); /* numeric, as everywhere in the JSON */
    ASSERT(strstr(out, "\"links\":[") != NULL);
    ASSERT(strstr(out, "\"stations\":[") != NULL);
    ASSERT(strstr(out, "\"variants\":[") != NULL);
    /* and NOT the doubled nesting, nor a count beside a thing that has a length */
    ASSERT(strstr(out, "\"entries\"") == NULL);
    ASSERT(strstr(out, "\"links\":{") == NULL && strstr(out, "\"stations\":{") == NULL);
    free(out);
    cJSON_Delete(root);
    return true;
}

static bool test_stat_station_create_and_find(void) {
    static stat_state_t s;
    memset(&s, 0, sizeof(s));
    stat_station_t *const a = stat_station_find_or_create(&s, 0x111, time(NULL));
    ASSERT(a != NULL && a->station_id == 0x111); /* create on an EMPTY table */
    ASSERT_EQ_INT(s.stations_count, 1);
    ASSERT(stat_station_find_or_create(&s, 0x111, time(NULL)) == a); /* find, not re-create */
    ASSERT_EQ_INT(s.stations_count, 1);                              /* must NOT double-count */
    (void)stat_station_find_or_create(&s, 0x222, time(NULL));        /* appended after the first */
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
    for (int i = 0; i < STAT_STATIONS_MAX; i++) {
        stat_station_t *const e = stat_station_find_or_create(&s, (uint16_t)(0x400 + i), time(NULL));
        e->last_seen = 9000 - i; /* DEscending: the stalest is the LAST slot, not the first, so an
                                    "always evict index 0" bug cannot pass by coincidence */
    }
    ASSERT_EQ_INT(s.stations_count, STAT_STATIONS_MAX);
    ASSERT_COUNT_STAT(&s);

    (void)stat_station_find_or_create(&s, 0xABC, time(NULL)); /* full -> evicts the stalest */
    ASSERT_EQ_INT(s.stations_count, STAT_STATIONS_MAX);       /* count saturates */
    ASSERT_COUNT_STAT(&s);
    ASSERT(stat_station_present(&s, 0xABC));
    ASSERT(!stat_station_present(&s, 0x400 + STAT_STATIONS_MAX - 1)); /* the stalest went */
    ASSERT(stat_station_present(&s, 0x400));                          /* the freshest stayed */
    ASSERT(stat_station_present(&s, 0x400 + STAT_STATIONS_MAX - 2));  /* and only one went */
    return true;
}

static bool test_stat_station_all_findable(void) {
    static stat_state_t s;
    memset(&s, 0, sizeof(s));
    for (int i = 0; i < STAT_STATIONS_MAX; i++)
        (void)stat_station_find_or_create(&s, (uint16_t)(0x600 + i), time(NULL));
    int found = 0; /* a scan bound that cut short would re-create instead of finding */
    for (int i = 0; i < s.stations_count; i++)
        if (stat_station_find_or_create(&s, s.stations[i].station_id, time(NULL)) == &s.stations[i])
            found++;
    ASSERT_EQ_INT(found, STAT_STATIONS_MAX);
    ASSERT_EQ_INT(s.stations_count, STAT_STATIONS_MAX);
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
    stat_on_peer(&s, 0xBBB, 1, 1, 0); /* second peer is appended */
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

/* The only removal in either table, and so the only way an entry can MOVE: the last peer is
   swapped into the hole. Three things can go wrong and none is visible from the count alone --
   the tail is left behind as a ghost, the moved entry arrives as an id without its payload, or
   removing the last entry (where the swap is a self-move) corrupts it. */
static bool test_stat_mesh_peer_remove_compacts(void) {
    static stat_state_t s;
    memset(&s, 0, sizeof(s));
    stat_on_peer(&s, 0xAAA, 1, 1, 0);
    stat_on_peer(&s, 0xBBB, 2, 2, 0);
    stat_on_peer(&s, 0xCCC, 3, 3, 0);
    ASSERT_EQ_INT(s.peers_count, 3);
    ASSERT_COUNT_STAT(&s);

    ASSERT(stat_mesh_peer_remove(&s, 0xBBB));
    ASSERT_EQ_INT(s.peers_count, 2);
    ASSERT_COUNT_STAT(&s); /* no ghost past the count */
    ASSERT(!stat_peer_present(&s, 0xBBB));
    ASSERT(stat_peer_present(&s, 0xAAA) && stat_peer_present(&s, 0xCCC));
    ASSERT_EQ_INT((int)s.peers[1].station_id, 0xCCC); /* moved down from the end */
    ASSERT_EQ_INT((int)s.peers[1].generation, 3);     /* the WHOLE entry moved, not just its id */
    ASSERT_EQ_INT((int)s.peers[1].cost, 3);

    ASSERT(!stat_mesh_peer_remove(&s, 0xBBB)); /* gone, and a second removal is a no-op */
    ASSERT_EQ_INT(s.peers_count, 2);

    ASSERT(stat_mesh_peer_remove(&s, 0xCCC)); /* the LAST entry: the swap is a self-move */
    ASSERT_EQ_INT(s.peers_count, 1);
    ASSERT_COUNT_STAT(&s);
    ASSERT(stat_peer_present(&s, 0xAAA));

    ASSERT(stat_mesh_peer_remove(&s, 0xAAA)); /* down to empty */
    ASSERT_EQ_INT(s.peers_count, 0);
    ASSERT_COUNT_STAT(&s);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------------------------------------
// Ctrl (MQTT -> node CONTROL) tests
//
// ctrl_from_message() runs on the mosquitto thread and only STAGES a payload; ctrl_tick() drains it
// on the main loop. These drive the real handler and inspect the staged bytes, which is where the
// mesh-management commands used to become MANAGE frames and now become CONTROL kvr entries.
// A kvr entry on the wire is { key, vlen, value... }.
// -----------------------------------------------------------------------------------------------------------------------------------------

#define CTRL_TEST_STATION 0x001 /* the gateway's own id: tests target 0x123 so the diag local path stays out of it */

static void ctrl_test_state(ctrl_state_t *const st) {
    memset(st, 0, sizeof(*st));
    iotdata_node_init(&t_inode, CTRL_TEST_STATION, NULL, 0u);
    st->inode = &t_inode;
    st->pool = &t_pool;
    buffer_queue_init(&st->queue, st->queue_slot, CTRL_QUEUE_MAX, &t_pool);
    g_ctrl = st;
}

/* These tests look at the head of the staging queue rather than draining it with ctrl_tick(), so
   each one hands the buffers back itself -- the pool leak check at the end of main is watching.
   peek() is the right tool here precisely because this queue has one thread in the test: the
   handle cannot go stale under us the way it could on a queue another thread takes from. */
static bool ctrl_staged(const ctrl_state_t *const st) {
    return buffer_queue_peek(&st->queue, hw_time_ms(), NULL, NULL) != BUFFER_NONE;
}

static uint16_t ctrl_staged_target(const ctrl_state_t *const st) {
    uint32_t key = 0;
    return buffer_queue_peek(&st->queue, hw_time_ms(), NULL, &key) != BUFFER_NONE ? (uint16_t)key : 0;
}

static uint16_t ctrl_staged_len(const ctrl_state_t *const st) {
    const buffer_handle_t h = buffer_queue_peek(&st->queue, hw_time_ms(), NULL, NULL);
    return h != BUFFER_NONE ? buffer_len(st->pool, h) : 0;
}

/* drain one the way ctrl_tick does, so a test can check what comes out and in what order */
/* Returns the ACTION of the staged command: every command shares the one CONTROL key now, so the
   key alone says nothing and the subject/action pair is what identifies it. Layout is
   [key][vlen][subject][action][args...]. */
static bool ctrl_test_take(ctrl_state_t *const st, uint8_t *const key0, uint16_t *const target) {
    uint32_t k = 0;
    const buffer_handle_t h = buffer_queue_take(&st->queue, hw_time_ms(), NULL, &k);
    if (h == BUFFER_NONE)
        return false;
    const uint8_t *const data = buffer_data(st->pool, h);
    if (key0 != NULL)
        *key0 = (data != NULL && data[1] >= 2) ? data[3] : 0;
    if (target != NULL)
        *target = (uint16_t)k;
    buffer_unref(st->pool, h);
    return true;
}

static const uint8_t *ctrl_staged_bytes(const ctrl_state_t *const st) {
    static const uint8_t none[8] = { 0 }; /* so a FAIL print on an empty queue is not a crash */
    const buffer_handle_t h = buffer_queue_peek(&st->queue, hw_time_ms(), NULL, NULL);
    const uint8_t *const p = h != BUFFER_NONE ? buffer_data(st->pool, h) : NULL;
    return p ? p : none;
}

static void ctrl_test_drop(ctrl_state_t *const st) {
    buffer_queue_clear(&st->queue);
}

static void ctrl_test_feed(ctrl_state_t *const st, const char *const json) {
    g_ctrl = st;
    ctrl_from_message("test/manage/req", (const unsigned char *)json, (int)strlen(json));
}

static const uint8_t telemetry_0538[] = { 0x35, 0x38, 0x00, 0x01, 0x2C, 0xC1, 0x68, 0x97, 0x3E };
static const uint8_t telemetry_0AF9[] = { 0x6A, 0xF9, 0x00, 0x1F, 0x2C, 0xD8, 0xEA, 0xBA, 0x73, 0x60, 0x04 };

static void mesh_node_test_state(process_state_t *ps, mesh_state_t *ms, stat_state_t *ss) {
    fwd_test_state(ps, ms, ss, true, true);
    filter_init(&ps->filter);
    g_exec = ps;
}

static const uint8_t split_head[] = { 0x05, 0xBF, 0x00, 0x05, 0x40, 0x02, 0x35, 0x00, 0x0D, 0x64, 0x61, 0x66, 0x63, 0x31, 0x36, 0x38, 0x2D, 0x64, 0x69, 0x72, 0x74, 0x79, 0x02, 0x07, 0x65, 0x73, 0x70, 0x33, 0x32, 0x63, 0x33 };

static const uint8_t split_tail[] = { 0x0D, 0x69, 0x6F, 0x74, 0x64, 0x61, 0x74, 0x61, 0x5F, 0x72, 0x65, 0x6C, 0x61, 0x79, 0x06, 0x0C, 0x37, 0x30, 0x41, 0x46, 0x30, 0x39, 0x31, 0x35, 0x35, 0x32, 0x46, 0x30 };

static int build_report(uint8_t *out, size_t size, uint16_t station, uint16_t sequence, uint8_t type) {
    uint8_t kv[32];
    iotdata_kvr_t b;
    iotdata_kvr_init(&b, kv, sizeof(kv));
    const uint8_t rq[1] = { IOTDATA_NODE_TLV_VERSION };
    const uint8_t cmd[2] = { IOTDATA_NODE_SUBJECT_MESH, IOTDATA_NODE_ACTION_MESH_PEERS_CLEAR };
    iotdata_kvr_add(&b, IOTDATA_NODE_CONTROL_REQUEST, rq, (uint8_t)sizeof(rq));
    iotdata_kvr_add(&b, IOTDATA_NODE_CONTROL_CONTROL, cmd, (uint8_t)sizeof(cmd));
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

static bool test_ctrl_short_names(void) {
    static const struct {
        const char *cmd;
        uint8_t expect[8];
        int expect_len;
    } cases[] = {
        /* [key][vlen][subject]... -- REQUEST is 0x00 and CONTROL is 0x01; everything else is
           vocabulary. Subjects are the TLV types verbatim, plus NODE (0x40) and MESH (0x41). */
        { "{\"cmd\":\"recv\",\"target\":\"0x123\"}", { 0x00, 0x01, 0x00 }, 3 },
        { "{\"cmd\":\"vers\",\"target\":\"0x123\"}", { 0x00, 0x01, 0x01 }, 3 },
        { "{\"cmd\":\"vari\",\"target\":\"0x123\"}", { 0x00, 0x01, 0x02 }, 3 },
        { "{\"cmd\":\"ctrl\",\"target\":\"0x123\"}", { 0x00, 0x01, 0x03 }, 3 },
        { "{\"cmd\":\"stat\",\"target\":\"0x123\"}", { 0x00, 0x01, 0x04 }, 3 }, /* no scope = the default groups */
        { "{\"cmd\":\"stat\",\"scope\":\"mesh\",\"target\":\"0x123\"}", { 0x00, 0x02, 0x04, 0x02 }, 4 },
        { "{\"cmd\":\"conf\",\"target\":\"0x123\"}", { 0x00, 0x01, 0x05 }, 3 },
        { "{\"cmd\":\"diag\",\"target\":\"0x123\"}", { 0x00, 0x01, 0x06 }, 3 },
        { "{\"cmd\":\"cont\",\"target\":\"0x123\"}", { 0x00, 0x01, 0x07 }, 3 },
        { "{\"cmd\":\"boot\",\"target\":\"0x123\"}", { 0x01, 0x02, 0x40, 0x00 }, 4 }, /* NODE/reboot */
        /* the tables have no command of their own: each is `stat` with the table's scope bit */
        { "{\"cmd\":\"stat\",\"scope\":\"stations\",\"target\":\"0x123\"}", { 0x00, 0x02, 0x04, 0x04 }, 4 },
        { "{\"cmd\":\"stat\",\"scope\":\"filters\",\"target\":\"0x123\"}", { 0x00, 0x02, 0x04, 0x08 }, 4 },
        { "{\"cmd\":\"stat\",\"scope\":\"peers\",\"target\":\"0x123\"}", { 0x00, 0x02, 0x04, 0x10 }, 4 },
        /* a dump is a command, because a scope cannot express "print it on your own console" */
        { "{\"cmd\":\"node-stations-dump\",\"target\":\"0x123\"}", { 0x01, 0x02, 0x04, 0x00 }, 4 },
        { "{\"cmd\":\"node-filters-dump\",\"target\":\"0x123\"}", { 0x01, 0x02, 0x04, 0x01 }, 4 },
        { "{\"cmd\":\"mesh-peers-dump\",\"target\":\"0x123\"}", { 0x01, 0x02, 0x41, 0x02 }, 4 },
        { "{\"cmd\":\"mesh-peers-clear\",\"target\":\"0x123\"}", { 0x01, 0x02, 0x41, 0x01 }, 4 },
        { "{\"cmd\":\"node-filters-update\",\"station\":\"0x456\",\"action\":\"block\",\"target\":\"0x123\"}", { 0x01, 0x05, 0x04, 0x02, 0x04, 0x56, 0x01 }, 7 },
        { "{\"cmd\":\"node-filters-update\",\"station\":\"0x456\",\"action\":\"allow\",\"target\":\"0x123\"}", { 0x01, 0x05, 0x04, 0x02, 0x04, 0x56, 0x02 }, 7 },
        { "{\"cmd\":\"node-filters-update\",\"station\":\"0x456\",\"action\":\"none\",\"target\":\"0x123\"}", { 0x01, 0x05, 0x04, 0x02, 0x04, 0x56, 0x00 }, 7 },
        { "{\"cmd\":\"mesh-peers-update\",\"station\":\"0x456\",\"action\":\"remove\",\"target\":\"0x123\"}", { 0x01, 0x05, 0x41, 0x00, 0x04, 0x56, 0x00 }, 7 },
        /* the recorder's actions hang off the DIAGNOSTICS subject, where its reports already are */
        { "{\"cmd\":\"diag-bbox-enable\",\"target\":\"0x123\"}", { 0x01, 0x03, 0x06, 0x00, 0x01 }, 5 },
        { "{\"cmd\":\"diag-bbox-disable\",\"target\":\"0x123\"}", { 0x01, 0x03, 0x06, 0x00, 0x00 }, 5 },
        { "{\"cmd\":\"diag-bbox-clear\",\"target\":\"0x123\"}", { 0x01, 0x02, 0x06, 0x01 }, 4 },
        { "{\"cmd\":\"diag-bbox-dump\",\"target\":\"0x123\"}", { 0x01, 0x02, 0x06, 0x02 }, 4 },
        /* the cart hangs off DIAGNOSTICS too, and carries its open/close as a fixed argument the
           command name chooses -- the same shape as the recorder's enable/disable above */
        { "{\"cmd\":\"diag-cart-enable\",\"target\":\"0x123\"}", { 0x01, 0x03, 0x06, 0x03, 0x01 }, 5 },
        { "{\"cmd\":\"diag-cart-disable\",\"target\":\"0x123\"}", { 0x01, 0x03, 0x06, 0x03, 0x00 }, 5 },
        /* a filter scope is carried even when zero: SCOPE_ALL is a real instruction, where an
           absent STATUS scope already means the default groups */
        { "{\"cmd\":\"node-filters-clear\",\"target\":\"0x123\"}", { 0x01, 0x03, 0x04, 0x03, 0x00 }, 5 },
        { "{\"cmd\":\"node-filters-clear\",\"scope\":\"manual\",\"target\":\"0x123\"}", { 0x01, 0x03, 0x04, 0x03, 0x01 }, 5 },
        { "{\"cmd\":\"node-filters-clear\",\"scope\":\"auto\",\"target\":\"0x123\"}", { 0x01, 0x03, 0x04, 0x03, 0x02 }, 5 },
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
    /* walked through the framework's own accessor, so a command an application plugs in is
       covered by this too rather than only the built-ins */
    for (uint8_t i = 0;; i++) {
        const iotdata_node_control_command_t *const c = iotdata_node_control_at(i);
        if (c == NULL)
            break;
        if (c->arg == IOTDATA_NODE_CONTROL_ARG_REPORTS)
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
    ASSERT(ctrl_from_iotdata(IOTDATA_NODE_TLV_STATUS, IOTDATA_NODE_ACTION_NODE_FILTERS_UPDATE, (const uint8_t[]){ 0x0A, 0xF9, IOTDATA_NODE_CONTROL_NODE_FILTERS_BLOCK }, IOTDATA_NODE_CONTROL_UPDATE_ENTRY_SIZE, NULL));

    ASSERT_EQ_INT(exec_node_table_count(IOTDATA_NODE_STATUS_STATIONS_ENTRY), 2);
    ASSERT_EQ_INT(exec_node_table_count(IOTDATA_NODE_STATUS_MESH_PEERS_ENTRY), 1);
    ASSERT_EQ_INT(exec_node_table_count(IOTDATA_NODE_STATUS_FILTERS_ENTRY), 1);

    /* a peer row, read back exactly as a reader would */
    uint8_t row[IOTDATA_NODE_STATUS_ENTRY_MAX];
    memset(row, 0, sizeof(row));
    ASSERT(exec_node_table_row(IOTDATA_NODE_STATUS_MESH_PEERS_ENTRY, 0, row));
    ASSERT_EQ_INT(iotdata_node_entry_get_u16(row, 0), 0x05BF);
    ASSERT_EQ_INT(iotdata_node_entry_get_u16(row, 2), 0x0001); /* the gateway is its own gateway */
    ASSERT_EQ_INT(iotdata_node_entry_get_u16(row, 5), 7);      /* generation */
    ASSERT_EQ_INT(row[10] & IOTDATA_NODE_STATUS_MESH_PEERS_FLAG_ACCEPTING, IOTDATA_NODE_STATUS_MESH_PEERS_FLAG_ACCEPTING);
    ASSERT_EQ_INT(row[10] & IOTDATA_NODE_STATUS_MESH_PEERS_FLAG_PARENT, 0); /* a root has no parent */

    /* a filter row */
    memset(row, 0, sizeof(row));
    ASSERT(exec_node_table_row(IOTDATA_NODE_STATUS_FILTERS_ENTRY, 0, row));
    ASSERT_EQ_INT(iotdata_node_entry_get_u16(row, 0), 0x0AF9);
    ASSERT_EQ_INT(row[2], IOTDATA_NODE_CONTROL_NODE_FILTERS_BLOCK);
    ASSERT_EQ_INT(row[3], IOTDATA_NODE_CONTROL_NODE_FILTERS_SCOPE_MANUAL);

    /* an index past the end is refused rather than returning a stale row */
    ASSERT(!exec_node_table_row(IOTDATA_NODE_STATUS_MESH_PEERS_ENTRY, 5, row));
    ASSERT(!exec_node_table_row(IOTDATA_NODE_TLV_STATUS, 0, row)); /* not a table */
    g_exec = NULL;
    return true;
}

/* Removal leaves a hole, and a dense report index must skip it -- otherwise the Nth row is the Nth
   slot and a removed entry makes the report short or repeats one. */
/* A removal must show up in the dump, and the dump must stay walkable 0..count-1 across it. There
   are no holes to skip any more -- stat_mesh_peer_remove() compacts -- and the rows come out in
   station-id order regardless of where the survivors ended up in storage. */
static bool test_mesh_node_table_after_remove(void) {
    process_state_t ps;
    mesh_state_t ms;
    static stat_state_t ss;
    mesh_node_test_state(&ps, &ms, &ss);
    stat_on_peer(&ss, 0x0501, 1, 1, 0);
    stat_on_peer(&ss, 0x0502, 1, 1, 0);
    stat_on_peer(&ss, 0x0503, 1, 1, 0);
    ASSERT(stat_mesh_peer_remove(&ss, 0x0502)); /* from the middle: 0x0503 moves down into its place */
    ASSERT_EQ_INT(exec_node_table_count(IOTDATA_NODE_STATUS_MESH_PEERS_ENTRY), 2);

    uint8_t row[IOTDATA_NODE_STATUS_ENTRY_MAX];
    ASSERT(exec_node_table_row(IOTDATA_NODE_STATUS_MESH_PEERS_ENTRY, 0, row));
    ASSERT_EQ_INT(iotdata_node_entry_get_u16(row, 0), 0x0501);
    ASSERT(exec_node_table_row(IOTDATA_NODE_STATUS_MESH_PEERS_ENTRY, 1, row));
    ASSERT_EQ_INT(iotdata_node_entry_get_u16(row, 0), 0x0503); /* the survivor, wherever it now sits */
    ASSERT(!exec_node_table_row(IOTDATA_NODE_STATUS_MESH_PEERS_ENTRY, 2, row));
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
    iotdata_node_init(&t_inode, 0x0001, NULL, 0u);
    ns.inode = &t_inode;
    memset(&ss, 0, sizeof(ss));
    ns.stat = &ss;
    ns.tx = test_packet_handler;
    iotdata_node_down_init(&ns.down, &t_pool);

    uint8_t kv[64];
    iotdata_kvr_t b;
    iotdata_kvr_init(&b, kv, sizeof(kv));
    uint8_t row[IOTDATA_NODE_STATUS_MESH_PEERS_ENTRY_SIZE];
    memset(row, 0, sizeof(row));
    iotdata_node_entry_put_u16(row, 0, 0x0F1B);
    iotdata_node_entry_put_u16(row, 2, 0x0001); /* a NUL byte, mid-row, on purpose */
    row[4] = 1;
    iotdata_node_entry_put_u16(row, 5, 9);
    row[7] = (uint8_t)(int8_t)-42;
    iotdata_node_entry_put_u16(row, 8, 3);
    row[10] = IOTDATA_NODE_STATUS_MESH_PEERS_FLAG_ACCEPTING;
    iotdata_kvr_add_u8(&b, IOTDATA_NODE_STATUS_MESH_PEERS_COUNT, 1);
    iotdata_kvr_add(&b, IOTDATA_NODE_STATUS_MESH_PEERS_ENTRY, row, (uint8_t)sizeof(row));

    ASSERT(iotdata_node_status_entry_size(IOTDATA_NODE_STATUS_MESH_PEERS_ENTRY) == sizeof(row));
    cJSON *const root = cJSON_CreateObject();
    ASSERT(root != NULL);
    cJSON *const data = cJSON_AddObjectToObject(root, "data");
    node_json_kvr(NULL, data, IOTDATA_NODE_TLV_STATUS, kv, b.len);
    char *const out = cJSON_PrintUnformatted(root);
    ASSERT(out != NULL);

    /* the whole table's count, and a row decoded into named fields */
    ASSERT(strstr(out, "\"mesh_peers_count\":1") != NULL);
    ASSERT(strstr(out, "\"peers\":[") != NULL);
    /* NUMERIC IN JSON, hex only where a person reads it (topics, log lines, the console): a
       document a program parses should not make it parse a base too. 0xF1B == 3867, 0x001 == 1. */
    ASSERT(strstr(out, "\"station\":3867") != NULL);
    ASSERT(strstr(out, "\"gateway\":1") != NULL); /* survived the NUL */
    ASSERT(strstr(out, "\"cost\":1") != NULL);
    ASSERT(strstr(out, "\"rssi\":-42") != NULL);
    ASSERT(strstr(out, "\"accepting\":true") != NULL);
    ASSERT(strstr(out, "\"parent\":false") != NULL);
    free(out);
    cJSON_Delete(root);

    /* a row of the wrong width is dropped, not misread as another table's row */
    iotdata_kvr_init(&b, kv, sizeof(kv));
    iotdata_kvr_add_u8(&b, IOTDATA_NODE_STATUS_MESH_PEERS_COUNT, 1);
    iotdata_kvr_add(&b, IOTDATA_NODE_STATUS_MESH_PEERS_ENTRY, row, 4); /* a FILTERS-width row in a PEERS report */
    cJSON *const r2 = cJSON_CreateObject();
    cJSON *const d2 = cJSON_AddObjectToObject(r2, "data");
    node_json_kvr(NULL, d2, IOTDATA_NODE_TLV_STATUS, kv, b.len);
    char *const o2 = cJSON_PrintUnformatted(r2);
    ASSERT(strstr(o2, "\"peers\":[]") != NULL);
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
    iotdata_node_init(&t_inode, 0x0001, NULL, 0u);
    ns.inode = &t_inode;
    memset(&ss, 0, sizeof(ss));
    ns.stat = &ss;

    uint8_t kv[64];
    iotdata_kvr_t b;
    iotdata_kvr_init(&b, kv, sizeof(kv));
    iotdata_kvr_add_str(&b, IOTDATA_NODE_VERSION_HARDWARE, "esp32c3/riscv32");
    iotdata_kvr_add_str(&b, IOTDATA_NODE_VERSION_FIRMWARE, "idf/6.1+bl1.20");

    cJSON *const root = cJSON_CreateObject();
    node_json_kvr(&ns, root, IOTDATA_NODE_TLV_VERSION, kv, b.len);
    char *const out = cJSON_PrintUnformatted(root);
    ASSERT(out != NULL);
    ASSERT(strstr(out, "\"hardware\":\"esp32c3/riscv32\"") != NULL);
    /* the '+' in a firmware value survives JSON intact -- it is part of the grammar, not an escape */
    ASSERT(strstr(out, "\"firmware\":\"idf/6.1+bl1.20\"") != NULL);
    free(out);
    cJSON_Delete(root);
    return true;
}

/* What a `vers` request actually puts on manage/from. The four buckets arrive as strings, and
   capabilities -- 16-bit entries, so hex by any generic rule -- must arrive DECODED, or whoever
   asked has to own a copy of the bit registry to read the answer. */
static bool test_version_report_as_the_monitor_sees_it(void) {
    node_state_t ns;
    stat_state_t ss;
    memset(&ns, 0, sizeof(ns));
    iotdata_node_init(&t_inode, 0x0001, NULL, 0u);
    ns.inode = &t_inode;
    memset(&ss, 0, sizeof(ss));
    ns.stat = &ss;

    iotdata_node_version_caps_t caps;
    memset(&caps, 0, sizeof(caps));
    (void)iotdata_node_version_caps_add(&caps, IOTDATA_NODE_VERSION_CAPS_RADIO, IOTDATA_NODE_VERSION_RADIO_E22_USB);
    (void)iotdata_node_version_caps_add(&caps, IOTDATA_NODE_VERSION_CAPS_FEATURES, IOTDATA_NODE_VERSION_FEATURE_MESH | IOTDATA_NODE_VERSION_FEATURE_BLACKBOX);
    (void)iotdata_node_version_caps_add(&caps, IOTDATA_NODE_VERSION_CAPS_PROPRIETARY, 0x008); /* a bit with no name */

    uint8_t kv[200];
    iotdata_kvr_t b;
    iotdata_kvr_init(&b, kv, sizeof(kv));
    const iotdata_node_version_t self = { .caps = &caps };
    ASSERT(iotdata_node_version_pack(&b, &self) > 0);

    cJSON *const root = cJSON_CreateObject();
    node_json_kvr(&ns, root, IOTDATA_NODE_TLV_VERSION, kv, b.len);
    char *const out = cJSON_PrintUnformatted(root);
    ASSERT(out != NULL);

    /* the four buckets, by the names the key table gives -- which is what a reader keys off */
    ASSERT(strstr(out, "\"hardware\":\"") != NULL);
    ASSERT(strstr(out, "\"firmware\":\"") != NULL);
    ASSERT(strstr(out, "\"serial\":\"") != NULL);
    /* software carries the grammar a reader parses: app/semver/stamp */
    ASSERT(strstr(out, "\"software\":\"" IOTDATA_NODE_VERSION_APP "/" IOTDATA_NODE_VERSION_SEMVER "/") != NULL);

    /* capabilities decoded, not hex */
    ASSERT(strstr(out, "\"capabilities\"") != NULL);
    /* rendered in the order the categories were declared, bits low-to-high within one */
    ASSERT(strstr(out, "radio/e22-usb,mesh,bbox,proprietary/0x008") != NULL);
    /* and the raw entries travel too, so a newer registry can decode what this one could not */
    ASSERT(strstr(out, "\"category\":\"radio\"") != NULL);
    ASSERT(strstr(out, "\"entries\"") != NULL);
    free(out);
    cJSON_Delete(root);

    /* THE IMAGE HASH ARRIVES AS 16 RAW BYTES and must come out of the gateway as hex, because
       that is what a manager and a person can both read. The gateway cannot hash its own binary,
       so the key is absent from a report it packs for itself -- but it decodes reports from
       DEVICES, which carry it. Fed by hand, because only a node can produce one. */
    iotdata_kvr_init(&b, kv, sizeof(kv));
    const uint8_t imagehash[IOTDATA_NODE_VERSION_IMAGE_BYTES] = { 0x9f, 0x86, 0xd0, 0x81, 0x88, 0x4c, 0x7d, 0x65, 0x9a, 0x2f, 0xea, 0xa0, 0xc5, 0x5a, 0xd0, 0x15 };
    iotdata_kvr_add(&b, IOTDATA_NODE_VERSION_IMAGE, imagehash, (uint8_t)sizeof(imagehash));
    cJSON *const img = cJSON_CreateObject();
    node_json_kvr(&ns, img, IOTDATA_NODE_TLV_VERSION, kv, b.len);
    char *const imgout = cJSON_PrintUnformatted(img);
    ASSERT(imgout != NULL && strstr(imgout, "\"image\":\"9f86d081884c7d659a2feaa0c55ad015\"") != NULL);
    free(imgout);
    cJSON_Delete(img);

    /* a malformed value is reported as malformed rather than rendered as a guess */
    iotdata_kvr_init(&b, kv, sizeof(kv));
    const uint8_t odd[3] = { 0x00, 0x01, 0x02 };
    iotdata_kvr_add(&b, IOTDATA_NODE_VERSION_CAPABILITIES, odd, sizeof(odd));
    cJSON *const bad = cJSON_CreateObject();
    node_json_kvr(&ns, bad, IOTDATA_NODE_TLV_VERSION, kv, b.len);
    char *const badout = cJSON_PrintUnformatted(bad);
    ASSERT(badout != NULL && strstr(badout, "\"capabilities\":\"malformed\"") != NULL);
    free(badout);
    cJSON_Delete(bad);
    return true;
}

static bool test_ctrl_status_scope_differs_from_node_status(void) {
    ctrl_state_t st;

    /* One command, and the scope decides. `stat` with a scope carries the byte; without one it
       carries nothing, because STATUS_REQUEST is WIDTH_VARIABLE and absent already means every
       group -- sending an explicit zero would say the same thing twice and cost a byte. */
    ctrl_test_state(&st);
    ctrl_test_feed(&st, "{\"cmd\":\"stat\",\"scope\":\"mesh\",\"target\":\"0x123\"}");
    ASSERT(ctrl_staged(&st));
    ASSERT_EQ_INT(ctrl_staged_bytes(&st)[0], IOTDATA_NODE_CONTROL_REQUEST);
    ASSERT_EQ_INT(ctrl_staged_bytes(&st)[1], 2);
    ASSERT_EQ_INT(ctrl_staged_bytes(&st)[2], IOTDATA_NODE_TLV_STATUS);
    ASSERT_EQ_INT(ctrl_staged_bytes(&st)[3], IOTDATA_NODE_STATUS_SCOPE_MESH);
    ctrl_test_drop(&st); /* ctrl_test_state below memsets the slot: give this one back first */

    ctrl_test_state(&st);
    ctrl_test_feed(&st, "{\"cmd\":\"stat\",\"target\":\"0x123\"}");
    ASSERT(ctrl_staged(&st));
    ASSERT_EQ_INT(ctrl_staged_bytes(&st)[0], IOTDATA_NODE_CONTROL_REQUEST);
    ASSERT_EQ_INT(ctrl_staged_bytes(&st)[1], 1); /* the subject alone: no scope = the default groups */
    ASSERT_EQ_INT(ctrl_staged_bytes(&st)[2], IOTDATA_NODE_TLV_STATUS);
    ctrl_test_drop(&st);
    return true;
}

static bool test_ctrl_mesh_update_entry_shape(void) {
    ctrl_state_t st;
    ctrl_test_state(&st);
    ctrl_test_feed(&st, "{\"cmd\":\"node-filters-update\",\"station\":\"0xABC\",\"action\":\"block\",\"target\":\"0x123\"}");
    ASSERT(ctrl_staged(&st));
    ASSERT_EQ_INT(ctrl_staged_len(&st), 2 + 2 + IOTDATA_NODE_CONTROL_UPDATE_ENTRY_SIZE);
    ASSERT_EQ_INT(ctrl_staged_bytes(&st)[0], IOTDATA_NODE_CONTROL_CONTROL);
    ASSERT_EQ_INT(ctrl_staged_bytes(&st)[1], 2 + IOTDATA_NODE_CONTROL_UPDATE_ENTRY_SIZE);
    ASSERT_EQ_INT(ctrl_staged_bytes(&st)[2], IOTDATA_NODE_TLV_STATUS);
    ASSERT_EQ_INT(ctrl_staged_bytes(&st)[3], IOTDATA_NODE_ACTION_NODE_FILTERS_UPDATE);
    ASSERT_EQ_INT((uint16_t)((ctrl_staged_bytes(&st)[4] << 8) | ctrl_staged_bytes(&st)[5]), 0x0ABC);
    ASSERT_EQ_INT(ctrl_staged_bytes(&st)[6], IOTDATA_NODE_CONTROL_NODE_FILTERS_BLOCK);
    ctrl_test_drop(&st);
    return true;
}

/* Drains a staged trace frame so a test can see exactly what the gateway put on the air. */
static int trace_test_take(trace_state_t *const st, uint8_t *const out, const int cap) {
    const buffer_handle_t h = buffer_queue_take(&st->queue, hw_time_ms(), NULL, NULL);
    if (h == BUFFER_NONE)
        return -1;
    int n = -1;
    const int len = (int)buffer_len(st->pool, h);
    if (len <= cap) {
        memcpy(out, buffer_data(st->pool, h), (size_t)len);
        n = len;
    }
    buffer_unref(st->pool, h);
    return n;
}

static bool test_trace_request_shapes_each_leg(void) {
    trace_state_t tv;
    uint8_t got[64];
    iotdata_mesh_diag_t dg;
    iotdata_mesh_diag_trace_t tr;
    iotdata_mesh_nbr_entry_t hop;
    const char *why = "";

    iotdata_node_init(&t_inode, 0x001, NULL, 0u);
    trace_begin(&tv, "pre", &t_inode, &t_pool);

    /* UP asks the subject to record: a REQUEST, carrying no hops of its own. */
    ASSERT(trace_request(&tv, 0x0A5, 0x5BF, false, false, false, &why));
    int n = trace_test_take(&tv, got, (int)sizeof(got));
    ASSERT_EQ_INT(n, IOTDATA_MESH_DIAG_TRACE_HDR_SIZE);
    ASSERT(iotdata_mesh_unpack_diag(got, n, &dg));
    ASSERT_EQ_INT(dg.kind, IOTDATA_MESH_DIAG_TRACE);
    ASSERT_EQ_INT(dg.op, IOTDATA_MESH_DIAG_OP_REQUEST);
    ASSERT(iotdata_mesh_unpack_diag_trace(got, n, &tr));
    ASSERT_EQ_INT(tr.hop_count, 0);
    ASSERT_EQ_INT(tr.subject_station, 0x5BF);
    ASSERT_EQ_INT((tr.flags & IOTDATA_MESH_DIAG_TRACE_F_DOWN), 0);

    /* DOWN cannot ask anyone -- the recording happens on the way out -- so the gateway originates
       the RECORD itself, as hop 0, and the relays that flood it append. */
    ASSERT(trace_request(&tv, 0x0A6, 0x5BF, true, false, true, &why));
    n = trace_test_take(&tv, got, (int)sizeof(got));
    ASSERT_EQ_INT(n, IOTDATA_MESH_DIAG_TRACE_HDR_SIZE + IOTDATA_MESH_DIAG_TRACE_HOP);
    ASSERT(iotdata_mesh_unpack_diag(got, n, &dg));
    ASSERT_EQ_INT(dg.op, IOTDATA_MESH_DIAG_OP_RECORD);
    ASSERT(iotdata_mesh_unpack_diag_trace(got, n, &tr));
    ASSERT_EQ_INT(tr.hop_count, 1);
    ASSERT(iotdata_mesh_diag_trace_hop(got, n, 0, &hop));
    ASSERT_EQ_INT(hop.station, 0x001);
    ASSERT_EQ_INT(hop.cost, 0);
    ASSERT((tr.flags & IOTDATA_MESH_DIAG_TRACE_F_DOWN) != 0);
    ASSERT((tr.flags & IOTDATA_MESH_DIAG_TRACE_F_RESPOND) != 0);

    /* Directive 0 means "unasked" to a node reporting on its own, so it cannot be requested --
       otherwise a solicited record and an unsolicited one are indistinguishable at the gateway. */
    ASSERT(!trace_request(&tv, 0u, 0x5BF, false, false, false, &why));
    ASSERT(!trace_request(&tv, 0x0A7, IOTDATA_MESH_STATION_RESERVED, false, false, false, &why));
    ASSERT(!trace_request(&tv, 0x0A8, IOTDATA_STATION_BROADCAST, false, false, false, &why));
    ASSERT_EQ_INT(trace_test_take(&tv, got, (int)sizeof(got)), -1); /* and nothing was staged */

    /* Ids are compared, never ordered, and 0 is stepped over. */
    tv.next_id = 0x0FFFu;
    ASSERT_EQ_INT(trace_next_id(&tv), 1);

    trace_end(&tv);
    return true;
}

static bool test_ctrl_response_topic_is_shared(void) {
    node_state_t ns;
    memset(&ns, 0, sizeof(ns));
    iotdata_node_init(&t_inode, 0x0001, NULL, 0u);
    ns.inode = &t_inode;
    stat_state_t ss;
    memset(&ss, 0, sizeof(ss));
    iotdata_node_init(&t_inode, 0x0001, NULL, 0u);
    ASSERT(node_begin(&ns, &t_inode, NULL, &ss, &t_pool, test_packet_handler, NULL, NULL, NULL, NULL, NULL, 0, "pre"));

    ctrl_state_t st;
    ctrl_test_state(&st);
    /* ctrl_begin sets the topics before it subscribes, so the (expected) subscribe failure with no
       broker does not stop us checking them */
    iotdata_node_init(&t_inode, 0x0001, NULL, 0u);
    survey_state_t sv;
    trace_state_t tv;
    survey_begin(&sv, "pre", &t_inode, &t_pool);
    trace_begin(&tv, "pre", &t_inode, &t_pool);
    (void)ctrl_begin(&st, "pre", &t_inode, &sv, &tv, &t_pool);

    ASSERT(strcmp(st.topic_req, "pre/manage/req") == 0);
    ASSERT(strcmp(st.topic_from, "pre/manage/from") == 0);
    ASSERT(strcmp(ns.topic_from, st.topic_from) == 0);
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

static bool test_ctrl_target_defaults_to_broadcast(void) {
    ctrl_state_t st;
    ctrl_test_state(&st);
    ctrl_test_feed(&st, "{\"cmd\":\"mesh-peers-dump\"}");
    ASSERT(ctrl_staged(&st));
    ASSERT_EQ_INT(ctrl_staged_target(&st), IOTDATA_STATION_BROADCAST);
    ctrl_test_drop(&st);
    return true;
}

static bool test_ctrl_requests_queue_in_order(void) {
    ctrl_state_t st;
    ctrl_test_state(&st);
    ctrl_test_feed(&st, "{\"cmd\":\"mesh-peers-dump\",\"target\":\"0x123\"}");
    ctrl_test_feed(&st, "{\"cmd\":\"node-filters-dump\",\"target\":\"0x456\"}");
    ASSERT_EQ_INT(buffer_queue_count(&st.queue), 2u);
    ASSERT_EQ_INT(buffer_queue_rejected(&st.queue), 0u);

    uint8_t key = 0;
    uint16_t target = 0;
    ASSERT(ctrl_test_take(&st, &key, &target));
    ASSERT_EQ_INT(key, IOTDATA_NODE_ACTION_MESH_PEERS_DUMP);
    ASSERT_EQ_INT(target, 0x123); /* the target travels with each one, not just with the last */
    ASSERT(ctrl_test_take(&st, &key, &target));
    ASSERT_EQ_INT(key, IOTDATA_NODE_ACTION_NODE_FILTERS_DUMP /* likewise */);
    ASSERT_EQ_INT(target, 0x456);
    ASSERT(!ctrl_test_take(&st, NULL, NULL));
    return true;
}

static bool test_ctrl_queue_full_is_refused(void) {
    ctrl_state_t st;
    ctrl_test_state(&st);
    for (int i = 0; i < CTRL_QUEUE_MAX + 2; i++)
        ctrl_test_feed(&st, "{\"cmd\":\"mesh-peers-dump\",\"target\":\"0x123\"}");
    ASSERT_EQ_INT(buffer_queue_count(&st.queue), CTRL_QUEUE_MAX);
    /* Refused by the ctrl layer BEFORE a buffer is acquired, so the queue never sees the attempt:
       a request naming several targets has to be checked against the room for all of them at once,
       and refusing at the queue could only discover that half way through the fan-out. */
    ASSERT_EQ_INT(buffer_queue_rejected(&st.queue), 0u);
    ASSERT_EQ_INT(st.stat_req_bad, 2u); /* each refusal answered, not dropped on the floor */
    ctrl_test_drop(&st);
    return true;
}

/* One request, several stations: the wire carries one station per DOWN frame, so the gateway has
   to emit one per target -- and must refuse the lot if they will not all fit. */
static bool test_ctrl_multiple_targets(void) {
    ctrl_state_t st;
    ctrl_test_state(&st);
    ctrl_test_feed(&st, "{\"cmd\":\"mesh-peers-dump\",\"target\":[\"0x123\",\"0x456\",\"0x789\"]}");
    ASSERT_EQ_INT(buffer_queue_count(&st.queue), 3u); /* one frame each */
    ASSERT_EQ_INT(st.stat_req_bad, 0u);
    /* a repeated station is airtime for nothing, and a broadcast anywhere absorbs the rest */
    ctrl_test_drop(&st);
    ctrl_test_feed(&st, "{\"cmd\":\"mesh-peers-dump\",\"target\":[\"0x123\",\"0x123\"]}");
    ASSERT_EQ_INT(buffer_queue_count(&st.queue), 1u);
    ctrl_test_drop(&st);
    ctrl_test_feed(&st, "{\"cmd\":\"mesh-peers-dump\",\"target\":[\"0x123\",\"all\",\"0x456\"]}");
    ASSERT_EQ_INT(buffer_queue_count(&st.queue), 1u);
    ctrl_test_drop(&st);
    /* more targets than one request may name: refused, never truncated to what happens to fit */
    char many[256] = "{\"cmd\":\"mesh-peers-dump\",\"target\":[";
    for (int i = 0; i < IOTDATA_NODE_CONTROL_TARGETS_MAX + 1; i++)
        (void)snprintf(many + strlen(many), sizeof(many) - strlen(many), "%s\"0x%03X\"", i ? "," : "", 0x100 + i);
    (void)snprintf(many + strlen(many), sizeof(many) - strlen(many), "]}");
    ctrl_test_feed(&st, many);
    ASSERT_EQ_INT(buffer_queue_count(&st.queue), 0u);
    ASSERT_EQ_INT(st.stat_req_bad, 1u);
    /* and a list that FITS but the queue has no room for: also refused whole, nothing staged */
    ctrl_test_feed(&st, "{\"cmd\":\"mesh-peers-dump\",\"target\":\"0x001\"}");
    ASSERT_EQ_INT(buffer_queue_count(&st.queue), 1u);
    char full[256] = "{\"cmd\":\"mesh-peers-dump\",\"target\":[";
    for (int i = 0; i < IOTDATA_NODE_CONTROL_TARGETS_MAX; i++)
        (void)snprintf(full + strlen(full), sizeof(full) - strlen(full), "%s\"0x%03X\"", i ? "," : "", 0x200 + i);
    (void)snprintf(full + strlen(full), sizeof(full) - strlen(full), "]}");
    ctrl_test_feed(&st, full);
    ASSERT_EQ_INT(buffer_queue_count(&st.queue), 1u); /* still just the one */
    ASSERT_EQ_INT(st.stat_req_bad, 2u);
    ctrl_test_drop(&st);
    return true;
}

static bool test_ctrl_staged_command_expires(void) {
    ctrl_state_t st;
    ctrl_test_state(&st);
    ctrl_test_feed(&st, "{\"cmd\":\"mesh-peers-dump\",\"target\":\"0x123\"}");
    ASSERT_EQ_INT(buffer_queue_count(&st.queue), 1u);
    const uint32_t now_ms = hw_time_ms();
    ASSERT_EQ_INT(buffer_queue_expire(&st.queue, now_ms + CTRL_QUEUE_TTL_MS - 1000), 0u); /* not yet */
    ASSERT_EQ_INT(buffer_queue_expire(&st.queue, now_ms + CTRL_QUEUE_TTL_MS + 1000), 1u);
    ASSERT_EQ_INT(buffer_queue_count(&st.queue), 0u);
    ASSERT_EQ_INT(buffer_queue_expired(&st.queue), 1u);
    return true;
}

/* THE RECORDER'S CLOCK IS MONOTONIC AND SEEDED, which is the shape of a bug this had: it ticked
   from time(NULL), so on a box with no RTC the first NTP step fed the whole gap in as a delta --
   forwards it credits years of uptime and fires a batched flush, backwards it goes negative and an
   unsigned delta of 4.29 billion does both. The two properties that make that impossible are
   asserted here: the FIRST tick advances nothing however large now_ms already is, and every tick
   after it advances by exactly the difference. */
/* THE RESPONSE TOPIC IS KEYED THE SAME WAY THE TELEMETRY TOPIC IS. A station rendered any other
   way -- lower case, un-padded, decimal -- would put a node's answers somewhere a subscriber
   watching its telemetry would never think to look, and nothing else would complain. */
static bool test_manage_from_topic_is_keyed_by_station(void) {
    char topic[GW_TOPIC_STATION_MAX];
    ASSERT(strcmp(gateway_topic_for_station(topic, sizeof(topic), "iotdata/manage/from", 0xF1B), "iotdata/manage/from/F1B") == 0);
    /* three digits, zero-padded and upper case, exactly as the telemetry topic spells it */
    ASSERT(strcmp(gateway_topic_for_station(topic, sizeof(topic), "iotdata/manage/from", 0x001), "iotdata/manage/from/001") == 0);
    ASSERT(strcmp(gateway_topic_for_station(topic, sizeof(topic), "iotdata/manage/from", 0xABC), "iotdata/manage/from/ABC") == 0);
    /* and the prefix is whatever the deployment uses, not a literal */
    ASSERT(strcmp(gateway_topic_for_station(topic, sizeof(topic), "site7/manage/from", 0x445), "site7/manage/from/445") == 0);
    return true;
}

static bool test_diagnostics_clock_is_monotonic_and_seeded(void) {
    ASSERT(iotdata_node_diagnostics_begin(0, true));
    blackbox_handle_t *const h = iotdata_node_diagnostics_handle();
    ASSERT(h != NULL);
    blackbox_status_t bs;

    /* a process up for an hour before the recorder starts owes the recorder nothing */
    iotdata_node_diagnostics_tick(3600000u);
    ASSERT(blackbox_status(h, &bs));
    ASSERT_EQ_INT(bs.uptime_ms, 0u);

    iotdata_node_diagnostics_tick(3600000u + 1000u);
    iotdata_node_diagnostics_tick(3600000u + 1500u);
    ASSERT(blackbox_status(h, &bs));
    ASSERT_EQ_INT(bs.uptime_ms, 1500u); /* the elapsed difference, not the clock's value */

    /* and a pass that takes no measurable time is not an error, it is a zero */
    iotdata_node_diagnostics_tick(3600000u + 1500u);
    ASSERT(blackbox_status(h, &bs));
    ASSERT_EQ_INT(bs.uptime_ms, 1500u);
    return true;
}

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

static bool test_mesh_node_filter_update_blocks_rx(void) {
    process_state_t ps;
    mesh_state_t ms;
    stat_state_t ss;
    mesh_node_test_state(&ps, &ms, &ss);

    ASSERT(filter_allows(&ps.filter, 0x0537)); /* nothing filtered yet */

    const uint8_t block[IOTDATA_NODE_CONTROL_UPDATE_ENTRY_SIZE] = { 0x05, 0x37, IOTDATA_NODE_CONTROL_NODE_FILTERS_BLOCK };
    ASSERT(ctrl_from_iotdata(IOTDATA_NODE_TLV_STATUS, IOTDATA_NODE_ACTION_NODE_FILTERS_UPDATE, block, (uint8_t)sizeof(block), NULL));
    ASSERT(!filter_allows(&ps.filter, 0x0537)); /* blocked */
    ASSERT(filter_allows(&ps.filter, 0x0538));  /* and only that one */
    ASSERT_EQ_INT(filter_count(&ps.filter), 1);

    /* NONE removes it: there is no separate remove command, by design */
    const uint8_t none[IOTDATA_NODE_CONTROL_UPDATE_ENTRY_SIZE] = { 0x05, 0x37, IOTDATA_NODE_CONTROL_NODE_FILTERS_NONE };
    ASSERT(ctrl_from_iotdata(IOTDATA_NODE_TLV_STATUS, IOTDATA_NODE_ACTION_NODE_FILTERS_UPDATE, none, (uint8_t)sizeof(none), NULL));
    ASSERT(filter_allows(&ps.filter, 0x0537));
    ASSERT_EQ_INT(filter_count(&ps.filter), 0);
    g_exec = NULL;
    return true;
}

static bool test_mesh_node_block_drops_peer(void) {
    process_state_t ps;
    mesh_state_t ms;
    stat_state_t ss;
    mesh_node_test_state(&ps, &ms, &ss);
    stat_on_peer(&ss, 0x05BF, 1, 1, 0);
    ASSERT_EQ_INT(ss.peers_count, 1);

    const uint8_t block[IOTDATA_NODE_CONTROL_UPDATE_ENTRY_SIZE] = { 0x05, 0xBF, IOTDATA_NODE_CONTROL_NODE_FILTERS_BLOCK };
    ASSERT(ctrl_from_iotdata(IOTDATA_NODE_TLV_STATUS, IOTDATA_NODE_ACTION_NODE_FILTERS_UPDATE, block, (uint8_t)sizeof(block), NULL));
    ASSERT_EQ_INT(ss.peers_count, 0);
    g_exec = NULL;
    return true;
}

static bool test_mesh_node_peers_update_and_clear(void) {
    process_state_t ps;
    mesh_state_t ms;
    stat_state_t ss;
    mesh_node_test_state(&ps, &ms, &ss);
    stat_on_peer(&ss, 0x05BF, 1, 1, 0);
    stat_on_peer(&ss, 0x0F1B, 1, 1, 0);
    ASSERT_EQ_INT(ss.peers_count, 2);

    const uint8_t forget[IOTDATA_NODE_CONTROL_UPDATE_ENTRY_SIZE] = { 0x05, 0xBF, IOTDATA_NODE_CONTROL_MESH_PEER_NONE };
    ASSERT(ctrl_from_iotdata(IOTDATA_NODE_SUBJECT_MESH, IOTDATA_NODE_ACTION_MESH_PEERS_UPDATE, forget, (uint8_t)sizeof(forget), NULL));
    ASSERT_EQ_INT(ss.peers_count, 1);
    ASSERT(ctrl_from_iotdata(IOTDATA_NODE_SUBJECT_MESH, IOTDATA_NODE_ACTION_MESH_PEERS_UPDATE, forget, (uint8_t)sizeof(forget), NULL)); /* again */
    ASSERT_EQ_INT(ss.peers_count, 1);                                                                                                   /* still 1: idempotent */

    /* the hole a removal leaves must be reusable, or the table leaks slots */
    stat_on_peer(&ss, 0x0AF9, 1, 1, 0);
    ASSERT_EQ_INT(ss.peers_count, 2);

    ASSERT(ctrl_from_iotdata(IOTDATA_NODE_SUBJECT_MESH, IOTDATA_NODE_ACTION_MESH_PEERS_CLEAR, NULL, 0, NULL));
    ASSERT_EQ_INT(ss.peers_count, 0);
    ASSERT(ctrl_from_iotdata(IOTDATA_NODE_SUBJECT_MESH, IOTDATA_NODE_ACTION_MESH_PEERS_CLEAR, NULL, 0, NULL)); /* again */
    ASSERT_EQ_INT(ss.peers_count, 0);
    g_exec = NULL;
    return true;
}

static bool test_mesh_node_filter_update_takes_a_list(void) {
    process_state_t ps;
    mesh_state_t ms;
    stat_state_t ss;
    mesh_node_test_state(&ps, &ms, &ss);

    const uint8_t three[3 * IOTDATA_NODE_CONTROL_UPDATE_ENTRY_SIZE] = {
        0x05, 0x37, IOTDATA_NODE_CONTROL_NODE_FILTERS_BLOCK, 0x05, 0x38, IOTDATA_NODE_CONTROL_NODE_FILTERS_BLOCK, 0x0A, 0xF9, IOTDATA_NODE_CONTROL_NODE_FILTERS_ALLOW,
    };
    ASSERT(ctrl_from_iotdata(IOTDATA_NODE_TLV_STATUS, IOTDATA_NODE_ACTION_NODE_FILTERS_UPDATE, three, (uint8_t)sizeof(three), NULL));
    ASSERT_EQ_INT(filter_count(&ps.filter), 3);
    ASSERT(!filter_allows(&ps.filter, 0x0537));
    ASSERT(!filter_allows(&ps.filter, 0x0538));
    ASSERT(filter_allows(&ps.filter, 0x0AF9));
    /* an allow entry exists, so a station with no entry at all is now excluded (whitelist) */
    ASSERT(!filter_allows(&ps.filter, 0x06ED));
    g_exec = NULL;
    return true;
}

/* A config WRITE is built here and applied on the node. The gateway never resolves a name and never
   checks a bound -- both belong to whoever owns the table -- so what this asserts is only that the
   records on the wire are the ones asked for, and that malformed input is refused rather than sent. */
/* The gateway's own settings, now a node config table: the adapters read it, a runtime change is
   written back to an OVERLAY beside the operator's file, and a restart picks that up. */
/* The console is polled from the middle of the exec loop, so it must return whether or not anybody
   is typing. It did not: stdin was left blocking and the first poll sat in read() forever, which
   stopped the gateway dead after one pass. This is the test that would have said so. */
static int g_console_hits = 0;
static void test_console_cmd(const iotdata_node_console_emit_fn emit, __attribute__((unused)) const uint32_t now_ms, int argc, char **argv) {
    (void)argc;
    (void)argv;
    g_console_hits++;
    emit("hit\n");
}

static bool test_console_poll_never_blocks(void) {
    static const iotdata_node_console_t cons[] = { { "hit", test_console_cmd, "count a dispatch" } };
    int fds[2];
    ASSERT(pipe(fds) == 0);
    const int saved = dup(STDIN_FILENO);
    ASSERT(saved >= 0);
    ASSERT(dup2(fds[0], STDIN_FILENO) >= 0);
    iotdata_node_console_init(cons, 1);

    /* nothing written yet: this must RETURN, and would hang the suite if it did not */
    g_console_hits = 0;
    iotdata_node_console_tick((uint32_t)time(NULL));
    ASSERT_EQ_INT(g_console_hits, 0);

    /* a line arrives and is dispatched, without waiting for a second one */
    ASSERT(write(fds[1], "hit\n", 4) == 4);
    iotdata_node_console_tick((uint32_t)time(NULL));
    ASSERT_EQ_INT(g_console_hits, 1);

    /* a partial line waits, and does not block while it does */
    ASSERT(write(fds[1], "hi", 2) == 2);
    iotdata_node_console_tick((uint32_t)time(NULL));
    ASSERT_EQ_INT(g_console_hits, 1);
    ASSERT(write(fds[1], "t\n", 2) == 2);
    iotdata_node_console_tick((uint32_t)time(NULL));
    ASSERT_EQ_INT(g_console_hits, 2);

    /* the writer closes: EOF must stop it asking rather than spin */
    (void)close(fds[1]);
    iotdata_node_console_tick((uint32_t)time(NULL));
    iotdata_node_console_tick((uint32_t)time(NULL));
    ASSERT_EQ_INT(g_console_hits, 2);

    (void)dup2(saved, STDIN_FILENO);
    (void)close(saved);
    (void)close(fds[0]);
    return true;
}

__attribute__((format(printf, 1, 2))) static void test_quiet_emit(const char *fmt, ...) {
    (void)fmt;
}

static bool test_gateway_config_load_and_overlay(void) {
    char dir[] = "/tmp/iotdata-gwcfg-XXXXXX";
    ASSERT(mkdtemp(dir) != NULL);
    char cfg[256], ovl[256], line[512];
    snprintf(cfg, sizeof(cfg), "%s/gw.cfg", dir);
    snprintf(ovl, sizeof(ovl), "%s/gw.runtime.cfg", dir);

    FILE *f = fopen(cfg, "w");
    ASSERT(f != NULL);
    fputs("# the operator's file\nmqtt-server = mqtt://10.0.0.1:1883\nlora-channel = 42\nmesh-beacon-interval-s = 90\n", f);
    fclose(f);

    /* the table is at its defaults until something loads */
    iotdata_node_config_defaults();
    /* against the SHARED block's default, which is where the gateway's now comes from: the local
       MQTT_*_DEFAULT overrides are gone, so this is the one statement of it left to test against */
    ASSERT(!strcmp(iotdata_node_config_string(MQTT_SERVER), IOTDATA_NODE_CONFIG_MQTT_SERVER));
    ASSERT_EQ_INT(iotdata_node_config_u8(LORA_CHANNEL), IOTDATA_NODE_CONFIG_LORA_CHANNEL);

    /* the operator's file, then the overlay (absent), then nothing on the command line */
    char *argv[] = { (char *)(uintptr_t)"gw", (char *)(uintptr_t)"--config", cfg, (char *)(uintptr_t)"--overlay", ovl };
    ASSERT(gateway_config_load(5, argv));
    ASSERT(!strcmp(iotdata_node_config_string(MQTT_SERVER), "mqtt://10.0.0.1:1883"));
    ASSERT_EQ_INT(iotdata_node_config_u8(LORA_CHANNEL), 42);
    ASSERT_EQ_INT(iotdata_node_config_u16(MESH_BEACON_INTERVAL_S), 90);
    ASSERT(!iotdata_node_config_bool(MESH_ENABLE)); /* untouched: the table's default stands */

    /* BY NAME, which is the only way the file loader can reach a row: every other read in this
       test is a compile-time id, so without this nothing would notice the day a row's spelling and
       its key stopped agreeing. Dashes for underscores, and case-insensitive, as a file writes it. */
    ASSERT_EQ_INT(iotdata_node_config_index_by_name("mesh-beacon-interval-s"), (int)IOTDATA_NODE_CFG_IX_MESH_BEACON_INTERVAL_S);
    ASSERT_EQ_INT(iotdata_node_config_index_by_name("MESH_BEACON_INTERVAL_S"), (int)IOTDATA_NODE_CFG_IX_MESH_BEACON_INTERVAL_S);
    ASSERT_EQ_INT(iotdata_node_config_index_by_name("nonesuch"), -1);

    /* THROUGH THE CONSOLE, which is the path an operator actually uses: `conf debug true`. */
    gateway_config_overlay_path = ovl;
    iotdata_node_config_saver_attach(gateway_config_save);
    iotdata_node_config_console_attach(NULL); /* the overlay IS the store here; there is no datastore */
    {
        char *cargv[3] = { (char *)(uintptr_t)"conf", (char *)(uintptr_t)"debug", (char *)(uintptr_t)"true" };
        iotdata_node_config_console(test_quiet_emit, 1000u, 3, cargv);
        ASSERT(iotdata_node_config_bool(DEBUG));
        f = fopen(ovl, "r");
        ASSERT(f != NULL);
        bool wrote = false;
        while (fgets(line, sizeof(line), f) != NULL)
            if (strstr(line, "debug") != NULL && strstr(line, "true") != NULL)
                wrote = true;
        fclose(f);
        ASSERT(wrote); /* the console said it persisted; the file has to agree */
    }

    /* AND NOTHING ELSE. The operator's own settings are in their file; repeating them here would
       make the overlay shadow it, so editing the file would quietly stop working. */
    {
        f = fopen(ovl, "r");
        ASSERT(f != NULL);
        int keys = 0;
        bool leaked_theirs = false;
        while (fgets(line, sizeof(line), f) != NULL) {
            if (line[0] == '#' || line[0] == '\n')
                continue;
            keys++;
            if (strstr(line, "mqtt-server") != NULL || strstr(line, "lora-channel") != NULL || strstr(line, "mesh-beacon-interval-s") != NULL)
                leaked_theirs = true; /* these came from the .cfg, not from a runtime change */
        }
        fclose(f);
        ASSERT_EQ_INT(keys, 1);
        ASSERT(!leaked_theirs);
    }

    /* a runtime change -- as the console or a CONTROL write would make it -- and the write-back */
    gateway_config_overlay_path = ovl;
    iotdata_node_config_saver_attach(gateway_config_save);
    {
        iotdata_node_config_update_t u;
        iotdata_node_config_update_begin(&u, false);
        ASSERT(iotdata_node_config_update_stage(&u, IOTDATA_NODE_CFG_MESH_BEACON_INTERVAL_S, &(iotdata_node_config_value_t){ .u = 120 }));
        ASSERT(iotdata_node_config_update_commit(&u, NULL, NULL));
    }
    ASSERT_EQ_INT(iotdata_node_config_u16(MESH_BEACON_INTERVAL_S), 120);

    /* the overlay is TEXT, beside the operator's file, and names the key the way the file does */
    f = fopen(ovl, "r");
    ASSERT(f != NULL);
    bool seen = false;
    while (fgets(line, sizeof(line), f) != NULL)
        if (strstr(line, "mesh-beacon-interval-s") != NULL && strstr(line, "120") != NULL)
            seen = true;
    fclose(f);
    ASSERT(seen);

    /* the operator's file is untouched -- still says 90 */
    f = fopen(cfg, "r");
    ASSERT(f != NULL);
    bool original = false;
    while (fgets(line, sizeof(line), f) != NULL)
        if (strstr(line, "mesh-beacon-interval-s") != NULL && strstr(line, "90") != NULL)
            original = true;
    fclose(f);
    ASSERT(original);

    /* a restart: defaults, then the file, then the overlay -- the runtime change wins */
    iotdata_node_config_saver_attach(NULL);
    iotdata_node_config_defaults();
    memset(gateway_config_owned, 0, sizeof(gateway_config_owned));
    ASSERT(gateway_config_load(5, argv));
    ASSERT_EQ_INT(iotdata_node_config_u16(MESH_BEACON_INTERVAL_S), 120); /* the overlay beat the file */
    ASSERT_EQ_INT(iotdata_node_config_u8(LORA_CHANNEL), 42);             /* and the file still holds what it holds */

    (void)remove(cfg);
    (void)remove(ovl);
    (void)rmdir(dir);
    iotdata_node_config_defaults();
    return true;
}

/*
 * ONE FILE CARRIES BOTH, and a pin is a pin.
 *
 * The two things worth holding down: a "node-" line is a protocol SETTING, loaded and written back
 * by the same machinery as the config rows and out of the same file; and a value stated on the
 * command line is immutable for the run, refused from the console as firmly as from the radio.
 */
static bool test_gateway_settings_fused_and_pinned(void) {
    char dir[] = "/tmp/iotdata-gwfuse-XXXXXX";
    ASSERT(mkdtemp(dir) != NULL);
    char cfg[256], ovl[256], line[512];
    snprintf(cfg, sizeof(cfg), "%s/gw.cfg", dir);
    snprintf(ovl, sizeof(ovl), "%s/gw.runtime.cfg", dir);

    FILE *f = fopen(cfg, "w");
    ASSERT(f != NULL);
    fputs("node-station = 7\nnode-interval = 90\nnode-report-status-period = 1800\n", f);
    fclose(f);

    iotdata_node_config_defaults();
    memset(gateway_config_owned, 0, sizeof(gateway_config_owned));
    memset(gateway_settings_owned, 0, sizeof(gateway_settings_owned));
    _iotdata_node_settings_pin = 0;
    _iotdata_node_settings_pin_report = 0;

    char *argv[] = { (char *)(uintptr_t)"gw", (char *)(uintptr_t)"--config", cfg, (char *)(uintptr_t)"--overlay", ovl };
    ASSERT(gateway_config_load(5, argv));

    /* the station is a SETTING and nothing else: no config row seeds it any more, so a file that
       says nothing leaves this node answering as whatever its hardware derived */
    ASSERT_EQ_INT(iotdata_node_settings_station(&gateway_settings), 7);
    ASSERT_EQ_INT(iotdata_node_settings_interval_ms(&gateway_settings) / 60000u, 90); /* the interval is MINUTES */
    ASSERT_EQ_INT(iotdata_node_settings_period_s(&gateway_settings, IOTDATA_NODE_TLV_STATUS), 1800);
    /* and one nobody mentioned is at the seeded default, not at zero */
    ASSERT_EQ_INT(iotdata_node_settings_window_ms(&gateway_settings), IOTDATA_NODE_SETTINGS_DEFAULT_WINDOW_MS);

    /* A RUNTIME CHANGE goes back to the same file -- and only what moved. */
    gateway_config_overlay_path = ovl;
    iotdata_node_settings_saver_attach(gateway_settings_save);
    iotdata_node_config_saver_attach(gateway_config_save);
    gateway_settings.window_ms = 2500;
    ASSERT(iotdata_node_settings_commit(&gateway_settings, NULL));
    {
        f = fopen(ovl, "r");
        ASSERT(f != NULL);
        bool wrote_window = false, leaked_theirs = false;
        while (fgets(line, sizeof(line), f) != NULL) {
            if (strstr(line, "node-window") != NULL && strstr(line, "2500") != NULL)
                wrote_window = true;
            if (strstr(line, "node-interval") != NULL || strstr(line, "node-report-status-period") != NULL)
                leaked_theirs = true; /* those came from the operator's file and are still theirs */
        }
        fclose(f);
        ASSERT(wrote_window);
        ASSERT(!leaked_theirs);
    }

    /* A RESTART: the file, then the overlay. The runtime change survives and the file still holds
       what the operator wrote. */
    iotdata_node_settings_saver_attach(NULL);
    iotdata_node_config_saver_attach(NULL);
    iotdata_node_config_defaults();
    memset(gateway_config_owned, 0, sizeof(gateway_config_owned));
    memset(gateway_settings_owned, 0, sizeof(gateway_settings_owned));
    ASSERT(gateway_config_load(5, argv));
    ASSERT_EQ_INT(iotdata_node_settings_window_ms(&gateway_settings), 2500);
    ASSERT_EQ_INT(iotdata_node_settings_interval_ms(&gateway_settings) / 60000u, 90);

    /* AND WITH NO LINE AT ALL it derives, rather than falling back to some number a config row
       used to hold: the station is the hardware's answer until somebody states otherwise. */
    {
        char bare[256];
        snprintf(bare, sizeof(bare), "%s/bare.cfg", dir);
        FILE *b = fopen(bare, "w");
        ASSERT(b != NULL);
        fputs("# nothing said about the station\n", b);
        fclose(b);
        char barov[256];
        snprintf(barov, sizeof(barov), "%s/bare.runtime.cfg", dir);
        iotdata_node_config_defaults();
        memset(gateway_config_owned, 0, sizeof(gateway_config_owned));
        memset(gateway_settings_owned, 0, sizeof(gateway_settings_owned));
        char *bargv[] = { (char *)(uintptr_t)"gw", (char *)(uintptr_t)"--config", bare, (char *)(uintptr_t)"--overlay", barov };
        ASSERT(gateway_config_load(5, bargv));
        ASSERT_EQ_INT(iotdata_node_settings_station(&gateway_settings), iotdata_node_station_from_mac("test"));
        ASSERT(iotdata_station_is_assignable(iotdata_node_settings_station(&gateway_settings)));
        (void)remove(bare);
        (void)remove(barov);
    }

    /* PINNING. Restart again, this time stating two values on the command line: one config row and
       one setting. Both take, and then neither can be moved. */
    iotdata_node_config_defaults();
    memset(gateway_config_owned, 0, sizeof(gateway_config_owned));
    memset(gateway_settings_owned, 0, sizeof(gateway_settings_owned));
    _iotdata_node_settings_pin = 0;
    _iotdata_node_settings_pin_report = 0;
    {
        char *pargv[] = { (char *)(uintptr_t)"gw",  (char *)(uintptr_t)"--config", cfg, (char *)(uintptr_t)"--overlay", ovl, (char *)(uintptr_t)"--mesh-beacon-interval-s", (char *)(uintptr_t)"45", (char *)(uintptr_t)"--node-window",
                          (char *)(uintptr_t)"7000" };
        ASSERT(gateway_config_load(9, pargv));
    }
    ASSERT_EQ_INT(iotdata_node_config_u16(MESH_BEACON_INTERVAL_S), 45);
    ASSERT_EQ_INT(iotdata_node_settings_window_ms(&gateway_settings), 7000); /* the argument beat the overlay */
    ASSERT(iotdata_node_config_is_pinned(IOTDATA_NODE_CFG_MESH_BEACON_INTERVAL_S));
    ASSERT(iotdata_node_settings_is_pinned(IOTDATA_NODE_SETTINGS_PIN_WINDOW));
    ASSERT(!iotdata_node_settings_is_pinned(IOTDATA_NODE_SETTINGS_PIN_INTERVAL)); /* that one came from the file */

    /* refused over the air */
    {
        iotdata_node_config_update_t u;
        iotdata_node_config_update_begin(&u, true);
        ASSERT(!iotdata_node_config_update_stage(&u, IOTDATA_NODE_CFG_MESH_BEACON_INTERVAL_S, &(iotdata_node_config_value_t){ .u = 120 }));
    }
    /* and refused from the console, which is the part that is a decision rather than a mechanism */
    {
        iotdata_node_config_update_t u;
        iotdata_node_config_update_begin(&u, false);
        ASSERT(!iotdata_node_config_update_stage(&u, IOTDATA_NODE_CFG_MESH_BEACON_INTERVAL_S, &(iotdata_node_config_value_t){ .u = 120 }));
    }
    ASSERT_EQ_INT(iotdata_node_config_u16(MESH_BEACON_INTERVAL_S), 45);
    ASSERT(iotdata_node_config_refusal(IOTDATA_NODE_CFG_MESH_BEACON_INTERVAL_S, false) != NULL);

    /* a pinned SETTING likewise, over the air: the RECEIVE triple arrives whole and the pinned
       field alone stays put, which is why it is pinned per field rather than per key */
    {
        uint8_t kvbuf[32];
        iotdata_kvr_t kv;
        iotdata_kvr_init(&kv, kvbuf, sizeof(kvbuf));
        uint8_t r[IOTDATA_NODE_SETTINGS_RECEIVE_SIZE];
        iotdata_node_settings_put_u16(r, 0, 1234); /* window: pinned */
        iotdata_node_settings_put_u16(r, 2, 30);   /* interval: not */
        iotdata_node_settings_put_u16(r, 4, 0);
        iotdata_kvr_add(&kv, IOTDATA_NODE_SETTINGS_RECEIVE, r, (uint8_t)sizeof(r));
        ASSERT(iotdata_node_settings_apply(&gateway_settings, kvbuf, kv.len));
    }
    ASSERT_EQ_INT(iotdata_node_settings_window_ms(&gateway_settings), 7000);          /* held */
    ASSERT_EQ_INT(iotdata_node_settings_interval_ms(&gateway_settings) / 60000u, 30); /* took */

    _iotdata_node_settings_pin = 0;
    _iotdata_node_settings_pin_report = 0;
    memset(_iotdata_node_config_pinned, 0, sizeof(_iotdata_node_config_pinned));
    (void)remove(cfg);
    (void)remove(ovl);
    (void)rmdir(dir);
    iotdata_node_config_defaults();
    return true;
}

static bool test_config_set_builds_records(void) {
    uint8_t out[64];
    const char *why = NULL;
    cJSON *root = cJSON_Parse("{\"cmd\":\"conf-set\",\"target\":\"5BF\",\"set\":[{\"id\":81,\"type\":1,\"value\":20},{\"id\":66,\"type\":5,\"value\":300000}]}");
    ASSERT(root != NULL);
    const size_t n = ctrl_config_build(root, out, sizeof(out), &why);
    cJSON_Delete(root);
    ASSERT_EQ_INT(n, 3u + 6u); /* [id|type]+u8, then [id|type]+u32 */
    /* 0x051 as u8: the record is (id << 4) | type */
    ASSERT_EQ_INT(((uint16_t)out[0] << 8) | out[1], (uint16_t)((81u << 4) | 1u));
    ASSERT_EQ_INT(out[2], 20u);
    ASSERT_EQ_INT(((uint16_t)out[3] << 8) | out[4], (uint16_t)((66u << 4) | 5u));
    ASSERT_EQ_INT(((uint32_t)out[5] << 24) | ((uint32_t)out[6] << 16) | ((uint32_t)out[7] << 8) | out[8], 300000u);
    ASSERT(iotdata_node_config_rec_id((uint16_t)(((uint16_t)out[3] << 8) | out[4])) == 66u); /* and reads back */
    return true;
}

static bool test_config_set_rejects_malformed(void) {
    uint8_t out[64];
    const char *why = NULL;
    /* no set array at all */
    cJSON *a = cJSON_Parse("{\"cmd\":\"conf-set\"}");
    ASSERT(a != NULL && ctrl_config_build(a, out, sizeof(out), &why) == 0 && why != NULL);
    cJSON_Delete(a);
    /* an id past the 12-bit space */
    cJSON *b = cJSON_Parse("{\"set\":[{\"id\":5000,\"type\":1,\"value\":1}]}");
    ASSERT(b != NULL && ctrl_config_build(b, out, sizeof(out), &why) == 0);
    cJSON_Delete(b);
    /* a variable-width type, which has no fixed record width to write */
    cJSON *c = cJSON_Parse("{\"set\":[{\"id\":81,\"type\":10,\"value\":1}]}");
    ASSERT(c != NULL && ctrl_config_build(c, out, sizeof(out), &why) == 0);
    cJSON_Delete(c);
    /* more records than one frame can carry: refused whole, never truncated */
    cJSON *d = cJSON_Parse("{\"set\":[{\"id\":81,\"type\":7,\"value\":1},{\"id\":82,\"type\":7,\"value\":1}]}");
    ASSERT(d != NULL && ctrl_config_build(d, out, 9, &why) == 0);
    cJSON_Delete(d);
    return true;
}

static bool test_mesh_node_unimplemented_key_refused(void) {
    process_state_t ps;
    mesh_state_t ms;
    stat_state_t ss;
    mesh_node_test_state(&ps, &ms, &ss);
    ASSERT(!ctrl_from_iotdata(IOTDATA_NODE_SUBJECT_MESH, 0x7F, NULL, 0, NULL));                            /* a MESH action nobody defines */
    ASSERT(!ctrl_from_iotdata(IOTDATA_NODE_SUBJECT_NODE, IOTDATA_NODE_ACTION_NODE_REBOOT, NULL, 0, NULL)); /* the node layer's, not ours */
    g_exec = NULL;
    return true;
}

static bool test_mesh_node_control_keys_are_all_handled(void) {
    process_state_t ps;
    mesh_state_t ms;
    static stat_state_t ss;
    mesh_node_test_state(&ps, &ms, &ss);
    /* The advertised list now includes the recorder's keys, and those reach their state through
       g_ctrl exactly as the mesh ones reach theirs through g_exec. A node must not advertise a key
       it cannot service, so the test stands up BOTH -- otherwise it walks straight into a null
       g_ctrl and the crash says nothing about whether the key is handled. */
    static ctrl_state_t cs;
    memset(&cs, 0, sizeof(cs));
    cs.inode = ms.inode; /* same station: the accessors dereference this, a zeroed state cannot */
    /* A REAL RECORDER, because enable/clear/pull reach into the store and a recorder that never
       started is a no-op rather than an answer -- which would let an unhandled DIAG key pass. */
    ASSERT(iotdata_node_diagnostics_begin(0, true));
    /* AND A REAL CONTENT STORE, for the same reason and in RAM for the same reason: the gateway's
       own is a directory out of its config, and a suite must not go near a filesystem. The handler
       refuses every key when no store is attached, which would make the CONTENT keys below pass
       while doing nothing. */
    static filestore_ram_t content_fs;
    static iotdata_node_content_t content_store;
    iotdata_node_content_init(&content_store, filestore_ram_init(&content_fs));
    iotdata_node_content_control_init(&content_store, NULL);
    cs.pool = &t_pool;
    buffer_queue_init(&cs.queue, cs.queue_slot, CTRL_QUEUE_MAX, &t_pool);
    g_ctrl = &cs;
    /* AND EVERY MODULE'S KEYS ARE IN IT. The list is one spliced `static const` array now, so a
       count of zero is no longer reachable -- but a module's pairs being left OUT of the splice
       still is, and the walk below would then pass by walking less. The 7 is this gateway's OWN
       keys and is deliberately a literal: what a node advertises is protocol, and changing it
       should mean changing a line that says what it advertises. The module counts stay symbolic,
       so a key added to the recorder or the store lands here without touching this. */
    ASSERT_EQ_INT(CTRL_FROM_IOTDATA_KEYS_COUNT, 7u + IOTDATA_NODE_DIAGNOSTICS_CONTROL_ACTIONS_COUNT + IOTDATA_NODE_CONTENT_CONTROL_ACTIONS_COUNT);
    /* the advertised list is (subject, action) pairs: every one must be serviced, or a manager is
       sent on a wild goose chase by the node's own inventory */
    for (size_t i = 0; i + 1 < sizeof(ctrl_from_iotdata_keys); i += 2) {
        const uint8_t subject = ctrl_from_iotdata_keys[i], action = ctrl_from_iotdata_keys[i + 1];
        const uint8_t entry[IOTDATA_NODE_CONTROL_UPDATE_ENTRY_SIZE] = { 0x05, 0x37, 0x00 };
        if (!ctrl_from_iotdata(subject, action, entry, (uint8_t)sizeof(entry), NULL)) {
            printf("FAIL (advertised %02X/%02X is not handled)\n", subject, action);
            g_exec = NULL;
            g_ctrl = NULL;
            return false;
        }
    }
    g_exec = NULL;
    g_ctrl = NULL;
    return true;
}

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

static bool test_telemetry_decodes_to_json(void) {
    static iotdata_decode_to_json_scratch_t scratch;
    char *json = NULL;
    ASSERT(iotdata_decode_to_json(telemetry_0538, sizeof(telemetry_0538), &json, &scratch) == IOTDATA_OK);
    ASSERT(json != NULL);
    ASSERT(strstr(json, "\"variant\":3") != NULL);
    free(json);
    return true;
}

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

static bool test_whole_frame_still_decodes(void) {
    process_state_t ps;
    memset(&ps, 0, sizeof(ps));

    uint8_t kv[64];
    iotdata_kvr_t b;
    iotdata_kvr_init(&b, kv, sizeof(kv));
    iotdata_kvr_add_str(&b, IOTDATA_NODE_VERSION_HARDWARE, "esp32c3/riscv32");
    iotdata_kvr_add_str(&b, IOTDATA_NODE_VERSION_FIRMWARE, "idf/6.1+bl1.20");
    iotdata_kvr_add_str(&b, IOTDATA_NODE_VERSION_SOFTWARE, "relay/1.0.0/202609121607");
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

static bool test_control_report_is_published(void) {
    static node_state_t ns;
    static stat_state_t ss;
    memset(&ns, 0, sizeof(ns));
    iotdata_node_init(&t_inode, 0x0001, NULL, 0u);
    ns.inode = &t_inode;
    memset(&ss, 0, sizeof(ss));
    ns.stat = &ss;
    ns.tx = test_packet_handler;
    iotdata_node_down_init(&ns.down, &t_pool);

    uint8_t frame[TEST_FRAME_MAX];
    const int n = build_report(frame, sizeof(frame), 0x0F1B, 5, IOTDATA_NODE_TLV_CONTROL);
    ASSERT(n > 0);
    ASSERT(node_on_packet(&ns, frame, (size_t)n, 0x0F1B));
    ASSERT_EQ_INT(ns.stat_rx, 1u);
    return true;
}

static bool test_down_echo_is_not_published(void) {
    static node_state_t ns;
    static stat_state_t ss;
    memset(&ns, 0, sizeof(ns));
    iotdata_node_init(&t_inode, 0x0001, NULL, 0u);
    ns.inode = &t_inode;
    memset(&ss, 0, sizeof(ss));
    ns.stat = &ss;
    ns.tx = test_packet_handler;
    iotdata_node_down_init(&ns.down, &t_pool);

    uint8_t frame[TEST_FRAME_MAX];
    const int n = build_report(frame, sizeof(frame), 0x0F1B, IOTDATA_SEQUENCE_DOWN, IOTDATA_NODE_TLV_CONTROL);
    ASSERT(n > 0);
    ASSERT(!node_on_packet(&ns, frame, (size_t)n, 0x0F1B));
    ASSERT_EQ_INT(ns.stat_rx, 0u); /* nothing published */
    return true;
}

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

/* A gateway is the top of the pile but still a station: with two or three of them, one can send a
   CONTROL frame at another's station id, across relay hops. That has to reach the SAME handler an
   MQTT request reaches, or a gateway becomes the one node in the fleet that cannot be driven over
   the air. It silently did nothing before -- node_on_packet skipped every DOWN frame outright. */
static bool test_control_over_the_air_reaches_the_same_handler(void) {
    process_state_t ps;
    mesh_state_t ms;
    static stat_state_t ss;
    mesh_node_test_state(&ps, &ms, &ss);
    node_state_t *const ns = ps.state_node;
    iotdata_node_init(&t_inode, 0x0001, NULL, 0u);
    ns->inode = &t_inode;

    /* another gateway addresses a CONTROL at ours: a DOWN frame whose station field is US */
    uint8_t kv[16], frame[TEST_FRAME_MAX];
    iotdata_kvr_t b;
    iotdata_kvr_init(&b, kv, sizeof(kv));
    const uint8_t rq_vers[1] = { IOTDATA_NODE_TLV_VERSION };
    iotdata_kvr_add(&b, IOTDATA_NODE_CONTROL_REQUEST, rq_vers, (uint8_t)sizeof(rq_vers));

    iotdata_encoder_t enc;
    size_t len = 0;
    ASSERT(iotdata_encode_begin(&enc, frame, sizeof(frame), 0, 0x0001, IOTDATA_SEQUENCE_DOWN) == IOTDATA_OK);
    ASSERT(iotdata_encode_tlv(&enc, IOTDATA_NODE_TLV_CONTROL, kv, (uint8_t)b.len) == IOTDATA_OK);
    ASSERT(iotdata_encode_end(&enc, &len) == IOTDATA_OK);

    const uint32_t before = ns->stat_requests;
    (void)node_on_packet(ns, frame, len, 0x0001);
    ASSERT(ns->stat_requests == before + 1); /* the request was acted on, not discarded */

    /* the same frame aimed at somebody else is OUR OWN ECHO coming back off a relay, and must not
       be executed -- a gateway never sends a DOWN frame to itself, so an exact match cannot be one */
    iotdata_kvr_init(&b, kv, sizeof(kv));
    iotdata_kvr_add(&b, IOTDATA_NODE_CONTROL_REQUEST, rq_vers, (uint8_t)sizeof(rq_vers));
    ASSERT(iotdata_encode_begin(&enc, frame, sizeof(frame), 0, 0x0537, IOTDATA_SEQUENCE_DOWN) == IOTDATA_OK);
    ASSERT(iotdata_encode_tlv(&enc, IOTDATA_NODE_TLV_CONTROL, kv, (uint8_t)b.len) == IOTDATA_OK);
    ASSERT(iotdata_encode_end(&enc, &len) == IOTDATA_OK);
    const uint32_t after_ours = ns->stat_requests;
    (void)node_on_packet(ns, frame, len, 0x0537);
    ASSERT(ns->stat_requests == after_ours); /* not ours: left alone */
    return true;
}

int main(void) {
    setbuf(stdout, NULL);

    BUFFER_POOL_INIT(t_pool, TEST_POOL_COUNT, TEST_FRAME_MAX + 8, 0);

    printf("\n=== Mesh Tests ===\n\n");

    RUN_TEST(mesh_begin_disabled);
    RUN_TEST(mesh_begin_enabled);
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
    RUN_TEST(ddup_peers_parse_mqtt_token);
    RUN_TEST(ddup_peer_insert_upsert);
    RUN_TEST(ddup_peer_expire);
    RUN_TEST(ddup_reconfigure_preserves_auto);
    RUN_TEST(ddup_check_and_add_new);
    RUN_TEST(ddup_check_and_add_duplicate);
    RUN_TEST(ddup_check_and_add_different_station);
    RUN_TEST(ddup_check_and_add_different_seq);
    RUN_TEST(ddup_check_and_add_with_pending);
    RUN_TEST(ddup_admit_lowest_publishes);
    RUN_TEST(ddup_admit_not_lowest_holds);
    RUN_TEST(ddup_admit_static_peer_holds);
    RUN_TEST(ddup_hold_ms_effective);
    RUN_TEST(ddup_hold_early_drop);
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

    printf("\n=== Netw Table Tests ===\n\n");

    RUN_TEST(netw_upsert_on_empty);
    RUN_TEST(netw_upsert_in_place);
    RUN_TEST(netw_sequential_upserts);
    RUN_TEST(netw_fill_and_evict_stalest);
    RUN_TEST(netw_all_locatable);

    printf("\n=== Stat Table Tests ===\n\n");

    RUN_TEST(stat_json_shape);
    RUN_TEST(stat_station_create_and_find);
    RUN_TEST(stat_station_fill_and_evict);
    RUN_TEST(stat_station_all_findable);
    RUN_TEST(stat_mesh_peer_create_and_update);
    RUN_TEST(stat_mesh_peer_fill_and_evict);
    RUN_TEST(stat_mesh_peer_remove_compacts);

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
    RUN_TEST(version_report_as_the_monitor_sees_it);
    RUN_TEST(control_over_the_air_reaches_the_same_handler);
    RUN_TEST(mesh_node_table_report);
    RUN_TEST(mesh_node_table_after_remove);
    RUN_TEST(mesh_node_status_absent_when_mesh_off);
    RUN_TEST(mesh_node_filter_update_blocks_rx);
    RUN_TEST(mesh_node_block_drops_peer);
    RUN_TEST(mesh_node_peers_update_and_clear);
    RUN_TEST(mesh_node_filter_update_takes_a_list);
    RUN_TEST(console_poll_never_blocks);
    RUN_TEST(gateway_config_load_and_overlay);
    RUN_TEST(gateway_settings_fused_and_pinned);
    RUN_TEST(config_set_builds_records);
    RUN_TEST(config_set_rejects_malformed);
    RUN_TEST(mesh_node_unimplemented_key_refused);
    RUN_TEST(mesh_node_control_keys_are_all_handled);

    printf("\n=== Ctrl Tests ===\n\n");

    RUN_TEST(ctrl_short_names);
    RUN_TEST(ctrl_table_matches_key_widths);
    RUN_TEST(ctrl_status_scope_differs_from_node_status);
    RUN_TEST(ctrl_mesh_update_entry_shape);
    RUN_TEST(trace_request_shapes_each_leg);
    RUN_TEST(ctrl_response_topic_is_shared);
    RUN_TEST(ctrl_unknown_command_stages_nothing);
    RUN_TEST(ctrl_bad_json_stages_nothing);
    RUN_TEST(ctrl_target_defaults_to_broadcast);
    RUN_TEST(ctrl_requests_queue_in_order);
    RUN_TEST(ctrl_queue_full_is_refused);
    RUN_TEST(ctrl_multiple_targets);
    RUN_TEST(ctrl_staged_command_expires);
    RUN_TEST(manage_from_topic_is_keyed_by_station);
    RUN_TEST(diagnostics_clock_is_monotonic_and_seeded);

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
