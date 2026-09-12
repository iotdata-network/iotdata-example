// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
//
// iotdata_gateway_state.h - the process state, and the pointer the node hooks reach it through.
//
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

typedef struct {
    const char *mqtt_topic_prefix;
    bool capture_rssi_packet;
    bool capture_rssi_channel;
    time_t interval_rssi_channel;
    time_t interval_rssi_channel_last;
    time_t stat_display_interval;
    time_t stat_display_interval_last;
    time_t stat_publish_interval;
    time_t stat_publish_interval_last;
    time_t stat_netw_interval;
    time_t stat_netw_interval_last;
    netw_t network;  /* stations heard (mesh + sensor), printed every stat-network-interval */
    filter_t filter; /* per-station RX allow/block, exactly as a relay has one */
    node_state_t *state_node;
    mesh_state_t *state_mesh;
    ddup_state_t *state_ddup;
    stat_state_t *state_stat;
    char _buffer_mqtt_topic[256], _buffer_mqtt_message[1024];
    iotdata_decode_to_json_scratch_t _iotdata_scratch;
    iotdata_decoder_t _iotdata_dec; /* for the validity gate below, kept out of the stack */
    buffer_pool_t *pool;
    /* The receive buffer, held ACROSS cycles. Most cycles read nothing, so acquiring and releasing
       one per cycle was a pair of pool operations to no purpose: this keeps the same buffer until a
       frame actually lands in it and is done with. One buffer stays out of the pool for the life of
       the run, which at this depth is the right trade. BUFFER_NONE means we hold none. */
    buffer_handle_t rx_held;
    bool debug;
    bool debug_data;
} process_state_t;

static process_state_t *g_exec = NULL;

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
