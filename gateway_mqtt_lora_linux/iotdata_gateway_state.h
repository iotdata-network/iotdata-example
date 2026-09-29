// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
//
// iotdata_gateway_state.h - the process state, and the pointer the node hooks reach it through.
//
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

typedef struct {
    time_t stat_display_interval_last;
    time_t stat_publish_interval_last;
    time_t stat_netw_interval_last;
    netw_t network;  /* stations heard (mesh + sensor), printed every stat-network-interval */
    filter_t filter; /* per-station RX allow/block, exactly as a relay has one */
    node_state_t *state_node;
    mesh_state_t *state_mesh;
    ddup_state_t *state_ddup;
    /* borrowed, and NULL unless ddup auto-peering is on: the presence channel is the only thing
       that keeps an auto peer table fresh, and it is ticked from the process loop */
    struct peer_presence_s *state_peer;
    stat_state_t *state_stat;
    struct survey_state_s *state_survey;
    char _buffer_mqtt_topic[256], _buffer_mqtt_message[1024];
    iotdata_decode_to_json_scratch_t _iotdata_scratch;
    iotdata_decoder_t _iotdata_dec; /* for the validity gate below, kept out of the stack */
    buffer_pool_t *pool;
    buffer_handle_t rx_held;
    bool debug;
    bool debug_data;
} process_state_t;

static process_state_t *g_exec = NULL;

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
