// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
//
// iotdata_gateway_state.h - the process state, and the pointer the node hooks reach it through.
//
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

typedef struct {
    netw_t network;
    filter_t filter;
    node_state_t *state_node;
    mesh_state_t *state_mesh;
    survey_state_t *state_survey;
    trace_state_t *state_trace;
    ddup_state_t *state_ddup;
    ddup_peers_t *peers_ddup;
    stat_state_t *state_stat;
    //
    uint32_t stat_display_interval_last_ms;
    uint32_t stat_publish_interval_last_ms;
    uint32_t stat_netw_interval_last_ms; /* the console print: for someone watching */
    uint32_t stat_netw_publish_last_ms;  /* the MQTT publish: for keeping */
    //
    buffer_pool_t *buffer_pool;
    buffer_handle_t buffer_rx;
    //
    bool debug;
    bool debug_data;
    //
    char _buffer_netw_topic[160];
    char _buffer_mqtt_topic[256], _buffer_mqtt_message[1024];
    iotdata_decode_to_json_scratch_t _iotdata_scratch;
    iotdata_decoder_t _iotdata_dec; /* for the validity gate below, kept out of the stack */
} process_state_t;

static process_state_t *g_exec = NULL;

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
