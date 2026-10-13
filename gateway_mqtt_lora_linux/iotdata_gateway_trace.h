
#ifndef IOTDATA_GATEWAY_TRACE_H
#define IOTDATA_GATEWAY_TRACE_H

// -----------------------------------------------------------------------------------------------------------------------------------------
// iotdata_gateway_trace.h - DIAG kind TRACE at the gateway: a path arrives, and is published whole.
//
// A TRACE IS ONE ARTIFACT, NOT A COLUMN OF OBSERVATIONS, which is what makes this different from
// survey. A survey response is N independent sightings and each is published on its own, because
// each stands alone. A trace is a single ordered path, and splitting it into one message per hop
// would hand a consumer the job of reassembling it -- including deciding which hops belong to which
// traversal when two arrive at once. So the hops go out as an array, in order, in one message.
//
// THE GATEWAY IS THE LAST HOP of an inward record, and says so. The RSSI it heard the final relay
// at is the margin on the last link; without it the path has one unmeasured leg, which is the leg
// closest to the operator reading it. A RESPONSE is NOT appended to: its hops describe the DOWNWARD
// leg, and adding our reception of the return journey would mix two legs into one list. That is the
// same RECORD-vs-RESPONSE distinction the relays rely on, doing the same job here.
//
// -----------------------------------------------------------------------------------------------------------------------------------------

#ifndef GW_TRACE_QUEUE_MAX
#define GW_TRACE_QUEUE_MAX 4
#endif
#define GW_TRACE_TAG_FRAME 1
/* A gateway originates at most a header plus its own hop; relays grow it from there. */
#define TRACE_FRAME_GW_MAX (IOTDATA_MESH_DIAG_TRACE_HDR_SIZE + IOTDATA_MESH_DIAG_TRACE_HOP)

typedef struct trace_state_s {
    iotdata_node_t *inode;
    buffer_pool_t *pool;
    char topic_from[128];
    buffer_queue_t queue;
    buffer_queue_entry_t queue_slot[GW_TRACE_QUEUE_MAX];
    uint16_t next_id;
    uint32_t stat_issued, stat_records, stat_responses, stat_errors, stat_malformed, stat_truncated;
} trace_state_t;

static const char *trace_tag_name(const uint8_t tag) {
    return (tag == GW_TRACE_TAG_FRAME) ? "trace" : "?";
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void trace_publish_note(trace_state_t *const st, const char *const text) {
    cJSON *const root = cJSON_CreateObject();
    if (root == NULL)
        return;
    const uint16_t self = iotdata_node_station(st->inode);
    cJSON_AddStringToObject(root, "type", "trace");
    cJSON_AddStringToObject(root, "cmd", "trace");
    /* WHO IT IS FROM, which is us: it was missing here, and the topic is keyed on it */
    cJSON_AddNumberToObject(root, "station", (double)self);
    cJSON_AddStringToObject(root, "note", text);
    cJSON_AddNumberToObject(root, "time", (double)time(NULL));
    char *const out = cJSON_PrintUnformatted(root);
    if (out != NULL) {
        char topic[GW_TOPIC_STATION_MAX];
        (void)mqtt_send(gateway_topic_for_station(topic, sizeof(topic), st->topic_from, self), out, (int)strlen(out));
        free(out);
    }
    cJSON_Delete(root);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* `final` is the gateway's own hop, appended to an inward RECORD and NULL for anything else. */
static void trace_publish(trace_state_t *const st, const iotdata_mesh_diag_t *const dg, const uint8_t *const buf, const int len, const iotdata_mesh_nbr_entry_t *const final) {
    iotdata_mesh_diag_trace_t tr;
    if (iotdata_mesh_unpack_diag_trace(buf, len, &tr)) {
        cJSON *const root = cJSON_CreateObject();
        if (root != NULL) {
                    const uint16_t self = iotdata_node_station(st->inode);
            cJSON_AddStringToObject(root, "type", "trace");
            cJSON_AddStringToObject(root, "cmd", "trace");
            /* WHO ANSWERED (us), as against the `subject` the trace is ABOUT -- the topic is keyed
               on this one, so a trace of F1B arrives on the gateway's topic, not on F1B's */
            cJSON_AddNumberToObject(root, "station", (double)self);
            cJSON_AddNumberToObject(root, "directive", (double)dg->directive_id);
            cJSON_AddNumberToObject(root, "subject", (double)tr.subject_station);
            cJSON_AddStringToObject(root, "leg", ((tr.flags & IOTDATA_MESH_DIAG_TRACE_F_DOWN) != 0) ? "down" : "up");
            cJSON_AddStringToObject(root, "mode", ((tr.flags & IOTDATA_MESH_DIAG_TRACE_F_ONE) != 0) ? "one" : "all");
            /* Stated either way: "the path is complete" is a claim worth making, not an absence. */
            cJSON_AddBoolToObject(root, "truncated", (tr.flags & IOTDATA_MESH_DIAG_TRACE_F_TRUNCATED) != 0);
            cJSON *const hops = cJSON_AddArrayToObject(root, "hops");
            if (hops != NULL)
                for (int i = 0; i <= tr.hop_count; i++) {
                    iotdata_mesh_nbr_entry_t h;
                    if (i < tr.hop_count) {
                        if (!iotdata_mesh_diag_trace_hop(buf, len, i, &h))
                            break;
                    } else if (final != NULL)
                        h = *final;
                    else
                        break;
                    cJSON *const o = cJSON_CreateObject();
                    if (o == NULL)
                        break;
                    cJSON_AddNumberToObject(o, "station", (double)h.station);
                    /* OMITTED rather than sent as a number: 0xFF means "has no gradient", which is what a
                       sensor or a roaming instrument is, and 255 would plot as a very distant relay. */
                    if (h.cost != IOTDATA_MESH_NBR_COST_SENSOR)
                        cJSON_AddNumberToObject(o, "cost", (double)h.cost);
                    /* Same three states survey publishes, for the same reason: an absent reading must not
                       average in as a number, and saturation is a reception rather than a measurement. */
                    if (iotdata_mesh_rssi_is_reading(h.rssi_dbm))
                        cJSON_AddNumberToObject(o, "rssi", (double)h.rssi_dbm);
                    else if (h.rssi_dbm == IOTDATA_MESH_DIAG_RSSI_SAT)
                        cJSON_AddBoolToObject(o, "rssi_saturated", true);
                    cJSON_AddItemToArray(hops, o);
                }
            cJSON_AddNumberToObject(root, "time", (double)time(NULL));
            char *const out = cJSON_PrintUnformatted(root);
            if (out != NULL) {
                char topic[GW_TOPIC_STATION_MAX];
                (void)mqtt_send(gateway_topic_for_station(topic, sizeof(topic), st->topic_from, self), out, (int)strlen(out));
                free(out);
            }
            cJSON_Delete(root);
        }
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static bool trace_on_frame(trace_state_t *const st, const uint8_t *const buf, const int len, const int rssi_dbm) {
    iotdata_mesh_diag_t dg;
    if (iotdata_mesh_unpack_diag(buf, len, &dg) && dg.kind == IOTDATA_MESH_DIAG_TRACE) {
        char note[192];
        iotdata_mesh_diag_trace_t tr;
        if (dg.op == IOTDATA_MESH_DIAG_OP_RECORD || dg.op == IOTDATA_MESH_DIAG_OP_RESPONSE) {
            if (iotdata_mesh_unpack_diag_trace(buf, len, &tr)) {
                if ((tr.flags & IOTDATA_MESH_DIAG_TRACE_F_TRUNCATED) != 0)
                    st->stat_truncated++;
                if (dg.op == IOTDATA_MESH_DIAG_OP_RECORD) {
                    /* We are the end of this path, and the link we heard it over is part of it. */
                    const iotdata_mesh_nbr_entry_t me = { .cost = 0u, .rssi_dbm = rssi_dbm, .station = iotdata_node_station(st->inode) };
                    trace_publish(st, &dg, buf, len, &me);
                    st->stat_records++;
                } else {
                    trace_publish(st, &dg, buf, len, NULL); /* a finished DOWN leg: not ours to extend */
                    st->stat_responses++;
                }
                return true;
            } else {
                st->stat_malformed++;
                trace_publish_note(st, snprintf_inline(note, sizeof(note), "%03" PRIX16 ": malformed trace -- body is not a whole number of hop records", dg.sender_station));
                return false;
            }
        } else if (dg.op == IOTDATA_MESH_DIAG_OP_ERROR) {
            const uint8_t reason = (len > IOTDATA_MESH_DIAG_HDR_SIZE) ? buf[IOTDATA_MESH_DIAG_HDR_SIZE] : 0xFFu;
            const char *const text = (reason == IOTDATA_MESH_DIAG_ERR_UNSUPPORTED) ? "does not implement trace" : (reason == IOTDATA_MESH_DIAG_ERR_MALFORMED) ? "could not parse the request" : "refused the request";
            st->stat_errors++;
            trace_publish_note(st, snprintf_inline(note, sizeof(note), "%03" PRIX16 ": directive %03" PRIX16 " -- %s", dg.sender_station, dg.directive_id, text));
            return true;
        }
    }
    return false; /* a request passing by: somebody else's business */
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static uint16_t trace_next_id(trace_state_t *const st) {
    st->next_id = (uint16_t)((st->next_id + 1u) & 0x0FFFu);
    if (st->next_id == 0u)
        st->next_id = 1u;
    return st->next_id;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static bool trace_stage(trace_state_t *const st, const uint8_t *const frame, const int len) {
    bool ok = false;
    if (len > 0) {
        const buffer_handle_t h = buffer_acquire(st->pool);
        if (h != BUFFER_NONE) {
            (void)buffer_write(st->pool, h, frame, (uint16_t)len);
            const uint32_t now_ms = hw_time_ms();
            ok = buffer_queue_add(&st->queue, h, now_ms, now_ms + 10000u, GW_TRACE_TAG_FRAME, 0);
            buffer_unref(st->pool, h);
        }
    }
    return ok;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* THE TWO LEGS ARE DIFFERENT FRAMES, not one frame with a flag, and that is the point of splitting
   request from record:
   - UP asks the subject to start recording, so it is a REQUEST: small, carrying no hops, and the
     record it provokes travels inward collecting them.
   - DOWN cannot ask anyone -- the recording has to happen on the way out -- so the gateway
     originates the RECORD itself, with itself as hop 0, and every relay that floods it appends.
   RESPOND defaults differently for the same reason: an upward record ends at us and is published,
   so there is nothing to answer; a downward one ends at the subject, and without an answer the
   operator learns nothing. */
static bool trace_request(trace_state_t *const st, const uint16_t directive_id, const uint16_t subject, const bool down, const bool one, const bool respond, const char **const why) {
    if (!iotdata_station_is_assignable(subject)) { /* the protocol's own rule, not a local guess */
        *why = "that is not an assignable station";
        return false;
    }
    if (directive_id == 0u) {
        *why = "directive 0 means 'unasked' and cannot be requested";
        return false;
    }
    const uint8_t flags = (uint8_t)((down ? IOTDATA_MESH_DIAG_TRACE_F_DOWN : 0u) | (one ? IOTDATA_MESH_DIAG_TRACE_F_ONE : 0u) | (respond ? IOTDATA_MESH_DIAG_TRACE_F_RESPOND : 0u));
    uint8_t frame[TRACE_FRAME_GW_MAX];
    /* Peeked: a sequence is spent only once the frame is real AND the queue owns it. A pack that
       would not fit must leave the counter alone, or the gap reads as a frame that went missing
       rather than one that never existed. */
    const uint16_t seq = iotdata_node_sequence(st->inode);
    int len;
    if (down) {
        const iotdata_mesh_nbr_entry_t me = { .cost = 0u, .rssi_dbm = IOTDATA_MESH_DIAG_RSSI_NONE, .station = iotdata_node_station(st->inode) };
        len = iotdata_mesh_pack_diag_trace(frame, (int)sizeof(frame), iotdata_node_station(st->inode), seq, IOTDATA_MESH_DIAG_OP_RECORD, directive_id, flags, subject, &me, 1);
    } else
        len = iotdata_mesh_pack_diag_trace_request(frame, (int)sizeof(frame), iotdata_node_station(st->inode), seq, directive_id, flags, subject);
    if (len <= 0) {
        *why = "the frame would not build";
        return false;
    }
    if (!trace_stage(st, frame, len)) {
        *why = "busy: nothing staged";
        return false;
    }
    iotdata_node_sequence_used(st->inode);
    st->stat_issued++;
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void trace_tick(trace_state_t *const st, mesh_state_t *const mesh) {
    const uint32_t now_ms = hw_time_ms();
    const uint16_t stale = buffer_queue_expire(&st->queue, now_ms);
    if (stale > 0)
        PRINTF_WARN("trace: %u staged frame(s) expired unsent\n", (unsigned)stale);
    uint8_t tag = 0;
    uint32_t key = 0;
    const buffer_handle_t h = buffer_queue_take(&st->queue, now_ms, &tag, &key);
    if (h != BUFFER_NONE) {
        const uint8_t *const buf = buffer_data(st->pool, h);
        const int len = (int)buffer_len(st->pool, h);
        if (mesh->packet_handler != NULL && mesh->packet_handler(buf, len))
            PRINTF_INFO("trace: aired a %d byte DIAG frame\n", len);
        else
            PRINTF_ERROR("trace: could not air a staged DIAG frame (%d bytes)\n", len);
        buffer_unref(st->pool, h);
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void trace_begin(trace_state_t *const st, const char *const topic_prefix, iotdata_node_t *const inode, buffer_pool_t *const pool) {
    assert(st && pool);
    *st = (trace_state_t){ 0 };
    st->inode = inode;
    st->pool = pool;
    buffer_queue_init(&st->queue, st->queue_slot, GW_TRACE_QUEUE_MAX, pool);
    buffer_queue_set_tag_name(&st->queue, trace_tag_name);
    snprintf(st->topic_from, sizeof(st->topic_from), "%s" IOTDATA_NODE_MQTT_MANAGE_TOPIC_FROM, topic_prefix);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void trace_end(trace_state_t *const st) {
    buffer_queue_clear(&st->queue);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#endif
