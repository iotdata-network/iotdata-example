#ifndef IOTDATA_GATEWAY_SURVEY_H
#define IOTDATA_GATEWAY_SURVEY_H

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
//
// iotdata_gateway_survey.h - the operator end of DIAG SURVEY (spec Appendix G.3.7, G.4.10-G.4.11).
//
// Three jobs, which are three different things wearing one name:
//   1. ISSUE a directive -- build the DIAG frame and get it on the air;
//   2. OBSERVE, because a gateway hears stations directly too and its own receptions are part of
//      the same measurement. They cost no airtime at all, which makes them the cheapest data in
//      the survey and the easiest to forget to collect;
//   3. COLLECT what the relays send back and publish it.
//
// EVERYTHING COMES BACK ON ONE SUBTREE, the management channel, as {"type":"survey"}. A
// survey produces a stream of small facts from several places at once, and giving each observer
// its own topic would make a client reassemble by subscription rather than by reading a field.
//
// THE RADIO BELONGS TO THE MAIN LOOP. A request arrives on the mosquitto thread, which must not
// touch the radio, so a built frame is STAGED here and aired by the main loop -- the same handoff
// the control queue makes, for the same reason.
//
// A NOTE ON WHAT A SURVEY MEASURES. Only a DIRECT reception counts: a packet heard from the
// station itself, not one relayed here inside a FORWARD. The relays apply the same rule, so what
// arrives is one measurement per node per packet, and joining them on {station, sequence} gives
// the reception pattern for that transmission.
//
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#ifndef GW_SURVEY_DIRECTIVES_MAX
#define GW_SURVEY_DIRECTIVES_MAX 2
#endif
#ifndef GW_SURVEY_STATIONS_MAX
#define GW_SURVEY_STATIONS_MAX 8
#endif
#ifndef GW_SURVEY_QUEUE_MAX
#define GW_SURVEY_QUEUE_MAX 4
#endif
#ifndef GW_SURVEY_QUEUE_TTL_MS
#define GW_SURVEY_QUEUE_TTL_MS 10000u
#endif
#define GW_SURVEY_TAG_FRAME 1

// -----------------------------------------------------------------------------------------------------------------------------------------

/* The survey encoding reuses the radio driver's own dBm convention so that a caller can pass a
   reading straight through untranslated. That only holds while the two agree about what "no
   reading" is -- if they ever diverge, an absent reading becomes a number again, silently. */
_Static_assert(LORA_RSSI_NONE == IOTDATA_MESH_DIAG_RSSI_NONE, "radio and survey disagree about 'no rssi reading'");

typedef struct {
    bool used;
    uint16_t directive_id;
    uint16_t station[GW_SURVEY_STATIONS_MAX];
    int station_count;
    time_t expiry; /* wall clock: a gateway has one, and an operator reads seconds not ticks */
    uint32_t observed;
} gw_survey_directive_t;

typedef struct survey_state_s {
    iotdata_node_t *inode;
    buffer_pool_t *pool;
    char topic_from[128];
    gw_survey_directive_t d[GW_SURVEY_DIRECTIVES_MAX];
    buffer_queue_t queue;
    buffer_queue_entry_t queue_slot[GW_SURVEY_QUEUE_MAX];
    uint16_t next_id;
    uint32_t stat_issued, stat_observed, stat_collected, stat_refused;
} survey_state_t;

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static const char *survey_tag_name(const uint8_t tag) {
    return (tag == GW_SURVEY_TAG_FRAME) ? "DIAG" : "NONE";
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static int survey_locate(const survey_state_t *const st, const uint16_t directive_id, int *const spare) {
    int slot = -1, free_slot = -1;
    for (int i = 0; i < (int)(sizeof(st->d) / sizeof(st->d[0])); i++) {
        const gw_survey_directive_t *const d = &st->d[i];
        if (!d->used) {
            if (free_slot < 0)
                free_slot = i;
        } else if (d->directive_id == directive_id) {
            slot = i;
            break;
        }
    }
    if (spare != NULL)
        *spare = free_slot;
    return slot;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static int survey_count(const survey_state_t *const st) {
    int n = 0;
    for (int i = 0; i < (int)(sizeof(st->d) / sizeof(st->d[0])); i++)
        if (st->d[i].used)
            n++;
    return n;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* One observation, from wherever it was made. `observer` is the node that heard it -- this
   gateway for a direct reception, a relay for one that arrived in a response -- and that
   distinction is the entire point of the survey, so it is never implied. */
static void survey_publish(survey_state_t *const st, const uint16_t directive_id, const uint16_t observer, const uint16_t station, const uint16_t sequence, const int rssi_dbm, const int8_t snr_q4,
                           const iotdata_mesh_diag_position_t *const observer_pos) {
    cJSON *const root = cJSON_CreateObject();
    if (root == NULL)
        return;
    cJSON_AddStringToObject(root, "type", "survey");
    cJSON_AddStringToObject(root, "cmd", "survey");
    cJSON_AddNumberToObject(root, "directive", (double)directive_id);
    cJSON_AddNumberToObject(root, "observer", (double)observer);
    cJSON_AddNumberToObject(root, "station", (double)station);
    cJSON_AddNumberToObject(root, "seq", (double)sequence);
    /* OMITTED when there is no reading, rather than sent as a sentinel: anything that averages or
       plots an rssi column would fold a sentinel silently into the result. Saturation is a
       category of its own and says so -- it is a real reception, just not a measurable one. */
    if (iotdata_mesh_rssi_is_reading(rssi_dbm))
        cJSON_AddNumberToObject(root, "rssi", (double)rssi_dbm);
    else if (rssi_dbm == IOTDATA_MESH_DIAG_RSSI_SAT)
        cJSON_AddBoolToObject(root, "rssi_saturated", true);
    /* Absent, not zero: a radio that cannot measure SNR must not appear to have measured 0 dB. */
    if (snr_q4 != IOTDATA_MESH_DIAG_SNR_NONE)
        cJSON_AddNumberToObject(root, "snr", (double)snr_q4 / 4.0);
    /* Carried with every observation rather than published once and left to be joined: an
       observer that says where it stands is what turns a column of numbers into a map. */
    if (observer_pos != NULL) {
        cJSON_AddNumberToObject(root, "obs_lat", (double)observer_pos->latitude_e7 / 1e7);
        cJSON_AddNumberToObject(root, "obs_lon", (double)observer_pos->longitude_e7 / 1e7);
        cJSON_AddNumberToObject(root, "obs_alt", (double)observer_pos->altitude_m);
    }
    cJSON_AddNumberToObject(root, "time", (double)time(NULL));
    char *const out = cJSON_PrintUnformatted(root);
    if (out != NULL) {
        char topic[GW_TOPIC_STATION_MAX];
        (void)mqtt_send(gateway_topic_for_station(topic, sizeof(topic), st->topic_from, station), out, (int)strlen(out));
        free(out);
    }
    cJSON_Delete(root);
}

static void survey_publish_note(survey_state_t *const st, const char *const cmd, const char *const text) {
    cJSON *const root = cJSON_CreateObject();
    if (root == NULL)
        return;
    cJSON_AddStringToObject(root, "type", "survey");
    cJSON_AddStringToObject(root, "cmd", cmd);
    const uint16_t self = iotdata_node_station(st->inode);
    cJSON_AddNumberToObject(root, "station", (double)self);
    cJSON_AddStringToObject(root, "text", text);
    char *const out = cJSON_PrintUnformatted(root);
    if (out != NULL) {
        char topic[GW_TOPIC_STATION_MAX];
        (void)mqtt_send(gateway_topic_for_station(topic, sizeof(topic), st->topic_from, self), out, (int)strlen(out));
        free(out);
    }
    cJSON_Delete(root);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

/* Stage a built frame for the main loop to air. Called on the mosquitto thread. */
static bool survey_stage(survey_state_t *const st, const uint8_t *const frame, const int len) {
    bool ok = false;
    if (len > 0) {
        const buffer_handle_t h = buffer_acquire(st->pool);
        if (h != BUFFER_NONE) {
            (void)buffer_write(st->pool, h, frame, (uint16_t)len);
            const uint32_t now_ms = hw_time_ms();
            ok = buffer_queue_add(&st->queue, h, now_ms, now_ms + GW_SURVEY_QUEUE_TTL_MS, GW_SURVEY_TAG_FRAME, 0);
            buffer_unref(st->pool, h);
        }
    }
    return ok;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* Issue a directive: onto the air, and into our own watch list -- a gateway that asks the question
   without listening for the answer itself would throw away the one observation that is free. */
static bool survey_request(survey_state_t *const st, const uint16_t directive_id, const uint16_t duration_m, const uint16_t batch_s, const uint16_t *const stations, const int count, const char **const why) {
    if (count <= 0 || count > GW_SURVEY_STATIONS_MAX) {
        *why = "no stations, or more than a node will hold";
        return false;
    }
    if (duration_m == 0u) {
        *why = "a directive with no lifetime does nothing";
        return false;
    }
    if (batch_s == 0u) {
        *why = "a reporting window of zero would spin";
        return false;
    }
    int spare = -1;
    if (survey_locate(st, directive_id, &spare) >= 0) {
        *why = "that directive id is already running here";
        return false;
    }
    uint8_t frame[IOTDATA_MAX_PACKET_SIZE]; // XXX
    /* Peeked. A sequence is spent only once the frame is real AND the staging queue has taken
       ownership of it -- a pack that would not fit, or a queue with no room, must leave the counter
       alone, or the gap reads as a frame that went missing rather than one that never existed. */
    const uint16_t seq = iotdata_node_sequence(st->inode);
    const int len = iotdata_mesh_pack_diag_survey_request(frame, (int)sizeof(frame), iotdata_node_station(st->inode), seq, directive_id, duration_m, batch_s, stations, count);
    if (len <= 0) {
        *why = "the request would not fit a frame";
        return false;
    }
    if (!survey_stage(st, frame, len)) {
        *why = "busy: nothing staged";
        return false;
    }
    iotdata_node_sequence_used(st->inode);
    if (spare >= 0) {
        gw_survey_directive_t *const d = &st->d[spare];
        *d = (gw_survey_directive_t){ 0 };
        d->used = true;
        d->directive_id = directive_id;
        d->station_count = count;
        for (int i = 0; i < count; i++)
            d->station[i] = stations[i];
        d->expiry = time(NULL) + (time_t)duration_m * 60;
    } /* else: we still issue it, we just cannot watch here as well -- said so in the reply */
    st->stat_issued++;
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* 12 bits, never 0 -- 0 is what CANCEL_ALL carries, and an id that collides with it would make a
   cancel of one look like a cancel of everything. */
static uint16_t survey_next_id(survey_state_t *const st) {
    st->next_id = (uint16_t)((st->next_id + 1u) & 0x0FFFu);
    if (st->next_id == 0u)
        st->next_id = 1u;
    return st->next_id;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static bool survey_cancel(survey_state_t *const st, const uint16_t directive_id, const bool all, const char **const why) {
    uint8_t frame[IOTDATA_MESH_DIAG_CANCEL_SIZE];
    const uint16_t seq = iotdata_node_sequence(st->inode); /* peeked, spent below only if it is staged */
    const int len = all ? iotdata_mesh_pack_diag_cancel_all(frame, (int)sizeof(frame), iotdata_node_station(st->inode), seq, IOTDATA_MESH_DIAG_SURVEY)
                        : iotdata_mesh_pack_diag_cancel(frame, (int)sizeof(frame), iotdata_node_station(st->inode), seq, IOTDATA_MESH_DIAG_SURVEY, directive_id);
    if (len <= 0 || !survey_stage(st, frame, len)) {
        *why = "busy: nothing staged";
        return false;
    }
    iotdata_node_sequence_used(st->inode);
    for (int i = 0; i < (int)(sizeof(st->d) / sizeof(st->d[0])); i++) {
        gw_survey_directive_t *const d = &st->d[i];
        if (d->used && (all || d->directive_id == directive_id))
            d->used = false;
    }
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* A packet heard DIRECTLY by this gateway. Published immediately: there is no airtime to save
   here, and a batch would only add latency to the one observation that did not need any. */
static int survey_on_direct(survey_state_t *const st, const uint16_t station, const uint16_t sequence, const int rssi_dbm) {
    int n = 0;
    for (int i = 0; i < (int)(sizeof(st->d) / sizeof(st->d[0])); i++) {
        gw_survey_directive_t *const d = &st->d[i];
        if (d->used) {
            for (int j = 0; j < d->station_count; j++)
                if (d->station[j] == station) {
                    survey_publish(st, d->directive_id, iotdata_node_station(st->inode), station, sequence, rssi_dbm, IOTDATA_MESH_DIAG_SNR_NONE, NULL);
                    d->observed++;
                    st->stat_observed++;
                    n++;
                    break;
                }
        }
    }
    return n;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* A DIAG frame from a relay -- always arriving inside a FORWARD, since a response is carried like
   data so it is acknowledged and retried. An empty RESPONSE is published as a note rather than
   dropped: "live here, heard nothing" is the answer that stops silence reading as no coverage. */
static bool survey_on_frame(survey_state_t *const st, const uint8_t *const buf, const int len) {
    iotdata_mesh_diag_t dg;
    if (iotdata_mesh_unpack_diag(buf, len, &dg) && dg.kind == IOTDATA_MESH_DIAG_SURVEY) {
        char note[160];
        if (dg.op == IOTDATA_MESH_DIAG_OP_RESPONSE) {
            if (!iotdata_mesh_diag_survey_response_ok(buf, len)) {
                survey_publish_note(st, "survey", snprintf_inline(note, sizeof(note), "%03" PRIX16 ": malformed response -- body is not a whole number of records", dg.sender_station));
                return false;
            }
            iotdata_mesh_diag_position_t pos;
            const bool has_pos = iotdata_mesh_diag_survey_position(buf, len, &pos);
            int off = IOTDATA_MESH_DIAG_SURVEY_RSP_SIZE, count = 0;
            iotdata_mesh_diag_survey_entry_t e;
            while (iotdata_mesh_diag_survey_entry_next(buf, len, &off, &e)) {
                survey_publish(st, dg.directive_id, dg.sender_station, e.station, e.sequence, e.rssi_dbm, e.snr_q4, has_pos ? &pos : NULL);
                st->stat_collected++;
                count++;
            }
            if (count == 0)
                survey_publish_note(st, "survey", snprintf_inline(note, sizeof(note), "%03" PRIX16 ": directive %03" PRIX16 " live, heard nothing this window", dg.sender_station, dg.directive_id));
            return true;
        } else if (dg.op == IOTDATA_MESH_DIAG_OP_ERROR) {
            const uint8_t reason = (len > IOTDATA_MESH_DIAG_HDR_SIZE) ? buf[IOTDATA_MESH_DIAG_HDR_SIZE] : 0xFFu;
            const char *const text = (reason == IOTDATA_MESH_DIAG_ERR_FULL)          ? "no room for another directive"
                                     : (reason == IOTDATA_MESH_DIAG_ERR_MALFORMED)   ? "could not parse the request"
                                     : (reason == IOTDATA_MESH_DIAG_ERR_UNSUPPORTED) ? "does not implement survey"
                                                                                     : "refused";
            st->stat_refused++;
            survey_publish_note(st, "survey", snprintf_inline(note, sizeof(note), "%03" PRIX16 " refused directive %03" PRIX16 ": %s", dg.sender_station, dg.directive_id, text));
            return true;
        }
    }
    return false;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

/* Main loop: air one staged frame, and retire directives whose lifetime has run out. */
static void survey_tick(survey_state_t *const st, mesh_state_t *const mesh) {
    const uint32_t now_ms = hw_time_ms();
    const uint16_t stale = buffer_queue_expire(&st->queue, now_ms);
    if (stale > 0)
        PRINTF_WARN("survey: %u staged frame(s) expired unsent\n", (unsigned)stale);
    uint32_t key = 0;
    uint8_t tag = 0;
    const buffer_handle_t h = buffer_queue_take(&st->queue, now_ms, &tag, &key);
    if (h != BUFFER_NONE) {
        const uint8_t *const buf = buffer_data(st->pool, h);
        const int len = (int)buffer_len(st->pool, h);
        if (mesh->packet_handler != NULL && mesh->packet_handler(buf, len))
            PRINTF_INFO("survey: aired a %d byte DIAG frame\n", len);
        else
            PRINTF_ERROR("survey: could not air a staged DIAG frame (%d bytes)\n", len);
        buffer_unref(st->pool, h);
    }
    const time_t now = time(NULL);
    for (int i = 0; i < (int)(sizeof(st->d) / sizeof(st->d[0])); i++) {
        gw_survey_directive_t *const d = &st->d[i];
        if (d->used && now >= d->expiry) {
            d->used = false;
            PRINTF_INFO("survey: directive %03" PRIX16 " expired here (%" PRIu32 " observed directly)\n", d->directive_id, d->observed);
        }
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static void survey_begin(survey_state_t *const st, const char *const topic_prefix, iotdata_node_t *const inode, buffer_pool_t *const pool) {
    assert(st && pool);
    *st = (survey_state_t){ 0 };
    st->inode = inode;
    st->pool = pool;
    buffer_queue_init(&st->queue, st->queue_slot, GW_SURVEY_QUEUE_MAX, pool);
    buffer_queue_set_tag_name(&st->queue, survey_tag_name);
    snprintf(st->topic_from, sizeof(st->topic_from), "%s" IOTDATA_NODE_MQTT_MANAGE_TOPIC_FROM, topic_prefix);
}

static void survey_end(survey_state_t *const st) {
    buffer_queue_clear(&st->queue);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#endif /* IOTDATA_GATEWAY_SURVEY_H */
