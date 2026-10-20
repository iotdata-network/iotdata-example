#ifndef APP_SURVEY_H
#define APP_SURVEY_H

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
//
// app_survey.h - a handheld survey instrument: one button, four gestures, and both halves of the
// measurement.
//
// IT IS NOT A SENSOR, and stopped pretending to be one. A sensor wakes, reads the world, speaks and
// sleeps; this listens continuously and speaks on purpose. It has no variant, no telemetry, and
// nothing in the gateway's sensor tables -- it appears only as survey data, because it is an
// instrument rather than a thing being measured.
//
// BOTH HALVES, which is the whole reason for the rewrite. A survey that only asks "who hears me"
// silently overstates coverage: the two ends of a LoRa link are not symmetric -- different transmit
// powers, different antenna heights, different noise floors -- so a spot where the beacons come in
// perfectly can be a spot where nothing can hear you. Relays report what they hear of us; this
// reports what it hears of them; the two are joined by POSITION and TIME, not by sequence, because
// they are different events. That is the second reason the position has to be on the wire.
//
// ONE FRAME DOES BOTH JOBS. What it transmits is a DIAG SURVEY RESPONSE carrying its observations
// and its position -- and that same transmission is what the relays measure. There is no separate
// beacon to arrange and nothing to correlate.
//
// IT GETS HOME AS A LEAF. The response is wrapped in a FORWARD and sent bare. A relay's gradient
// check treats a sender it has never heard beacon as infinitely far away, so any joined relay is
// closer and carries it -- with dedup, hop ACK and retry, and no new code anywhere. This node is
// structurally a relay that only ever originates.
//
//   single press   report now -- what was heard, and where. Survey or no survey.
//   double press   start a survey, or stop the one running
//   long  press    start reporting on a timer, or stop it
//   xlong press    cancel everything and reboot
//
// The two modes stay independent: a survey is about who hears THIS node, the timer is about how
// often it speaks. Note they measure different halves -- with the timer off, the relays have
// nothing of ours to hear and only this node's own observations arrive.
//
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static const char *__tag_survey = "survey";

#ifndef CONSOLE_DRAIN_MS
#define CONSOLE_DRAIN_MS 100
#endif

/* The directive is short-lived and RE-ISSUED while the walk continues, rather than issued once for
   an optimistic duration. Re-issue is idempotent by design -- a relay already holding the id passes
   it on and carries on unchanged -- and the useful consequence is the one that matters in a field:
   switch the node off, or walk out of range, and every relay stops surveying by itself.

   THE LIFETIME IS DERIVED, not chosen. It only has to outlast a few missed re-issues, and a walk
   goes to the edge of coverage on purpose -- so the relay most likely to miss one is exactly the
   relay worth keeping. Three intervals tolerates two consecutive misses. */
#ifndef SURVEY_REISSUE_MIN
#define SURVEY_REISSUE_MIN 5u
#endif
#ifndef SURVEY_DURATION_M
#define SURVEY_DURATION_M (3u * SURVEY_REISSUE_MIN)
#endif
#ifndef SURVEY_BATCH_S
#define SURVEY_BATCH_S 30u
#endif
#define SURVEY_REISSUE_MS (SURVEY_REISSUE_MIN * 60u * 1000u)

#ifndef SURVEY_AUTO_MS
#define SURVEY_AUTO_MS 30000u
#endif
#ifndef SURVEY_PRESS_LONG_MS
#define SURVEY_PRESS_LONG_MS 2000u
#endif
#ifndef SURVEY_PRESS_XLONG_MS
#define SURVEY_PRESS_XLONG_MS 8000u
#endif

/* WHAT THE LIGHT SAYS. One LED, so it gets three resting states and one event -- about the limit
   before it needs a manual.
     no fix          fast pulse   "not ready; a point pressed now carries no position"
     surveying       slow pulse   "a directive of mine is out there"
     idle, fix       dark, with a blink every few seconds so dark is distinguishable from DEAD,
                     which on a handheld is worth the current
     report sent     two blinks, then back to whichever of the above applies
   NO FIX OUTRANKS SURVEYING, because it is the one that changes what you do next. */
#ifndef SURVEY_LED_BEAT_MS
#define SURVEY_LED_BEAT_MS 5000u
#endif
#define SURVEY_LED_EVENT_BLINKS 2u

#ifndef SURVEY_OBSERVE_MAX
#define SURVEY_OBSERVE_MAX 16
#endif

#define SURVEY_DIRECTIVE_NONE 0u

#ifndef SURVEY_TRACE_MS
#define SURVEY_TRACE_MS (5u * 60u * 1000u)
#endif

// -----------------------------------------------------------------------------------------------------------------------------------------

typedef struct {
    iotdata_node_t *node; /* our station, and the sequence a frame is spent from */
    gpio_num_t button_gpio;
    gpio_num_t led_gpio;
    bool (*send)(const uint8_t *buf, size_t len);      /* the radio is already up; put these bytes out */
    bool (*position)(iotdata_mesh_diag_position_t *p); /* false when this node does not have a fix */
    uint32_t *tx_count, *tx_errors;
} survey_config_t;

/* survey_init() RETAINS a pointer to the config above for the life of the node, so whatever is
   passed must outlive the call. A compound literal will NOT: it dies with the enclosing block, and
   the first call through a function pointer afterwards jumps into whatever the stack has since
   become -- an instruction fetch from a data address, which is a confusing way to find out. */

typedef struct {
    bool active;
    uint16_t directive_id;
    uint32_t issued_ms;
    bool auto_report;
    uint32_t auto_last_ms;
    led_pattern_t led_rest;
    uint16_t led_rest_arg;
    uint32_t led_event_until_ms, led_beat_ms;
    iotdata_mesh_diag_survey_entry_t obs[SURVEY_OBSERVE_MAX];
    int obs_count;
    uint32_t obs_total, obs_dropped;
    uint32_t trace_last_ms, trace_count;
    button_handle_t button_handle;
} survey_t;

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static const survey_config_t *_survey_cfg = NULL;
static led_config_t _survey_led_config;
static led_handle_t _survey_led;

static bool survey_have_fix(void) {
    iotdata_mesh_diag_position_t p;
    return _survey_cfg->position != NULL && _survey_cfg->position(&p);
}

/* A momentary indication. It restores NOTHING itself -- survey_led_tick() does that once the window
   has passed, so no call site can forget to. A light that forgets is a light that lies. */
static void survey_led_event(survey_t *const s, const unsigned blinks) {
    led_start(&_survey_led, LED_PATTERN_BLINK, (uint16_t)blinks);
    s->led_event_until_ms = hw_time_ms() + (blinks * 2u * LED_BLINK_MS_DEFAULT) + 50u;
    s->led_rest = LED_PATTERN_OFF; /* forget what was showing; the tick re-applies it */
    s->led_rest_arg = 0;
}

static void survey_led_tick(survey_t *const s) {
    if (s->led_event_until_ms != 0u) {
        if ((int32_t)(hw_time_ms() - s->led_event_until_ms) < 0)
            return; /* an event is still playing */
        s->led_event_until_ms = 0u;
        s->led_rest = LED_PATTERN_OFF; /* the blink left it dark; re-apply below */
    }
    led_pattern_t want = LED_PATTERN_OFF;
    uint16_t arg = 0;
    if (!survey_have_fix()) {
        want = LED_PATTERN_PULSE;
        arg = LED_PULSE_FAST_MS_DEFAULT;
    } else if (s->active) {
        want = LED_PATTERN_PULSE;
        arg = LED_PULSE_SLOW_MS_DEFAULT;
    }
    if (want != s->led_rest || arg != s->led_rest_arg) {
        if (want == LED_PATTERN_OFF)
            led_stop(&_survey_led);
        else
            led_start(&_survey_led, want, arg);
        s->led_rest = want;
        s->led_rest_arg = arg;
        s->led_beat_ms = hw_time_ms();
    }
    if (want == LED_PATTERN_OFF && (hw_time_ms() - s->led_beat_ms) >= SURVEY_LED_BEAT_MS) {
        s->led_beat_ms = hw_time_ms();
        survey_led_event(s, 1u);
    }
}

static void survey_observe(survey_t *const s, const uint16_t station, const uint16_t sequence, const int rssi_dbm) {
    s->obs_total++;
    /* A RING, DROPPING THE OLDEST. A full batch that refuses the newest keeps the observations
       taken furthest away and discards the ones taken here -- and where an observation was taken
       is the whole of its meaning on a node that is being carried. Whatever is dropped is counted;
       nothing is lost quietly. */
    if (s->obs_count >= SURVEY_OBSERVE_MAX) {
        memmove(&s->obs[0], &s->obs[1], (size_t)(SURVEY_OBSERVE_MAX - 1) * sizeof(s->obs[0]));
        s->obs_count = SURVEY_OBSERVE_MAX - 1;
        s->obs_dropped++;
    }
    {
        iotdata_mesh_diag_survey_entry_t *const e = &s->obs[s->obs_count++];
        e->station = station;
        e->sequence = sequence;
        e->rssi_dbm = rssi_dbm; /* the driver's own convention, untranslated */
        e->snr_q4 = IOTDATA_MESH_DIAG_SNR_NONE;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void survey_report(survey_t *const s, const char *const why) {
    const survey_config_t *const c = _survey_cfg;
    const uint16_t station = iotdata_node_station(c->node);
    const uint16_t seq = iotdata_node_sequence(c->node); /* peeked: spent only if it goes */

    iotdata_mesh_diag_position_t posn;
    const bool have_posn = (c->position != NULL) && c->position(&posn);

    uint8_t inner[IOTDATA_MESH_DIAG_SURVEY_RSP_SIZE + SURVEY_OBSERVE_MAX * IOTDATA_MESH_DIAG_SURVEY_RSP_ENTRY + IOTDATA_MESH_DIAG_SURVEY_POS_SIZE];
    const int ilen = iotdata_mesh_pack_diag_survey_response(inner, (int)sizeof(inner), station, seq, s->active ? s->directive_id : (uint16_t)SURVEY_DIRECTIVE_NONE, s->obs, s->obs_count, have_posn ? &posn : NULL);
    uint8_t frame[IOTDATA_MAX_PACKET_SIZE];
    const int flen = (ilen > 0) ? iotdata_mesh_pack_forward(frame, (int)sizeof(frame), station, seq, IOTDATA_MESH_TTL_DEFAULT, inner, ilen) : 0;
    if (flen > 0) {
        if (c->send(frame, (size_t)flen)) {
            iotdata_node_sequence_used(c->node);
            (*c->tx_count)++;
            ESP_LOGI(__tag_survey, "report: %s -- %d heard, %s (%d bytes)", why, s->obs_count, have_posn ? "positioned" : "no fix", flen);
            s->obs_count = 0;
            survey_led_event(s, SURVEY_LED_EVENT_BLINKS);
        } else {
            (*c->tx_errors)++;
            ESP_LOGW(__tag_survey, "report: %s would not go out", why);
        }
    } else
        ESP_LOGW(__tag_survey, "report: %s would not build", why);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static bool survey_directive_send(survey_t *const s, const bool begin) {
    const survey_config_t *const c = _survey_cfg;
    const uint16_t station = iotdata_node_station(c->node);
    const uint16_t seq = iotdata_node_sequence(c->node);
    uint8_t frame[IOTDATA_MAX_PACKET_SIZE];
    const int len = begin ? iotdata_mesh_pack_diag_survey_request(frame, (int)sizeof(frame), station, seq, s->directive_id, SURVEY_DURATION_M, SURVEY_BATCH_S, &station, 1)
                          : iotdata_mesh_pack_diag_cancel(frame, (int)sizeof(frame), station, seq, IOTDATA_MESH_DIAG_SURVEY, s->directive_id);
    if (len > 0 && c->send(frame, (size_t)len)) {
        iotdata_node_sequence_used(c->node);
        s->issued_ms = hw_time_ms();
        return true;
    }
    return false;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static void survey_begin(survey_t *const s) {
    const uint16_t id = (uint16_t)((s->directive_id + 1u) & 0x0FFFu);
    s->directive_id = (id == 0u) ? 1u : id;
    if (survey_directive_send(s, true)) {
        s->active = true;
        ESP_LOGI(__tag_survey, "STARTED directive=%03" PRIX16 " watching station=%03" PRIX16 " (%um lifetime, %us batches)", s->directive_id, iotdata_node_station(_survey_cfg->node), (unsigned)SURVEY_DURATION_M, (unsigned)SURVEY_BATCH_S);
        /* the first point is where the walk starts, and the report cadence runs from here */
        s->auto_last_ms = s->trace_last_ms = hw_time_ms();
        survey_report(s, "survey start");
    } else
        ESP_LOGW(__tag_survey, "could not air the directive -- not started");
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void survey_end(survey_t *const s) {
    s->active = false;
    ESP_LOGI(__tag_survey, "STOPPED directive=%03" PRIX16 "%s", s->directive_id, survey_directive_send(s, false) ? "" : " (cancel not aired; it will time out anyway)");
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void survey_trace(survey_t *const s, const char *const why) {
    const survey_config_t *const c = _survey_cfg;
    if (c == NULL || c->send == NULL)
        return;
    const uint16_t station = iotdata_node_station(c->node), seq = iotdata_node_sequence(c->node); /* peeked: spent only if it goes */
    const iotdata_mesh_nbr_entry_t me = { .cost = IOTDATA_MESH_NBR_COST_SENSOR, .rssi_dbm = IOTDATA_MESH_DIAG_RSSI_NONE, .station = station };
    uint8_t inner[IOTDATA_MESH_DIAG_TRACE_HDR_SIZE + IOTDATA_MESH_DIAG_TRACE_HOP];
    const int ilen = iotdata_mesh_pack_diag_trace(inner, (int)sizeof(inner), station, seq, IOTDATA_MESH_DIAG_OP_RECORD, SURVEY_DIRECTIVE_NONE, 0u, station, &me, 1);
    uint8_t frame[IOTDATA_MAX_PACKET_SIZE];
    const int flen = (ilen > 0) ? iotdata_mesh_pack_forward(frame, (int)sizeof(frame), station, seq, IOTDATA_MESH_TTL_DEFAULT, inner, ilen) : 0;
    if (flen > 0) {
        if (c->send(frame, (size_t)flen)) {
            iotdata_node_sequence_used(c->node);
            (*c->tx_count)++;
            s->trace_count++;
            ESP_LOGI(__tag_survey, "trace: %s -- asking the mesh to record its own path home (%d bytes)", why, flen);
            survey_led_event(s, SURVEY_LED_EVENT_BLINKS);
        } else {
            (*c->tx_errors)++;
            ESP_LOGW(__tag_survey, "trace: %s would not go out", why);
        }
    } else
        ESP_LOGW(__tag_survey, "trace: %s would not build", why);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void survey_poll(survey_t *const s) {
    button_event_t ev;
    while (button_event_fetch(&ev)) {
        if (ev.type == BUTTON_EVENT_SINGLE_PRESS)
            survey_report(s, "pressed");
        else if (ev.type == BUTTON_EVENT_DOUBLE_PRESS) {
            if (s->active)
                survey_end(s);
            else
                survey_begin(s);
        } else if (ev.type == BUTTON_EVENT_LONG_PRESS) {
            s->auto_report = !s->auto_report;
            s->auto_last_ms = hw_time_ms();
            ESP_LOGI(__tag_survey, "automatic reporting and trace %s", s->auto_report ? "ON" : s->active ? "OFF -- the survey keeps reporting until it stops" : "OFF -- relays now have nothing of ours to hear");
            s->trace_last_ms = hw_time_ms();
            if (s->auto_report)
                survey_report(s, "auto start");
        } else if (ev.type == BUTTON_EVENT_EXTRA_LONG_PRESS) {
            ESP_LOGW(__tag_survey, "button held -- cancelling everything and restarting");
            s->auto_report = false;
            if (s->active)
                survey_end(s);
            iotdata_node_diagnostics_event(IOTDATA_NODE_BB_LC_STOP, 0);
            iotdata_node_diagnostics_flush();
            hw_delay_ms_yieldable(CONSOLE_DRAIN_MS);
            esp_restart();
        } else if (ev.type == BUTTON_EVENT_STUCK)
            ESP_LOGW(__tag_survey, "button reads stuck -- check the pull-up and the wiring");
    }
    survey_led_tick(s);

    /* A SURVEY REPORTS TOO. The directive only asks the relays to listen; without our packets to hear
       they batch up nothing, so a running survey keeps the reports going whether or not the long press
       turned them on. Each report is also the position the relays' observations are joined to. */
    const bool reporting = s->auto_report || s->active;
    if (reporting && (hw_time_ms() - s->auto_last_ms) >= SURVEY_AUTO_MS) {
        s->auto_last_ms = hw_time_ms();
        survey_report(s, s->active ? "survey" : "auto");
    }
    if (reporting && (hw_time_ms() - s->trace_last_ms) >= SURVEY_TRACE_MS) {
        s->trace_last_ms = hw_time_ms();
        survey_trace(s, "periodic");
    }
    if (s->active && (hw_time_ms() - s->issued_ms) >= SURVEY_REISSUE_MS)
        if (!survey_directive_send(s, true))
            ESP_LOGW(__tag_survey, "re-issue did not go out (will retry)");
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static bool survey_init(survey_t *const s, const survey_config_t *const cfg) {
    _survey_cfg = cfg;
    s->obs_count = 0;
    if (!button_conf(&s->button_handle, &(button_config_t){
                                            .pins = { .gpio_button = cfg->button_gpio },
                                            .debounce_ms = BUTTON_DEBOUNCE_MS_DEFAULT,
                                            .double_click_ms = BUTTON_DOUBLE_CLICK_MS_DEFAULT,
                                            .long_press_ms = SURVEY_PRESS_LONG_MS,
                                            .extra_long_press_ms = SURVEY_PRESS_XLONG_MS,
                                            .stuck_timeout_ms = BUTTON_STUCK_MS_DEFAULT,
                                        })) {
        ESP_LOGW(__tag_survey, "button would not configure -- this node has no way to take a point");
        return false;
    }
    button_hardware_init(&s->button_handle);
    const button_status_t bs = button_status(&s->button_handle);
    if (bs == BUTTON_STATUS_OK) {
        _survey_led_config = (led_config_t){
            .pins = { .gpio_led = cfg->led_gpio },
            .blink_ms = LED_BLINK_MS_DEFAULT,
            .pulse_slow_ms = LED_PULSE_SLOW_MS_DEFAULT,
            .pulse_fast_ms = LED_PULSE_FAST_MS_DEFAULT,
        };
        if (led_conf(&_survey_led, &_survey_led_config)) {
            led_hardware_init(&_survey_led); /* led_conf() only validates and stores; this drives the pin */
            const led_status_t ls = led_status(&_survey_led);
            ESP_LOGI(__tag_survey, "led GPIO%d, %s -- fast pulse = no fix, slow pulse = surveying, two blinks = report sent", (int)cfg->led_gpio, led_status_to_str(ls));
        } else
            ESP_LOGW(__tag_survey, "led would not configure -- this node has no way to tell you anything");
        s->led_rest = LED_PATTERN_OFF;
        s->led_rest_arg = 0;
        s->led_event_until_ms = 0u;
        s->led_beat_ms = hw_time_ms();

        button_events_clear();
        ESP_LOGI(__tag_survey, "button GPIO%d, %s -- single=report, double=survey on/off, %us=auto on/off (every %us), %us=cancel all + reboot", (int)cfg->button_gpio, button_status_to_str(bs), (unsigned)(SURVEY_PRESS_LONG_MS / 1000u),
                 (unsigned)(SURVEY_AUTO_MS / 1000u), (unsigned)(SURVEY_PRESS_XLONG_MS / 1000u));
        if (s->active) {
            ESP_LOGW(__tag_survey, "directive %03" PRIX16 " was running before the restart -- letting it time out", s->directive_id);
            s->active = false;
        }
        s->auto_report = false;
    } else {
        ESP_LOGW(__tag_survey, "button reads pressed while idle -- check the pull-up and the wiring");
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#endif /* APP_SURVEY_H */
