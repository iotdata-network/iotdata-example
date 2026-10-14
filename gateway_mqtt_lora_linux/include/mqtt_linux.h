
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <mosquitto.h>

// -----------------------------------------------------------------------------------------------------------------------------------------

typedef struct {
    const char *server;
    const char *client;
    bool synchronous;
    bool tls_insecure;
    unsigned int reconnect_delay;
    unsigned int reconnect_delay_max;
    bool debug;
} mqtt_config_t;

// -----------------------------------------------------------------------------------------------------------------------------------------

struct mosquitto *mosq = NULL;
void (*mqtt_message_callback)(const char *, const unsigned char *, const int) = NULL;

/*
 * A SECOND SUBSCRIBER, ROUTED BY TOPIC PREFIX.
 *
 * mqtt_subscribe() keeps ONE global callback, so a second call used to silently replace the first
 * -- which was fine while the control channel was the only subscriber and is not fine now that ddup
 * peer presence is another. Rather than give every caller a router, the one extra channel gets a
 * prefix and a handler of its own, and the dispatcher below sends a message to whichever matches.
 *
 * Two is not a general mechanism and does not pretend to be. A third subscriber is the point at
 * which this wants a real table.
 */
static char mqtt_aux_prefix[96] = "";
static void (*mqtt_aux_callback)(void *, const char *, const void *, int) = NULL;
static void *mqtt_aux_ctx = NULL;
bool mqtt_synchronous = false;
bool mqtt_connected = false;
uint32_t mqtt_stat_connects = 0;
uint32_t mqtt_stat_disconnects = 0;
uint32_t mqtt_stat_reconnects = 0;
time_t mqtt_stat_last_connect_time = 0;
uint32_t mqtt_stat_publishes = 0;
uint64_t mqtt_stat_publish_bytes = 0;
uint32_t mqtt_stat_publish_errors = 0;

static unsigned int mqtt_reconnect_delay_base = 0;
static unsigned int mqtt_reconnect_delay_max_base = 0;
static unsigned int mqtt_reconnect_delay_current = 0;
static time_t mqtt_reconnect_next_attempt = 0;

// -----------------------------------------------------------------------------------------------------------------------------------------

bool mqtt_send(const char *topic, const char *message, const int length) {
    if (!mosq || !mqtt_connected) {
        mqtt_stat_publish_errors++;
        return false;
    }
    const int result = mosquitto_publish(mosq, NULL, topic, length, message, MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN);
    if (result != MOSQ_ERR_SUCCESS) {
        mqtt_stat_publish_errors++;
        PRINTF_ERROR("mqtt: publish error: %s\n", mosquitto_strerror(result));
        return false;
    }
    mqtt_stat_publishes++;
    mqtt_stat_publish_bytes += (uint64_t)length;
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void __mqtt_message_callback_wrapper(struct mosquitto *m, void *o __attribute__((unused)), const struct mosquitto_message *message) {
    if (m != mosq)
        return;
    /* the aux channel first and exclusively: its prefix is what distinguishes it, and a message it
       owns must not also reach the control handler, which would try to parse it as a command */
    if (mqtt_aux_callback != NULL && mqtt_aux_prefix[0] != '\0' && strncmp((const char *)message->topic, mqtt_aux_prefix, strlen(mqtt_aux_prefix)) == 0) {
        mqtt_aux_callback(mqtt_aux_ctx, (const char *)message->topic, message->payload, message->payloadlen);
        return;
    }
    if (mqtt_message_callback)
        mqtt_message_callback((const char *)message->topic, message->payload, message->payloadlen);
}

/* Subscribe a second channel, with its own context. `prefix` is what the dispatcher matches on --
   the subscription pattern's fixed part, e.g. "iotdata/ddup/" for "iotdata/ddup/#". */
bool mqtt_subscribe_aux(const char *const pattern, const char *const prefix, void (*callback)(void *, const char *, const void *, int), void *const ctx) {
    if (!mosq)
        return false;
    snprintf(mqtt_aux_prefix, sizeof(mqtt_aux_prefix), "%s", prefix);
    mqtt_aux_callback = callback;
    mqtt_aux_ctx = ctx;
    const int result = mosquitto_subscribe(mosq, NULL, pattern, MQTT_PUBLISH_QOS);
    if (result != MOSQ_ERR_SUCCESS) {
        PRINTF_ERROR("mqtt: subscribe error: %s\n", mosquitto_strerror(result));
        return false;
    }
    PRINTF_INFO("mqtt: subscribed to aux topic '%s'\n", pattern);
    return true;
}

/*
 * RETAINED publish, and a NULL payload is how a retained topic is CLEARED -- an empty retained
 * message is the broker's own way of saying "this is no longer true", and it is what a last will
 * publishes on our behalf if we die without saying it.
 */
bool mqtt_send_retained(const char *const topic, const char *const message, const int length) {
    if (!mosq || !mqtt_connected) {
        mqtt_stat_publish_errors++;
        return false;
    }
    const int result = mosquitto_publish(mosq, NULL, topic, length, (length > 0) ? message : NULL, MQTT_PUBLISH_QOS, true);
    if (result != MOSQ_ERR_SUCCESS) {
        mqtt_stat_publish_errors++;
        PRINTF_ERROR("mqtt: retained publish error: %s\n", mosquitto_strerror(result));
        return false;
    }
    mqtt_stat_publishes++;
    mqtt_stat_publish_bytes += (uint64_t)((length > 0) ? length : 0);
    return true;
}

/* THE WILL IS REGISTERED AT CONNECT, so this has to be called BEFORE mqtt_begin(). Held here and
   applied there, rather than taking a second connect path. */
static char mqtt_will_topic[96] = "";
void mqtt_set_will(const char *const topic) {
    snprintf(mqtt_will_topic, sizeof(mqtt_will_topic), "%s", (topic != NULL) ? topic : "");
}

// -----------------------------------------------------------------------------------------------------------------------------------------

bool mqtt_subscribe(const char *topic, const int qos, void (*callback)(const char *, const unsigned char *, const int)) {
    if (!mosq)
        return false;
    mqtt_message_callback = callback;
    const int result = mosquitto_subscribe(mosq, NULL, topic, qos);
    if (result != MOSQ_ERR_SUCCESS) {
        PRINTF_ERROR("mqtt: subscribe error: %s\n", mosquitto_strerror(result));
        return false;
    }
    PRINTF_INFO("mqtt: subscribed to topic '%s' (qos=%d)\n", topic, qos);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

bool mqtt_unsubscribe(const char *topic) {
    if (!mosq)
        return false;
    const int result = mosquitto_unsubscribe(mosq, NULL, topic);
    if (result != MOSQ_ERR_SUCCESS) {
        PRINTF_ERROR("mqtt: unsubscribe error: %s\n", mosquitto_strerror(result));
        return false;
    }
    PRINTF_INFO("mqtt: unsubscribed from topic '%s'\n", topic);
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static bool __mqtt_parse(const char *string, char *host, const int length, int *port, bool *ssl) {
    host[0] = '\0';
    *port = 1883;
    *ssl = false;
    if (strncmp(string, "mqtt://", 7) == 0)
        strncpy(host, string + 7, (size_t)length - 1);
    else if (strncmp(string, "mqtts://", 8) == 0) {
        strncpy(host, string + 8, (size_t)length - 1);
        *ssl = true;
        *port = 8883;
    } else
        strncpy(host, string, (size_t)length - 1);
    char *port_str = strchr(host, ':');
    if (port_str) {
        *port_str = '\0'; // Terminate host string at colon
        *port = atoi(port_str + 1);
    }
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void __mqtt_connect_callback(struct mosquitto *m, void *o __attribute__((unused)), int r) {
    if (m != mosq)
        return;
    if (r != 0) {
        PRINTF_ERROR("mqtt: connect failed: %s\n", mosquitto_connack_string(r));
        return;
    }
    mqtt_connected = true;
    mqtt_stat_connects++;
    mqtt_stat_last_connect_time = hw_clock();
    mqtt_reconnect_delay_current = mqtt_reconnect_delay_base;
    mqtt_reconnect_next_attempt = 0;
    PRINTF_INFO("mqtt: connected\n");
}

static void __mqtt_disconnect_callback(struct mosquitto *m, void *o __attribute__((unused)), int r) {
    if (m != mosq)
        return;
    mqtt_connected = false;
    mqtt_stat_disconnects++;
    if (r == 0)
        PRINTF_INFO("mqtt: disconnected (clean)\n");
    else
        PRINTF_ERROR("mqtt: disconnected unexpectedly: rc=%d\n", r);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void __mqtt_reconnect_attempt(void) {
    const time_t now = hw_clock();
    if (now < mqtt_reconnect_next_attempt)
        return;
    if (mqtt_reconnect_delay_current == 0)
        mqtt_reconnect_delay_current = mqtt_reconnect_delay_base ? mqtt_reconnect_delay_base : 1;
    PRINTF_INFO("mqtt: attempting reconnect\n");
    const int r = mosquitto_reconnect(mosq);
    if (r == MOSQ_ERR_SUCCESS) {
        mqtt_stat_reconnects++;
        mqtt_reconnect_delay_current = mqtt_reconnect_delay_base;
        mqtt_reconnect_next_attempt = 0;
    } else {
        PRINTF_ERROR("mqtt: reconnect failed: %s (next attempt in %us)\n", mosquitto_strerror(r), mqtt_reconnect_delay_current);
        mqtt_reconnect_next_attempt = now + mqtt_reconnect_delay_current;
        mqtt_reconnect_delay_current *= 2;
        if (mqtt_reconnect_delay_max_base > 0 && mqtt_reconnect_delay_current > mqtt_reconnect_delay_max_base)
            mqtt_reconnect_delay_current = mqtt_reconnect_delay_max_base;
    }
}

void mqtt_loop(const int timeout_ms) {
    if (!mosq)
        return;
    const int rc = mosquitto_loop(mosq, timeout_ms, 1);
    if (rc != MOSQ_ERR_SUCCESS) {
        if (mqtt_connected) {
            mqtt_connected = false;
            mqtt_stat_disconnects++;
            PRINTF_ERROR("mqtt: loop error: %s\n", mosquitto_strerror(rc));
        }
        __mqtt_reconnect_attempt();
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

bool mqtt_begin(const mqtt_config_t *cfg) {
    char host[256];
    int port;
    bool ssl;
    if (!__mqtt_parse(cfg->server, host, sizeof(host), &port, &ssl)) {
        PRINTF_ERROR("mqtt: error parsing details in '%s'\n", cfg->server);
        return false;
    }
    PRINTF_INFO("mqtt: connecting (host='%s', port=%d, ssl=%s, client='%s')\n", host, port, ssl ? "true" : "false", cfg->client);
    char client_id[24];
    snprintf(client_id, sizeof(client_id), "%s-%06X", cfg->client ? cfg->client : "mqtt-linux", (unsigned int)(hw_clock() ^ getpid()) & 0xFFFFFF);
    mosquitto_lib_init();
    mosq = mosquitto_new(client_id, true, NULL);
    if (!mosq) {
        PRINTF_ERROR("mqtt: error creating client instance\n");
        return false;
    }
    if (ssl) {
        mosquitto_tls_set(mosq, NULL, NULL, NULL, NULL, NULL);
        if (cfg->tls_insecure) {
            mosquitto_tls_insecure_set(mosq, true);
            PRINTF_INFO("mqtt: WARNING tls certificate validation disabled\n");
        }
    }
    /* BEFORE the connect: a will set afterwards is a will the broker never heard about. Empty and
       retained, so a death clears our presence exactly as a clean shutdown would. */
    if (mqtt_will_topic[0] != '\0')
        (void)mosquitto_will_set(mosq, mqtt_will_topic, 0, NULL, MQTT_PUBLISH_QOS, true);
    mosquitto_connect_callback_set(mosq, __mqtt_connect_callback);
    mosquitto_disconnect_callback_set(mosq, __mqtt_disconnect_callback);
    mosquitto_message_callback_set(mosq, __mqtt_message_callback_wrapper);
    if (cfg->reconnect_delay > 0)
        mosquitto_reconnect_delay_set(mosq, cfg->reconnect_delay, cfg->reconnect_delay_max, true);
    mqtt_reconnect_delay_base = cfg->reconnect_delay > 0 ? cfg->reconnect_delay : 1;
    mqtt_reconnect_delay_max_base = cfg->reconnect_delay_max > 0 ? cfg->reconnect_delay_max : mqtt_reconnect_delay_base;
    mqtt_reconnect_delay_current = mqtt_reconnect_delay_base;
    mqtt_reconnect_next_attempt = 0;
    int result;
    if ((result = mosquitto_connect(mosq, host, port, MQTT_CONNECT_TIMEOUT)) != MOSQ_ERR_SUCCESS) {
        PRINTF_ERROR("mqtt: error connecting to broker: %s\n", mosquitto_strerror(result));
        mosquitto_destroy(mosq);
        mosq = NULL;
        return false;
    }
    mqtt_synchronous = cfg->synchronous;
    if (!mqtt_synchronous && (result = mosquitto_loop_start(mosq)) != MOSQ_ERR_SUCCESS) {
        PRINTF_ERROR("mqtt: error starting loop: %s\n", mosquitto_strerror(result));
        mosquitto_disconnect(mosq);
        mosquitto_destroy(mosq);
        mosq = NULL;
        return false;
    }
    if (mqtt_synchronous)
        for (int i = 0; i < 50 && !mqtt_connected; i++)
            mosquitto_loop(mosq, 100, 1);

    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

bool mqtt_is_connected(void) {
    return mosq && mqtt_connected;
}

void mqtt_end(void) {
    mqtt_connected = false;
    if (mosq) {
        if (!mqtt_synchronous)
            mosquitto_loop_stop(mosq, true);
        mosquitto_disconnect(mosq);
        mosquitto_destroy(mosq);
        mosq = NULL;
    }
    mosquitto_lib_cleanup();
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
