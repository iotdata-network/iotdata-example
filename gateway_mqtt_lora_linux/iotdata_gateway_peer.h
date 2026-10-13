
// -----------------------------------------------------------------------------------------------------------------------------------------
// iotdata_gateway_peer.h - ddup AUTO-PEERING via MQTT presence.
//
// THE PROBLEM IT SOLVES: a static `ddup-peers` list is a list of addresses somebody has to keep
// correct, on every gateway, as a fleet grows or a box changes address. Presence turns that round:
// each gateway says where IT is, and everybody else listens. `ddup-peers = mqtt` is the whole
// configuration.
//
// exec/main owns MQTT here; ddup only exposes the lockable peer add/remove. Presence is published
// retained to iotdata/ddup/<gwid> = {"ip":..,"port":..,"time":<epoch>}; the LWT is an empty retained
// message (offline). The topic is fleet-bridged (iotdata/ddup/#, both directions), so every gateway
// sees every other's address. On receipt we ddup_peer_insert/remove (keyed by gateway-id); our own
// presence (the bridge echo) is ignored. The `ts` lets a peer be aged out when its box/bridge dies and
// the LWT can't reach us (the LWT fires on the dead box's LOCAL broker) — see DDUP_PEER_TTL_SECS. A
// stale retained presence re-delivered on a bridge reconnect carries the OLD ts, so it can't refresh a
// dead peer. Public/private-IP/NAT is a sysadmin concern (VPN/tunnel).
// -----------------------------------------------------------------------------------------------------------------------------------------

#include <ifaddrs.h>
#include <net/if.h>
#include <cjson/cJSON.h>

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#ifndef PEER_PRESENCE_REPUBLISH_SECS
#define PEER_PRESENCE_REPUBLISH_SECS 30 // re-publish at least this often (recover from a broker restart)
#endif

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

typedef struct peer_presence_s {
    ddup_state_t *ddup;
    uint16_t gwid;
    char topic[64];   // iotdata/ddup/<gwid>
    char last_ip[64]; // last advertised IP (re-publish when it changes)
    time_t last_pub;
} peer_presence_t;

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

static void peer_presence_topic(uint16_t gwid, char *out, size_t outsz) {
    snprintf(out, outsz, "iotdata/ddup/%04" PRIX16, gwid);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

// Resolve our advertise IPv4 from the qualifier: a literal IP -> as-is; an interface name -> its
// IPv4; "" -> the first non-loopback IPv4. Returns true + writes ip.
static bool peer_resolve_advertise_ip(const char *qualifier, char *ip, size_t ipsz) {
    bool found = false;
    struct in_addr lit;
    if (qualifier[0] && inet_pton(AF_INET, qualifier, &lit) == 1) {
        snprintf(ip, ipsz, "%s", qualifier);
        found = true;
    } else {
        struct ifaddrs *ifap = NULL;
        if (getifaddrs(&ifap) == 0) {
            for (struct ifaddrs *p = ifap; p && !found; p = p->ifa_next) {
                if (!p->ifa_addr || p->ifa_addr->sa_family != AF_INET || (p->ifa_flags & IFF_LOOPBACK))
                    continue;
                if (qualifier[0] && strcmp(p->ifa_name, qualifier) != 0)
                    continue;
                char buf[INET_ADDRSTRLEN];
                if (inet_ntop(AF_INET, &((const struct sockaddr_in *)(const void *)p->ifa_addr)->sin_addr, buf, sizeof(buf))) {
                    snprintf(ip, ipsz, "%s", buf);
                    found = true;
                }
            }
            freeifaddrs(ifap);
        }
    }
    return found;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

// The aux MQTT handler: mqtt_linux routes anything under the presence prefix here. ctx = peer_presence_t*.
static void peer_presence_on_msg(void *ctx, const char *topic, const void *payload, int len) {
    peer_presence_t *pp = (peer_presence_t *)ctx;
    const char *slash = strrchr(topic, '/');
    if (!slash || !slash[1])
        return;
    const uint16_t gwid = (uint16_t)strtol(slash + 1, NULL, 16);
    if (gwid == pp->gwid)
        return;     // our own presence (bridge echo)
    if (len <= 0) { // empty retained = offline / LWT
        ddup_peer_remove(pp->ddup, gwid);
        return;
    }
    cJSON *j = cJSON_ParseWithLength(payload, (size_t)len);
    if (!j)
        return;
    const cJSON *jip = cJSON_GetObjectItem(j, "ip");
    const cJSON *jport = cJSON_GetObjectItem(j, "port");
    const cJSON *jts = cJSON_GetObjectItem(j, "time");
    if (!cJSON_IsNumber(jts))
        jts = cJSON_GetObjectItem(j, "ts");
    if (cJSON_IsString(jip) && jip->valuestring[0]) {
        const uint16_t port = cJSON_IsNumber(jport) ? (uint16_t)jport->valueint : pp->ddup->port;
        const time_t ts = cJSON_IsNumber(jts) ? (time_t)jts->valuedouble : 0; // 0 = un-stamped (legacy peer)
        // A STALE retained presence (the peer's box/bridge died, the LWT fired only on its local broker)
        // carries an old ts — treat it as offline instead of (re)inserting a ghost; otherwise insert/refresh.
        if (ts != 0 && (long long)(time(NULL) - ts) > DDUP_PEER_TTL_SECS)
            ddup_peer_remove(pp->ddup, gwid);
        else
            ddup_peer_insert(pp->ddup, gwid, jip->valuestring, port, ts);
    }
    cJSON_Delete(j);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

// Set ONLY the last-will. MUST be called BEFORE mqtt_begin() (the will is registered at
// connect). The aux subscription is deferred to peer_presence_start() because the handler calls
// into ddup, whose mutex isn't initialised until ddup_begin() (after the config connect/settle).
static void peer_presence_init(peer_presence_t *pp, ddup_state_t *ddup, uint16_t gwid) {
    *pp = (peer_presence_t){ 0 };
    pp->ddup = ddup;
    pp->gwid = gwid;
    peer_presence_topic(gwid, pp->topic, sizeof(pp->topic));
    mqtt_set_will(pp->topic); // LWT: empty retained = offline. BEFORE mqtt_begin(), or it is never registered.
    PRINTF_INFO("ddup-peer: auto-discovery via MQTT presence on %s (advertise=%s)\n", pp->topic, ddup->advertise_qualifier[0] ? ddup->advertise_qualifier : "auto");
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void peer_presence_tick(peer_presence_t *pp, time_t now) {
    if (!pp->ddup || !pp->ddup->auto_peer || !pp->ddup->enabled) // uninitialised (static peers) / ddup-enable master switch
        return;
    ddup_peers_expire(pp->ddup, now); // backstop: prune peers whose presence ts went stale (box/bridge death the LWT can't cover)
    char ip[64];
    if (peer_resolve_advertise_ip(pp->ddup->advertise_qualifier, ip, sizeof(ip))) {
        const bool changed = strcmp(ip, pp->last_ip) != 0;
        if (changed || pp->last_pub == 0 || (now - pp->last_pub) >= PEER_PRESENCE_REPUBLISH_SECS) {
            char payload[128];
            const int n = snprintf(payload, sizeof(payload), "{\"ip\":\"%s\",\"port\":%u,\"time\":%lld}", ip, (unsigned)pp->ddup->port, (long long)now);
            if (mqtt_send_retained(pp->topic, payload, n)) {
                if (changed)
                    PRINTF_INFO("ddup-peer: presence %s -> %s\n", pp->topic, payload);
                snprintf(pp->last_ip, sizeof(pp->last_ip), "%s", ip);
                pp->last_pub = now;
            }
        }
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void peer_presence_shutdown(peer_presence_t *pp) {
    if (pp->ddup && pp->ddup->auto_peer)
        (void)mqtt_send_retained(pp->topic, NULL, 0);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

static void peer_presence_start(peer_presence_t *pp) {
    (void)mqtt_subscribe_aux("iotdata/ddup/#", "iotdata/ddup/", peer_presence_on_msg, pp);
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
