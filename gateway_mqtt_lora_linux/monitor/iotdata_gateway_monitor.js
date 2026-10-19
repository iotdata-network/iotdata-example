#!/usr/bin/env node
//
// iotdata_gateway_monitor.js — live view of received sensor traffic, sized for a phone screen.
//
// Built for siting work: you carry the phone, the page shows what the gateway is actually
// hearing right now, so you can tell whether a spot works before you bolt anything to it.
//
// Newest packet is at the TOP, under a per-station summary, so the current state of the site
// is visible without scrolling — the list below is history, not the thing you read up a ladder.
//
// Stations named in the config get their surveyed position shown against each packet, along
// with the distance from the gateway's own surveyed position, so a reading has somewhere to
// sit: "-99 dBm" means little on its own, "-99 dBm at 543 m" is a data point you can use.
//
// One npm dependency, mqtt, pinned to the version the apex tier already uses. It replaced
// shelling out to mosquitto_sub when this started following a LoRaWAN network server: that tool
// accepts a password only as a command line argument, so an API key was visible in `ps` to every
// user on the box. It also brings reconnection in-process, where a broker going away no longer
// means respawning a child and re-parsing its output from a chunk boundary.
//
// Traffic to the browser stays Server-Sent Events rather than WebSockets, for the reason that
// used to apply to the broker too: EventSource needs no library at either end, and this is
// strictly one-way.

const fs = require('fs');
const os = require('os');
const path = require('path');
const http = require('http');
const readline = require('readline');
const mqtt = require('mqtt');

const USAGE = `
iotdata_gateway_monitor.js — live view of received sensor traffic

  Usage:
    iotdata-gateway-monitor [--config FILE] [--port N] [--broker HOST] [--topic PAT]
                            [--keep N] [--window N] [--snapshot FILE] [--no-restore]
    iotdata-gateway-monitor --help

  Command-line options override the config file, which overrides the built-in defaults.

  Config search order when --config is not given:
    ./iotdata_gateway_monitor.<hostname>.cfg   (running from the source directory)
    ./iotdata_gateway_monitor.cfg
    /etc/default/iotdata-gateway-monitor       (installed location)

  Browse to http://<host>:<port>/. Bound to all interfaces with no authentication:
  keep it on the LAN.
`;

// ---------------------------------------------------------------------------- config

const argv = process.argv.slice(2);
if (argv.includes('--help') || argv.includes('-h')) {
    console.log(USAGE.trim());
    process.exit(0);
}
const flag = (name) => {
    const i = argv.indexOf('--' + name);
    return i >= 0 && argv[i + 1] && !argv[i + 1].startsWith('--') ? argv[i + 1] : undefined;
};

// Same key=value/#-comment format the gateway itself uses, so both are edited the same way.
function loadConfig(file) {
    const cfg = {};
    for (const raw of fs.readFileSync(file, 'utf8').split('\n')) {
        const line = raw.trim();
        if (!line || line.startsWith('#')) continue;
        const eq = line.indexOf('=');
        if (eq < 0) continue;
        cfg[line.slice(0, eq).trim()] = line.slice(eq + 1).trim();
    }
    return cfg;
}

const CONFIG_CANDIDATES = flag('config') ? [flag('config')] : [path.join(__dirname, `iotdata_gateway_monitor.${os.hostname()}.cfg`), path.join(__dirname, 'iotdata_gateway_monitor.cfg'), '/etc/default/iotdata-gateway-monitor'];

let cfg = {};
let cfgFrom = '(defaults only)';
for (const c of CONFIG_CANDIDATES) {
    try {
        cfg = loadConfig(c);
        cfgFrom = c;
        break;
    } catch {
        /* try the next candidate */
    }
}
if (flag('config') && cfgFrom === '(defaults only)') {
    console.error(`error: cannot read config ${flag('config')}`);
    process.exit(1);
}

const num = (v, dflt) => (v === undefined || v === '' || isNaN(parseFloat(v)) ? dflt : parseFloat(v));
const PORT = parseInt(flag('port') ?? cfg['web-port'] ?? '8080', 10);
const BIND = cfg['web-bind'] || '0.0.0.0';
const BROKER = flag('broker') ?? cfg['mqtt-server'] ?? 'localhost';
const MQTT_PORT = cfg['mqtt-port'] || '1883';
const TOPIC = flag('topic') ?? cfg['mqtt-topic'] ?? 'iotdata/#';
// Which kinds of topic under the subscription are taken, by topicKind(). Only station traffic
// for now: a management report and the gateway's own stats documents carry a station field too,
// and would otherwise surface as phantom stations named after the last topic segment.
const ACCEPT = new Set(
    String(cfg['mqtt-accept'] ?? 'station')
        .split(',')
        .map((k) => k.trim())
        .filter(Boolean)
);
const KEEP = parseInt(flag('keep') ?? cfg['history-keep'] ?? '500', 10);
const WINDOW = parseInt(flag('window') ?? cfg['stats-window'] ?? '20', 10);

// Packet log. Off unless a file is configured. The interval is how often buffered records
// are written, NOT a sampling rate: every packet is logged. Batching exists because this
// runs from an SD card, where a trickle of tiny appends is the write pattern that wears one
// out fastest — and because a packet every few seconds does not deserve a syscall each.
const LOG_FILE = flag('log') ?? cfg['log-file'] ?? null;
const LOG_FLUSH = parseInt(flag('log-interval') ?? cfg['log-flush-interval'] ?? '300', 10);

// Where the live state is parked so a restart does not wipe the station table. /dev/shm and not
// the SD card: this is rewritten every minute for as long as the monitor runs, and the card is
// the part of this box that wears out. Losing it on reboot is intended — that is what the
// rebuild-from-log path exists for.
const SNAPSHOT_FILE = flag('snapshot') ?? cfg['snapshot-file'] ?? '/dev/shm/iotdata-gateway-monitor.state';
const SNAPSHOT_INTERVAL = parseInt(cfg['snapshot-interval'] ?? '60', 10);
const SNAPSHOT_VERSION = 1;
const RESTORE = !argv.includes('--no-restore');

// A sequence jump larger than this is a station restart or a counter wrap, not lost packets.
// Counting it as loss would put the loss figure permanently at ~100% after a single reboot.
const SEQ_MAX_GAP = parseInt(cfg['stats-max-gap'] ?? '1000', 10);
// A sequence number seen again from the same station within this many seconds is a duplicate:
// the same packet delivered twice (two gateways, or direct and via a relay, that ddup failed to
// collapse). Bounded in time because a station that restarts reuses low sequence numbers, and
// those are new packets, not repeats.
const DUP_WINDOW_MS = 1000 * parseInt(cfg['stats-dup-window'] ?? '300', 10);
// Battery is not in every packet, so the table shows the latest reading with its age, and calls
// it stale past this many hours: a level from yesterday says nothing about the battery today.
const BATTERY_STALE_S = 3600 * parseFloat(cfg['stats-battery-stale'] ?? '24');
// A relay is heard at least once a beacon round, when it rebroadcasts, and is stale after three
// missed: the rule a relay applies to its own parent, at the gateway's default 150s cadence.
const RELAY_STALE_S = parseFloat(cfg['stats-relay-stale'] ?? '450');

// The node's version TLV (type 1), which it sends on boot and then periodically. Found by format
// rather than position, since a packet may carry several TLVs in any order.
function versionOf(body) {
    for (const t of Array.isArray(body && body.data) ? body.data : [])
        if (t && (t.format === 'version' || t.type === 1) && t.data && typeof t.data === 'object') return t.data;
    return null;
}

// The gateway's own surveyed position. Distances are measured from here.
const REF =
    cfg['ref-latitude'] !== undefined && cfg['ref-longitude'] !== undefined
        ? {
              lat: num(cfg['ref-latitude']),
              lon: num(cfg['ref-longitude']),
              alt: num(cfg['ref-altitude'], null),
          }
        : null;

// station-<ID>-name / -latitude / -longitude / -altitude
const STATIONS = {};
// Topics carry station ids in hex, payloads in decimal; the table, the config and the page all
// use the topic form, three digits upper-case.
const hexId = (v) => (typeof v === 'number' ? v.toString(16) : String(v)).toUpperCase().padStart(3, '0');
for (const key of Object.keys(cfg)) {
    const m = /^station-([0-9A-Fa-f]{1,8})-(name|latitude|longitude|altitude|decoder|state)$/.exec(key);
    if (!m) continue;
    const id = hexId(m[1]);
    STATIONS[id] = STATIONS[id] || {};
    STATIONS[id][m[2]] = m[2] === 'name' || m[2] === 'decoder' || m[2] === 'state' ? cfg[key] : num(cfg[key], null);
}

// ignore-unspecified: only the stations named above are kept, live and on restore, so one that
// does not belong -- an old simulator id, a phantom from a corrupted frame -- is gone at the next
// restart without editing the snapshot. Each one is logged once, the first time it is dropped.
const IGNORE_UNSPECIFIED = /^true$/i.test(cfg['ignore-unspecified'] || '');
const ignoredLogged = new Set();
function specified(id) {
    const hex = hexId(id);
    if (!IGNORE_UNSPECIFIED || STATIONS[hex] !== undefined) return true;
    if (!ignoredLogged.has(hex)) {
        ignoredLogged.add(hex);
        console.log(`ignore-unspecified: ignoring station ${hex} (not in the config)`);
    }
    return false;
}

// Decoder plugins: decoder-<name>=<path to module>. A plugin exports decodeMonitor(body)
// and returns display fields, or null when the packet is not its own. A summary, if it
// gives one, is a few words for the station table's reading column.
//
// Loaded by path from config and never referenced by this repo, so a device-specific wire
// format can be decoded here without the monitor depending on that device's code. A plugin
// that is missing or broken is reported and skipped: the monitor must keep running on a host
// where the plugin simply is not installed.
const DECODERS = {};
for (const key of Object.keys(cfg)) {
    const m = /^decoder-(.+)$/.exec(key);
    if (!m) continue;
    const file = path.resolve(cfg[key]);
    try {
        const mod = require(file);
        if (typeof mod.decodeMonitor !== 'function') throw new Error('exports no decodeMonitor(body) function');
        DECODERS[m[1]] = { name: m[1], file, mod };
    } catch (e) {
        console.error(`decoder ${m[1]}: not loaded (${e.message})`);
    }
}

// A station may name the decoder to use; otherwise every loaded decoder is offered the
// packet and the first to claim it wins. Plugins identify their own traffic (the TSA one
// looks for its TLV type), so an unbound decoder declining is the normal case, not an error.
function runDecoders(station, body) {
    const bound = (STATIONS[station.toUpperCase()] || {}).decoder;
    const list = bound ? (DECODERS[bound] ? [DECODERS[bound]] : []) : Object.values(DECODERS);
    for (const d of list) {
        try {
            const out = d.mod.decodeMonitor(body);
            if (out) return out;
        } catch (e) {
            return { name: d.name, fields: { error: e.message } };
        }
    }
    return null;
}

// Great-circle distance. The alternative — treating a degree as a fixed number of metres —
// is wrong by enough to matter at this latitude, where a degree of longitude is barely half
// a degree of latitude.
function haversine(lat1, lon1, lat2, lon2) {
    const R = 6371008.8; // IUGG mean Earth radius, metres
    const rad = (d) => (d * Math.PI) / 180;
    const dLat = rad(lat2 - lat1);
    const dLon = rad(lon2 - lon1);
    const a = Math.sin(dLat / 2) ** 2 + Math.cos(rad(lat1)) * Math.cos(rad(lat2)) * Math.sin(dLon / 2) ** 2;
    return 2 * R * Math.asin(Math.min(1, Math.sqrt(a)));
}

for (const id of Object.keys(STATIONS)) {
    const s = STATIONS[id];
    s.distance = REF && s.latitude != null && s.longitude != null ? haversine(REF.lat, REF.lon, s.latitude, s.longitude) : null;
}

// ---------------------------------------------------------------------------- tracker config

// A GNSS tracker followed through its LoRaWAN network server's MQTT feed, rather than through
// this site's own gateway. The point is survey work: carry the tracker to a candidate spot,
// press the button, and the position it reports lands here next to the RSSI the local stations
// are producing — so a placement decision has a coordinate attached to it rather than a memory.
//
// This is a SECOND broker connection, outbound to the internet, entirely separate from the
// local one. Off unless tracker-host is configured.
//
//   tracker-host/-port/-user/-pass   the network server's MQTT endpoint and credentials
//   tracker-tls                      on by default for port 8883, off otherwise
//   tracker-cafile                   extra CA bundle; unset means node's built-in store
//   tracker-topic                    subscription pattern (the whole application, by default)
//   tracker-type                     wire format of the device (see TRACKER_TYPES below)
//   tracker-keep                     positions held in memory and in the snapshot
//   tracker-show                     positions the page lists when the panel is opened
//   tracker-log-file                 JSON Lines log of positions, one per line
const TRACKER = cfg['tracker-host']
    ? {
          host: cfg['tracker-host'],
          port: cfg['tracker-port'] || '8883',
          user: cfg['tracker-user'] || null,
          pass: cfg['tracker-pass'] || null,
          // On by default for the conventional MQTT-over-TLS port and off otherwise, so the
          // usual case needs no setting and a broker on this machine needs no exception. The
          // credential below is a network server API key and this connection leaves the LAN,
          // so defaulting it off would be the wrong way round.
          tls: /^(true|false)$/i.test(cfg['tracker-tls'] || '') ? /^true$/i.test(cfg['tracker-tls']) : String(cfg['tracker-port'] || '8883') === '8883',
          // Only needed when the server's chain is not in node's built-in store, which for a
          // public network server it will be.
          cafile: cfg['tracker-cafile'] && cfg['tracker-cafile'] !== 'none' ? cfg['tracker-cafile'] : null,
          topic: cfg['tracker-topic'] || '#',
          type: (cfg['tracker-type'] || 'sensecap-t1000b').toLowerCase(),
      }
    : null;
const TRACKER_KEEP = parseInt(cfg['tracker-keep'] ?? '200', 10);
const TRACKER_SHOW = parseInt(cfg['tracker-show'] ?? '16', 10);
const TRACKER_LOG = cfg['tracker-log-file'] ?? null;
const TRACKER_LOG_FLUSH = parseInt(cfg['tracker-log-flush-interval'] ?? '60', 10);

// SenseCAP T1000-B, as The Things Stack renders it: the uplink carries a decoded_payload with a
// nested messages array, each entry a {measurementId, measurementValue, type} triple. Keying on
// `type` rather than the numeric id because the network server has already done that translation
// and the strings are stable across firmware, where the id table is the thing that grows.
//
// Position and battery arrive in the same uplink as the event that triggered it, so one uplink
// is one row here — the fix, what caused it to be sent, and how it was heard.
function decodeSensecapT1000b(body) {
    const up = body && body.uplink_message;
    const dev = body && body.end_device_ids;
    if (!up || !dev || !up.decoded_payload) return null;
    const groups = up.decoded_payload.messages;
    if (!Array.isArray(groups)) return null;

    const pos = {
        device: dev.device_id || '?',
        lat: null,
        lon: null,
        battery: null,
        event: null,
        t: null,
    };
    for (const group of groups)
        for (const m of Array.isArray(group) ? group : [group]) {
            if (!m || typeof m !== 'object') continue;
            if (m.timestamp && !pos.t) pos.t = m.timestamp;
            switch (m.type) {
                case 'Latitude':
                    pos.lat = num(m.measurementValue, null);
                    break;
                case 'Longitude':
                    pos.lon = num(m.measurementValue, null);
                    break;
                case 'Battery':
                    pos.battery = num(m.measurementValue, null);
                    break;
                case 'Event Status':
                    // An array of {eventName, id}; the name is what is worth showing.
                    pos.event = Array.isArray(m.measurementValue)
                        ? m.measurementValue
                              .map((e) => (e && e.eventName ? String(e.eventName).replace(/\.$/, '') : null))
                              .filter(Boolean)
                              .join(', ') || null
                        : null;
                    break;
            }
        }

    // A fix with no coordinates is a status or battery uplink: real traffic, but nothing to plot.
    if (pos.lat === null || pos.lon === null) return null;

    // Strongest reception wins when several of the site's gateways heard the same uplink — that is
    // the number that describes the link, and the others are only interesting as a count.
    const rx = Array.isArray(up.rx_metadata) ? up.rx_metadata : [];
    let best = null;
    for (const r of rx) if (r && (best === null || (r.rssi ?? -999) > (best.rssi ?? -999))) best = r;
    const lora = ((up.settings || {}).data_rate || {}).lora || {};

    pos.t = pos.t || (up.received_at ? Date.parse(up.received_at) : Date.now());
    pos.receivedAt = up.received_at ? Date.parse(up.received_at) : Date.now();
    pos.fcnt = up.f_cnt;
    pos.rssi = best ? (best.rssi ?? null) : null;
    pos.snr = best ? (best.snr ?? null) : null;
    pos.gateway = best && best.gateway_ids ? best.gateway_ids.gateway_id || null : null;
    pos.gateways = rx.length || 0;
    pos.sf = lora.spreading_factor ?? null;
    pos.freq = up.settings ? num(up.settings.frequency, null) : null;
    return pos;
}

// name -> decoder. One entry today; the config names the type so a second device does not mean
// a second code path everywhere else. The misspelling is accepted because it is the one already
// written down in the field notes.
const TRACKER_TYPES = {
    'sensecap-t1000b': decodeSensecapT1000b,
    'senscap-t1000b': decodeSensecapT1000b,
};

// ---------------------------------------------------------------------------- state

const history = []; // newest last; the page reverses it
const clients = new Set();
const stats = new Map();
const gnss = []; // newest last; tracker positions, independent of the packet history
// Gateways and relays, from the gateway's own publications rather than from station packets:
// stats/gateway (its links and counters), stats/network (its view of the mesh -- relays, their
// cost, who hears whom) and ddup (where it can be reached). Keyed by the gateway's hex id, and
// only the latest of each is kept: these are status reports, superseded by the next one, not
// history -- so they go to neither the packet list nor the packet log.
const gateways = new Map();
// Each node's latest version report, keyed by hex id. Nodes send these as management messages
// (manage/from/<id>, tlv "version") on boot and then periodically -- not inside their data
// packets -- and relays send them too, so this is shared by the station table and the relay rows.
const versions = new Map();
// Survey instruments, from the reports they send: the gateway publishes each observation in one
// with the instrument as `observer`. Keyed by that observer's hex id: when last heard, how many
// observations, and where it was standing.
const surveys = new Map();
let counter = 0;

function send(event, payload, to) {
    const frame = `event: ${event}\ndata: ${JSON.stringify(payload)}\n\n`;
    for (const c of to ? [to] : clients) {
        try {
            c.write(frame);
        } catch {
            /* removed on close */
        }
    }
}

function statFor(id) {
    if (!stats.has(id))
        stats.set(id, {
            id,
            count: 0,
            lost: 0,
            dups: 0,
            recent: [], // [sequence, t] within DUP_WINDOW_MS, for duplicate detection
            prevSeq: undefined,
            times: [],
            rssis: [],
        });
    return stats.get(id);
}

// A packet's measurements as [name, value] pairs, for the station table's reading column.
// Containers flatten one level ("temperature", not "environment.temperature"), and what is not a
// measurement stays out: framing, the battery (it has its own column), the station's link report,
// position, clock, flags and raw TLVs. A decoder plugin's summary stands in for the TLV it decoded.
const READING_SKIP = new Set(['variant', 'station', 'sequence', 'rssi_packet', 'packed_bits', 'packed_bytes', 'battery', 'link', 'position', 'datetime', 'flags', 'data']);
function readingOf(rec) {
    const out = [];
    for (const [k, v] of Object.entries(rec.body || {})) {
        if (READING_SKIP.has(k) || v == null) continue;
        if (typeof v === 'number') out.push([k, v]);
        else if (typeof v === 'object' && !Array.isArray(v)) for (const [c, n] of Object.entries(v)) if (typeof n === 'number') out.push([c, n]);
    }
    const d = rec.decoded;
    if (d && !(d.fields && d.fields.error)) {
        const text = d.summary || Object.entries(d.fields || {}).map(([k, v]) => `${v} ${k}`).join(', ');
        if (text) out.push([d.name, text]);
    }
    return out;
}

// Returns true when rec is a duplicate. A duplicate is counted and otherwise ignored: it is not
// a packet the station sent, so it must not inflate pkts or the interval, refresh last-seen, or
// move the sequence baseline -- the old code took a repeated sequence for a restart, which reset
// that baseline and could hide real loss behind it.
function observe(rec) {
    const s = statFor(rec.station);
    if (rec.sequence !== undefined) {
        s.recent = s.recent.filter(([, t]) => rec.t - t <= DUP_WINDOW_MS);
        if (s.recent.some(([q]) => q === rec.sequence)) {
            s.dups++;
            return true;
        }
        s.recent.push([rec.sequence, rec.t]);
        if (s.recent.length > 64) s.recent.shift();
    }
    s.count++;
    s.last = rec.t;
    // Latest battery, reading and version, each with when it arrived: none is in every packet, and
    // the age is what makes an old reading recognisable as one.
    const bat = rec.body && rec.body.battery;
    if (bat && bat.level != null) s.battery = { level: bat.level, charging: !!bat.state, t: rec.t };
    const reading = readingOf(rec);
    if (reading.length) s.reading = { values: reading, t: rec.t };
    const ver = versionOf(rec.body);
    if (ver) versions.set(hexId(rec.station), { data: ver, t: rec.t });
    s.times.push(rec.t);
    if (s.times.length > WINDOW) s.times.shift();
    if (rec.rssi !== undefined && rec.rssi !== null) {
        s.rssis.push(rec.rssi);
        if (s.rssis.length > WINDOW) s.rssis.shift();
    }
    if (rec.sequence !== undefined) {
        if (s.prevSeq !== undefined) {
            const d = rec.sequence - s.prevSeq;
            if (d > 1 && d <= SEQ_MAX_GAP) s.lost += d - 1;
            // d <= 0 (restart/wrap) or d > SEQ_MAX_GAP: not loss we can attribute, so the
            // baseline simply moves to the new value.
        }
        s.prevSeq = rec.sequence;
    }
    return false;
}

// Counts are for the whole session; interval and average RSSI are over the last WINDOW packets, so
// they track what is happening NOW rather than being dragged back by an hour of earlier data —
// which is the point when you are walking a station around the site.
function snapshot() {
    const now = Date.now();
    return [...stats.values()]
        .map((s) => {
            const meta = STATIONS[s.id] || {};
            const n = s.times.length;
            const span = n > 1 ? (s.times[n - 1] - s.times[0]) / 1000 : 0;
            const expected = s.count + s.lost;
            return {
                id: s.id,
                name: meta.name || null,
                lat: meta.latitude ?? null,
                lon: meta.longitude ?? null,
                distance: meta.distance ?? null,
                state: meta.state || null, /* free text from station-<ID>-state: deployed, broken, ... */
                count: s.count,
                lost: s.lost,
                dups: s.dups,
                battery: s.battery ? s.battery.level : null,
                charging: s.battery ? s.battery.charging : null,
                batteryAge: s.battery ? Math.round((now - s.battery.t) / 1000) : null,
                batteryStale: s.battery ? (now - s.battery.t) / 1000 > BATTERY_STALE_S : false,
                reading: s.reading ? s.reading.values : null,
                readingAge: s.reading ? Math.round((now - s.reading.t) / 1000) : null,
                version: versions.has(s.id) ? versions.get(s.id).data : null,
                versionAge: versions.has(s.id) ? Math.round((now - versions.get(s.id).t) / 1000) : null,
                lossPct: expected > 0 ? (s.lost * 100) / expected : 0,
                // Mean seconds between packets: "one every 2m30s" reads better than "0.4/m".
                interval: span > 0 ? span / (n - 1) : null,
                rssiAvg: s.rssis.length ? s.rssis.reduce((a, b) => a + b, 0) / s.rssis.length : null,
                age: s.last ? Math.round((now - s.last) / 1000) : null,
            };
        })
        .sort((a, b) => a.id.localeCompare(b.id));
}

function record(rec) {
    if (observe(rec)) rec.dup = true; // before log and push, so both carry the flag
    history.push(rec);
    while (history.length > KEEP) history.shift();
    logAppend(rec);
    send('packet', rec);
    send('stats', snapshot());
}

// The same fix arrives several times over. The T1000-B sends confirmed uplinks and repeats them
// until the network acknowledges, so one button press produces two or three uplinks carrying an
// identical payload under consecutive f_cnt values. Sixteen rows of the same coordinate would be
// a useless panel, so a repeat updates the row it duplicates instead of adding one.
//
// Identity is the measurement timestamp and the coordinate, not f_cnt — f_cnt is what differs
// between the copies. Repeats are counted, and the reception kept is the strongest one, since
// during a survey the question is how well the spot CAN be heard, not how badly.
// Merge one position into the buffer; returns the entry that now represents it. Separate from
// recordPosition so that replaying the log during a rebuild goes through exactly the same
// dedupe as live traffic does, without writing the log back out or pushing to browsers.
function mergePosition(pos) {
    pos.distance = REF && pos.lat != null && pos.lon != null ? haversine(REF.lat, REF.lon, pos.lat, pos.lon) : null;

    const dup = gnss.find((g) => g.device === pos.device && g.t === pos.t && g.lat === pos.lat && g.lon === pos.lon);
    if (dup) {
        // Live traffic arrives one reception at a time and increments; a replayed log record
        // already carries the running total, so taking the larger keeps a rebuild from summing
        // the same fix's successive log lines into a nonsense count.
        dup.rx = pos.rx ? Math.max(dup.rx || 1, pos.rx) : (dup.rx || 1) + 1;
        dup.fcnt = pos.fcnt; // the newest copy's counter, so a gap in the panel is still visible
        if (pos.rssi != null && (dup.rssi == null || pos.rssi > dup.rssi)) {
            dup.rssi = pos.rssi;
            dup.snr = pos.snr;
            dup.gateway = pos.gateway;
        }
        return dup;
    }

    pos.rx = pos.rx || 1;
    gnss.push(pos);
    while (gnss.length > TRACKER_KEEP) gnss.shift();
    return pos;
}

// The badge counts every position held, the panel lists only the most recent tracker-show of
// them: "[#40]" after forty fixes is the useful number, and scrolling forty rows on a phone at
// the foot of a mast is not.
const gnssView = () => ({ n: gnss.length, rows: gnss.slice(-TRACKER_SHOW) });

function recordPosition(pos) {
    const entry = mergePosition(pos);
    gnssLogAppend(entry);
    send('gnss', gnssView());
}

// A tracker uplink off the network server's feed. A completely different payload from a station
// packet, so it gets its own parser rather than teaching the station one about LoRaWAN.
function handleTrackerMessage(topic, text) {
    let body;
    try {
        body = JSON.parse(text);
    } catch {
        return; // join accepts, downlink acks, service status: not uplinks
    }
    const decode = TRACKER_TYPES[TRACKER.type];
    if (!decode) return;
    let pos;
    try {
        pos = decode(body);
    } catch (e) {
        console.error(`tracker: decode failed (${e.message})`);
        return;
    }
    if (pos) recordPosition(pos);
}

// Which topic kinds feed the gateways-and-relays section, and where each one is filed. They are
// taken whatever mqtt-accept says: that setting decides what counts as a station packet, and
// none of these is one.
const GATEWAY_KINDS = { gateway: 'gw', network: 'net', ddup: 'ddup' };

function handleGatewayMessage(kind, topic, body) {
    const id = hexId(topic.split('/').pop());
    if (!specified(id)) return;
    const g = gateways.get(id) || { id };
    g[GATEWAY_KINDS[kind]] = { t: Date.now(), body };
    gateways.set(id, g);
    send('gateways', gatewayView());
}

// One row per gateway, followed by the relays it reports. Ages are counted from when each report
// ARRIVED plus whatever age it carried, so a gateway that stops reporting shows its relays going
// stale too, rather than frozen at the last values it sent.
function gatewayView() {
    const now = Date.now();
    const since = (part) => (part ? Math.round((now - part.t) / 1000) : null);
    const name = (id) => (STATIONS[id] || {}).name || null;
    const rows = [];
    for (const g of [...gateways.values()].sort((a, b) => a.id.localeCompare(b.id))) {
        const gw = (g.gw && g.gw.body) || {};
        const net = (g.net && g.net.body) || {};
        const ddup = (g.ddup && g.ddup.body) || null;
        const ages = [since(g.gw), since(g.net)].filter((a) => a != null);
        rows.push({
            role: 'gateway',
            id: g.id,
            name: name(g.id),
            version: gw.version ?? null,
            uptime: gw.uptime_secs != null ? gw.uptime_secs + since(g.gw) : null,
            age: ages.length ? Math.min(...ages) : since(g.ddup),
            addr: ddup && ddup.ip ? `${ddup.ip}${ddup.port ? `:${ddup.port}` : ''}` : null,
            software: versions.has(g.id) ? versions.get(g.id).data.software || null : null,
            stations: Array.isArray(gw.stations) ? gw.stations.length : null,
            links: (Array.isArray(gw.links) ? gw.links : []).map((l) => {
                const c = l.config || {}, rx = l.rx || {}, tx = l.tx || {};
                return {
                    name: l.name || l.type || 'link',
                    channel: c.channel ?? null,
                    mhz: c.frequency_khz != null ? c.frequency_khz / 1000 : null,
                    dbm: c.transmit_power_dbm ?? null,
                    bps: c.packet_rate_bps ?? null,
                    rssi: (l.rssi || {}).packet_dbm ?? null,
                    rx: rx.packets ?? null,
                    rxBad: (rx.errors || 0) + (rx.drops || 0),
                    tx: tx.packets ?? null,
                    txBad: tx.errors || 0,
                };
            }),
        });
        const netAge = since(g.net) || 0;
        for (const r of (Array.isArray(net.network) ? net.network : []).filter((n) => n && n.kind === 'relay' && specified(n.station))) {
            const id = hexId(r.station);
            rows.push({
                role: 'relay',
                id,
                name: name(id),
                gateway: g.id,
                age: r.age_secs != null ? r.age_secs + netAge : null,
                staleAfter: RELAY_STALE_S,
                packets: r.packets ?? null,
                rssi: r.rssi ?? null,
                cost: r.cost ?? null,
                accepting: r.accepting ?? null,
                beacons: r.beacons ?? null,
                forwards: r.forwards ?? null,
                hears: Array.isArray(r.hears) ? r.hears.length : null,
                software: versions.has(id) ? versions.get(id).data.software || null : null,
                versionAge: versions.has(id) ? Math.round((now - versions.get(id).t) / 1000) : null,
            });
        }
    }
    // Survey instruments: a node announcing itself as one ("survey/..." software), or seen as the
    // observer in survey reports -- unless it is already a gateway or relay above, which is what an
    // observer in a gateway-directed survey is.
    const listed = new Set(rows.map((r) => r.id));
    const ids = new Set(surveys.keys());
    for (const [id, v] of versions) if (String(v.data.software || '').startsWith('survey/')) ids.add(id);
    for (const id of [...ids].sort()) {
        if (listed.has(id)) continue;
        const sv = surveys.get(id) || null, ver = versions.get(id) || null;
        const ages = [since(sv), since(ver)].filter((a) => a != null);
        rows.push({
            role: 'survey',
            id,
            name: name(id),
            age: ages.length ? Math.min(...ages) : null,
            software: ver ? ver.data.software || null : null,
            versionAge: since(ver),
            observations: sv ? sv.n : null,
            lat: sv && sv.lat != null ? sv.lat : null,
            lon: sv && sv.lon != null ? sv.lon : null,
            alt: sv && sv.alt != null ? sv.alt : null,
            // what the latest report heard, strongest first
            heard: sv
                ? Object.entries(sv.heard || {})
                      .map(([sid, h]) => ({ id: sid, name: name(sid), rssi: h.rssi }))
                      .sort((a, b) => (b.rssi ?? -999) - (a.rssi ?? -999))
                : [],
        });
    }
    return rows;
}

// A topic's kind from its shape. Level 1 after the prefix is always a CATEGORY, and station
// telemetry lives in exactly one of them: <prefix>/variant/<name>/<station-hex> is 'station'.
// Anything else is named by its second-to-last segment, so iotdata/manage/from/<station> is
// 'from' and iotdata/stats/gateway/<station> is 'gateway'.
//
// THE HEX IN THE LAST SEGMENT IS NOT ENOUGH ANY MORE, which is what this used to test. Every
// category now ends in a station id -- the gateway's own stats document, the network table, a
// management report -- and each of those carries a `station` field as well, so the shape test
// and the payload test both passed and all three surfaced as phantom stations whose fields were
// rendered as if they were a reading.
function topicKind(topic) {
    const parts = String(topic).split('/');
    const last = parts[parts.length - 1];
    if (parts.length >= 3 && parts[parts.length - 3] === 'variant' && /^[0-9A-Fa-f]{1,8}$/.test(last)) return 'station';
    return parts.length >= 2 ? parts[parts.length - 2] : last;
}

// A sensor packet is identified by carrying a station id, which is what separates it from the
// gateway's own stats/status publications on neighbouring topics. Matching on that rather than
// on a topic pattern means new variants appear here without this script needing to learn them.
function handleMessage(topic, text) {
    let body;
    try {
        body = JSON.parse(text);
    } catch {
        return; // not JSON: the gateway's own status publications, retained junk
    }
    if (!body || typeof body !== 'object') return;
    // A node's report about itself: <prefix>/manage/from/<id>. Only the version TLV is used for now.
    const tp = topic.split('/');
    if (tp.length >= 3 && tp[tp.length - 3] === 'manage' && tp[tp.length - 2] === 'from') {
        if (body.tlv === 'version' && body.data && typeof body.data === 'object') {
            if (!specified(tp[tp.length - 1])) return;
            versions.set(hexId(tp[tp.length - 1]), { data: body.data, t: Date.now() });
            send('stats', snapshot());
            send('gateways', gatewayView());
        } else if (body.type === 'survey' && body.cmd === 'survey' && body.observer != null && body.station != null) {
            if (!specified(body.observer) || !specified(body.station)) return;
            const id = hexId(body.observer);
            const sv = surveys.get(id) || { n: 0, recent: [], heard: {} };
            // Every gateway that hears the instrument's report publishes it, so each observation can
            // arrive once per path. Its (station, seq) identifies it: count it once.
            const key = `${body.station}:${body.seq}`;
            if (sv.recent.includes(key)) return;
            sv.recent.push(key);
            if (sv.recent.length > 256) sv.recent.shift();
            // One report is a run of observations sharing the instrument's position and timestamp;
            // a new one replaces what the last one heard, so the map shows the latest stand.
            const report = `${body.time}:${body.obs_lat}:${body.obs_lon}`;
            if (report !== sv.report) {
                sv.report = report;
                sv.heard = {};
                sv.lat = body.obs_lat ?? null;
                sv.lon = body.obs_lon ?? null;
                sv.alt = body.obs_alt ?? null;
            }
            sv.heard[hexId(body.station)] = { rssi: body.rssi ?? null, seq: body.seq };
            sv.t = Date.now();
            sv.n++;
            surveys.set(id, sv);
            send('gateways', gatewayView());
        }
        return;
    }
    const kind = topicKind(topic);
    if (GATEWAY_KINDS[kind]) return handleGatewayMessage(kind, topic, body);
    if (body.station === undefined) return;
    if (!ACCEPT.has(kind)) return;

    // Topic is <prefix>/variant/<variant-name>/<station-hex>; the variant NAME lives only here
    // (the payload carries the numeric id), so it is worth taking rather than maintaining a table.
    const parts = topic.split('/');
    const station = parts.length >= 1 ? parts[parts.length - 1] : '?';
    if (!specified(station)) return;
    const meta = STATIONS[hexId(station)] || null;

    record({
        id: ++counter,
        t: Date.now(),
        topic,
        variant: parts.length >= 2 ? parts[parts.length - 2] : '?',
        variantId: body.variant,
        station,
        sequence: body.sequence,
        rssi: body.rssi_packet,
        site: meta
            ? {
                  name: meta.name || null,
                  lat: meta.latitude ?? null,
                  lon: meta.longitude ?? null,
                  distance: meta.distance ?? null,
              }
            : null,
        decoded: runDecoders(station, body),
        body,
    });
}

// ---------------------------------------------------------------------------- log

// One JSON object per line (JSON Lines), not one big array: a growing array would have to be
// rewritten whole on every flush, and a file cut short by a power loss would be unparseable.
// Line-per-record appends, survives a truncated tail, and reads directly into jq or pandas.
// Bounded so that a full or unwritable filesystem costs a known amount of memory rather than
// growing until the process is killed — this box has little to spare.
const LOG_BUFFER_MAX = 10000;

// There are two of these now — received packets, and tracker positions — with identical
// batching, bounding and flush-on-shutdown requirements. The behaviour is written once here and
// instantiated per file rather than copied, so a fix to one is a fix to both.
function makeLog(file, flushSecs, label) {
    const buffer = [];
    let written = 0;
    let dropped = 0;

    function append(rec) {
        if (!file) return;
        if (buffer.length >= LOG_BUFFER_MAX) {
            dropped++;
            return;
        }
        buffer.push(JSON.stringify(rec) + '\n');
    }

    function flush(sync) {
        if (!file || !buffer.length) return;
        const n = buffer.length;
        const chunk = buffer.splice(0, n).join('');
        const failed = (e) => console.error(`${label}: append to ${file} failed, ${n} record(s) lost: ${e.message}`);
        try {
            if (sync) fs.appendFileSync(file, chunk);
            else
                fs.appendFile(file, chunk, (e) => {
                    if (e) failed(e);
                    else written += n;
                });
            if (sync) written += n;
        } catch (e) {
            failed(e);
        }
    }

    if (file) {
        setInterval(flush, flushSecs * 1000);
        // Registered before the subscriber's handlers (which call process.exit), so the tail of
        // the buffer reaches disk on a restart instead of being dropped.
        for (const sig of ['SIGINT', 'SIGTERM']) process.on(sig, () => flush(true));
    }

    return {
        file,
        append,
        flush,
        stat: () => ({ written, dropped, pending: buffer.length }),
    };
}

const packetLog = makeLog(LOG_FILE, LOG_FLUSH, 'log');
const gnssLog = makeLog(TRACKER_LOG, TRACKER_LOG_FLUSH, 'tracker log');

function logAppend(rec) {
    packetLog.append(rec);
}
function gnssLogAppend(pos) {
    gnssLog.append(pos);
}

// ---------------------------------------------------------------------------- snapshot

// Everything above the packet list — which stations have been heard at all, how many packets
// each, how much loss, how long since the last one — lives only in memory. So a restart for a
// config edit silently resets the site's accumulated history to nothing, and the stations that
// went quiet days ago vanish from the table rather than showing as stale. During a survey that
// is precisely the information being accumulated, and losing it is not a small thing.
//
// Three ways it comes back, in order of preference:
//
//   snapshotLoad()  read /dev/shm — the fast path, a few hundred KB, effectively instant
//   snapshotMake()  rebuild by streaming the logs — the slow path, seconds, and what runs
//                   after a reboot (which empties /dev/shm) or on the first run after an upgrade
//   neither         start empty, exactly as before
//
// snapshotSave() writes the fast path on a timer and once more on the way out.

function snapshotSave() {
    if (!SNAPSHOT_FILE) return;
    const data = {
        version: SNAPSHOT_VERSION,
        savedAt: Date.now(),
        counter,
        stats: [...stats.values()],
        history,
        gnss,
        gateways: [...gateways.values()],
        versions: [...versions.entries()],
        surveys: [...surveys.entries()],
    };
    // Written to a sibling and renamed rather than in place: rename is atomic, so a restart
    // landing mid-write finds either the previous snapshot or the new one, never a half-written
    // file that has to be thrown away — which would defeat the point on the one occasion it matters.
    //
    // Synchronous even on the timer: a few hundred KB to tmpfs costs less than the bookkeeping
    // needed to do it asynchronously without racing the shutdown path.
    const tmp = SNAPSHOT_FILE + '.tmp';
    try {
        fs.writeFileSync(tmp, JSON.stringify(data));
        fs.renameSync(tmp, SNAPSHOT_FILE);
    } catch (e) {
        console.error(`snapshot: save to ${SNAPSHOT_FILE} failed: ${e.message}`);
    }
}

function snapshotLoad() {
    if (!SNAPSHOT_FILE) return null;
    let data;
    try {
        data = JSON.parse(fs.readFileSync(SNAPSHOT_FILE, 'utf8'));
    } catch {
        return null; // absent is the normal case after a reboot; unreadable is treated the same
    }
    if (!data || data.version !== SNAPSHOT_VERSION || !Array.isArray(data.stats)) return null;

    stats.clear();
    // Defaults first so a snapshot written by an older version, missing a field added since,
    // restores with a sane value rather than an undefined that breaks the stats arithmetic.
    for (const st of data.stats) if (st && st.id && specified(st.id)) stats.set(st.id, { count: 0, lost: 0, dups: 0, recent: [], times: [], rssis: [], ...st });
    history.length = 0;
    for (const rec of Array.isArray(data.history) ? data.history.slice(-KEEP) : []) if (rec && specified(rec.station)) history.push(rec);
    gnss.length = 0;
    for (const g of Array.isArray(data.gnss) ? data.gnss.slice(-TRACKER_KEEP) : []) gnss.push(g);
    // Absent from a snapshot written before the section existed, which simply restores none.
    gateways.clear();
    for (const g of Array.isArray(data.gateways) ? data.gateways : []) if (g && g.id && specified(g.id)) gateways.set(g.id, g);
    versions.clear();
    for (const e of Array.isArray(data.versions) ? data.versions : []) if (Array.isArray(e) && e[0] && e[1] && specified(e[0])) versions.set(e[0], e[1]);
    surveys.clear();
    for (const e of Array.isArray(data.surveys) ? data.surveys : [])
        if (Array.isArray(e) && e[0] && e[1] && specified(e[0])) {
            const sv = { recent: [], heard: {}, ...e[1] };
            for (const sid of Object.keys(sv.heard || {})) if (!specified(sid)) delete sv.heard[sid];
            surveys.set(e[0], sv);
        }
    counter = data.counter || history.length;
    return {
        savedAt: data.savedAt,
        stations: stats.size,
        packets: history.length,
        positions: gnss.length,
    };
}

// Read a JSON Lines file record by record. Streamed rather than read whole: the packet log is
// already tens of megabytes and grows for as long as the site runs, while this box has under
// half a gigabyte of RAM. A line that will not parse is counted and skipped — that is what a
// log cut short by a power loss looks like, and it should cost the tail record, not the rebuild.
function streamJsonLines(file, onRecord, done) {
    if (!file) return done(null, 0, 0);
    const stream = fs.createReadStream(file, { encoding: 'utf8' });
    let failed = false;
    stream.on('error', (e) => {
        failed = true;
        done(e, 0, 0);
    });
    const rl = readline.createInterface({ input: stream, crlfDelay: Infinity });
    let n = 0;
    let bad = 0;
    rl.on('line', (line) => {
        if (!line) return;
        let rec;
        try {
            rec = JSON.parse(line);
        } catch {
            bad++;
            return;
        }
        onRecord(rec);
        n++;
    });
    rl.on('close', () => {
        if (!failed) done(null, n, bad);
    });
}

function snapshotMake(done) {
    const t0 = Date.now();
    stats.clear();
    history.length = 0;
    gnss.length = 0;
    counter = 0;

    streamJsonLines(
        LOG_FILE,
        (rec) => {
            if (!rec || rec.station === undefined) return;
            // The log may predate the filter, or a narrower mqtt-accept, so it applies here too.
            if (rec.topic !== undefined && !ACCEPT.has(topicKind(rec.topic))) return;
            if (!specified(rec.station)) return;
            // observe() and not record(): the log is the SOURCE here, so appending it back to
            // itself, or pushing to browsers that have not connected yet, would both be wrong.
            observe(rec);
            if (typeof rec.id === 'number' && rec.id > counter) counter = rec.id;
            history.push(rec);
            if (history.length > KEEP) history.shift();
        },
        (err, packets, bad) => {
            if (err) console.error(`snapshot: rebuild from ${LOG_FILE} failed: ${err.message}`);
            streamJsonLines(
                TRACKER_LOG,
                (pos) => {
                    if (pos && pos.lat != null && pos.lon != null) mergePosition(pos);
                },
                (err2, lines, bad2) => {
                    if (err2) console.error(`snapshot: rebuild from ${TRACKER_LOG} failed: ${err2.message}`);
                    done({
                        packets,
                        bad: (bad || 0) + (bad2 || 0),
                        lines,
                        positions: gnss.length,
                        ms: Date.now() - t0,
                    });
                }
            );
        }
    );
}

// Fast path, then slow path, then empty. Deliberately finishes BEFORE the subscribers start:
// rebuilding from a log while new packets are being appended to it would double-count whatever
// arrived in between, and a few seconds of delayed subscription costs nothing here.
function restore(done) {
    if (!RESTORE) return done('skipped (--no-restore)');
    const loaded = snapshotLoad();
    if (loaded) {
        const age = Math.max(0, Math.round((Date.now() - (loaded.savedAt || Date.now())) / 1000));
        return done(`snapshot ${SNAPSHOT_FILE}, ${age}s old: ${loaded.stations} station(s), ${loaded.packets} packet(s), ${loaded.positions} position(s)`);
    }
    if (!LOG_FILE && !TRACKER_LOG) return done('nothing to restore from (no snapshot, no log configured)');
    console.log(`  restore no snapshot at ${SNAPSHOT_FILE}; rebuilding from the log, this takes a moment`);
    snapshotMake((r) => done(`rebuilt from log in ${(r.ms / 1000).toFixed(1)}s: ${r.packets} packet(s), ${gnss.length} position(s)` + (r.bad ? `, ${r.bad} unparseable line(s) skipped` : '')));
}

if (SNAPSHOT_FILE) {
    setInterval(snapshotSave, SNAPSHOT_INTERVAL * 1000);
    // After the log flushes and before the subscribers register their exit handlers, so the
    // snapshot written on shutdown is the last thing the state does before the process goes.
    for (const sig of ['SIGINT', 'SIGTERM']) process.on(sig, () => snapshotSave());
}

// ---------------------------------------------------------------------------- subscriber

// Two brokers: this site's own, and optionally the tracker's network server out on the internet.
// Both go away from time to time — a mosquitto restart, the site's uplink dropping — and both
// must come back without help, so the connection options are shared and only the endpoint,
// credentials and handler differ.
const brokers = new Set();

function subscriber(label, url, opts, topic, onMessage) {
    const client = mqtt.connect(url, {
        reconnectPeriod: 5000,
        connectTimeout: 30000,
        // A stable id per role so a reconnect displaces the previous session rather than leaving
        // a ghost subscribed on the broker; the hostname keeps two boxes on one network server
        // from taking turns evicting each other.
        clientId: `iotdata-monitor-${label.replace(/[^\w.-]/g, '-')}-${os.hostname()}`,
        ...opts,
    });
    brokers.add(client);

    client.on('connect', () => {
        console.log(`${label}: connected to ${url}`);
        client.subscribe(topic, { qos: 0 }, (err) => {
            if (err) console.error(`${label}: subscribe to ${topic} failed: ${err.message}`);
        });
    });
    // Logged, never fatal. Everything here is a link that is expected to fail sometimes, and the
    // library retries on its own — exiting would take the web view down with it, which is the
    // one thing that must stay up while someone is standing at the foot of a mast reading it.
    client.on('error', (e) => console.error(`${label}: ${e.message}`));
    client.on('reconnect', () => console.error(`${label}: reconnecting to ${url}`));
    client.on('message', (t, payload) => onMessage(t, payload.toString('utf8')));
    return client;
}

for (const sig of ['SIGINT', 'SIGTERM'])
    process.on(sig, () => {
        for (const c of brokers) c.end(true);
        process.exit(0);
    });

function subscribe() {
    subscriber('mqtt', `mqtt://${BROKER}:${MQTT_PORT}`, {}, TOPIC, handleMessage);
    if (!TRACKER) return;

    const opts = {};
    if (TRACKER.user) opts.username = TRACKER.user;
    if (TRACKER.pass) opts.password = TRACKER.pass;
    if (TRACKER.tls) {
        // Certificate verification stays on. The whole point of TLS on this connection is that
        // the API key is not handed to whatever happens to answer on port 8883.
        opts.rejectUnauthorized = true;
        if (TRACKER.cafile) {
            try {
                opts.ca = fs.readFileSync(TRACKER.cafile);
            } catch (e) {
                console.error(`tracker: cannot read ${TRACKER.cafile} (${e.message}); falling back to the built-in CA store`);
            }
        }
    }
    const url = `${TRACKER.tls ? 'mqtts' : 'mqtt'}://${TRACKER.host}:${TRACKER.port}`;
    subscriber(`tracker[${TRACKER.type}]`, url, opts, TRACKER.topic, handleTrackerMessage);
}

// Re-push stats on a timer as well as per packet, so "age" counts up on screen while a station
// is silent. Without this a station that stopped transmitting would sit there looking healthy.
setInterval(() => {
    send('stats', snapshot());
    send('gateways', gatewayView());
}, 5000);

// The gateway's own position in the title bar, linked the same way the station distances
// are. Empty when no reference is configured, which also disables distances.
const REF_LINK = REF ? `<a class="ref" href="https://www.google.com/maps/search/?api=1&amp;query=${REF.lat},${REF.lon}" target="_blank" rel="noopener noreferrer">${REF.lat.toFixed(6)}, ${REF.lon.toFixed(6)}</a>` : '';

// ---------------------------------------------------------------------------- map

// Google Maps needs a browser-side API key, and unlike the network server key this one is meant
// to be public: it names the project to bill, so it necessarily ships inside the page for the
// browser to present. Google's answer to that exposure is restriction rather than secrecy — tie
// the key to the Maps JavaScript API and to the referrers you serve from, in the Cloud console.
// It still lives in the config file rather than in this source, so the repo stays clean and each
// host can carry its own. With no key the header link disappears and /map says why.
const MAP_KEY = cfg['map-api-key'] ?? null;
// hybrid rather than roadmap: walking a site, the useful backdrop is the actual tree line,
// tracks and rooflines, with the road labels still drawn over them. roadmap, satellite and
// terrain are the other three.
const MAP_TYPE = cfg['map-type'] ?? 'hybrid';
// Only used when there is a single point to show. With more than one the view is fitted to them.
const MAP_ZOOM = parseInt(cfg['map-zoom'] ?? '17', 10);

// With twenty devices the gateway's bare coordinates stop being the useful thing in the title
// bar, so the map takes their place — the reference position is still one click away, now with
// everything else drawn around it. Falls back to the coordinates when no key is configured.
const MAP_LINK = MAP_KEY
    ? '<a class="ref" href="/map" target="_blank" rel="noopener noreferrer">[map]</a>'
    : REF_LINK;

// ---------------------------------------------------------------------------- page

// A radio mast throwing two arcs: this box is the LoRa gateway the rest of the mesh reports
// to, and none of the other machines' icons are radio-themed. Inlined as a data: URI so the
// page stays a single response with no second request for 400 bytes. Two things bite here:
// '#' must be %23-escaped or the URL truncates at a fragment, and the SVG uses single quotes
// so the href can keep its double ones. One mid-tone colour rather than the --good/--weak
// palette, which is semantic on this page and would be read as a health indicator; teal also
// stays legible against both light and dark browser chrome, which the page's own colours,
// swapped by prefers-color-scheme, would not.
const FAVICON = `<link rel="icon" type="image/svg+xml" href="data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 32 32' fill='none' stroke='%230d9488' stroke-width='3' stroke-linecap='round' stroke-linejoin='round'><path d='M16 12.5V27'/><path d='M10.5 27L16 20l5.5 7'/><path d='M21.2 8.3A6.5 6.5 0 0 1 21.2 16.7'/><path d='M10.8 8.3A6.5 6.5 0 0 0 10.8 16.7'/></svg>">`;

const PAGE = `<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>iotdata monitor</title>
${FAVICON}
<script>var q=new URLSearchParams(location.search),d=document.documentElement.dataset,t=q.get('theme'),k=q.get('kiosk');if(t)d.theme=t;if(k!=null&&!/^(off|no|false|0)$/i.test(k))d.kiosk='';</script>
<style>
  :root {
    --bg:#f6f7f9; --fg:#14171a; --dim:#6b7280; --line:#dfe3e8; --card:#fff;
    --good:#0a7d36; --ok:#a86400; --weak:#c02a2a;
  }
  @media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) { --bg:#101215; --fg:#e6e8ea; --dim:#9aa3ad; --line:#262b31; --card:#181b1f;
            --good:#4ade80; --ok:#fbbf24; --weak:#f87171; }
  }
  :root[data-theme="dark"] { --bg:#101215; --fg:#e6e8ea; --dim:#9aa3ad; --line:#262b31; --card:#181b1f;
            --good:#4ade80; --ok:#fbbf24; --weak:#f87171; }
  * { box-sizing:border-box; }
  body { margin:0; background:var(--bg); color:var(--fg);
         font:15px/1.4 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;
         padding-bottom:env(safe-area-inset-bottom); }
  header { position:sticky; top:0; z-index:2; background:var(--card);
           border-bottom:1px solid var(--line); }
  .bar { padding:8px 12px; display:flex; flex-wrap:wrap; align-items:center; gap:4px 10px; }
  .bar b { font-size:15px; font-weight:600; }
  :root[data-kiosk] .bar .ref { display:none; }
  .bar .ref { color:var(--dim); font-size:12px; text-decoration:underline;
              text-decoration-thickness:1px; text-underline-offset:2px;
              font-family:ui-monospace,SFMono-Regular,Menlo,monospace; }
  #stat { margin-left:auto; font-size:12px; color:var(--dim); display:flex; align-items:center; gap:6px; }
  #dot { width:9px; height:9px; border-radius:50%; background:var(--weak); }
  #dot.live { background:var(--good); }
  table { width:100%; border-collapse:collapse; font-size:12.5px;
          font-variant-numeric:tabular-nums; }
  th { text-align:right; font-weight:600; color:var(--dim); font-size:10.5px;
       text-transform:uppercase; letter-spacing:.04em; padding:2px 8px 4px; }
  th:first-child, td:first-child { text-align:left; }
  td { text-align:right; padding:3px 8px; border-top:1px solid var(--line);
       font-family:ui-monospace,SFMono-Regular,Menlo,monospace; }
  td.who { font-family:inherit; }
  td.who b { font-family:ui-monospace,SFMono-Regular,Menlo,monospace; font-weight:650; }
  td.who span { color:var(--dim); }
  td.who .nm { margin-left:6px; }
  td.who a { color:inherit; text-decoration:underline; text-decoration-thickness:1px;
             text-underline-offset:2px; }
  .stale { color:var(--weak); font-weight:600; }
  #list { padding:8px; display:flex; flex-direction:column; gap:8px; }
  .rec { background:var(--card); border:1px solid var(--line); border-radius:10px; padding:9px 11px; }
  .rec.new { animation:flash 1.2s ease-out; }
  @keyframes flash { from { border-color:var(--good); } to { border-color:var(--line); } }
  .top { display:flex; flex-wrap:wrap; align-items:baseline; gap:8px; }
  .time { font-variant-numeric:tabular-nums; color:var(--dim); font-size:12px;
          font-family:ui-monospace,SFMono-Regular,Menlo,monospace; }
  .station { font-weight:650; font-family:ui-monospace,SFMono-Regular,Menlo,monospace; }
  .variant { color:var(--dim); font-size:13px; }
  .seq { color:var(--dim); font-size:12px; font-variant-numeric:tabular-nums;
         font-family:ui-monospace,SFMono-Regular,Menlo,monospace; }
  .rssi { margin-left:auto; font-weight:700; font-size:17px; font-variant-numeric:tabular-nums;
          font-family:ui-monospace,SFMono-Regular,Menlo,monospace; }
  .rssi.good { color:var(--good); } .rssi.ok { color:var(--ok); } .rssi.weak { color:var(--weak); }
  .gap { color:var(--weak); font-size:11px; font-weight:600; border:1px solid var(--weak);
         border-radius:4px; padding:0 4px; }
  .site { margin-top:4px; font-size:12.5px; color:var(--dim); }
  .site b { color:var(--fg); font-weight:600; }
  .site code { font-family:ui-monospace,SFMono-Regular,Menlo,monospace; font-size:11.5px; }
  .dec { margin-top:7px; padding-left:8px; border-left:3px solid var(--line); }
  .dec-name { font-size:11px; color:var(--dim); text-transform:uppercase; letter-spacing:.04em; }
  .dec-name b { color:var(--fg); font-weight:650; font-size:12.5px; text-transform:none;
                letter-spacing:0; margin-left:0; }
  .dec-vals { margin-top:3px; font-family:ui-monospace,SFMono-Regular,Menlo,monospace;
              font-size:12.5px; line-height:1.5; word-spacing:.15em; }
  .chips { margin-top:6px; display:flex; flex-wrap:wrap; gap:5px; }
  .chip { background:var(--bg); border:1px solid var(--line); border-radius:6px;
          padding:2px 7px; font-size:13px; font-variant-numeric:tabular-nums; }
  .chip i { color:var(--dim); font-style:normal; font-size:11px; margin-right:4px;
            text-transform:uppercase; letter-spacing:.03em; }
  .gbtn { font:inherit; font-size:12px; color:var(--fg); background:var(--bg);
          border:1px solid var(--line); border-radius:6px; padding:1px 7px; cursor:pointer;
          font-variant-numeric:tabular-nums; font-family:ui-monospace,SFMono-Regular,Menlo,monospace; }
  .gbtn[aria-expanded="true"] { border-color:var(--good); color:var(--good); }
  #gnss { border-top:1px solid var(--line); background:var(--bg); padding:2px 0 6px; }
  #gnss .lbl { padding:5px 8px 1px; font-size:10.5px; color:var(--dim);
               text-transform:uppercase; letter-spacing:.04em; }
  #gnss td.ev { font-family:inherit; text-align:left; color:var(--dim); }
  #gnss a { color:inherit; text-decoration:underline; text-decoration-thickness:1px;
            text-underline-offset:2px; }
  #gnss .rx { color:var(--dim); font-size:11px; margin-left:3px; }
  #gnss .good { color:var(--good); } #gnss .ok { color:var(--ok); } #gnss .weak { color:var(--weak); }
  td.fw { font-size:11.5px; color:var(--dim); white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
  td.state { font-size:11.5px; color:var(--dim); white-space:nowrap; }
  /* The station table's reading takes the width the rest can spare: the numbers shrink to fit,
     and a long name is cut short (its tooltip has all of it). Every heading sorts its column. */
  #sum th { white-space:nowrap; cursor:pointer; user-select:none; }
  #sum th .u { text-transform:none; }
  #sum th.nm, #sum td.nm, #sum th.state, #sum td.state, #sum th.rd, #sum td.rd { text-align:left; }
  #sum td.id { font-weight:650; }
  #sum td.nm { font-family:inherit; color:var(--dim); }
  #sum td.rd span { color:var(--dim); }
  #sum td.id, #sum td.nm, #sum td.state, #sum td.n { width:1%; white-space:nowrap; }
  #sum td.nm span { display:inline-block; max-width:190px; overflow:hidden; text-overflow:ellipsis;
                    white-space:nowrap; vertical-align:bottom; }
  #sum td.n a { color:inherit; text-decoration:underline; text-decoration-thickness:1px;
                text-underline-offset:2px; }
  /* software | up | last close both tables, and get the same widths in each so the two line up */
  #gw th:nth-last-child(3), #gw td:nth-last-child(3), #sum th:nth-last-child(3), #sum td:nth-last-child(3) { width:210px; min-width:210px; max-width:210px; }
  #gw th:nth-last-child(-n+2), #gw td:nth-last-child(-n+2), #sum th:nth-last-child(-n+2), #sum td:nth-last-child(-n+2) { width:52px; min-width:52px; max-width:52px; }
  #gw { border-bottom:1px solid var(--line); padding:2px 0 4px; }
  #gw .lbl { padding:5px 8px 1px; font-size:10.5px; color:var(--dim);
             text-transform:uppercase; letter-spacing:.04em; }
  #gw td.role { font-family:inherit; text-align:left; color:var(--dim); }
  #gw tr.relay td.who { padding-left:20px; }
  #gw .good { color:var(--good); } #gw .ok { color:var(--ok); } #gw .weak { color:var(--weak); }
  .empty { color:var(--dim); padding:24px 12px; text-align:center; }
  .note { color:var(--dim); font-size:10.5px; padding:3px 8px 6px; }
</style>
</head>
<body>
<header>
  <div class="bar"><b>iotdata gateway</b>${MAP_LINK}<button id="gbtn" class="gbtn" hidden aria-expanded="false"></button><span id="stat"><span id="dot"></span><span id="msg">connecting</span></span></div>
  <div id="gnss" hidden><div class="lbl" id="glbl">tracker</div><table><tbody></tbody></table></div>
  <div id="gw" hidden><table><tbody></tbody></table></div>
  <table id="sum"><tbody></tbody></table>
  <div class="note">counts are cumulative; interval and avg rssi over the last ${WINDOW} packets</div>
</header>
<div id="list"><div class="empty">waiting for packets\u2026</div></div>
<script>
// Units by field name. Variants use both long and short forms of the same quantity, so both
// are mapped; an unmapped field still renders, just without a unit.
var UNITS = {
  temperature:'\\u00b0C', temp:'\\u00b0C', pressure:'hPa', pres:'hPa', humidity:'%', humid:'%',
  battery:'V', bat:'V', voltage:'V', wind:'m/s', wspd:'m/s', wgust:'m/s', wdir:'\\u00b0',
  rain:'mm', depth:'mm', solar:'W/m\\u00b2', light:'lx', lux:'lx',
  cpm:'cpm', dose:'\\u00b5Sv/h', rssi:'dBm', rssi_packet:'dBm',
  // Children of the containers that still flatten, taken from the library's own print_*: a
  // bare "speed 3.2" or "rate 0" is a number with its meaning left on the sensor.
  speed:'m/s', gust:'m/s', direction:'deg', rate:'mm/hr', size:'mm/d'
};
// Framing/metadata, not content — shown in the header line or not at all.
var SKIP = {variant:1, station:1, sequence:1, rssi_packet:1, packed_bits:1, packed_bytes:1};

// Containers whose children say nothing on their own. A flattened battery read "level 94" and
// "state false": the word battery, the unit, and any clue that false meant discharging had all
// gone. These render as one chip each, worded the way the library's own print_battery/_link/
// _solar word them, so the screen and an iotdata dump say the same thing rather than keeping
// two vocabularies. A container not listed here (environment, wind, rain, radiation, position)
// has self-describing children and keeps flattening, which is what flattening is for. A
// formatter that returns null declines, and the generic walk takes the packet instead -- a
// shape we did not expect is still better shown badly than not shown.
var FIELDS = {
  battery: function (v) {
    return v.level == null ? null : v.level + ' % \\u00b7 ' + (v.state ? 'charging' : 'discharging');
  },
  // Flattened, this rssi is indistinguishable from the packet rssi in the header line, and the
  // two are different measurements: this is what the STATION heard on its downlink, that is
  // what the gateway heard from the station.
  link: function (v) {
    return v.rssi == null ? null : v.rssi + ' dBm \\u00b7 snr ' + v.snr + ' dB';
  },
  solar: function (v) {
    return v.irradiance == null ? null : v.irradiance + ' W/m\\u00b2 \\u00b7 UV ' + v.ultraviolet;
  }
};

function el(tag, cls, text) {
  var e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text !== undefined && text !== null) e.textContent = text;
  return e;
}

function chip(label, value) {
  var d = el('span', 'chip');
  d.appendChild(el('i', null, label));
  d.appendChild(document.createTextNode(value));
  return d;
}

// One level of flattening: containers like "environment" contribute their children directly,
// because "temperature" is more use on a small screen than "environment.temperature".
function chipsFor(obj, into) {
  for (var k in obj) {
    if (!obj.hasOwnProperty(k) || SKIP[k]) continue;
    var v = obj[k];
    if (v === null || v === undefined) continue;
    if (Array.isArray(v)) {
      for (var n = 0; n < v.length; n++) {
        var e = v[n];
        if (e && typeof e === 'object' && e.data !== undefined)
          into.appendChild(chip(e.format || 'data', 'type ' + e.type + (obj.packed_bytes ? ' \\u00b7 ' + obj.packed_bytes + ' B' : '')));
        else into.appendChild(chip(k, typeof e === 'object' ? JSON.stringify(e) : String(e)));
      }
    } else if (typeof v === 'object') {
      var f = FIELDS[k] ? FIELDS[k](v) : null;
      if (f !== null) into.appendChild(chip(k, f));
      else chipsFor(v, into);
    } else {
      // Numbers only: several of these names (size, rate, direction) are generic enough to
      // turn up as a string somewhere else, and "small mm/d" is worse than "small".
      var u = typeof v === 'number' ? UNITS[k] : null;
      into.appendChild(chip(k, u ? v + ' ' + u : String(v)));
    }
  }
}

// LoRa demodulates below the noise floor, so a "weak" reading here is not a failed link — it is
// less margin than a strong one. Thresholds spread the range actually seen at this site so the
// colour changes as you move, which is the point during a walk test.
function rssiClass(r) { return r >= -95 ? 'good' : r >= -105 ? 'ok' : 'weak'; }
function every(t) {
  t = Math.round(t);
  if (t < 60) return t + 's';
  if (t < 3600) return Math.floor(t / 60) + 'm' + (t % 60 ? String(t % 60).padStart(2, '0') + 's' : '');
  return Math.floor(t / 3600) + 'h' + String(Math.floor((t % 3600) / 60)).padStart(2, '0') + 'm';
}
function metres(m) { return m == null ? '' : (m < 1000 ? Math.round(m) + ' m' : (m / 1000).toFixed(2) + ' km'); }
function two(n) { return (n < 10 ? '0' : '') + n; }

var lastSeq = {};

function row(r) {
  var e = el('div', 'rec new');

  var top = el('div', 'top');
  var t = new Date(r.t);
  top.appendChild(el('span', 'time', two(t.getHours()) + ':' + two(t.getMinutes()) + ':' + two(t.getSeconds())));
  top.appendChild(el('span', 'station', r.station));
  top.appendChild(el('span', 'variant', r.variant + (r.variantId !== undefined ? ' (' + r.variantId + ')' : '')));
  if (r.sequence !== undefined) top.appendChild(el('span', 'seq', '#' + r.sequence));

  // A skipped sequence number means packets that were sent were not heard — the measurement
  // that actually matters when siting, and one a signal strength reading alone will not show.
  // A duplicate is badged and leaves the gap tracking alone, as it does the table's counts.
  if (r.dup) top.appendChild(el('span', 'gap', 'dup'));
  else {
    var prev = lastSeq[r.station];
    if (prev !== undefined && r.sequence > prev + 1) top.appendChild(el('span', 'gap', '-' + (r.sequence - prev - 1)));
    if (r.sequence !== undefined) lastSeq[r.station] = r.sequence;
  }

  if (r.rssi !== undefined && r.rssi !== null) top.appendChild(el('span', 'rssi ' + rssiClass(r.rssi), r.rssi + ' dBm'));
  e.appendChild(top);

  if (r.site) {
    var s = el('div', 'site');
    if (r.site.name) s.appendChild(el('b', null, r.site.name));
    if (r.site.lat != null && r.site.lon != null) {
      s.appendChild(document.createTextNode(r.site.name ? ' \\u00b7 ' : ''));
      s.appendChild(el('code', null, r.site.lat.toFixed(6) + ', ' + r.site.lon.toFixed(6)));
    }
    if (r.site.distance != null) s.appendChild(document.createTextNode(' \\u00b7 ' + metres(r.site.distance)));
    e.appendChild(s);
  }

  var chips = el('div', 'chips');
  chipsFor(r.body, chips);
  if (chips.childNodes.length) e.appendChild(chips);

  // A plugin's output, when one claimed this packet: its label and value count, then the
  // values themselves. Deliberately plain — the point is to read the profile at a glance.
  if (r.decoded) {
    var d = el('div', 'dec');
    var head = el('div', 'dec-name');
    head.appendChild(el('b', null, r.decoded.name));
    var f = r.decoded.fields || {};
    for (var k in f) if (f.hasOwnProperty(k)) head.appendChild(document.createTextNode(' \u00b7 ' + f[k] + ' ' + k));
    d.appendChild(head);
    if (r.decoded.detail) d.appendChild(el('div', 'dec-vals', r.decoded.detail));
    e.appendChild(d);
  }
  return e;
}

// The reading column: value and unit run together, the way you would jot them down ("4.8°C,
// 1025hPa, 91%"), in the packet's own field order. Units come from UNITS where these do not say
// otherwise; a name with neither shows as "name value", and a plugin's summary, which arrives as
// text, shows as it is.
var COMPACT = {
  temperature: function (v) { return v.toFixed(1) + '\\u00b0C'; },
  humidity: function (v) { return Math.round(v) + '%'; },
  pressure: function (v) { return Math.round(v) + 'hPa'; },
  irradiance: function (v) { return Math.round(v) + 'W/m\\u00b2'; },
  ultraviolet: function (v) { return 'UV' + v; },
  direction: function (v) { return Math.round(v) + '\\u00b0'; },
  gust: function (v) { return 'gust ' + v + 'm/s'; },
  // DEPTH is centimetres in the protocol, whatever a variant calls it
  depth: function (v) { return v + 'cm'; },
  snow_depth: function (v) { return 'snow ' + v + 'cm'; },
  level: function (v) { return 'level ' + v + 'cm'; }
};
COMPACT.temp = COMPACT.water_temp = COMPACT.temperature;
COMPACT.humid = COMPACT.humidity;
COMPACT.pres = COMPACT.pressure;
COMPACT.wdir = COMPACT.direction;
COMPACT.wgust = COMPACT.gust;

function readingText(values) {
  var out = [];
  for (var i = 0; i < values.length; i++) {
    var k = values[i][0], v = values[i][1];
    if (typeof v !== 'number') out.push(String(v));
    else if (COMPACT[k]) out.push(COMPACT[k](v));
    else if (UNITS[k]) out.push(v + UNITS[k]);
    else out.push(k + ' ' + v);
  }
  return out.join(', ');
}

// What the reading column sorts on: temperature, the one thing nearly every station reports --
// the first temperature in it, or a plugin summary that leads with one ("22.1°C (...)").
function readingTemp(values) {
  for (var i = 0; values && i < values.length; i++) {
    var k = values[i][0], v = values[i][1];
    if (typeof v === 'number' && (k === 'temperature' || k === 'temp' || k === 'water_temp')) return v;
    if (typeof v === 'string' && /^-?[0-9.]+\\u00b0C/.test(v)) return parseFloat(v);
  }
  return null;
}

// The version report as one short cell: the software string, which is what changes between
// builds and says what the node is running ("relay/0.9.10/202610130940"). Firmware is the
// platform underneath (idf/6.1) and only stands in when a node sends no software field. The
// whole block -- hardware, serial, capabilities -- is in the tooltip.
function versionText(v) {
  if (!v) return '\\u2013';
  return v.software || v.firmware || v.FW || v.fw || '\\u2713';
}

// id, then name, then whatever else, on ONE line: the id stands out, the text after it is all dim.
function whoCell(id, name) {
  var who = el('td', 'who');
  who.appendChild(el('b', null, id));
  if (name) who.appendChild(el('span', 'nm', name));
  return who;
}

// The station table, a column at a time: its heading, what it sorts on, and its cell. A sort key of
// null -- nothing to sort on -- goes last whichever way the column runs, and ties fall back to
// station order.
var COLS = [
  { h: 'station', cls: 'id', key: function (s) { return parseInt(s.id, 16); },
    cell: function (s) { return el('td', 'id', s.id); } },
  { h: 'name', cls: 'nm', key: function (s) { return s.name ? s.name.toLowerCase() : null; },
    cell: function (s) {
      var td = el('td', 'nm');
      if (s.name) {
        var nm = el('span', null, s.name);
        nm.title = s.name; // a long one is cut short
        td.appendChild(nm);
      }
      return td;
    } },
  // The distance doubles as the way to go look at the spot. ?api=1&query= is the documented form
  // that hands off to the Maps app on a phone rather than the web page; rel=noopener because
  // target=_blank otherwise gives the opened page a handle back to this one.
  { h: 'dist', unit: 'm', cls: 'n', key: function (s) { return s.distance; },
    cell: function (s) {
      var td = el('td', 'n');
      if (s.distance == null) td.textContent = '\\u2013';
      else if (s.lat != null && s.lon != null) {
        var a = document.createElement('a');
        a.href = 'https://www.google.com/maps/search/?api=1&query=' + s.lat + ',' + s.lon;
        a.target = '_blank';
        a.rel = 'noopener noreferrer';
        a.textContent = String(Math.round(s.distance));
        td.appendChild(a);
      } else td.textContent = String(Math.round(s.distance));
      return td;
    } },
  { h: 'state', cls: 'state', key: function (s) { return s.state ? s.state.toLowerCase() : null; },
    cell: function (s) { return el('td', 'state', s.state || ''); } },
  // The last measurement and its age: a station whose recent packets carried only status keeps
  // showing what it last measured, and the age says how long ago that was.
  { h: 'reading', cls: 'rd', key: function (s) { return readingTemp(s.reading); },
    cell: function (s) {
      var td = el('td', 'rd', s.reading ? readingText(s.reading) : '\\u2013');
      if (s.reading) td.appendChild(el('span', null, ' (' + dur(s.readingAge) + ')'));
      return td;
    } },
  { h: 'pkts', cls: 'n', key: function (s) { return s.count; },
    cell: function (s) { return el('td', 'n', String(s.count)); } },
  { h: 'every', cls: 'n', key: function (s) { return s.interval; },
    cell: function (s) { return el('td', 'n', s.interval == null ? '\\u2013' : every(s.interval)); } },
  { h: 'lost', cls: 'n', key: function (s) { return s.lossPct; },
    cell: function (s) { return el('td', 'n', s.lost + (s.lost ? ' (' + s.lossPct.toFixed(1) + '%)' : '')); } },
  // ddup should keep this at zero; anything else is the same packet delivered twice.
  { h: 'dups', cls: 'n', key: function (s) { return s.dups || 0; },
    cell: function (s) { return el('td', s.dups ? 'n stale' : 'n', String(s.dups || 0)); } },
  { h: 'avg rssi', cls: 'n', key: function (s) { return s.rssiAvg; },
    cell: function (s) { return el('td', 'n', s.rssiAvg == null ? '\\u2013' : Math.round(s.rssiAvg) + ' dBm'); } },
  // Latest battery reading; red once it is older than the configured staleness, since a level
  // that has stopped being reported is itself the warning.
  { h: 'batt', cls: 'n', key: function (s) { return s.battery; },
    cell: function (s) {
      var td = el('td', s.batteryStale ? 'n stale' : 'n', s.battery == null ? '\\u2013' : s.battery + '%' + (s.charging ? ' \\u26a1' : ''));
      if (s.battery != null) td.title = 'reported ' + every(s.batteryAge) + ' ago';
      return td;
    } },
  { h: 'software', cls: 'fw', key: function (s) { return s.version ? versionText(s.version) : null; },
    cell: function (s) {
      var td = el('td', 'fw', versionText(s.version));
      if (s.version) td.title = JSON.stringify(s.version) + ' -- reported ' + every(s.versionAge) + ' ago';
      return td;
    } },
  // Time since its version report, which a sensor sends only on a cold boot: so, time since boot.
  // Not the status TLV's uptime, which on a sleeping node is just the seconds awake this cycle.
  { h: 'up', key: function (s) { return s.versionAge; },
    cell: function (s) { return el('td', null, dur(s.versionAge)); } },
  // Age is the honest answer to "is it still there" — a gap only shows when a LATER packet
  // arrives to reveal it, so a station that goes silent shows up here and nowhere else.
  { h: 'last', key: function (s) { return s.age; },
    cell: function (s) {
      return el('td', s.age != null && s.age > 300 ? 'stale' : null, s.age == null ? '\\u2013' : s.age < 90 ? s.age + 's' : Math.round(s.age / 60) + 'm');
    } }
];

// Click a heading to sort on it, again to reverse it; station order until then.
var sortCol = 0, sortDir = 1, lastRows = [];

function sortKey(col, s) {
  var k = col.key(s);
  return k == null || k !== k ? null : k; // NaN is nothing to sort on, too
}

function renderStats(rows) {
  lastRows = rows;
  var col = COLS[sortCol];
  var byId = function (a, b) { return parseInt(a.id, 16) - parseInt(b.id, 16); };
  rows = rows.slice().sort(function (a, b) {
    var x = sortKey(col, a), y = sortKey(col, b);
    if (x === null || y === null) return x === y ? byId(a, b) : x === null ? 1 : -1;
    var c = typeof x === 'number' && typeof y === 'number' ? x - y : String(x).localeCompare(String(y));
    return c ? c * sortDir : byId(a, b);
  });

  var tb = document.querySelector('#sum tbody');
  tb.innerHTML = '';
  var hr = document.createElement('tr');
  for (var i = 0; i < COLS.length; i++) {
    var th = el('th', COLS[i].cls || null, COLS[i].h);
    if (COLS[i].unit) th.appendChild(el('span', 'u', ' (' + COLS[i].unit + ')'));
    if (i === sortCol) th.appendChild(document.createTextNode(sortDir > 0 ? ' \\u25b2' : ' \\u25bc'));
    th.onclick = (function (n) {
      return function () {
        sortDir = n === sortCol ? -sortDir : 1;
        sortCol = n;
        renderStats(lastRows);
      };
    })(i);
    hr.appendChild(th);
  }
  tb.appendChild(hr);

  for (var n = 0; n < rows.length; n++) {
    var tr = document.createElement('tr');
    for (var c = 0; c < COLS.length; c++) tr.appendChild(COLS[c].cell(rows[n]));
    tb.appendChild(tr);
  }
}

// Tracker positions. The button in the title bar carries the count and nothing else until it
// is pressed: for most of this page's life there is no survey running, and a permanently empty
// panel above the station table would cost the one thing this layout is protecting — the top of
// the screen, which is what you read standing at the base of a mast.
var gbtn = document.getElementById('gbtn');
var gpanel = document.getElementById('gnss');
var GHEAD = ['fix', 'position', 'dist', 'batt', 'rssi', 'snr', 'event'];

gbtn.addEventListener('click', function () {
  var open = gpanel.hidden;
  gpanel.hidden = !open;
  gbtn.setAttribute('aria-expanded', open ? 'true' : 'false');
});

// A fix from today needs only a clock; one from an earlier day needs the date, because a survey
// runs over several and "14:32" alone would silently merge them.
function gtime(ms) {
  var d = new Date(ms), n = new Date();
  var hm = two(d.getHours()) + ':' + two(d.getMinutes()) + ':' + two(d.getSeconds());
  return d.toDateString() === n.toDateString() ? hm : two(d.getMonth() + 1) + '-' + two(d.getDate()) + ' ' + hm;
}

function renderGnss(view) {
  var rows = view.rows || [];
  gbtn.hidden = view.n === 0;
  gbtn.textContent = '[#' + view.n + ']';
  if (!view.n) gpanel.hidden = true;
  var tb = gpanel.querySelector('tbody');
  tb.innerHTML = '';
  var hr = document.createElement('tr');
  for (var i = 0; i < GHEAD.length; i++) hr.appendChild(el('th', null, GHEAD[i]));
  tb.appendChild(hr);

  // Say so when the panel is showing a window onto a longer list, rather than letting the
  // badge and the row count silently disagree.
  document.getElementById('glbl').textContent =
    view.n > rows.length ? 'tracker · latest ' + rows.length + ' of ' + view.n : 'tracker';

  for (var n = rows.length - 1; n >= 0; n--) {
    var g = rows[n], tr = document.createElement('tr');
    tr.appendChild(el('td', null, gtime(g.t)));

    var pos = el('td');
    var a = document.createElement('a');
    a.href = 'https://www.google.com/maps/search/?api=1&query=' + g.lat + ',' + g.lon;
    a.target = '_blank';
    a.rel = 'noopener noreferrer';
    a.textContent = g.lat.toFixed(5) + ', ' + g.lon.toFixed(5);
    pos.appendChild(a);
    tr.appendChild(pos);

    tr.appendChild(el('td', null, g.distance == null ? '\u2013' : metres(g.distance)));
    tr.appendChild(el('td', null, g.battery == null ? '\u2013' : g.battery + '%'));

    // Strongest reception of this fix, with a count when the tracker repeated it — a confirmed
    // uplink is sent again until acknowledged, so two or three copies of one press is normal.
    var sig = el('td', null, g.rssi == null ? '\u2013' : g.rssi + ' dBm');
    if (g.rssi != null) sig.className = rssiClass(g.rssi);
    if (g.rx > 1) sig.appendChild(el('span', 'rx', '\u00d7' + g.rx));
    tr.appendChild(sig);

    tr.appendChild(el('td', null, g.snr == null ? '\u2013' : g.snr.toFixed(1)));
    tr.appendChild(el('td', 'ev', g.event || '\u2013'));
    tb.appendChild(tr);
  }
}

// Gateways and relays, from the gateways' own stats rather than from station packets. Hidden
// until a gateway has reported, so a site with none loses no screen. Relays sit under the gateway
// that reported them, indented; one the gateway says is not accepting is called out, since that
// is the relay that will quietly stop carrying traffic.
var gwp = document.getElementById('gw');
var WHEAD = ['network', 'role', 'rssi', 'rx', 'tx/fwd', 'software', 'up', 'last'];

function dur(t) {
  if (t == null) return '\u2013';
  t = Math.round(t);
  return t < 90 ? t + 's' : t < 5400 ? Math.round(t / 60) + 'm' : t < 172800 ? Math.round(t / 3600) + 'h' : Math.round(t / 86400) + 'd';
}

function linkText(l) {
  var p = [];
  if (l.channel != null) p.push('ch' + l.channel);
  if (l.mhz != null) p.push(l.mhz.toFixed(3) + ' MHz');
  if (l.dbm != null) p.push(l.dbm + ' dBm');
  if (l.bps != null) p.push(l.bps + ' bps');
  return p.length ? p.join(' ') : l.name;
}

function renderGateways(rows) {
  gwp.hidden = !rows.length;
  var tb = gwp.querySelector('tbody');
  tb.innerHTML = '';
  var hr = document.createElement('tr');
  for (var i = 0; i < WHEAD.length; i++) hr.appendChild(el('th', null, WHEAD[i]));
  tb.appendChild(hr);

  for (var n = 0; n < rows.length; n++) {
    var r = rows[n], tr = document.createElement('tr'), sub = [], rssi, rx, tx;
    tr.className = r.role;
    if (r.role === 'gateway') {
      var links = r.links || [], rxn = 0, txn = 0, bad = 0;
      for (var k = 0; k < links.length; k++) {
        sub.push(linkText(links[k]));
        rxn += links[k].rx || 0; txn += links[k].tx || 0; bad += links[k].rxBad + links[k].txBad;
      }
      if (r.addr) sub.push(r.addr);
      rssi = links.length ? links[0].rssi : null;
      rx = el('td', null, links.length ? String(rxn) : '\u2013');
      if (bad) { rx.appendChild(document.createTextNode(' \u00b7 ' + bad + ' err')); rx.className = 'weak'; }
      tx = el('td', null, links.length ? String(txn) : '\u2013');
    } else if (r.role === 'survey') {
      if (r.lat != null) sub.push(r.lat.toFixed(5) + ', ' + r.lon.toFixed(5));
      rssi = null;
      rx = el('td', null, r.observations == null ? '–' : String(r.observations));
      tx = el('td', null, '–');
    } else {
      if (r.cost != null) sub.push('cost ' + r.cost);
      if (r.hears != null) sub.push('hears ' + r.hears);
      if (r.beacons != null) sub.push(r.beacons + ' beacons');
      rssi = r.rssi;
      rx = el('td', null, r.packets == null ? '\u2013' : String(r.packets));
      tx = el('td', null, r.forwards == null ? '\u2013' : String(r.forwards));
    }

    var who = whoCell(r.id, r.name);
    if (sub.length) who.appendChild(el('span', null, ' \u00b7 ' + sub.join(' \u00b7 ')));
    tr.appendChild(who);

    var role = el('td', 'role', r.role);
    if (r.role === 'relay' && r.accepting === false) { role.textContent = 'relay \u00b7 not accepting'; role.className = 'role weak'; }
    tr.appendChild(role);

    var sig = el('td', null, rssi == null ? '\u2013' : rssi + ' dBm');
    if (rssi != null) sig.className = rssiClass(rssi);
    tr.appendChild(sig);
    tr.appendChild(rx);
    tr.appendChild(tx);
    tr.appendChild(el('td', 'fw', r.software || (r.version ? 'v' + r.version : '\u2013')));
    // a gateway states its uptime; for a relay, time since its version report, which it sends on boot
    tr.appendChild(el('td', null, r.role === 'gateway' ? dur(r.uptime) : dur(r.versionAge)));
    var age = el('td', null, dur(r.age));
    if (r.age != null && r.age > (r.staleAfter || 300)) age.className = 'stale';
    tr.appendChild(age);
    tb.appendChild(tr);
  }
}

var list = document.getElementById('list');
var dot = document.getElementById('dot'), msg = document.getElementById('msg');
var count = 0;

function add(r) {
  if (count === 0) list.innerHTML = '';
  list.insertBefore(row(r), list.firstChild);
  count++;
  while (list.childNodes.length > ${KEEP}) list.removeChild(list.lastChild);
  msg.textContent = count;
}

var es = new EventSource('/events');
es.onopen = function () { dot.className = 'live'; msg.textContent = count ? count : 'listening'; };
es.onerror = function () { dot.className = ''; msg.textContent = 'reconnecting'; };
es.addEventListener('packet', function (ev) { add(JSON.parse(ev.data)); });
es.addEventListener('stats', function (ev) { renderStats(JSON.parse(ev.data)); });
es.addEventListener('gnss', function (ev) { renderGnss(JSON.parse(ev.data)); });
es.addEventListener('gateways', function (ev) { renderGateways(JSON.parse(ev.data)); });
</script>
</body>
</html>`;

// A second page rather than a panel on the first: the map is heavy, it is opened deliberately,
// and the monitor page stays as light as it was. It carries no data of its own — it subscribes
// to the same /events stream and plots what the stats and gnss views already contain, so there
// is one source of truth and nothing to keep in step. Per-hop mesh routing is the obvious thing
// to draw here later; the layer split below leaves room for it.
const MAP_PAGE = `<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>iotdata map</title>
${FAVICON}
<style>
  html, body { margin:0; height:100%; background:#14171a;
               font:13px/1.45 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif; }
  #map { position:absolute; inset:0; }
  #panel { position:absolute; z-index:5; top:10px; left:10px; width:min(58vw,290px);
           max-height:calc(100% - 20px); overflow:auto; background:rgba(255,255,255,.93);
           color:#14171a; border-radius:8px; padding:8px 10px;
           box-shadow:0 1px 5px rgba(0,0,0,.35); }
  #panel h1 { margin:0 0 5px; font-size:13px; font-weight:650;
              display:flex; gap:8px; align-items:baseline; }
  #panel h1 a { margin-left:auto; color:#0d9488; text-decoration:none;
                font-size:11.5px; font-weight:600; }
  .k { display:flex; align-items:center; gap:6px; margin:1px 0; font-size:12px; }
  .k i { width:9px; height:9px; border-radius:50%; flex:none; box-shadow:0 0 0 1.5px #fff; }
  .k b { font-family:ui-monospace,SFMono-Regular,Menlo,monospace; font-weight:650; }
  .k span { color:#6b7280; overflow:hidden; text-overflow:ellipsis; white-space:nowrap; }
  .muted { color:#6b7280; font-size:11.5px; margin-top:6px; }
  /* Marker labels sit directly on satellite imagery, which is arbitrarily light or dark. */
  .mlbl { text-shadow:0 0 3px rgba(0,0,0,.95), 0 0 7px rgba(0,0,0,.85); }
  .iw { font:12px/1.5 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;
        color:#14171a; min-width:150px; }
  .iw b { font-family:ui-monospace,SFMono-Regular,Menlo,monospace; }
  #err { position:absolute; inset:0; z-index:6; display:none; align-items:center;
         justify-content:center; padding:2em; text-align:center; color:#f87171; }
</style>
</head>
<body>
<div id="map"></div>
<div id="err"></div>
<div id="panel">
  <h1>iotdata gateway<a href="/">monitor &rarr;</a></h1>
  <div id="legend"></div>
  <div id="note" class="muted">waiting for the monitor stream</div>
</div>
<script>
var REF = ${REF ? `{ lat: ${REF.lat}, lng: ${REF.lon} }` : 'null'};
var MAPTYPE = '${MAP_TYPE}';
var ZOOM = ${MAP_ZOOM};

var map = null, live = false, info = null, pinned = false, fitted = false;
var gwm = null, sm = {}, tk = [], svm = [];
var data = { stats: [], gnss: { n: 0, rows: [] }, net: [] };

function esc(s) {
  return String(s == null ? '' : s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
}
function every(t) {
  t = Math.round(t);
  if (t < 60) return t + 's';
  if (t < 3600) return Math.floor(t / 60) + 'm' + (t % 60 ? String(t % 60).padStart(2, '0') + 's' : '');
  return Math.floor(t / 3600) + 'h' + String(Math.floor((t % 3600) / 60)).padStart(2, '0') + 'm';
}
function metres(m) {
  return m == null ? '-' : m < 1000 ? Math.round(m) + ' m' : (m / 1000).toFixed(2) + ' km';
}
function ago(a) {
  return a == null ? 'never' : a < 90 ? a + 's' : a < 5400 ? Math.round(a / 60) + 'm' : Math.round(a / 3600) + 'h';
}
// The same thresholds the station table uses for its stale styling, so the two pages agree.
function colour(a) {
  if (a == null) return '#6b7280';
  if (a <= 300) return '#0a7d36';
  if (a <= 3600) return '#a86400';
  return '#c02a2a';
}
// labelOrigin is expressed in the symbol's own units, before scale is applied, so one constant
// keeps the text just clear of the circle at whatever size it is drawn.
function sym(fill, scale, ring) {
  return { path: google.maps.SymbolPath.CIRCLE, scale: scale, fillColor: fill, fillOpacity: 1,
           strokeColor: '#fff', strokeWeight: ring, labelOrigin: new google.maps.Point(0, 2.5) };
}
function tag(text) {
  return { text: text, className: 'mlbl', color: '#fff', fontSize: '11px', fontWeight: '600' };
}
function fail(msg) {
  var e = document.getElementById('err');
  e.textContent = msg;
  e.style.display = 'flex';
}

// Hover to read a station, click to pin it open. Hover alone would be useless on the phone that
// actually gets carried around the site, so both gestures do the same thing and a pinned window
// simply survives the mouse leaving.
function show(marker, html, click) {
  if (pinned && !click) return;
  if (!info) info = new google.maps.InfoWindow();
  if (click) pinned = true;
  info.setContent(html);
  info.open({ anchor: marker, map: map });
}
function hide(force) {
  if (info && (!pinned || force)) info.close();
  if (force) pinned = false;
}
function bind(marker, body) {
  marker.addListener('mouseover', function () { show(marker, body(), false); });
  marker.addListener('mouseout', function () { hide(false); });
  marker.addListener('click', function () { pinned = false; show(marker, body(), true); });
}

function stationInfo(s) {
  var h = '<div class="iw"><b>' + esc(s.id) + '</b>';
  if (s.name) h += ' &middot; ' + esc(s.name);
  h += '<br>' + s.count + ' pkts';
  if (s.interval != null) h += ' &middot; every ' + every(s.interval);
  if (s.rssiAvg != null) h += '<br>' + Math.round(s.rssiAvg) + ' dBm avg';
  if (s.lost) h += '<br>' + s.lost + ' lost (' + s.lossPct.toFixed(1) + '%)';
  if (s.distance != null) h += '<br>' + metres(s.distance) + ' from gateway';
  return h + '<br>last seen ' + ago(s.age) + ' ago</div>';
}
function fixInfo(g) {
  var h = '<div class="iw"><b>' + esc(g.device) + '</b><br>' + new Date(g.t).toLocaleString();
  h += '<br>' + g.lat.toFixed(6) + ', ' + g.lon.toFixed(6);
  if (g.distance != null) h += '<br>' + metres(g.distance) + ' from gateway';
  if (g.battery != null) h += '<br>battery ' + g.battery + '%';
  if (g.rssi != null) h += '<br>' + g.rssi + ' dBm' + (g.snr != null ? ' / ' + g.snr + ' dB' : '');
  if (g.sf != null) h += ' &middot; SF' + g.sf;
  if (g.rx > 1) h += '<br>' + g.rx + ' receptions';
  if (g.event) h += '<br>' + esc(g.event);
  return h + '</div>';
}

function surveyInfo(r) {
  var h = '<div class="iw"><b>' + esc(r.id) + '</b>' + (r.name ? ' &middot; ' + esc(r.name) : '') + ' &middot; survey';
  h += '<br>' + r.lat.toFixed(6) + ', ' + r.lon.toFixed(6) + (r.alt != null ? ' &middot; ' + r.alt + ' m' : '');
  h += '<br>last report ' + ago(r.age) + ' ago';
  var heard = r.heard || [];
  if (heard.length) {
    h += '<br>heard ' + heard.length + ':';
    for (var i = 0; i < heard.length; i++)
      h += '<br>&nbsp;&nbsp;<b>' + esc(heard[i].id) + '</b> ' + (heard[i].rssi != null ? heard[i].rssi + ' dBm' : '') +
           (heard[i].name ? ' &middot; ' + esc(heard[i].name) : '');
  }
  return h + '</div>';
}

// The latest of each, keyed by whatever names the sender: a fix per tracker device, a stand per
// survey instrument. The trail of every fix is gone -- what matters is where each one is now.
function latestPer(rows, key) {
  var out = {}, i;
  for (i = 0; i < rows.length; i++) if (rows[i].lat != null && rows[i].lon != null) out[rows[i][key]] = rows[i];
  return out;
}

function draw() {
  if (!live) return;
  var site = [], i, s;

  if (REF && !gwm) {
    gwm = new google.maps.Marker({ position: REF, map: map, zIndex: 40, title: 'gateway',
                                   icon: sym('#0d9488', 8, 3), label: tag('gateway') });
    bind(gwm, function () {
      return '<div class="iw"><b>gateway</b><br>' + REF.lat.toFixed(6) + ', ' + REF.lng.toFixed(6) + '</div>';
    });
  }
  if (REF) site.push(REF);

  var seen = {};
  for (i = 0; i < data.stats.length; i++) {
    s = data.stats[i];
    if (s.lat == null || s.lon == null) continue;
    seen[s.id] = 1;
    var p = { lat: s.lat, lng: s.lon };
    site.push(p);
    if (sm[s.id]) {
      sm[s.id].row = s;
      sm[s.id].setIcon(sym(colour(s.age), 7, 2));
      sm[s.id].setPosition(p);
    } else {
      var m = new google.maps.Marker({ position: p, map: map, zIndex: 30, title: s.id,
                                       icon: sym(colour(s.age), 7, 2), label: tag(s.name || s.id) });
      m.row = s;
      bind(m, (function (mm) { return function () { return stationInfo(mm.row); }; })(m));
      sm[s.id] = m;
    }
  }
  for (var id in sm) if (sm.hasOwnProperty(id) && !seen[id]) { sm[id].setMap(null); delete sm[id]; }

  // Trackers and survey instruments, one marker each at its latest position. Redrawn wholesale:
  // there are only ever a handful.
  for (i = 0; i < tk.length; i++) tk[i].setMap(null);
  for (i = 0; i < svm.length; i++) svm[i].setMap(null);
  tk = []; svm = [];
  var path = [], fixes = latestPer(data.gnss.rows || [], 'device'), dev;
  for (dev in fixes) if (fixes.hasOwnProperty(dev)) {
    var g = fixes[dev], q = { lat: g.lat, lng: g.lon };
    path.push(q);
    var t = new google.maps.Marker({ position: q, map: map, zIndex: 35, title: g.device,
                                     icon: sym('#0d9488', 6, 2), label: tag(g.device || 'tracker') });
    bind(t, (function (gg) { return function () { return fixInfo(gg); }; })(g));
    tk.push(t);
  }
  var stands = latestPer((data.net || []).filter(function (r) { return r.role === 'survey'; }), 'id'), sid;
  for (sid in stands) if (stands.hasOwnProperty(sid)) {
    var r = stands[sid], sp = { lat: r.lat, lng: r.lon };
    path.push(sp);
    var v = new google.maps.Marker({ position: sp, map: map, zIndex: 36, title: r.id,
                                     icon: sym('#7c3aed', 7, 2), label: tag(r.name || r.id) });
    bind(v, (function (rr) { return function () { return surveyInfo(rr); }; })(r));
    svm.push(v);
  }

  // Fitted to the gateway and the stations, not to the track. The tracker can legitimately be a
  // long way from the site — it is in the lab while the gateway is on site — and letting it into
  // the bounds would zoom out to include both and show neither usefully. On site the two coincide
  // and this makes no difference. Only the track case falls back to fitting the fixes.
  if (!fitted) {
    var pts = site.length ? site : path;
    if (pts.length === 1) { map.setCenter(pts[0]); map.setZoom(ZOOM); fitted = true; }
    else if (pts.length > 1) {
      var b = new google.maps.LatLngBounds();
      for (i = 0; i < pts.length; i++) b.extend(pts[i]);
      map.fitBounds(b, 60);
      fitted = true;
    }
  }
}

function panel() {
  var html = '', missing = [], mapped = 0, i;
  if (REF) html += '<div class="k"><i style="background:#0d9488"></i><b>gateway</b></div>';
  for (i = 0; i < data.stats.length; i++) {
    var s = data.stats[i];
    if (s.lat == null || s.lon == null) { missing.push(s.id); continue; }
    mapped++;
    html += '<div class="k"><i style="background:' + colour(s.age) + '"></i><b>' + esc(s.id) +
            '</b> <span>' + (s.name ? esc(s.name) + ' &middot; ' : '') + ago(s.age) + '</span></div>';
  }

  var surveys = (data.net || []).filter(function (r) { return r.role === 'survey' && r.lat != null; });
  for (i = 0; i < surveys.length; i++)
    html += '<div class="k"><i style="background:#7c3aed"></i><b>' + esc(surveys[i].id) + '</b> <span>' +
            (surveys[i].name ? esc(surveys[i].name) + ' &middot; ' : '') + 'survey &middot; ' + ago(surveys[i].age) + '</span></div>';
  var trackers = Object.keys(latestPer(data.gnss.rows || [], 'device'));
  for (i = 0; i < trackers.length; i++)
    html += '<div class="k"><i style="background:#0d9488"></i><b>' + esc(trackers[i]) + '</b> <span>tracker &middot; latest fix</span></div>';
  document.getElementById('legend').innerHTML = html;

  var txt = mapped + ' station' + (mapped === 1 ? '' : 's') + ' mapped';
  if (trackers.length) txt += ', ' + trackers.length + ' tracker' + (trackers.length === 1 ? '' : 's');
  if (surveys.length) txt += ', ' + surveys.length + ' survey instrument' + (surveys.length === 1 ? '' : 's');
  if (missing.length) txt += '. No position configured for ' + esc(missing.join(', ')) + '.';
  document.getElementById('note').textContent = txt;
}

function start() {
  map = new google.maps.Map(document.getElementById('map'), {
    center: REF || { lat: 0, lng: 0 }, zoom: REF ? ZOOM : 2, mapTypeId: MAPTYPE,
    streetViewControl: false, tilt: 0
  });
  map.addListener('click', function () { hide(true); });
  live = true;
  draw();
}
window.start = start;
// Google reports a rejected or misrestricted key out of band rather than as a script error.
window.gm_authFailure = function () {
  fail('Google rejected the Maps API key: check it has the Maps JavaScript API enabled and that this origin passes its referrer restrictions.');
};
setTimeout(function () {
  if (!live) fail('Google Maps did not load. Check this machine can reach maps.googleapis.com.');
}, 15000);

// view=map suppresses the packet backlog the monitor page wants on connect: this page plots
// stats and positions, and has no use for five hundred individual records over cellular.
var es = new EventSource('/events?view=map');
es.addEventListener('stats', function (ev) { data.stats = JSON.parse(ev.data); panel(); draw(); });
es.addEventListener('gnss', function (ev) { data.gnss = JSON.parse(ev.data); panel(); draw(); });
es.addEventListener('gateways', function (ev) { data.net = JSON.parse(ev.data); panel(); draw(); });
</script>
<script async src="https://maps.googleapis.com/maps/api/js?key=${MAP_KEY}&loading=async&callback=start"></script>
</body>
</html>`;

// ---------------------------------------------------------------------------- server

const server = http.createServer((req, res) => {
    const url = req.url.split('?')[0];

    if (url === '/' || url === '/index.html') {
        res.writeHead(200, {
            'Content-Type': 'text/html; charset=utf-8',
            'Cache-Control': 'no-store',
        });
        res.end(PAGE);
        return;
    }

    if (url === '/events') {
        res.writeHead(200, {
            'Content-Type': 'text/event-stream',
            'Cache-Control': 'no-store',
            'Connection': 'keep-alive',
            'X-Accel-Buffering': 'no',
        });
        // Replay what we have so a phone arriving late, or reconnecting after the screen slept,
        // sees context rather than an empty page until the next packet lands.
        const q = new URLSearchParams(req.url.split('?')[1] || '');
        if (q.get('view') !== 'map') for (const rec of history) send('packet', rec, res);
        send('stats', snapshot(), res);
        send('gnss', gnssView(), res);
        send('gateways', gatewayView(), res);
        clients.add(res);
        // Phones aggressively idle-close background connections; a periodic comment keeps the
        // stream from being reaped during the quiet gaps between packets.
        const ka = setInterval(() => {
            try {
                res.write(': keepalive\n\n');
            } catch {
                /* closing */
            }
        }, 20000);
        req.on('close', () => {
            clearInterval(ka);
            clients.delete(res);
        });
        return;
    }

    if (url === '/map') {
        if (!MAP_KEY) {
            res.writeHead(503, { 'Content-Type': 'text/plain' });
            res.end('map not configured: set map-api-key in the config\n');
            return;
        }
        res.writeHead(200, {
            'Content-Type': 'text/html; charset=utf-8',
            'Cache-Control': 'no-store',
        });
        res.end(MAP_PAGE);
        return;
    }

    res.writeHead(404, { 'Content-Type': 'text/plain' });
    res.end('not found\n');
});

server.listen(PORT, BIND, () => {
    const known = Object.keys(STATIONS);
    console.log(`iotdata monitor: http://${os.hostname()}.local:${PORT}/`);
    console.log(`  config  ${cfgFrom}`);
    console.log(`  broker  ${BROKER}:${MQTT_PORT}  topic ${TOPIC}  keep ${KEEP}  window ${WINDOW}`);
    console.log(`  ref     ${REF ? `${REF.lat}, ${REF.lon}${REF.alt != null ? ` (${REF.alt} m)` : ''}` : '(not configured: no distances)'}`);
    // The key is a public one by nature, but a log is a different audience from a page:
    // enough to tell two hosts' keys apart, not enough to use.
    console.log(
        `  map     ${MAP_KEY ? `/map (${MAP_TYPE}, key ${MAP_KEY.slice(0, 6)}\u2026${MAP_KEY.slice(-4)})` : '(not configured: no map-api-key)'}`
    );
    for (const id of known) {
        const s = STATIONS[id];
        console.log(`  station ${id}  ${s.name || '(unnamed)'}${s.distance != null ? `  ${Math.round(s.distance)} m` : ''}${s.state ? `  (${s.state})` : ''}`);
    }
    if (!known.length) console.log('  station (none configured: packets still shown, without names or distances)');
    for (const d of Object.values(DECODERS)) console.log(`  decoder ${d.name}  ${d.file}`);
    if (LOG_FILE) {
        // Fail loudly at startup rather than once every flush interval for the next two months.
        try {
            fs.appendFileSync(LOG_FILE, '');
            console.log(`  log     ${LOG_FILE}  (every packet, flushed every ${LOG_FLUSH}s)`);
        } catch (e) {
            console.error(`  log     ${LOG_FILE}  NOT WRITABLE: ${e.message}`);
        }
    } else console.log('  log     (not configured: set log-file to record packets)');

    if (TRACKER) {
        console.log(`  tracker ${TRACKER.type}  ${TRACKER.host}:${TRACKER.port}  topic ${TRACKER.topic}  ${TRACKER.tls ? `tls${TRACKER.cafile ? ` (${TRACKER.cafile})` : ''}` : 'NO TLS'}`);
        if (!TRACKER_TYPES[TRACKER.type]) console.error(`  tracker UNKNOWN TYPE '${TRACKER.type}': positions will be ignored (known: ${Object.keys(TRACKER_TYPES).join(', ')})`);
        if (TRACKER_LOG) {
            // Same reasoning as the packet log: fail at startup, not once a minute for a month.
            try {
                fs.appendFileSync(TRACKER_LOG, '');
                console.log(`  tracker log ${TRACKER_LOG}  (every position, flushed every ${TRACKER_LOG_FLUSH}s)`);
            } catch (e) {
                console.error(`  tracker log ${TRACKER_LOG}  NOT WRITABLE: ${e.message}`);
            }
        } else console.log('  tracker log (not configured: set tracker-log-file to record positions)');
    } else console.log('  tracker (not configured: set tracker-host to follow a gnss tracker)');

    console.log(`  state   ${SNAPSHOT_FILE}, saved every ${SNAPSHOT_INTERVAL}s`);

    // State first, subscribers second: rebuilding from a log that is being appended to at the
    // same time would double-count everything that arrived during the rebuild.
    restore((how) => {
        console.log(`  restore ${how}`);
        subscribe();
    });
});
