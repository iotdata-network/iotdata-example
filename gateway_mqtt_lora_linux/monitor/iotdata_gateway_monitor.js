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
for (const key of Object.keys(cfg)) {
    const m = /^station-([0-9A-Fa-f]{1,8})-(name|latitude|longitude|altitude|decoder)$/.exec(key);
    if (!m) continue;
    const id = m[1].toUpperCase();
    STATIONS[id] = STATIONS[id] || {};
    STATIONS[id][m[2]] = m[2] === 'name' || m[2] === 'decoder' ? cfg[key] : num(cfg[key], null);
}

// Decoder plugins: decoder-<name>=<path to module>. A plugin exports decodeMonitor(body)
// and returns display fields, or null when the packet is not its own.
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
            prevSeq: undefined,
            times: [],
            rssis: [],
        });
    return stats.get(id);
}

function observe(rec) {
    const s = statFor(rec.station);
    s.count++;
    s.last = rec.t;
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
}

// Counts are for the whole session; rate and average RSSI are over the last WINDOW packets, so
// they track what is happening NOW rather than being dragged back by an hour of earlier data —
// which is the point when you are walking a station around the site.
function snapshot() {
    const now = Date.now();
    return [...stats.values()]
        .map((s) => {
            const meta = STATIONS[s.id] || {};
            const n = s.times.length;
            const mins = n > 1 ? (s.times[n - 1] - s.times[0]) / 60000 : 0;
            const expected = s.count + s.lost;
            return {
                id: s.id,
                name: meta.name || null,
                lat: meta.latitude ?? null,
                lon: meta.longitude ?? null,
                distance: meta.distance ?? null,
                count: s.count,
                lost: s.lost,
                lossPct: expected > 0 ? (s.lost * 100) / expected : 0,
                rate: mins > 0 ? (n - 1) / mins : null,
                rssiAvg: s.rssis.length ? s.rssis.reduce((a, b) => a + b, 0) / s.rssis.length : null,
                age: s.last ? Math.round((now - s.last) / 1000) : null,
            };
        })
        .sort((a, b) => a.id.localeCompare(b.id));
}

function record(rec) {
    history.push(rec);
    while (history.length > KEEP) history.shift();
    observe(rec);
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
    if (!body || typeof body !== 'object' || body.station === undefined) return;

    // Topic is <prefix>/<variant-name>/<station-hex>; the variant NAME lives only here (the
    // payload carries the numeric id), so it is worth taking rather than maintaining a table.
    const parts = topic.split('/');
    const station = parts.length >= 1 ? parts[parts.length - 1] : '?';
    const meta = STATIONS[station.toUpperCase()] || null;

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
    for (const st of data.stats) if (st && st.id) stats.set(st.id, { count: 0, lost: 0, times: [], rssis: [], ...st });
    history.length = 0;
    for (const rec of Array.isArray(data.history) ? data.history.slice(-KEEP) : []) history.push(rec);
    gnss.length = 0;
    for (const g of Array.isArray(data.gnss) ? data.gnss.slice(-TRACKER_KEEP) : []) gnss.push(g);
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
setInterval(() => send('stats', snapshot()), 5000);

// The gateway's own position in the title bar, linked the same way the station distances
// are. Empty when no reference is configured, which also disables distances.
const REF_LINK = REF ? `<a class="ref" href="https://www.google.com/maps/search/?api=1&amp;query=${REF.lat},${REF.lon}" target="_blank" rel="noopener noreferrer">${REF.lat.toFixed(6)}, ${REF.lon.toFixed(6)}</a>` : '';

// ---------------------------------------------------------------------------- page

const PAGE = `<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>iotdata monitor</title>
<style>
  :root {
    --bg:#f6f7f9; --fg:#14171a; --dim:#6b7280; --line:#dfe3e8; --card:#fff;
    --good:#0a7d36; --ok:#a86400; --weak:#c02a2a;
  }
  @media (prefers-color-scheme: dark) {
    :root { --bg:#101215; --fg:#e6e8ea; --dim:#9aa3ad; --line:#262b31; --card:#181b1f;
            --good:#4ade80; --ok:#fbbf24; --weak:#f87171; }
  }
  * { box-sizing:border-box; }
  body { margin:0; background:var(--bg); color:var(--fg);
         font:15px/1.4 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;
         padding-bottom:env(safe-area-inset-bottom); }
  header { position:sticky; top:0; z-index:2; background:var(--card);
           border-bottom:1px solid var(--line); }
  .bar { padding:8px 12px; display:flex; flex-wrap:wrap; align-items:center; gap:4px 10px; }
  .bar b { font-size:15px; font-weight:600; }
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
  td.who span { color:var(--dim); display:block; font-size:11.5px; }
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
  .empty { color:var(--dim); padding:24px 12px; text-align:center; }
  .note { color:var(--dim); font-size:10.5px; padding:3px 8px 6px; }
</style>
</head>
<body>
<header>
  <div class="bar"><b>iotdata gateway</b>${REF_LINK}<button id="gbtn" class="gbtn" hidden aria-expanded="false"></button><span id="stat"><span id="dot"></span><span id="msg">connecting</span></span></div>
  <div id="gnss" hidden><div class="lbl" id="glbl">tracker positions · newest first</div><table><tbody></tbody></table></div>
  <table id="sum"><tbody></tbody></table>
  <div class="note">counts are cumulative; rate and avg rssi over the last ${WINDOW} packets</div>
</header>
<div id="list"><div class="empty">waiting for packets\u2026</div></div>
<script>
// Units by field name. Variants use both long and short forms of the same quantity, so both
// are mapped; an unmapped field still renders, just without a unit.
var UNITS = {
  temperature:'\\u00b0C', temp:'\\u00b0C', pressure:'hPa', pres:'hPa', humidity:'%', humid:'%',
  battery:'V', bat:'V', voltage:'V', wind:'m/s', wspd:'m/s', wgust:'m/s', wdir:'\\u00b0',
  rain:'mm', depth:'mm', solar:'W/m\\u00b2', light:'lx', lux:'lx',
  cpm:'cpm', dose:'\\u00b5Sv/h', rssi:'dBm', rssi_packet:'dBm'
};
// Framing/metadata, not content — shown in the header line or not at all.
var SKIP = {variant:1, station:1, sequence:1, rssi_packet:1, packed_bits:1, packed_bytes:1};

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
      chipsFor(v, into);
    } else {
      var u = UNITS[k];
      into.appendChild(chip(k, u ? v + ' ' + u : String(v)));
    }
  }
}

// LoRa demodulates below the noise floor, so a "weak" reading here is not a failed link — it is
// less margin than a strong one. Thresholds spread the range actually seen at this site so the
// colour changes as you move, which is the point during a walk test.
function rssiClass(r) { return r >= -95 ? 'good' : r >= -105 ? 'ok' : 'weak'; }
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
  var prev = lastSeq[r.station];
  if (prev !== undefined && r.sequence > prev + 1) top.appendChild(el('span', 'gap', '-' + (r.sequence - prev - 1)));
  if (r.sequence !== undefined) lastSeq[r.station] = r.sequence;

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

var HEAD = ['station', 'pkts', 'rate', 'lost', 'avg rssi', 'last'];
function renderStats(rows) {
  var tb = document.querySelector('#sum tbody');
  tb.innerHTML = '';
  var hr = document.createElement('tr');
  for (var i = 0; i < HEAD.length; i++) hr.appendChild(el('th', null, HEAD[i]));
  tb.appendChild(hr);

  for (var n = 0; n < rows.length; n++) {
    var s = rows[n], tr = document.createElement('tr');
    var who = el('td', 'who');
    who.appendChild(el('b', null, s.id));
    if (s.name || s.distance != null) {
      var sub = el('span');
      if (s.name) sub.appendChild(document.createTextNode(s.name));
      if (s.distance != null) {
        if (s.name) sub.appendChild(document.createTextNode(' \\u00b7 '));
        // The distance doubles as the way to go look at the spot. ?api=1&query= is the
        // documented form that hands off to the Maps app on a phone rather than the web
        // page; rel=noopener because target=_blank otherwise gives the opened page a
        // handle back to this one.
        if (s.lat != null && s.lon != null) {
          var a = document.createElement('a');
          a.href = 'https://www.google.com/maps/search/?api=1&query=' + s.lat + ',' + s.lon;
          a.target = '_blank';
          a.rel = 'noopener noreferrer';
          a.textContent = metres(s.distance);
          sub.appendChild(a);
        } else sub.appendChild(document.createTextNode(metres(s.distance)));
      }
      who.appendChild(sub);
    }
    tr.appendChild(who);
    tr.appendChild(el('td', null, String(s.count)));
    tr.appendChild(el('td', null, s.rate == null ? '\\u2013' : s.rate.toFixed(1) + '/m'));
    tr.appendChild(el('td', null, s.lost + (s.lost ? ' (' + s.lossPct.toFixed(1) + '%)' : '')));
    tr.appendChild(el('td', null, s.rssiAvg == null ? '\\u2013' : Math.round(s.rssiAvg) + ' dBm'));
    // Age is the honest answer to "is it still there" — a gap only shows when a LATER packet
    // arrives to reveal it, so a station that goes silent shows up here and nowhere else.
    var age = el('td', null, s.age == null ? '\\u2013' : s.age < 90 ? s.age + 's' : Math.round(s.age / 60) + 'm');
    if (s.age != null && s.age > 300) age.className = 'stale';
    tr.appendChild(age);
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
    view.n > rows.length ? 'tracker positions · newest first · latest ' + rows.length + ' of ' + view.n
                         : 'tracker positions · newest first';

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
</script>
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
        for (const rec of history) send('packet', rec, res);
        send('stats', snapshot(), res);
        send('gnss', gnssView(), res);
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

    res.writeHead(404, { 'Content-Type': 'text/plain' });
    res.end('not found\n');
});

server.listen(PORT, BIND, () => {
    const known = Object.keys(STATIONS);
    console.log(`iotdata monitor: http://${os.hostname()}.local:${PORT}/`);
    console.log(`  config  ${cfgFrom}`);
    console.log(`  broker  ${BROKER}:${MQTT_PORT}  topic ${TOPIC}  keep ${KEEP}  window ${WINDOW}`);
    console.log(`  ref     ${REF ? `${REF.lat}, ${REF.lon}${REF.alt != null ? ` (${REF.alt} m)` : ''}` : '(not configured: no distances)'}`);
    for (const id of known) {
        const s = STATIONS[id];
        console.log(`  station ${id}  ${s.name || '(unnamed)'}${s.distance != null ? `  ${Math.round(s.distance)} m` : ''}`);
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
