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
// No npm dependencies, deliberately: this box has no MQTT library installed and pulling one in
// is not worth it. mosquitto_sub does the subscribing and this process parses its output.
// Traffic to the browser is Server-Sent Events rather than WebSockets for the same reason —
// EventSource needs no library at either end, and this is strictly one-way.

const fs = require('fs');
const os = require('os');
const path = require('path');
const http = require('http');
const { spawn } = require('child_process');

const USAGE = `
iotdata_gateway_monitor.js — live view of received sensor traffic

  Usage:
    iotdata-gateway-monitor [--config FILE] [--port N] [--broker HOST] [--topic PAT]
                            [--keep N] [--window N]
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

const CONFIG_CANDIDATES = flag('config')
    ? [flag('config')]
    : [
          path.join(__dirname, `iotdata_gateway_monitor.${os.hostname()}.cfg`),
          path.join(__dirname, 'iotdata_gateway_monitor.cfg'),
          '/etc/default/iotdata-gateway-monitor',
      ];

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

// A sequence jump larger than this is a station restart or a counter wrap, not lost packets.
// Counting it as loss would put the loss figure permanently at ~100% after a single reboot.
const SEQ_MAX_GAP = parseInt(cfg['stats-max-gap'] ?? '1000', 10);

// The gateway's own surveyed position. Distances are measured from here.
const REF =
    cfg['ref-latitude'] !== undefined && cfg['ref-longitude'] !== undefined
        ? { lat: num(cfg['ref-latitude']), lon: num(cfg['ref-longitude']), alt: num(cfg['ref-altitude'], null) }
        : null;

// station-<ID>-name / -latitude / -longitude / -altitude
const STATIONS = {};
for (const key of Object.keys(cfg)) {
    const m = /^station-([0-9A-Fa-f]{1,8})-(name|latitude|longitude|altitude)$/.exec(key);
    if (!m) continue;
    const id = m[1].toUpperCase();
    STATIONS[id] = STATIONS[id] || {};
    STATIONS[id][m[2]] = m[2] === 'name' ? cfg[key] : num(cfg[key], null);
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

// ---------------------------------------------------------------------------- state

const history = []; // newest last; the page reverses it
const clients = new Set();
const stats = new Map();
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
    if (!stats.has(id)) stats.set(id, { id, count: 0, lost: 0, prevSeq: undefined, times: [], rssis: [] });
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
    send('packet', rec);
    send('stats', snapshot());
}

// A sensor packet is identified by carrying a station id, which is what separates it from the
// gateway's own stats/status publications on neighbouring topics. Matching on that rather than
// on a topic pattern means new variants appear here without this script needing to learn them.
function handleLine(line) {
    const sp = line.indexOf(' ');
    if (sp < 0) return;
    const topic = line.slice(0, sp);
    let body;
    try {
        body = JSON.parse(line.slice(sp + 1));
    } catch {
        return; // not JSON: status lines, retained junk, a partial line at startup
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
        site: meta ? { name: meta.name || null, lat: meta.latitude ?? null, lon: meta.longitude ?? null, distance: meta.distance ?? null } : null,
        body,
    });
}

// ---------------------------------------------------------------------------- subscriber

// mosquitto_sub exits when the broker goes away (restart, network blip). Respawn rather than
// dying: this is meant to be left running for hours while siting work goes on.
let backoff = 1000;
function subscribe() {
    const child = spawn('mosquitto_sub', ['-h', BROKER, '-p', String(MQTT_PORT), '-t', TOPIC, '-v'], { stdio: ['ignore', 'pipe', 'pipe'] });
    let buf = '';

    child.stdout.on('data', (chunk) => {
        buf += chunk;
        // Payloads are single-line JSON, but never assume a chunk boundary falls on a newline.
        let nl;
        while ((nl = buf.indexOf('\n')) >= 0) {
            handleLine(buf.slice(0, nl));
            buf = buf.slice(nl + 1);
        }
        if (buf.length > 65536) buf = ''; // a line this long is a stuck stream, not a packet
        backoff = 1000;
    });
    child.stderr.on('data', (d) => process.stderr.write('mosquitto_sub: ' + d));
    child.on('exit', (code, sig) => {
        console.error(`mosquitto_sub exited (${sig || code}); retrying in ${backoff / 1000}s`);
        setTimeout(subscribe, backoff);
        backoff = Math.min(backoff * 2, 30000);
    });
    child.on('error', (e) => console.error('mosquitto_sub failed to start:', e.message));
    process.on('SIGINT', () => {
        child.kill();
        process.exit(0);
    });
    process.on('SIGTERM', () => {
        child.kill();
        process.exit(0);
    });
}

// Re-push stats on a timer as well as per packet, so "age" counts up on screen while a station
// is silent. Without this a station that stopped transmitting would sit there looking healthy.
setInterval(() => send('stats', snapshot()), 5000);

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
  .bar { padding:8px 12px; display:flex; align-items:center; gap:10px; }
  .bar b { font-size:15px; font-weight:600; }
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
  .chips { margin-top:6px; display:flex; flex-wrap:wrap; gap:5px; }
  .chip { background:var(--bg); border:1px solid var(--line); border-radius:6px;
          padding:2px 7px; font-size:13px; font-variant-numeric:tabular-nums; }
  .chip i { color:var(--dim); font-style:normal; font-size:11px; margin-right:4px;
            text-transform:uppercase; letter-spacing:.03em; }
  .empty { color:var(--dim); padding:24px 12px; text-align:center; }
  .note { color:var(--dim); font-size:10.5px; padding:3px 8px 6px; }
</style>
</head>
<body>
<header>
  <div class="bar"><b>iotdata</b><span id="stat"><span id="dot"></span><span id="msg">connecting</span></span></div>
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
    if (s.name || s.distance != null)
      who.appendChild(el('span', null, (s.name || '') + (s.name && s.distance != null ? ' \\u00b7 ' : '') + (s.distance != null ? metres(s.distance) : '')));
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

var list = document.getElementById('list');
var dot = document.getElementById('dot'), msg = document.getElementById('msg');
var count = 0;

function add(r) {
  if (count === 0) list.innerHTML = '';
  list.insertBefore(row(r), list.firstChild);
  count++;
  while (list.childNodes.length > ${KEEP}) list.removeChild(list.lastChild);
  msg.textContent = count + ' packets';
}

var es = new EventSource('/events');
es.onopen = function () { dot.className = 'live'; msg.textContent = count ? count + ' packets' : 'listening'; };
es.onerror = function () { dot.className = ''; msg.textContent = 'reconnecting'; };
es.addEventListener('packet', function (ev) { add(JSON.parse(ev.data)); });
es.addEventListener('stats', function (ev) { renderStats(JSON.parse(ev.data)); });
</script>
</body>
</html>`;

// ---------------------------------------------------------------------------- server

const server = http.createServer((req, res) => {
    const url = req.url.split('?')[0];

    if (url === '/' || url === '/index.html') {
        res.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8', 'Cache-Control': 'no-store' });
        res.end(PAGE);
        return;
    }

    if (url === '/events') {
        res.writeHead(200, {
            'Content-Type': 'text/event-stream',
            'Cache-Control': 'no-store',
            Connection: 'keep-alive',
            'X-Accel-Buffering': 'no',
        });
        // Replay what we have so a phone arriving late, or reconnecting after the screen slept,
        // sees context rather than an empty page until the next packet lands.
        for (const rec of history) send('packet', rec, res);
        send('stats', snapshot(), res);
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
    subscribe();
});
