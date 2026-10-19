#!/usr/bin/env node

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------
//
// iotdata_gateway_control.js - drive iotdata node CONTROL commands over MQTT.
//
// Publishes a JSON management request to <prefix>/manage/req; the gateway
// (iotdata_gateway_ctrl.h) turns it into a node CONTROL payload, executed locally when it
// is addressed to the gateway itself and aired as a DOWN frame to the target otherwise.
//
// There is ONE command namespace: every command here, mesh management included, is a
// CONTROL key from iotdata_node.h. (Mesh management used to be a parallel MANAGE
// vocabulary of its own; it is not any more.) Add a command by adding a CONTROL key
// there and a line for it in the gateway's ctrl_on_message.
//
// Everything a station says comes back as JSON under <prefix>/manage/from/<station> -- an answer
// to a request and a node's own periodic report arrive the same way -- and the "type" field
// says what it is -- there is no topic per producer to keep track of:
//   {"type":"node", "station":1471,"tlv":"status","data":{...}}          a node TLV report
//   {"type":"diag", "station":1,"cmd":"diag","source":"blackbox","text":"..."}
//   {"type":"diag", "station":1,"cmd":"diag-dump","source":"blackbox","index":0,"record":"..."}
//   {"type":"error","station":1,"cmd":"node-foo","text":"unknown tlv 'foo'"}
//   {"type":"survey","directive":801,"observer":257,"station":1335,"seq":77,"rssi":-119,"snr":-10}
//
// A SURVEY is the one command here that is NOT a node CONTROL key. It is a mesh directive: it
// asks every relay that takes it to report which packets it hears DIRECTLY from the named
// stations, and at what margin. Observations stream back on the same management subtree, one per
// observer per packet -- join them on {station, seq} to get the reception pattern for a single
// transmission. An observer reporting zero for a window says so in words; that is a result (no
// coverage there) and not the same as a relay that never got the directive and stays silent.
// A diag names its source because the blackbox recorder is one possible diagnostic, not the
// definition of one. Records stay strings: decoding one needs the @blackbox definitions header,
// which this tool has and the gateway does not -- pass --definitions and it is decoded in place.
//
// What a command produces still depends on the command: a report that has a TLV (the node ones,
// and mesh state as the mesh scope of STATUS) comes back on that topic; a table dump
// (stations/peers/filters) prints on the target's own console, so watch that via esp32-tool.
//
// Usage:
//   ./iotdata_gateway_control.js [options] <command> [args]
//   ./iotdata_gateway_control.js status --target all
//   ./iotdata_gateway_control.js status --target 0x5BF,0x537,0x538      (one frame each)
//   ./iotdata_gateway_control.js status --target 0x5BF --broker mqtt://192.168.0.61:1883
//   ./iotdata_gateway_control.js conf --target 0x5BF
//   ./iotdata_gateway_control.js conf-set --target 0x5BF 0x051:u8=20 0x042:u32=300000
//   ./iotdata_gateway_control.js raw '{"cmd":"status","target":"all"}'
//   ./iotdata_gateway_control.js survey 0x537 --duration 20 --batch 10 --watch 300
//   ./iotdata_gateway_control.js survey 0x537,0x538 --duration 120 --batch 600
//   ./iotdata_gateway_control.js survey-cancel all
//
// Setup:  npm install        (in this dir; pulls in mqtt)
//
// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------

'use strict';

// mqtt is require()d lazily in main() so --help, --dry-run and arg errors work before `npm install`.

const DEFAULTS = {
    broker: process.env.MQTT_BROKER || 'mqtt://localhost:1883',
    prefix: process.env.IOTDATA_PREFIX || 'iotdata',
    target: 'all',
    watch: 10, // seconds, when --watch is given with no number
    duration: 10, // minutes a survey directive lives for
    batch: 30, // seconds between a relay's survey reports
};

const opts = { broker: DEFAULTS.broker, prefix: DEFAULTS.prefix, target: DEFAULTS.target, watch: 0, dryRun: false, verbose: false, debug: false, definitions: null, duration: DEFAULTS.duration, batch: DEFAULTS.batch, id: null };

const display = {
    log: (...a) => console.log(...a),
    err: (...a) => console.error(...a),
    verbose: (...a) => opts.verbose && console.error('[verbose]', ...a),
    debug: (...a) => opts.debug && console.error('[debug]', ...a),
};

// ------------------------------------------------------------------------------------------------------------------------
// Target: "all" / "broadcast" / "*" -> broadcast; otherwise a station id (decimal or 0x..), 1..4094.
// ------------------------------------------------------------------------------------------------------------------------

const TARGETS_MAX = 8; // IOTDATA_CONTROL_TARGETS_MAX -- the gateway refuses a longer list outright

function parseTargetOne(t, whole) {
    const s = String(t).toLowerCase();
    if (s === 'all' || s === 'broadcast' || s === '*') return 'all';
    const n = s.startsWith('0x') ? parseInt(s, 16) : parseInt(s, 10);
    if (Number.isNaN(n) || n < 1 || n > 0xffe) throw new Error(`invalid target '${t}'${whole !== t ? ` in '${whole}'` : ''} (use all, or a station id 1..4094 / 0x001..0xFFE)`);
    return n;
}

// One station, or several comma-separated. A list goes over as a JSON array and the gateway sends
// one DOWN frame per station -- the wire addresses one station at a time, so "several" is the
// gateway's fan-out, not a wire feature. `all` anywhere makes the whole thing a broadcast.
function parseTarget(t) {
    if (t === undefined) throw new Error('--target needs a value');
    const parts = String(t).split(',').map((p) => p.trim()).filter((p) => p.length > 0);
    if (parts.length === 0) throw new Error('--target needs a value');
    if (parts.length === 1) return parseTargetOne(parts[0], t);
    const list = parts.map((p) => parseTargetOne(p, t));
    if (list.some((v) => v === 'all')) return 'all';
    const uniq = [...new Set(list)];
    if (uniq.length > TARGETS_MAX) throw new Error(`too many targets in '${t}': ${uniq.length}, at most ${TARGETS_MAX}`);
    return uniq;
}

// A concrete station id (decimal or 0x..) — for block/allow/unfilter/peers-remove.
function parseStation(s) {
    if (s === undefined) throw new Error('this command needs a <station> id (1..4094 or 0x..)');
    const n = String(s).toLowerCase().startsWith('0x') ? parseInt(s, 16) : parseInt(s, 10);
    if (Number.isNaN(n) || n < 1 || n > 0xffe) throw new Error(`invalid station '${s}' (1..4094 or 0x001..0xFFE)`);
    return n;
}

// ------------------------------------------------------------------------------------------------------------------------
// Command registry — extend here, AND in iotdata_node_control.h, which is the vocabulary that
// actually resolves a name: the gateway looks every `cmd` up there and answers "unknown command"
// for anything it does not hold. This table carries only what C has no room for -- the help text
// and what each command consumes -- so a name present here and absent there is a dead command, and
// `make check` fails on exactly that (see the control Makefile).
// build(args) returns the JSON request body; `target` is attached by buildRequest unless the command sets it.
// ------------------------------------------------------------------------------------------------------------------------

/*
 * The command vocabulary.
 *
 * A subject is FOUR LETTERS, matching the device's own USB CLI (`vers`, `stat`, `logl`, `boot`
 * there), and verbs are full words. So `diag enable` is the same words over MQTT as over serial,
 * and an operator driving a node both ways learns one set.
 *
 * A `mesh` or `node` prefix names WHOSE table is meant, and both are needed because both exist:
 * a node has its own stations and filters, and the mesh has its peers. `diag` prefixes the same
 * way, since the recorder and the cart are two unrelated things that both hang off DIAGNOSTICS.
 *
 * Subjects may be typed space-separated as on the serial CLI, or hyphenated -- `mesh peers clear`
 * and `mesh-peers-clear` are the same command. The wire always carries the hyphenated token.
 *
 * `takes` says what a command consumes from the remaining arguments:
 *   'station'  a station id, required   (which node the command is ABOUT; --target says who to ask)
 *   'scope'    an optional scope word, in that command's own vocabulary
 *   'content'  a content id, required, read as HEX -- §4's ids are (station << 12) | counter and
 *              are only legible that way, which is why this one is not parsed like a station
 */
const COMMANDS = {
    // --- the system TLVs: one request each --------------------------------------------------
    'recv': { summary: 'request RECEIVE — when the node is next reachable' },
    'vers': { summary: 'request VERSION — device firmware, platform, build, serial' },
    'vari': { summary: 'request VARIANT — iotdata variant suite produced' },
    'ctrl': { summary: 'request CONTROL — supported control operations' },
    'stat': { summary: 'request STATUS — operating status, and the tables', takes: 'scope', scopes: 'node | mesh | stations | filters | peers | content | all, comma-separated   (default: node,mesh)' },
    'conf': { summary: 'request CONFIG — the device\'s own settable values' },
    'conf-set': { summary: 'write one or more config values', takes: 'config', config: '<id>:<type>=<value> ...   e.g. 0x051:u8=20 0x042:u32=300000' },
    'diag': { summary: 'request DIAGNOSTICS — diagnostic operations and recordings' },
    'cont': { summary: 'request CONTENT — tbd' },
    'reports': { summary: 'request every report the node can produce, in type order' },

    // --- generic system control ---------------------------------------------------------------
    'boot': { summary: 'restart the node' },


    // --- diag: the recorder -------------------------------------------------------------------
    'diag-bbox-enable': { summary: 'enable recording' },
    'diag-bbox-disable': { summary: 'disable recording' },
    'diag-bbox-clear': { summary: 'clear recordings' },
    'diag-bbox-dump': { summary: 'dump recordings (to console)' },

    // --- diag: the cart -----------------------------------------------------------------------
    // The other half of CART_INTERVAL_MIN=0: with no periodic window, this is the ONLY way the
    // rail comes up, so it is the one command on this list a remote site depends on. Disable is
    // the way back from a window that will not close, short of the node's own two-hour backstop.
    'diag-cart-enable': { summary: 'open the cart window now (power the side unit)' },
    'diag-cart-disable': { summary: 'close the cart window now (cut the side unit)' },

    // --- node management ----------------------------------------------------------------------
    // The tables have no read command of their own: each is `stat` with that table's scope word,
    // since a table IS a STATUS scope. Only what a scope cannot say -- print it on your own
    // console, change it -- is a command.
    'node-stations-dump': { summary: 'print the stations table on the node\'s console' },
    'node-filters-update': { summary: 'update one filter in the table', takes: 'station+action', actions: 'block | allow | none' },
    'node-filters-clear': { summary: 'remove all filters', takes: 'scope', scopes: 'all | manual | auto   (default: all)' },
    'node-filters-dump': { summary: 'print the filters table on the node\'s console' },
    'node-rollback': { summary: 'boot the PREVIOUS firmware image and keep it -- refused if there is not one' },

    // --- mesh management ----------------------------------------------------------------------
    'mesh-peers-update': { summary: 'update one peer in the table', takes: 'station+action', actions: 'remove | none' },
    'mesh-peers-clear': { summary: 'remove all peers (forcing re-discovery)' },
    'mesh-peers-dump': { summary: 'print the peers table on the node\'s console' },

    // --- content ------------------------------------------------------------------------------
    // Insert and download are absent and are not an omission: both are a byte STREAM, and the
    // only thing here that can carry one is the node's own serial console (esp32-tool) until the
    // CONTENT TLVs exist. These four are what can be asked of a store from a distance.
    'content-list': { summary: 'print the content store on the node\'s console' },
    'content-delete': { summary: 'delete one file from the store', takes: 'content', contents: '<id in hex>   e.g. 1a2b3c' },
    'content-inspect': { summary: 'describe one file, and parse it if it is an OTA bundle', takes: 'content', contents: '<id in hex>   e.g. 1a2b3c' },
    'content-validate': { summary: 'read one file back and report its CRC-64', takes: 'content', contents: '<id in hex>   e.g. 1a2b3c' },
    'content-ota': { summary: 'what the firmware consumer is doing, and what the last attempt did' },
    'content-ota-apply': { summary: 'arm one held bundle to be applied now, clearing its refusal and attempt count', takes: 'content', contents: '<id in hex>   e.g. 1a2b3c' },

    // --- mesh diagnostics: a DIRECTIVE, not a node command --------------------------------------
    'survey': { summary: 'ask the mesh who hears a station, and how well', takes: 'stations', stations: '<station> [<station> ...] or a comma-separated list, with --duration/--batch/--id' },
    'survey-cancel': { summary: 'retire a survey directive (or every one)', takes: 'id', ids: '<id in hex> | all   (default: all)' },

    // --- power user ---------------------------------------------------------------------------
    'raw': { summary: 'send a raw JSON request, e.g. raw \'{"cmd":"stat"}\'' },
};

/* Resolve leading argument words into a command name, longest match first, so a space-separated
   subject reads as it does on the serial console. Returns [name, remaining args]. */
function resolveCommand(words) {
    for (let n = Math.min(4, words.length); n >= 1; n--) {
        const joined = words.slice(0, n).join('-');
        if (COMMANDS[joined] !== undefined)
            return [joined, words.slice(n)];
    }
    return [words[0] ?? '', words.slice(1)];
}

// A record arrives as a string field inside the response envelope, because decoding it needs the
// @blackbox definitions header that only this side has. With --definitions, decode it in place so
// the printed response is JSON the whole way down.
function convertResponse(text, convert) {
    let obj;
    try {
        obj = JSON.parse(text);
    } catch {
        // not the envelope: telemetry, or a bare line from something that does not wrap
        return text.split('\n').map((l) => convert(l.trim()) ?? l).join('\n');
    }
    if (obj !== null && typeof obj === 'object' && typeof obj.record === 'string') {
        const decoded = convert(obj.record.trim());
        if (decoded !== null) {
            try {
                return JSON.stringify({ ...obj, record: JSON.parse(decoded) });
            } catch {
                /* leave the raw record in place */
            }
        }
    }
    return text;
}

/* The CONFIG wire types, as iotdata_node_config.h numbers them. */
const CONFIG_TYPES = { bool: 0, u8: 1, i8: 2, u16: 3, i16: 4, u32: 5, i32: 6, u64: 7, i64: 8, float: 9, string: 10, blob: 11 };

/*
 * `<id>:<type>=<value>` — numeric id, explicit type.
 *
 * NUMERIC because the gateway does not resolve names, and should not: an id is what travels, and the
 * client nearest the operator is the one place that turns a name into one. A gateway with no name
 * table is a gateway that cannot fall out of step with the firmware in the field — and one that
 * could run on an ESP32.
 *
 * EXPLICITLY TYPED because a write is not self-describing the way a read is. A node reports
 * id+type+value, but "0x051 = 20" does not say whether 20 is a u8 or a u32, and the record on the
 * wire carries the type in its own header. Nothing here validates the VALUE: the node owns the
 * bounds, and it answers with what is now true — so a refused value is simply absent from the reply.
 */
function parseConfigAssignments(args) {
    if (args.length === 0) throw new Error('conf-set: needs at least one <id>:<type>=<value>');
    return args.map((a) => {
        const m = /^([^:]+):([a-z0-9]+)=(.+)$/i.exec(a);
        if (!m) throw new Error(`conf-set: '${a}' is not <id>:<type>=<value>, e.g. 0x051:u8=20`);
        const id = Number.parseInt(m[1], m[1].toLowerCase().startsWith('0x') ? 16 : 10);
        if (!Number.isInteger(id) || id < 0 || id > 0x0fff) throw new Error(`conf-set: id '${m[1]}' is not in 0x000..0xFFF`);
        const type = CONFIG_TYPES[m[2].toLowerCase()];
        if (type === undefined) throw new Error(`conf-set: unknown type '${m[2]}' (one of ${Object.keys(CONFIG_TYPES).join(', ')})`);
        const value = Number(m[3]);
        if (!Number.isFinite(value)) throw new Error(`conf-set: value '${m[3]}' is not a number`);
        return { id, type, value };
    });
}

function buildRequest(name, args) {
    if (name === 'raw') {
        if (args[0] === undefined) throw new Error("raw: needs a JSON request, e.g. raw '{\"cmd\":\"stat\"}'");
        let req;
        try {
            req = JSON.parse(args[0]);
        } catch (e) {
            throw new Error('raw: invalid JSON: ' + e.message);
        }
        if (req.target === undefined) req.target = opts.target;
        return req;
    }
    /* The name is sent as typed: the gateway resolves it against iotdata_node_control.h, so one
       place maps a name to a CONTROL key rather than two that must agree. This table is for help
       and local validation only -- which is why a name it holds and the C table does not fails
       here, locally, instead of travelling and coming back "unknown command". */
    const spec = COMMANDS[name];
    if (!spec) throw new Error(`unknown command '${name}' (try --help)`);
    const req = { cmd: name, target: opts.target };
    if (spec.takes === 'station') req.station = parseStation(args[0]);
    else if (spec.takes === 'station+action') {
        req.station = parseStation(args[0]);
        if (args[1] !== undefined) req.action = args[1];
    } else if (spec.takes === 'content') {
        const word = args[0];
        if (word === undefined) throw new Error(`${name}: name a content id (in hex)`);
        if (!/^(0[xX])?[0-9a-fA-F]{1,6}$/.test(word)) throw new Error(`${name}: '${word}' is not a content id`);
        req.content = word;
    } else if (spec.takes === 'scope' && args[0] !== undefined) req.scope = args[0];
    else if (spec.takes === 'config') req.set = parseConfigAssignments(args);
    else if (spec.takes === 'stations') {
        /* A survey is not addressed to anybody: the gateway airs it and whoever hears it decides.
           Sending a target would suggest otherwise, so it is left off. */
        delete req.target;
        const words = args.join(',').split(',').map((w) => w.trim()).filter((w) => w.length > 0);
        if (words.length === 0) throw new Error('survey: name at least one station to watch for');
        req.stations = words.map(parseStation);
        req.duration = opts.duration;
        req.batch = opts.batch;
        if (opts.id !== null) req.id = opts.id;
    } else if (spec.takes === 'id') {
        delete req.target;
        const word = (args[0] ?? 'all').toLowerCase();
        if (word !== 'all' && word !== '*') req.id = word;
    }
    return req;
}

// ------------------------------------------------------------------------------------------------------------------------

function usage() {
    display.log('iotdata_gateway_control.js - drive iotdata node CONTROL commands over MQTT\n');
    display.log('Usage: iotdata_gateway_control.js [options] <command> [args]\n');
    display.log('Options:');
    display.log(`  --broker <url>   MQTT broker      (default: ${DEFAULTS.broker}, or $MQTT_BROKER)`);
    display.log(`  --prefix <p>     topic prefix     (default: ${DEFAULTS.prefix}, or $IOTDATA_PREFIX)`);
    display.log('  --target <t>     all | broadcast | <station id: 1..4094 or 0x..> | a comma-separated list of ids (default: all)');
    display.log(`  --watch [secs]   after sending, print live telemetry for N seconds (default ${DEFAULTS.watch})`);
    display.log('  --definitions <hdr>  blackbox record header (with // @blackbox tag=...); while watching,');
    display.log('                   convert recognised record lines (e.g. a blackbox-dump reply) CSV -> JSON');
    display.log(`  --duration <min> survey: how long the directive lives, in MINUTES (default ${DEFAULTS.duration})`);
    display.log(`  --batch <secs>   survey: how often each relay reports, in SECONDS (default ${DEFAULTS.batch})`);
    display.log('  --id <hex>       survey: the directive id (default: the gateway picks one).');
    display.log('                   Re-sending the SAME id is safe -- relays already surveying pass it on and carry on unchanged');
    display.log('  --dry-run | -n   print the request that would be sent, do not connect');
    display.log('  --verbose | --debug | --help\n');
    display.log('Commands  (spaced or hyphenated: `mesh peers clear`):');
    /* Measured, not a constant: the names grew prefixes (node-, mesh-, diag-bbox-, diag-cart-)
       and a hardcoded column silently stopped aligning when they did. */
    const w = Math.max(...Object.keys(COMMANDS).map((n) => n.length));
    const pad = ''.padEnd(w);
    for (const [name, c] of Object.entries(COMMANDS)) {
        display.log(`  ${name.padEnd(w)} ${c.summary}`);
        if (c.takes === 'station') display.log(`  ${pad}   takes <station>`);
        if (c.takes === 'station+action') display.log(`  ${pad}   takes <station> <action>`);
        if (c.takes === 'config') display.log(`  ${pad}   takes ${c.config}`);
        if (c.takes === 'stations') display.log(`  ${pad}   takes ${c.stations}`);
        if (c.takes === 'id') display.log(`  ${pad}   takes ${c.ids}`);
        if (c.takes === 'content') display.log(`  ${pad}   takes ${c.contents}`);
        if (c.actions) display.log(`  ${pad}   action: ${c.actions}`);
        if (c.scopes) display.log(`  ${pad}   scope: ${c.scopes}`);
    }
}

// ------------------------------------------------------------------------------------------------------------------------

function parseArgv(argv) {
    const rest = [];
    for (let i = 0; i < argv.length; i++) {
        const a = argv[i];
        if (a === '--broker') opts.broker = argv[++i];
        else if (a === '--prefix') opts.prefix = argv[++i];
        else if (a === '--target') opts.target = parseTarget(argv[++i]);
        else if (a === '--definitions' || a === '--records') opts.definitions = argv[++i];
        else if (a === '--duration') opts.duration = Number(argv[++i]);
        else if (a === '--batch') opts.batch = Number(argv[++i]);
        else if (a === '--id') opts.id = argv[++i];
        else if (a === '--watch') {
            const n = Number(argv[i + 1]);
            if (!Number.isNaN(n) && argv[i + 1] !== undefined) {
                opts.watch = n;
                i++;
            } else opts.watch = DEFAULTS.watch;
        } else if (a === '--dry-run' || a === '-n') opts.dryRun = true;
        else if (a === '--verbose' || a === '-v') opts.verbose = true;
        else if (a === '--debug') opts.debug = true;
        else if (a === '--help' || a === '-h') {
            usage();
            process.exit(0);
        } else if (a.startsWith('--')) throw new Error(`unknown option '${a}'`);
        else rest.push(a);
    }
    return rest;
}

// ------------------------------------------------------------------------------------------------------------------------
// Record CSV -> JSON, using the blackbox csv2json module + a component's record header. Returns a
// function line->JSON-string for recognised records, or null for anything else (so callers keep the
// raw line). Returns null overall (records stay CSV) if csv2json or the header can't be loaded.
// ------------------------------------------------------------------------------------------------------------------------

function makeRecordConverter(defsPath) {
    const fs = require('fs');
    const path = require('path');
    // csv2json lives in the blackbox repo, a sibling under iotdata-depend; env override for other layouts.
    const modPath = process.env.BLACKBOX_CSV2JSON || path.resolve(__dirname, '../../../iotdata-depend/blackbox/js/csv2json.js');
    let parseDefinitions, csvLineToJson;
    try {
        ({ parseDefinitions, csvLineToJson } = require(modPath));
    } catch (e) {
        display.err(`warning: csv2json not found at ${modPath} (${e.message}); records stay CSV`);
        return null;
    }
    let schema;
    try {
        schema = parseDefinitions(fs.readFileSync(defsPath, 'utf8'));
    } catch (e) {
        display.err(`warning: cannot read definitions '${defsPath}' (${e.message}); records stay CSV`);
        return null;
    }
    const tags = Object.keys(schema);
    display.verbose(`csv2json: ${tags.length} record type(s) from ${defsPath}: ${tags.join(', ') || '(none)'}`);
    return (line) => {
        if (!line) return null;
        const tag = line.split(',', 1)[0];
        if (!Object.prototype.hasOwnProperty.call(schema, tag)) return null; // not a known record → leave as CSV/text
        try {
            return JSON.stringify(csvLineToJson(line, schema));
        } catch {
            return null;
        }
    };
}

// ------------------------------------------------------------------------------------------------------------------------

/* When every address of a name fails, Node raises an AggregateError whose message is EMPTY, with the
   reasons in .errors -- so say what failed, and what each address said. */
function mqttError(e) {
    const what = e.message || e.code || String(e);
    const each = Array.isArray(e.errors) ? e.errors.map((x) => `${x.address}${x.port ? ':' + x.port : ''} ${x.code || x.message}`) : [];
    return each.length ? `${what} (${each.join(', ')})` : what;
}

async function main() {
    let rest;
    try {
        rest = parseArgv(process.argv.slice(2));
    } catch (e) {
        display.err('error: ' + e.message);
        process.exit(2);
    }
    if (rest.length === 0) {
        usage();
        process.exit(1);
    }

    /* longest-match, so `mesh peers clear` reads as it does on the serial console */
    const [command, args] = resolveCommand(rest);
    let request;
    try {
        request = buildRequest(command, args);
    } catch (e) {
        display.err('error: ' + e.message);
        process.exit(2);
    }

    const topicReq = `${opts.prefix}/manage/req`;
    const payload = JSON.stringify(request);

    if (opts.dryRun) {
        display.log(`(dry-run) -> ${topicReq}  ${payload}`);
        return;
    }

    let mqtt;
    try {
        mqtt = require('mqtt');
    } catch {
        display.err("error: the 'mqtt' package is not installed — run: npm install (in this dir)");
        process.exit(1);
    }

    /* THE BROKER BY ITS IPv4 ADDRESS, when it has one: the fleet's broker listens on IPv4 only, and an
       mDNS lookup for either family can come back with the IPv6 addresses alone -- every one refused.
       TLS keeps the name, which its certificate is checked against. */
    let broker = opts.broker;
    try {
        const u = new URL(opts.broker);
        if (u.protocol === 'mqtt:' || u.protocol === 'tcp:') {
            const { address } = await require('dns').promises.lookup(u.hostname, { family: 4 });
            if (address !== u.hostname) {
                u.hostname = address;
                broker = u.toString();
            }
        }
    } catch {
        /* no IPv4 address for it: connect by name, as given */
    }
    display.verbose(`connecting to ${opts.broker}${broker !== opts.broker ? ` (${broker})` : ''}`);
    const client = mqtt.connect(broker);

    client.on('error', (e) => {
        display.err(`error: mqtt: ${mqttError(e)}`);
        process.exit(1);
    });

    await new Promise((resolve) => client.on('connect', resolve));
    display.verbose('connected');

    await new Promise((resolve, reject) => client.publish(topicReq, payload, { qos: 0 }, (e) => (e ? reject(e) : resolve()))).catch((e) => {
        display.err(`error: publish failed: ${e.message}`);
        process.exit(1);
    });
    display.log(`-> ${topicReq}  ${payload}`);

    if (opts.watch > 0) {
        const watchTopic = `${opts.prefix}/#`;
        const convert = opts.definitions ? makeRecordConverter(opts.definitions) : null;
        display.log(`watching ${watchTopic} for ${opts.watch}s  (replies and reports arrive on ${opts.prefix}/manage/from/<station> as JSON; table dumps go to the target's own console)`);
        client.on('message', (topic, msg) => {
            const text = msg.toString();
            display.log(`<- ${topic}  ${convert ? convertResponse(text, convert) : text}`);
        });
        await new Promise((resolve, reject) => client.subscribe(watchTopic, (e) => (e ? reject(e) : resolve()))).catch((e) => {
            display.err(`error: subscribe failed: ${e.message}`);
            process.exit(1);
        });
        await new Promise((resolve) => setTimeout(resolve, opts.watch * 1000));
    }

    client.end();
}

main();
