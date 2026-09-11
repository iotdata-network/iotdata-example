#!/usr/bin/env node

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------
//
// iotdata_gateway_command.js - drive iotdata node CONTROL commands over MQTT.
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
// Every response comes back as JSON on ONE topic, <prefix>/manage/resp, and the "resp" field
// says what it is -- there is no topic per producer to keep track of:
//   {"resp":"node", "station":"05BF","tlv":"status","data":{...}}        a node TLV report
//   {"resp":"diag", "station":"0001","cmd":"diag","source":"blackbox","text":"..."}
//   {"resp":"diag", "station":"0001","cmd":"diag-dump","source":"blackbox","index":0,"record":"..."}
//   {"resp":"error","station":"0001","cmd":"node-foo","text":"unknown tlv 'foo'"}
// A diag names its source because the blackbox recorder is one possible diagnostic, not the
// definition of one. Records stay strings: decoding one needs the @blackbox definitions header,
// which this tool has and the gateway does not -- pass --definitions and it is decoded in place.
//
// What a command produces still depends on the command: a report that has a TLV (the node ones,
// and mesh state as the mesh scope of STATUS) comes back on that topic; a table dump
// (stations/peers/filters) prints on the target's own console, so watch that via esp32-tool.
//
// Usage:
//   ./iotdata_gateway_command.js [options] <command> [args]
//   ./iotdata_gateway_command.js status --target all
//   ./iotdata_gateway_command.js status --target 0x5BF --broker mqtt://192.168.0.61:1883
//   ./iotdata_gateway_command.js raw '{"cmd":"status","target":"all"}'
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
};

const opts = { broker: DEFAULTS.broker, prefix: DEFAULTS.prefix, target: DEFAULTS.target, watch: 0, dryRun: false, verbose: false, debug: false, definitions: null };

const display = {
    log: (...a) => console.log(...a),
    err: (...a) => console.error(...a),
    verbose: (...a) => opts.verbose && console.error('[verbose]', ...a),
    debug: (...a) => opts.debug && console.error('[debug]', ...a),
};

// ------------------------------------------------------------------------------------------------------------------------
// Target: "all" / "broadcast" / "*" -> broadcast; otherwise a station id (decimal or 0x..), 1..4094.
// ------------------------------------------------------------------------------------------------------------------------

function parseTarget(t) {
    if (t === undefined) throw new Error('--target needs a value');
    const s = String(t).toLowerCase();
    if (s === 'all' || s === 'broadcast' || s === '*') return 'all';
    const n = s.startsWith('0x') ? parseInt(s, 16) : parseInt(s, 10);
    if (Number.isNaN(n) || n < 1 || n > 0xffe) throw new Error(`invalid target '${t}' (use all, or a station id 1..4094 / 0x001..0xFFE)`);
    return n;
}

// A concrete station id (decimal or 0x..) — for block/allow/unfilter/peers-remove.
function parseStation(s) {
    if (s === undefined) throw new Error('this command needs a <station> id (1..4094 or 0x..)');
    const n = String(s).toLowerCase().startsWith('0x') ? parseInt(s, 16) : parseInt(s, 10);
    if (Number.isNaN(n) || n < 1 || n > 0xffe) throw new Error(`invalid station '${s}' (1..4094 or 0x001..0xFFE)`);
    return n;
}

// ------------------------------------------------------------------------------------------------------------------------
// Command registry — extend here (mirror each with a CONTROL key in iotdata_node.h and a line in
// the gateway's ctrl_on_message).
// build(args) returns the JSON request body; `target` is attached by buildRequest unless the command sets it.
// ------------------------------------------------------------------------------------------------------------------------

/*
 * The command vocabulary.
 *
 * A subject is FOUR LETTERS, matching the device's own USB CLI (`vers`, `stat`, `logl`, `boot`
 * there), and verbs are full words. So `diag enable` is the same words over MQTT as over serial,
 * and an operator driving a node both ways learns one set.
 *
 * The mesh subjects keep a `mesh` prefix because it names a real group in the key space (0x40
 * upward, as against `type << 3` for a TLV subject). There is deliberately NO `node` prefix: since
 * mesh management became node CONTROL, such a prefix would be on every command and would therefore
 * distinguish nothing.
 *
 * Subjects may be typed space-separated as on the serial CLI, or hyphenated -- `mesh peers clear`
 * and `mesh-peers-clear` are the same command. The wire always carries the hyphenated token.
 *
 * `takes` says what a command consumes from the remaining arguments:
 *   'station'  a station id, required   (which node the command is ABOUT; --target says who to ask)
 *   'scope'    an optional scope word, in that command's own vocabulary
 */
const COMMANDS = {
    // --- the system TLVs: one request each --------------------------------------------------
    'vers': { summary: 'request VERSION — device firmware, platform, build, serial' },
    'vari': { summary: 'request VARIANT — iotdata variant suite produced' },
    'ctrl': { summary: 'request CONTROL — supported control operations' },
    'stat': { summary: 'request STATUS — operating status (system, network)', takes: 'scope', scopes: 'node | mesh | node,mesh   (default: every group)' },
    'conf': { summary: 'request CONFIG — tbd' },
    'diag': { summary: 'request DIAGNOSTICS — diagnostic operations and recordings' },
    'cont': { summary: 'request CONTENT — tbd' },
    'reports': { summary: 'request every report the node can produce, in type order' },

    // --- generic system control ---------------------------------------------------------------
    'boot': { summary: 'restart the node' },

    // --- the recorder -------------------------------------------------------------------------
    'diag-enable': { summary: 'enable recording' },
    'diag-disable': { summary: 'disable recording' },
    'diag-clear': { summary: 'clear recordings' },
    'diag-dump': { summary: 'dump recordings (to console)' },

    // --- mesh management ----------------------------------------------------------------------
    'mesh-stations': { summary: 'request the stations table' },
    'mesh-stations-dump': { summary: 'print the table on the node\'s console' },
    'mesh-peers': { summary: 'request the peers table' },
    'mesh-peers-update': { summary: 'update one peer in the table', takes: 'station+action', actions: 'remove | none' },
    'mesh-peers-clear': { summary: 'remove all peers (forcing re-discovery)' },
    'mesh-peers-dump': { summary: 'print the table on the node\'s console' },
    'mesh-filters': { summary: 'request the filters tabl' },
    'mesh-filters-update': { summary: 'update one filter in the table', takes: 'station+action', actions: 'block | allow | none' },
    'mesh-filters-clear': { summary: 'remove all filters', takes: 'scope', scopes: 'all | manual | auto   (default: all)' },
    'mesh-filters-dump': { summary: 'print the table on the node\'s console' },

    // --- compatibility ------------------------------------------------------------------------
    'status': { summary: 'the MESH group of STATUS' },

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
    /* An alias is sent as typed: the gateway knows both vocabularies, so there is one place that
       maps a name to a CONTROL key rather than two that must agree. The alias table here is for
       help and validation. */
    const spec = COMMANDS[name];
    if (!spec) throw new Error(`unknown command '${name}' (try --help)`);
    const req = { cmd: name, target: opts.target };
    if (spec.takes === 'station') req.station = parseStation(args[0]);
    else if (spec.takes === 'station+action') {
        req.station = parseStation(args[0]);
        if (args[1] !== undefined) req.action = args[1];
    } else if (spec.takes === 'scope' && args[0] !== undefined) req.scope = args[0];
    return req;
}

// ------------------------------------------------------------------------------------------------------------------------

function usage() {
    display.log('iotdata_gateway_command.js - drive iotdata node CONTROL commands over MQTT\n');
    display.log('Usage: iotdata_gateway_command.js [options] <command> [args]\n');
    display.log('Options:');
    display.log(`  --broker <url>   MQTT broker      (default: ${DEFAULTS.broker}, or $MQTT_BROKER)`);
    display.log(`  --prefix <p>     topic prefix     (default: ${DEFAULTS.prefix}, or $IOTDATA_PREFIX)`);
    display.log('  --target <t>     all | broadcast | <station id: 1..4094 or 0x..>   (default: all)');
    display.log(`  --watch [secs]   after sending, print live telemetry for N seconds (default ${DEFAULTS.watch})`);
    display.log('  --definitions <hdr>  blackbox record header (with // @blackbox tag=...); while watching,');
    display.log('                   convert recognised record lines (e.g. a blackbox-dump reply) CSV -> JSON');
    display.log('  --dry-run | -n   print the request that would be sent, do not connect');
    display.log('  --verbose | --debug | --help\n');
    display.log('Commands  (spaced or hyphenated: `mesh peers clear`):');
    for (const [name, c] of Object.entries(COMMANDS)) {
        display.log(`  ${name.padEnd(18)} ${c.summary}`);
        if (c.takes === 'station') display.log(`  ${''.padEnd(18)}   takes <station>`);
        if (c.takes === 'station+action') display.log(`  ${''.padEnd(18)}   takes <station> <action>`);
        if (c.actions) display.log(`  ${''.padEnd(18)}   action: ${c.actions}`);
        if (c.scopes) display.log(`  ${''.padEnd(18)}   scope: ${c.scopes}`);
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

    display.verbose(`connecting to ${opts.broker}`);
    const client = mqtt.connect(opts.broker);

    client.on('error', (e) => {
        display.err(`error: mqtt: ${e.message}`);
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
        display.log(`watching ${watchTopic} for ${opts.watch}s  (responses arrive on ${opts.prefix}/manage/resp as JSON; table dumps go to the target's own console)`);
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
