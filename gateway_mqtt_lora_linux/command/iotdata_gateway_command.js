#!/usr/bin/env node

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------
//
// iotdata_gateway_command.js - drive iotdata mesh MANAGE commands over MQTT.
//
// Publishes a JSON management request to <prefix>/manage/req; the gateway
// (iotdata_gateway_mqtt.h) turns it into a mesh MANAGE frame on air, addressed to one
// node or broadcast to all. The node executes the command — e.g. STATUS makes it dump
// its status line to its own USB console (the response is NOT on MQTT, so watch the
// relay console via esp32-deploy to see it).
//
// Extend COMMANDS as the management vocabulary grows (reboot, drop-parent, set-param,
// report/uplink-diagnostics, ...). Each command builds the JSON request; a matching
// dedicated builder is added on the C side (iotdata_mesh_pack_manage_*).
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

const opts = { broker: DEFAULTS.broker, prefix: DEFAULTS.prefix, target: DEFAULTS.target, watch: 0, dryRun: false, verbose: false, debug: false };

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
// Command registry — extend here (mirror each with a dedicated iotdata_mesh_pack_manage_* on the C side).
// build(args) returns the JSON request body; `target` is attached by buildRequest unless the command sets it.
// ------------------------------------------------------------------------------------------------------------------------

const COMMANDS = {
    'status': {
        summary: 'dump status + peers + filter to the node console',
        usage: 'status',
        build() {
            return { cmd: 'status' };
        },
    },
    'peers': {
        summary: 'dump the node peer (neighbour) table',
        usage: 'peers',
        build() {
            return { cmd: 'peers' };
        },
    },
    'stations': {
        summary: 'dump the "stations heard" table (mesh peers + sensors)',
        usage: 'stations',
        build() {
            return { cmd: 'stations' };
        },
    },
    'flush': {
        summary: 'clear the peer table (force re-discovery)',
        usage: 'flush',
        build() {
            return { cmd: 'peers-clear' };
        },
    },
    'peers-remove': {
        summary: 'forget one peer',
        usage: 'peers-remove <station>',
        build(a) {
            return { cmd: 'peers-remove', station: parseStation(a[0]) };
        },
    },
    'block': {
        summary: 'block RX from a station (blacklist) — e.g. block the gateway to force a hop',
        usage: 'block <station>',
        build(a) {
            return { cmd: 'block', station: parseStation(a[0]) };
        },
    },
    'allow': {
        summary: 'whitelist a station (when any exist, only allowed stations pass)',
        usage: 'allow <station>',
        build(a) {
            return { cmd: 'allow', station: parseStation(a[0]) };
        },
    },
    'unfilter': {
        summary: 'remove a station from the filter',
        usage: 'unfilter <station>',
        build(a) {
            return { cmd: 'unfilter', station: parseStation(a[0]) };
        },
    },
    'filters': {
        summary: 'dump the station filter table',
        usage: 'filters',
        build() {
            return { cmd: 'filters' };
        },
    },
    'filter-clear': {
        summary: 'clear the filter table (scope: all | manual | auto)',
        usage: 'filter-clear [scope]',
        build(a) {
            return a[0] ? { cmd: 'filter-clear', scope: a[0] } : { cmd: 'filter-clear' };
        },
    },
    'raw': {
        summary: 'send a raw JSON request (power user), e.g. raw \'{"cmd":"status"}\'',
        usage: 'raw <json>',
        build(args) {
            if (!args[0]) throw new Error('raw needs a JSON string argument');
            try {
                return JSON.parse(args[0]);
            } catch (e) {
                throw new Error('raw: invalid JSON: ' + e.message);
            }
        },
    },
    // future: reboot, drop-parent (force orphan), set-param, report (uplink diagnostics), ...
};

function buildRequest(name, args) {
    const command = COMMANDS[name];
    if (!command) throw new Error(`unknown command '${name}' (try: ${Object.keys(COMMANDS).join(', ')})`);
    const req = command.build(args);
    if (req.target === undefined) req.target = opts.target; // raw may set its own
    return req;
}

// ------------------------------------------------------------------------------------------------------------------------

function usage() {
    display.log('iotdata_gateway_command.js - drive iotdata mesh MANAGE commands over MQTT\n');
    display.log('Usage: iotdata_gateway_command.js [options] <command> [args]\n');
    display.log('Options:');
    display.log(`  --broker <url>   MQTT broker      (default: ${DEFAULTS.broker}, or $MQTT_BROKER)`);
    display.log(`  --prefix <p>     topic prefix     (default: ${DEFAULTS.prefix}, or $IOTDATA_PREFIX)`);
    display.log('  --target <t>     all | broadcast | <station id: 1..4094 or 0x..>   (default: all)');
    display.log(`  --watch [secs]   after sending, print live telemetry for N seconds (default ${DEFAULTS.watch})`);
    display.log('  --dry-run | -n   print the request that would be sent, do not connect');
    display.log('  --verbose | --debug | --help\n');
    display.log('Commands:');
    for (const [name, c] of Object.entries(COMMANDS)) display.log(`  ${name.padEnd(8)} ${c.summary}`);
}

// ------------------------------------------------------------------------------------------------------------------------

function parseArgv(argv) {
    const rest = [];
    for (let i = 0; i < argv.length; i++) {
        const a = argv[i];
        if (a === '--broker') opts.broker = argv[++i];
        else if (a === '--prefix') opts.prefix = argv[++i];
        else if (a === '--target') opts.target = parseTarget(argv[++i]);
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

    const [command, ...args] = rest;
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
        display.log(`watching ${watchTopic} for ${opts.watch}s  (node STATUS output is on the node console, not MQTT)`);
        client.on('message', (topic, msg) => display.log(`<- ${topic}  ${msg.toString()}`));
        await new Promise((resolve, reject) => client.subscribe(watchTopic, (e) => (e ? reject(e) : resolve()))).catch((e) => {
            display.err(`error: subscribe failed: ${e.message}`);
            process.exit(1);
        });
        await new Promise((resolve) => setTimeout(resolve, opts.watch * 1000));
    }

    client.end();
}

main();
