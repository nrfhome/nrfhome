const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const ota = require('zigbee-herdsman-converters/lib/ota');
const extend = require('zigbee-herdsman-converters/lib/extend');
const e = exposes.presets;
const ea = exposes.access;

const utils = require('zigbee-herdsman-converters/lib/utils');

// These mimic entries from zigbee-herdsman/converters/toZigbee.js
const tz_serial_commands = [{
    key: ['serial_command'],
    convertSet: async (entity, key, value, meta) => {
        await entity.command('manuSpecificMultifunctionAV',
                             'serial_command',
                             {'string': value},
                             utils.getOptions(meta.mapped, entity));
    },
}, {
    key: ['start_serial_repeat'],
    convertSet: async (entity, key, value, meta) => {
        await entity.command('manuSpecificMultifunctionAV',
                             'start_serial_repeat',
                             {'string': value},
                             utils.getOptions(meta.mapped, entity));
    },
}, {
    key: ['stop_serial_repeat'],
    convertSet: async (entity, key, value, meta) => {
        await entity.command('manuSpecificMultifunctionAV',
                             'stop_serial_repeat',
                             {},
                             utils.getOptions(meta.mapped, entity));
    },
}, {
    key: ['serial_pause'],
    convertSet: async (entity, key, value, meta) => {
        await entity.command('manuSpecificMultifunctionAV',
                             'serial_pause',
                             {'milliseconds':parseInt(value)},
                             utils.getOptions(meta.mapped, entity));
    },
}];

const tz_ir_command = {
    key: ['ir_command'],
    convertSet: async (entity, key, value, meta) => {
        var s = value;
        var payload = {};

        var rep = s.match(/^([0-9]+)\*(.+)/)
        if (rep !== null) {
            payload.repeat = parseInt(rep[1]);
            s = rep[2];
        } else {
            payload.repeat = 1;
        }

        var dash = s.indexOf("-");
        var protocol = s.substr(0, dash).toLowerCase();

        if (protocol === "nec" || protocol === "nec1") {
            payload.protocol = 0x01;
        } else if (protocol === "nec2") {
            payload.protocol = 0x02;
        } else if (protocol === "necx1") {
            payload.protocol = 0x03;
        } else if (protocol === "necx2") {
            payload.protocol = 0x04;
        } else if (protocol === "pause") {
            payload.protocol = 0xff;
        } else {
            payload.protocol = 0x00;
        }

        payload.code = parseInt(s.substr(dash+1), 16);

        await entity.command('manuSpecificMultifunctionAV',
                             'ir_command',
                             payload,
                             utils.getOptions(meta.mapped, entity));
    },
};

const tz_rf_command = {
    key: ['rf_command'],
    convertSet: async (entity, key, value, meta) => {
        var s = value;
        var payload = {};

        var rep = s.match(/^([0-9]+)\*(.+)/)
        if (rep !== null) {
            payload.repeat = parseInt(rep[1]);
            s = rep[2];
        } else {
            payload.repeat = 1;
        }

        var dash = s.indexOf("-");
        var protocol = s.substr(0, dash).toLowerCase();

        if (protocol === "rhine") {
            payload.protocol = 0x01;
        } else if (protocol === "chungear12") {
            payload.protocol = 0x02;
        } else if (protocol === "chungear24") {
            payload.protocol = 0x03;
        } else if (protocol === "pause") {
            payload.protocol = 0xff;
        } else {
            payload.protocol = 0x00;
        }

        payload.code = parseInt(s.substr(dash+1), 16);

        await entity.command('manuSpecificMultifunctionAV',
                             'rf_command',
                             payload,
                             utils.getOptions(meta.mapped, entity));
    },
};

// Add "manuSpecificMultifunctionAV" to zigbee-herdsman/src/zcl/definition/cluster.ts

const definition = {
    zigbeeModel: ['Multifunction_AV_Controller'],
    model: 'Multifunction_AV_Controller',
    vendor: 'Nordic',
    description: 'Multifunction AV Controller',
    extend: extend.switch(),
    // extend, above, fills in the fz/tz defaults
    //fromZigbee: [fz.on_off, fz.ignore_basic_report],
    fromZigbee: [fz.electrical_measurement],
    toZigbee: extend.switch().toZigbee.concat(tz_serial_commands, [tz_ir_command, tz_rf_command]),
    ota: ota.zigbeeOTA,
    // Endpoint names must come from a whitelist; see utils.endpointNames[]
    exposes: [e.switch().withEndpoint('l1'),
              e.switch().withEndpoint('l2'),
              //new CommandString('serial_command'),
              exposes.text('serial_command', ea.SET),
              exposes.text('start_serial_repeat', ea.SET),
              exposes.text('stop_serial_repeat', ea.SET),
              exposes.numeric('serial_pause', ea.SET),
              exposes.text('ir_command', ea.SET),
              exposes.text('rf_command', ea.SET)],
    endpoint: (device) => {
        return {'l1': 10, 'l2': 11, 'default': 20};
    },
    meta: {multiEndpoint: true},
};

module.exports = definition;
