// add to data/configuration.yaml:
// external_converters:
//  - nrf-switch.js

const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const ota = require('zigbee-herdsman-converters/lib/ota');
const extend = require('zigbee-herdsman-converters/lib/extend');
const e = exposes.presets;
const ea = exposes.access;

const utils = require('zigbee-herdsman-converters/lib/utils');

// Adapted from fz.xiaomi_multistate_action
// Legacy switch implementation (fake multistate endpoint)
const multistate_action = {
        cluster: 'genMultistateInput',
        type: ['attributeReport'],
        convert: (model, msg, publish, options, meta) => {
            const action = msg.data['presentValue'];
            return {button: action};
        },
    };

// New switch implementation
// "action" and "button" were used because they appear in
// lib/state.ts:dontCacheProperties

// To get separate MQTT messages on button press + release:
// zigbee2mqtt/0xf4ce... {"action":0,"button":0,"linkquality":47}
// zigbee2mqtt/0xf4ce... {"action":1,"button":0,"linkquality":47}
//
// Reporting tab. Endpoint 10..15 (button 0..5). Cluster genBinaryInput.
// Attribute presentValue. Min rep interval 0. Apply
// Button 0 is special: if held, it resets the device
const binary_input_action = {
        cluster: 'genBinaryInput',
        type: ['attributeReport'],
        convert: (model, msg, publish, options, meta) => {
            const state = msg.data['presentValue'];
            return {action: state, button: parseInt(msg.endpoint.ID) - 10};
        },
    };

// To get a single MQTT message on each button click:
// zigbee2mqtt/0xf4ce... {"action":"click","button":1,"linkquality":47}
//
// Bind tab. Endpoint 10..15 (button 0..5). Destination Coordinator,
// endpoint 1. Clusters: OnOff. Bind
const toggle_cmd_action = {
        cluster: 'genOnOff',
        type: ['commandToggle'],
        convert: (model, msg, publish, options, meta) => {
            return {action: "click", button: parseInt(msg.endpoint.ID) - 10};
        },
    };

const definition = {
    zigbeeModel: ['Light_Switch'],
    model: 'Light_Switch',
    vendor: 'Nordic',
    description: 'Light switch remote',
    fromZigbee: [multistate_action, binary_input_action, toggle_cmd_action],
    toZigbee: [],
    ota: ota.zigbeeOTA,
    exposes: [],
};

module.exports = definition;
