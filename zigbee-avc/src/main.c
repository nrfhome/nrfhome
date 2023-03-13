/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/reboot.h>

#include <nrfx_pwm.h>
#include <nrfx_saadc.h>
#include <nrfx_wdt.h>

#include <stdio.h>
#include <soc.h>

#include <zboss_api.h>
#include <zboss_api_addons.h>
#include "zb_mem_config_custom_max.h"

#include <zigbee/zigbee_app_utils.h>
#include <zigbee/zigbee_error_handler.h>
#include <zigbee/zigbee_zcl_scenes.h>
#include <zb_nrf_platform.h>

#if CONFIG_ZIGBEE_FOTA
#include <zigbee/zigbee_fota.h>
#include <zephyr/dfu/mcuboot.h>
#endif /* CONFIG_ZIGBEE_FOTA */

#define ZB_HA_DEFINE_DEVICE_ON_OFF_OUTPUT
#include "zb_ha_on_off_output.h"

#include <zcl/zb_zcl_el_measurement.h>

#include "cc1101.h"

#define EP_RELAY_0			10
#define EP_RELAY_1			11
#define EP_CUSTOM			20

#define BASIC_MANUF_NAME		"Nordic"
#define BASIC_MODEL_ID			"Multifunction_AV_Controller"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/* Main application customizable context.
 * Stores all settings and static values.
 */
typedef struct {
	zb_zcl_basic_attrs_ext_t         basic_attr;
	zb_zcl_identify_attrs_t          identify_attr;
	zb_zcl_scenes_attrs_t            scenes_attr;
	zb_zcl_groups_attrs_t            groups_attr;
	zb_zcl_on_off_attrs_t            relay_0_on_off_attr;
	zb_zcl_on_off_attrs_t            relay_1_on_off_attr;
} app_device_ctx_t;

static const struct gpio_dt_spec relay_gpios[] = {
		GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(relays), gpios, 0),
		GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(relays), gpios, 1),
};

#define STRAP_VALUE_AVC_IR		0x00
#define STRAP_VALUE_AVC_IR_NO_LED	0x01
#define STRAP_VALUE_CC1101		0x02
#define STRAP_VALUE_AC_SENSOR		0x04
static uint8_t strap_value = STRAP_VALUE_AVC_IR;

/* Zigbee device application context storage. */
static app_device_ctx_t dev_ctx;

ZB_ZCL_DECLARE_IDENTIFY_ATTRIB_LIST(
	identify_attr_list,
	&dev_ctx.identify_attr.identify_time);

ZB_ZCL_DECLARE_GROUPS_ATTRIB_LIST(
	groups_attr_list,
	&dev_ctx.groups_attr.name_support);

ZB_ZCL_DECLARE_SCENES_ATTRIB_LIST(
	scenes_attr_list,
	&dev_ctx.scenes_attr.scene_count,
	&dev_ctx.scenes_attr.current_scene,
	&dev_ctx.scenes_attr.current_group,
	&dev_ctx.scenes_attr.scene_valid,
	&dev_ctx.scenes_attr.name_support);

ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST_EXT(
	basic_attr_list,
	&dev_ctx.basic_attr.zcl_version,
	&dev_ctx.basic_attr.app_version,
	&dev_ctx.basic_attr.stack_version,
	&dev_ctx.basic_attr.hw_version,
	dev_ctx.basic_attr.mf_name,
	dev_ctx.basic_attr.model_id,
	dev_ctx.basic_attr.date_code,
	&dev_ctx.basic_attr.power_source,
	dev_ctx.basic_attr.location_id,
	&dev_ctx.basic_attr.ph_env,
	dev_ctx.basic_attr.sw_ver);

ZB_ZCL_DECLARE_DIAGNOSTICS_ATTRIB_LIST(
	diag_attr_list);

// Based on ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACPOWER_MULTIPLIER_ID,
//          ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACPOWER_DIVISOR_ID
#define ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACCURRENT_MULTIPLIER_ID(data_ptr) \
{                                                                                                 \
  ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACCURRENT_MULTIPLIER_ID,                                       \
  ZB_ZCL_ATTR_TYPE_U16,                                                                           \
  ZB_ZCL_ATTR_ACCESS_READ_ONLY,                                                                   \
  (void*) data_ptr                                                                           \
}

#define ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACCURRENT_DIVISOR_ID(data_ptr) \
{                                                                                              \
  ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACCURRENT_DIVISOR_ID,                                       \
  ZB_ZCL_ATTR_TYPE_U16,                                                                        \
  ZB_ZCL_ATTR_ACCESS_READ_ONLY,                                                                \
  (void*) data_ptr                                                                        \
}

// Based on ZB_ZCL_DECLARE_ELECTRICAL_MEASUREMENT_ATTRIB_LIST
#define ZB_ZCL_DECLARE_AC_MEASUREMENT_ATTRIB_LIST(attr_list, measurement_type, current, multiplier, divisor) \
  ZB_ZCL_START_DECLARE_ATTRIB_LIST_CLUSTER_REVISION(attr_list, ZB_ZCL_ELECTRICAL_MEASUREMENT)         \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_MEASUREMENT_TYPE_ID, (measurement_type))    \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMSCURRENT_ID, (current))                   \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACCURRENT_MULTIPLIER_ID, (multiplier))      \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACCURRENT_DIVISOR_ID, (divisor))            \
  ZB_ZCL_FINISH_DECLARE_ATTRIB_LIST

struct ac_measurement_attrs {
	zb_uint32_t measurement_type;
	zb_uint16_t rmscurrent;
	zb_uint16_t accurrent_multiplier;
	zb_uint16_t accurrent_divisor;
};

// Set divisor=1000 to indicate that we're reporting milliamps
static struct ac_measurement_attrs ac_measurement_attrs = {
	.measurement_type = ZB_ZCL_ELECTRICAL_MEASUREMENT_PHASE_A_MEASUREMENT,
	.rmscurrent = ZB_ZCL_ELECTRICAL_MEASUREMENT_RMSCURRENT_DEFAULT_VALUE,
	.accurrent_multiplier = 1,
	.accurrent_divisor = 1000,
};

// Let users set up reporting on rmsCurrent
#define AC_MEASUREMENT_REPORT_ATTR_COUNT	1

ZB_ZCL_DECLARE_AC_MEASUREMENT_ATTRIB_LIST(
	elec_attr_list,
	&ac_measurement_attrs.measurement_type,
	&ac_measurement_attrs.rmscurrent,
	&ac_measurement_attrs.accurrent_multiplier,
	&ac_measurement_attrs.accurrent_divisor);

#define CUSTOM_CLUSTER_ID_SERVER_ROLE_INIT	custom_cluster_init_server
#define CUSTOM_CLUSTER_ID_CLIENT_ROLE_INIT	custom_cluster_init_client

#define CUSTOM_CLUSTER_ID			0xfc00
#define CUSTOM_CLUSTER_MFR_CODE			0x1234

// Serial commands can be in the 0x0X range
// IR/RF commands can be in the 0x1X range
#define CUSTOM_CLUSTER_MASK			0xf0

#define CUSTOM_CLUSTER_CMD_SERIAL		0x00
#define CUSTOM_CLUSTER_CMD_IR			0x10
#define CUSTOM_CLUSTER_CMD_RF			0x18

static void pulse_activity_led(void);

zb_ret_t check_value_custom_server(zb_uint16_t attr_id, zb_uint8_t endpoint, zb_uint8_t *value)
{
	LOG_ERR("%s(0x%08x): not implemented", __func__, attr_id);
	return RET_OK;
}

#define UART_DATA_MAX				30
#define UART_RING_SIZE				8

#define UART_CMD_SEND_STRING			0x00
#define UART_CMD_REPEAT_STRING			0x01
#define UART_CMD_STOP_REPEAT			0x02
#define UART_CMD_PAUSE				0x03

// This should add up to exactly 32 bytes, because message queue data items
// are allocated in powers of 2
struct uart_command {
	zb_uint8_t cmd;
	zb_uint8_t len;
	union {
		zb_uint8_t data[UART_DATA_MAX];
		zb_uint16_t delay_ms;
	};
} __packed;
K_MSGQ_DEFINE(uart_msgq, sizeof(struct uart_command), UART_RING_SIZE, 4 /* align */);

struct uart_state {
	const struct device *device;
	struct k_timer uart_timer;

	struct k_spinlock lock;
	bool uart_running;
	struct uart_command current_command;
	zb_uint32_t repeated_count;
};
static struct uart_state uart_state;

static void uart_kick(struct uart_state *uart_state);

// 16000000 MHz / 400 = 40 kHz = 25us per symbol
#define FAN_PERIOD_TICKS			400
// Fan duty cycle is either 100% or 0%
#define FAN_MARK_SYMBOL				0x8190
#define FAN_SPACE_SYMBOL			0x8000

// https://fccid.io/CHQ7078T
#define RF_PROTOCOL_RHINE			0x01
// https://fccid.io/KUJCE10007
#define RF_PROTOCOL_CHUNGEAR12			0x02
// https://fccid.io/KUJCE10407
#define RF_PROTOCOL_CHUNGEAR24			0x03

#define RHINE_HEADER_PULSE			4
#define RHINE_SHORT_PULSE			3
#define RHINE_LONG_PULSE			7

#define RHINE_COMMAND_MS			25
// (3+1) * 25us = 100us
#define RHINE_REPEATS_PER_SYMBOL		3

#define CHUNGEAR_COMMAND_MS			40
// (14+1) * 25us = 375us
#define CHUNGEAR_REPEATS_PER_SYMBOL		14

// 16000000 MHz / 421 = 38004.75 Hz = 26.3125us per symbol
#define IR_PWMCLK				NRF_PWM_CLK_16MHz
#define IR_PERIOD_TICKS				421

// number of times to repeat the 26.3125us cycle per mark/space
// 1 symbol = 21*26.3125 = 552.5625us
#define IR_REPEATS_PER_SYMBOL			21

// total time for a NEC command
#define IR_COMMAND_MS				108

// 105/421 (1/16000000 sec) - duty 25%
// bit 15 is polarity: first edge in PWM period is falling
#define IR_MARK_SYMBOL				0x8069
#define IR_SPACE_SYMBOL				0x8000

#define IR_PROTOCOL_NEC1			0x01
#define IR_PROTOCOL_NEC2			0x02
#define IR_PROTOCOL_NECx1			0x03
#define IR_PROTOCOL_NECx2			0x04

#define XMIT_PROTOCOL_NOP			0x00
#define XMIT_PROTOCOL_PAUSE			0xff

#define MAX_XMIT_SYMBOLS			192
#define XMIT_RING_SIZE				8

struct xmit_command {
	zb_uint8_t protocol;
	zb_uint8_t repeat;
	zb_uint32_t command;
} __packed;
K_MSGQ_DEFINE(ir_msgq, sizeof(struct xmit_command), XMIT_RING_SIZE, 4 /* align */);
K_MSGQ_DEFINE(rf_msgq, sizeof(struct xmit_command), XMIT_RING_SIZE, 4 /* align */);

// Used to compute the pin number to send to non-GPIO drivers (PWM, UART, ...)
// (gpio0 base 0 OR gpio1 base 32) + pin offset
#define RAW_GPIO_PIN(dt_alias) \
	(DT_PROP(DT_GPIO_CTLR(DT_ALIAS(dt_alias), gpios), port) * 32 + \
	 DT_GPIO_PIN(DT_ALIAS(dt_alias), gpios))

struct xmit_state {
	struct k_msgq *msgq;
	const struct gpio_dt_spec *gpio;
	zb_uint32_t pwm_gpio_pin;
	nrfx_pwm_t pwm;

	bool is_rf;
	float rf_freq;
	struct k_work freq_change_work;

	nrf_pwm_values_common_t pwm_values[MAX_XMIT_SYMBOLS];
	int pwm_value_count;
	zb_uint16_t mark_symbol;
	zb_uint16_t space_symbol;
	zb_uint16_t repeats_per_symbol;
	zb_uint32_t command_repeat_interval;

	struct k_timer xmit_timer;

	struct k_spinlock lock;
	bool xmit_running;
	struct xmit_command current_command;
	zb_uint32_t repeated_count;
};
static struct xmit_state ir_state;
static struct xmit_state rf_state;

static void xmit_kick(struct xmit_state *state);

static void handle_serial_cmd(zb_uint8_t cmd, zb_uint8_t param)
{
	LOG_HEXDUMP_DBG(zb_buf_begin(param), zb_buf_len(param), "uart_arg");
	zb_uint8_t len = zb_buf_len(param);
	zb_uint8_t *buf = zb_buf_begin(param);

	struct uart_command entry;
	entry.cmd = cmd;

	switch (cmd) {
	case UART_CMD_SEND_STRING:
	case UART_CMD_REPEAT_STRING:
		if (len > (buf[0] + 1))
			len = buf[0];
		else
			len--;

		if (len > (UART_DATA_MAX - 1))
			len = UART_DATA_MAX - 1;
		entry.len = len + 1;
		memcpy(&entry.data, &buf[1], len);
		entry.data[len] = '\r';
		break;
	case UART_CMD_STOP_REPEAT:
		break;
	case UART_CMD_PAUSE:
		entry.delay_ms = *((zb_uint16_t *)buf);
		break;
	default:
		LOG_WRN("Unrecognized command: 0x%x", cmd);
		return;
	}

	if (k_msgq_put(&uart_msgq, &entry, K_NO_WAIT) != 0) {
		LOG_WRN("UART message queue overrun");
	} else {
		uart_kick(&uart_state);
	}
}

static void handle_xmit_cmd(zb_uint8_t param, zb_uint8_t ir_or_rf)
{
	struct xmit_command *ic = zb_buf_begin(param);
	LOG_HEXDUMP_DBG(ic, zb_buf_len(param), "xmit_arg");

	if (zb_buf_len(param) != sizeof(*ic)) {
		LOG_WRN("xmit command with wrong length: %d", zb_buf_len(param));
		return;
	}

	struct xmit_state *state = &ir_state;
	if (ir_or_rf == CUSTOM_CLUSTER_CMD_RF)
		state = &rf_state;

	if (k_msgq_put(state->msgq, ic, K_NO_WAIT) != 0) {
		LOG_WRN("xmit message queue overrun");
	} else {
		xmit_kick(state);
	}
}

static zb_bool_t process_custom_srv(zb_uint8_t param)
{
	if (param == ZB_ZCL_GENERAL_GET_CMD_LISTS_PARAM) {
		return ZB_FALSE;
	}

	zb_zcl_parsed_hdr_t cmd_info;
	ZB_ZCL_COPY_PARSED_HEADER(param, &cmd_info);

	if ((cmd_info.cmd_id & CUSTOM_CLUSTER_MASK) ==
	    CUSTOM_CLUSTER_CMD_SERIAL) {
		handle_serial_cmd(cmd_info.cmd_id, param);
	} else if (cmd_info.cmd_id == CUSTOM_CLUSTER_CMD_IR ||
		   cmd_info.cmd_id == CUSTOM_CLUSTER_CMD_RF) {
		handle_xmit_cmd(param, cmd_info.cmd_id);
	} else {
		// XXX is this right?
		return ZB_FALSE;
	}

	if (cmd_info.disable_default_response) {
		zb_buf_free(param);
	} else {
		ZB_ZCL_SEND_DEFAULT_RESP(param,
			ZB_ZCL_PARSED_HDR_SHORT_DATA(&cmd_info).source.u.short_addr,
			ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
			ZB_ZCL_PARSED_HDR_SHORT_DATA(&cmd_info).src_endpoint,
			ZB_ZCL_PARSED_HDR_SHORT_DATA(&cmd_info).dst_endpoint,
			cmd_info.profile_id,
			CUSTOM_CLUSTER_ID,
			cmd_info.seq_number,
			cmd_info.cmd_id,
			ZB_ZCL_STATUS_SUCCESS);
	}
	return ZB_TRUE;
}

void custom_cluster_init_server()
{
	zb_zcl_add_cluster_handlers(CUSTOM_CLUSTER_ID,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		check_value_custom_server,
		(zb_zcl_cluster_write_attr_hook_t)NULL,
		process_custom_srv);
}

void custom_cluster_init_client()
{
	k_oops();
}

zb_zcl_cluster_desc_t custom_clusters[] =
{
	ZB_ZCL_CLUSTER_DESC(CUSTOM_CLUSTER_ID,
		0,				/* attr_count */
		NULL,				/* attr_desc_list */
		ZB_ZCL_CLUSTER_SERVER_ROLE,	/* cluster_role_mask */
		CUSTOM_CLUSTER_MFR_CODE		/* manuf_code */
	),
	ZB_ZCL_CLUSTER_DESC(ZB_ZCL_CLUSTER_ID_DIAGNOSTICS,
		ZB_ZCL_ARRAY_SIZE(diag_attr_list, zb_zcl_attr_t),	/* attr_count */
		diag_attr_list,						/* attr_desc_list */
		ZB_ZCL_CLUSTER_SERVER_ROLE,				/* cluster_role_mask */
		ZB_ZCL_MANUF_CODE_INVALID				/* manuf_code */
	),
	ZB_ZCL_CLUSTER_DESC(ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
		ZB_ZCL_ARRAY_SIZE(elec_attr_list, zb_zcl_attr_t),	/* attr_count */
		elec_attr_list,						/* attr_desc_list */
		ZB_ZCL_CLUSTER_SERVER_ROLE,				/* cluster_role_mask */
		ZB_ZCL_MANUF_CODE_INVALID				/* manuf_code */
	),
};

ZB_DECLARE_SIMPLE_DESC(3, 0);
ZB_AF_SIMPLE_DESC_TYPE(3, 0) simple_desc_custom_ep =
{
	EP_CUSTOM,		/* endpoint */
	ZB_AF_HA_PROFILE_ID,	/* app_profile_id */
	ZB_HA_TEST_DEVICE_ID,	/* app_device_id */
	0,			/* app_device_version */
	0,			/* reserved */
	3,			/* app_input_cluster_count */
	0,			/* app_output_cluster_count */
	{			/* app_cluster_list[] */
		CUSTOM_CLUSTER_ID,
		ZB_ZCL_CLUSTER_ID_DIAGNOSTICS,
		ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
	}
};

ZBOSS_DEVICE_DECLARE_REPORTING_CTX(
	reporting_info_custom_ep,
	AC_MEASUREMENT_REPORT_ATTR_COUNT);

ZB_AF_DECLARE_ENDPOINT_DESC(custom_ep,
	EP_CUSTOM,		/* ep_id */
	ZB_AF_HA_PROFILE_ID,	/* profile_id */
	0,			/* unused */
	NULL,			/* unused */
	3,			/* cluster_number */
	custom_clusters,	/* cluster_list */
	(zb_af_simple_desc_1_1_t*)&simple_desc_custom_ep, /* simple_desc */
	AC_MEASUREMENT_REPORT_ATTR_COUNT,	/* rep_count */
	reporting_info_custom_ep,		/* rep_ctx */
	0,			/* lev_ctrl_count */
	NULL);			/* lev_ctrl_ctx */

/* relay 0 */

ZB_ZCL_DECLARE_ON_OFF_ATTRIB_LIST(
	relay_0_on_off_attr_list,
	&dev_ctx.relay_0_on_off_attr.on_off);

ZB_HA_DECLARE_ON_OFF_OUTPUT_CLUSTER_LIST(
	relay_0_on_off_clusters,
	relay_0_on_off_attr_list,
	basic_attr_list,
	identify_attr_list,
	groups_attr_list,
	scenes_attr_list);

ZB_HA_DECLARE_ON_OFF_OUTPUT_EP(
	relay_0_on_off_ep,
	EP_RELAY_0,
	relay_0_on_off_clusters);

/* relay 1 */

ZB_ZCL_DECLARE_ON_OFF_ATTRIB_LIST(
	relay_1_on_off_attr_list,
	&dev_ctx.relay_1_on_off_attr.on_off);

ZB_HA_DECLARE_ON_OFF_OUTPUT_CLUSTER_LIST(
	relay_1_on_off_clusters,
	relay_1_on_off_attr_list,
	basic_attr_list,
	identify_attr_list,
	groups_attr_list,
	scenes_attr_list);

// HACK: defining two of the same endpoints causes a redefinition
#undef ZB_DECLARE_SIMPLE_DESC
#define ZB_DECLARE_SIMPLE_DESC(...)

ZB_HA_DECLARE_ON_OFF_OUTPUT_EP(
	relay_1_on_off_ep,
	EP_RELAY_1,
	relay_1_on_off_clusters);

#ifndef CONFIG_ZIGBEE_FOTA
ZBOSS_DECLARE_DEVICE_CTX_3_EP(
	zigbee_device_ctx,
	relay_0_on_off_ep,
	relay_1_on_off_ep,
	custom_ep);
#else
extern zb_af_endpoint_desc_t zigbee_fota_client_ep;
ZBOSS_DECLARE_DEVICE_CTX_4_EP(
	zigbee_device_ctx,
	zigbee_fota_client_ep,
	relay_0_on_off_ep,
	relay_1_on_off_ep,
	custom_ep);
#endif

struct sw {
	const struct gpio_dt_spec *sw_gpio;
	struct k_work button_work;
	struct k_timer button_timer;
	bool timer_expired;
	struct gpio_callback button_cb;
};
static struct sw sw;

#define SW_RESET_DURATION_MS			2000

enum state_led_status {
	LEDS_OFF = 0,
	SOLID_RED,
	FAST_BLINK_RED,
	SLOW_BLINK_WHITE,
	FAST_BLINK_BLUE,
	SLOW_BLINK_PURPLE,
	SOLID_GREEN,
};

struct led_status {
	struct k_spinlock lock;
	bool is_resetting;
	bool reset_button_held;
	bool is_connected;
	bool is_paired;
	bool is_upgrading;

	enum state_led_status state;
	int counter;
	struct k_timer state_led_timer;

	struct k_timer activity_led_timer;

	const struct gpio_dt_spec *activity_gpio;
	const struct gpio_dt_spec *green_gpio;
	const struct gpio_dt_spec *red_gpio;
	const struct gpio_dt_spec *blue_gpio;
};
static struct led_status led_status;

static void button_pressed_zb_callback(zb_uint8_t param)
{
	k_spinlock_key_t key = k_spin_lock(&led_status.lock);
	led_status.is_resetting = true;
	k_spin_unlock(&led_status.lock, key);

	LOG_INF("Erasing NVRAM and then restarting...");
	zb_nvram_erase();
	k_sleep(K_SECONDS(2));
	sys_reboot(SYS_REBOOT_COLD);
}

static void button_pressed(const struct device *dev, struct gpio_callback *cb,
			   uint32_t pin_pos)
{
	if (!sw.timer_expired) {
		k_work_submit(&sw.button_work);
	}
}

static void button_pressed_worker(struct k_work *work)
{
	struct sw *sw = CONTAINER_OF(work, struct sw, button_work);

	int val = gpio_pin_get_dt(sw->sw_gpio);

	k_spinlock_key_t key = k_spin_lock(&led_status.lock);
	led_status.reset_button_held = !!val;
	k_spin_unlock(&led_status.lock, key);

	LOG_INF("User button state change: %d", val);

	if (sw->timer_expired) {
		ZB_SCHEDULE_APP_CALLBACK(button_pressed_zb_callback, 0);
		return;
	}

	if (val) {
		k_timer_start(&sw->button_timer,
			      K_MSEC(SW_RESET_DURATION_MS),
			      K_NO_WAIT);
	} else {
		k_timer_stop(&sw->button_timer);
	}
}

static void button_cnt_timer(struct k_timer *work)
{
	struct sw *button_sw = CONTAINER_OF(work, struct sw, button_timer);
	button_sw->timer_expired = true;
	k_work_submit(&button_sw->button_work);
}

static void activity_led_timeout(struct k_timer *work)
{
	gpio_pin_set_dt(led_status.activity_gpio, 0);
}

static void pulse_activity_led(void)
{
	if (strap_value == STRAP_VALUE_AVC_IR_NO_LED) {
		return;
	}

	gpio_pin_set_dt(led_status.activity_gpio, 1);
	k_timer_start(&led_status.activity_led_timer, K_MSEC(100), K_NO_WAIT);
}

static void configure_button(void)
{
	memset(&sw, 0, sizeof(sw));
	k_work_init(&sw.button_work, button_pressed_worker);
	k_timer_init(&sw.button_timer, button_cnt_timer, NULL);

	static const struct gpio_dt_spec sw_gpio =
		GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
	sw.sw_gpio = &sw_gpio;
	gpio_pin_configure_dt(&sw_gpio, GPIO_INPUT);

	// Special case: if the button is held down at boot time,
	// erase zb_nvram without starting Zigbee (in case NVRAM is
	// corrupt and causing ZBOSS assertions)
	k_sleep(K_MSEC(1));

	struct led_status *s = &led_status;
	uint32_t start_time = k_cycle_get_32();
	while (gpio_pin_get_dt(&sw_gpio)) {
		gpio_pin_set_dt(s->red_gpio, 1);

		uint32_t now = k_cycle_get_32();
		if (k_cyc_to_ms_floor32(now - start_time) > SW_RESET_DURATION_MS) {
			zb_nvram_erase();
			gpio_pin_set_dt(s->red_gpio, 0);
			k_sleep(K_SECONDS(2));
			sys_reboot(SYS_REBOOT_COLD);
		}
	}
	gpio_pin_set_dt(s->red_gpio, 0);

	gpio_pin_interrupt_configure_dt(&sw_gpio, GPIO_INT_EDGE_BOTH);
	gpio_init_callback(&sw.button_cb, button_pressed, BIT(sw_gpio.pin));
	gpio_add_callback(sw_gpio.port, &sw.button_cb);
}

static void configure_straps(void)
{
#if DT_NODE_EXISTS(DT_NODELABEL(strap_pins))
	const struct gpio_dt_spec straps[] = {
		GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(strap_pins), gpios, 0),
		GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(strap_pins), gpios, 1),
		GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(strap_pins), gpios, 2),
	};

	strap_value = 0;
	for (int i = 2; i >= 0; i--) {
		strap_value <<= 1;
		gpio_pin_configure_dt(&straps[i], GPIO_INPUT);
		k_sleep(K_MSEC(1));
		strap_value |= !!gpio_pin_get_dt(&straps[i]);
	}

	LOG_INF("HW strap value: 0x%02x", strap_value);
#endif
}

static void configure_gpio(void)
{
	gpio_pin_configure_dt(&relay_gpios[0], GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&relay_gpios[1], GPIO_OUTPUT_INACTIVE);

	k_timer_init(&led_status.activity_led_timer, activity_led_timeout, NULL);

	static const struct gpio_dt_spec activity_gpio =
		GPIO_DT_SPEC_GET(DT_ALIAS(led_activity), gpios);
	led_status.activity_gpio = &activity_gpio;
	gpio_pin_configure_dt(&activity_gpio, GPIO_OUTPUT_INACTIVE);

	static const struct gpio_dt_spec green_gpio =
		GPIO_DT_SPEC_GET(DT_ALIAS(led_state_green), gpios);
	led_status.green_gpio = &green_gpio;
	gpio_pin_configure_dt(&green_gpio, GPIO_OUTPUT_INACTIVE);

	static const struct gpio_dt_spec red_gpio =
		GPIO_DT_SPEC_GET(DT_ALIAS(led_state_red), gpios);
	led_status.red_gpio = &red_gpio;
	gpio_pin_configure_dt(&red_gpio, GPIO_OUTPUT_INACTIVE);

	static const struct gpio_dt_spec blue_gpio =
		GPIO_DT_SPEC_GET(DT_ALIAS(led_state_blue), gpios);
	led_status.blue_gpio = &blue_gpio;
	gpio_pin_configure_dt(&blue_gpio, GPIO_OUTPUT_INACTIVE);
}

static enum state_led_status calculate_led_state(struct led_status *s)
{
	if (s->is_resetting)
		return SOLID_RED;
	else if (s->reset_button_held)
		return FAST_BLINK_RED;
	else if (s->is_upgrading)
		return SLOW_BLINK_PURPLE;
	else if (s->is_connected)
		return SOLID_GREEN;
	else if (s->is_paired)
		return SLOW_BLINK_WHITE;
	else
		return FAST_BLINK_BLUE;
}

static bool update_state_led_gpios(struct led_status *s,
				   enum state_led_status state,
				   int counter)
{
	bool red, blue, green;
	bool reset_counter = false;
	unsigned int blink_period = 0;

	if (strap_value == STRAP_VALUE_AVC_IR_NO_LED) {
		return true;
	}

	switch (state) {
	case LEDS_OFF:
		red = blue = green = false;
		break;
	case SOLID_RED:
		red = true;
		blue = green = false;
		break;
	case SOLID_GREEN:
		green = true;
		blue = red = false;
		break;
	case SLOW_BLINK_WHITE:
		red = green = blue = true;
		blink_period = 20;
		break;
	case SLOW_BLINK_PURPLE:
		red = blue = true;
		green = false;
		blink_period = 20;
		break;
	case FAST_BLINK_RED:
		green = blue = false;
		red = true;
		blink_period = 2;
		break;
	case FAST_BLINK_BLUE:
		green = red = false;
		blue = true;
		blink_period = 2;
		break;
	}

	if (blink_period) {
		if (counter >= (blink_period / 2))
			red = blue = green = false;
		if (counter >= (blink_period - 1))
			reset_counter = true;
	}

	gpio_pin_set_dt(s->green_gpio, green);
	gpio_pin_set_dt(s->red_gpio, red);
	gpio_pin_set_dt(s->blue_gpio, blue);

	return reset_counter;
}

static void state_led_timeout(struct k_timer *work)
{
	k_spinlock_key_t key = k_spin_lock(&led_status.lock);
	enum state_led_status new_state = calculate_led_state(&led_status);
	k_spin_unlock(&led_status.lock, key);

	if (led_status.state != new_state)
		led_status.counter = 0;
	led_status.state = new_state;

	if (update_state_led_gpios(&led_status, new_state, led_status.counter))
		led_status.counter = 0;
	else
		led_status.counter++;
}

static void start_led_update_loop(void)
{
	led_status.state = LEDS_OFF;
	led_status.counter = 0;

	k_timer_init(&led_status.state_led_timer, state_led_timeout, NULL);
	k_timer_start(&led_status.state_led_timer, K_MSEC(100), K_MSEC(100));
}

static void on_off_set_value(zb_uint8_t endpoint, zb_bool_t on)
{
	LOG_INF("Set ON/OFF value: %d: %i", endpoint, on);

	ZB_ZCL_SET_ATTRIBUTE(
		endpoint,
		ZB_ZCL_CLUSTER_ID_ON_OFF,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
		(zb_uint8_t *)&on,
		ZB_FALSE);

	if (endpoint == EP_RELAY_0) {
		gpio_pin_set_dt(&relay_gpios[0], on);
	} else {
		gpio_pin_set_dt(&relay_gpios[1], on);
	}
}

/**@brief Function for initializing all clusters attributes.
 */
static void device_clusters_attr_init(void)
{
	/* Basic cluster attributes data */
	memset(&dev_ctx.basic_attr, 0, sizeof(zb_zcl_basic_attrs_ext_t));
	dev_ctx.basic_attr.zcl_version   = ZB_ZCL_VERSION;

	/* Use ZB_ZCL_SET_STRING_VAL to set strings, because the first byte
	 * should contain string length without trailing zero.
	 *
	 * For example "test" string wil be encoded as:
	 *   [(0x4), 't', 'e', 's', 't']
	 */
	ZB_ZCL_SET_STRING_VAL(
		dev_ctx.basic_attr.mf_name,
		BASIC_MANUF_NAME,
		ZB_ZCL_STRING_CONST_SIZE(BASIC_MANUF_NAME));

	ZB_ZCL_SET_STRING_VAL(
		dev_ctx.basic_attr.model_id,
		BASIC_MODEL_ID,
		ZB_ZCL_STRING_CONST_SIZE(BASIC_MODEL_ID));

	ZB_ZCL_SET_STRING_VAL(
		dev_ctx.basic_attr.date_code,
		CONFIG_DATE_CODE,
		ZB_ZCL_STRING_CONST_SIZE(CONFIG_DATE_CODE));

#ifdef CONFIG_ZIGBEE_FOTA
	ZB_ZCL_SET_STRING_VAL(
		dev_ctx.basic_attr.sw_ver,
		CONFIG_MCUBOOT_IMAGE_VERSION,
		ZB_ZCL_STRING_CONST_SIZE(CONFIG_MCUBOOT_IMAGE_VERSION));

	LOG_INF("FW version: %s (%s)", CONFIG_MCUBOOT_IMAGE_VERSION, CONFIG_DATE_CODE);
#endif

	dev_ctx.basic_attr.power_source = ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE;

	/* Identify cluster attributes data. */
	dev_ctx.identify_attr.identify_time =
		ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;

	/* On/Off cluster attributes data. */
	dev_ctx.relay_0_on_off_attr.on_off = (zb_bool_t)ZB_ZCL_ON_OFF_IS_OFF;
	dev_ctx.relay_1_on_off_attr.on_off = (zb_bool_t)ZB_ZCL_ON_OFF_IS_OFF;

	on_off_set_value(EP_RELAY_0, false);
	on_off_set_value(EP_RELAY_1, false);
}

/**@brief Callback function for handling ZCL commands.
 *
 * @param[in]   bufid   Reference to Zigbee stack buffer
 *                      used to pass received data.
 */
static void zcl_device_cb(zb_bufid_t bufid)
{
	zb_uint8_t cluster_id;
	zb_uint8_t attr_id;
	zb_zcl_device_callback_param_t  *device_cb_param =
		ZB_BUF_GET_PARAM(bufid, zb_zcl_device_callback_param_t);

	/* Set default response value. */
	device_cb_param->status = RET_OK;

	switch (device_cb_param->device_cb_id) {
#ifdef CONFIG_ZIGBEE_FOTA
	case ZB_ZCL_OTA_UPGRADE_VALUE_CB_ID:
		zigbee_fota_zcl_cb(bufid);
		break;
#endif
	case ZB_ZCL_SET_ATTR_VALUE_CB_ID:
		cluster_id = device_cb_param->cb_param.
			     set_attr_value_param.cluster_id;
		attr_id = device_cb_param->cb_param.
			  set_attr_value_param.attr_id;

		if (cluster_id == ZB_ZCL_CLUSTER_ID_ON_OFF) {
			uint8_t value =
				device_cb_param->cb_param.set_attr_value_param
				.values.data8;

			LOG_INF("on/off attribute setting to %hd", value);
			pulse_activity_led();
			if (attr_id == ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
				on_off_set_value(device_cb_param->endpoint,
						 (zb_bool_t)value);
			}
		} else {
			/* Other clusters can be processed here */
			LOG_INF("Unhandled cluster attribute id: %d",
				cluster_id);
		}
		break;

	default:
		LOG_INF("%s id %hd %d", __func__, device_cb_param->device_cb_id,
			device_cb_param->endpoint);

		if (zcl_scenes_cb(bufid) == ZB_FALSE) {
			device_cb_param->status = RET_ERROR;
		}
		break;
	}

	//LOG_INF("%s status: %hd", __func__, device_cb_param->status);
}

static void set_pairing_status(bool status)
{
	k_spinlock_key_t key = k_spin_lock(&led_status.lock);
	led_status.is_paired = status;
	k_spin_unlock(&led_status.lock, key);
}

#ifdef CONFIG_ZIGBEE_FOTA
static void confirm_image(void)
{
	if (!boot_is_img_confirmed()) {
		int ret = boot_write_img_confirmed();

		if (ret) {
			LOG_ERR("Couldn't confirm image: %d", ret);
		} else {
			LOG_INF("Marked image as OK");
		}
	}
}

static void ota_evt_handler(const struct zigbee_fota_evt *evt)
{
	static int failure_count = 0;

	switch (evt->id) {
	case ZIGBEE_FOTA_EVT_PROGRESS:
		//LOG_INF("OTA progress: %d%%", evt->dl.progress);
		led_status.is_upgrading = true;
		break;

	case ZIGBEE_FOTA_EVT_FINISHED:
		LOG_INF("Reboot application.");
		sys_reboot(SYS_REBOOT_COLD);
		break;

	case ZIGBEE_FOTA_EVT_ERROR:
		led_status.is_upgrading = false;

		failure_count++;
		LOG_ERR("OTA image transfer failed: attempt %d", failure_count);

		if (failure_count >= 3) {
			sys_reboot(SYS_REBOOT_COLD);
		}
		break;

	default:
		break;
	}
}
#endif /* CONFIG_ZIGBEE_FOTA */

/**@brief Zigbee stack event handler.
 *
 * @param[in]   bufid   Reference to the Zigbee stack buffer
 *                      used to pass signal.
 */
void zboss_signal_handler(zb_bufid_t bufid)
{
	zb_zdo_app_signal_hdr_t *sg_p = NULL;
	zb_zdo_app_signal_type_t  sig    = zb_get_app_signal(bufid, &sg_p);
	zb_ret_t status = ZB_GET_APP_SIGNAL_STATUS(bufid);
#if 0
	LOG_DBG("%s %d", __func__, sig);
#endif

#ifdef CONFIG_ZIGBEE_FOTA
	/* Pass signal to the OTA client implementation. */
	zigbee_fota_signal_handler(bufid);
#endif /* CONFIG_ZIGBEE_FOTA */

	/* Update network status LED. Logic is from zigbee_led_status_update() */
	switch (sig) {
	case ZB_BDB_SIGNAL_DEVICE_REBOOT:
		/* fall-through */
	case ZB_BDB_SIGNAL_STEERING:
		if (status == RET_OK) {
			set_pairing_status(true);
		} else {
			set_pairing_status(false);
		}
		break;

	case ZB_ZDO_SIGNAL_LEAVE:
		set_pairing_status(false);
		break;

	default:
		break;
	}

	if (sig == ZB_NLME_STATUS_INDICATION) {
		// Suppress "I: Unimplemented signal (signal: 50, status: 0)"
#if 0
		zb_zdo_signal_nlme_status_indication_params_t *nlme_status_ind =
			ZB_ZDO_SIGNAL_GET_PARAMS(sg_p, zb_zdo_signal_nlme_status_indication_params_t);
		LOG_DBG("NLME: 0x%x", nlme_status_ind->nlme_status.status);
#endif

		zb_buf_free(bufid);
		return;
	}

	/* No application-specific behavior is required.
	 * Call default signal handler.
	 */
	ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));

	/* All callbacks should either reuse or free passed buffers.
	 * If bufid == 0, the buffer is invalid (not passed).
	 */
	if (bufid) {
		zb_buf_free(bufid);
	}
}

static void uart_fetch_next_cmd_locked(struct uart_state *uart_state)
{
	uart_state->uart_running = false;

next_command:
	if (k_msgq_get(&uart_msgq, &uart_state->current_command, K_NO_WAIT) != 0) {
		return;
	}

	struct uart_command *c = &uart_state->current_command;
	switch (c->cmd) {
	case UART_CMD_SEND_STRING:
	case UART_CMD_REPEAT_STRING: {
		int rc = uart_tx(uart_state->device, c->data, c->len, 0);
		__ASSERT(rc == 0, "uart_tx failed");
		pulse_activity_led();
		break;
	}
	case UART_CMD_PAUSE:
		k_timer_start(&uart_state->uart_timer, K_MSEC(c->delay_ms), K_NO_WAIT);
		break;
	case UART_CMD_STOP_REPEAT:
	default:
		goto next_command;
	}

	uart_state->uart_running = true;
	uart_state->repeated_count = 0;
}

static void uart_kick(struct uart_state *uart_state)
{
	k_spinlock_key_t key = k_spin_lock(&uart_state->lock);
	if (!uart_state->uart_running)
		uart_fetch_next_cmd_locked(uart_state);
	k_spin_unlock(&uart_state->lock, key);
}

static void uart_repeat_again(struct uart_state *uart_state, bool is_timer)
{
	if (is_timer) {
		// timer expired, so send the string again
		struct uart_command *c = &uart_state->current_command;
		int rc = uart_tx(uart_state->device, c->data, c->len, 0);
		__ASSERT(rc == 0, "uart_tx failed");
		pulse_activity_led();
	} else {
		// TX finished, so schedule the timer again
		if (uart_state->repeated_count < 2) {
			uart_state->repeated_count++;
			k_timer_start(&uart_state->uart_timer, K_MSEC(300), K_NO_WAIT);
		} else {
			k_timer_start(&uart_state->uart_timer, K_MSEC(100), K_NO_WAIT);
		}
	}
}

static void uart_event_handler_common(struct uart_state *uart_state, bool is_timer)
{
	k_spinlock_key_t key = k_spin_lock(&uart_state->lock);
	__ASSERT(uart_state->uart_running, "Got event when UART wasn't running");

	// As long as no UART_CMD_STOP_REPEAT (or any other command, for that
	// matter) has arrived, then keep repeating
	if (uart_state->current_command.cmd == UART_CMD_REPEAT_STRING &&
	    k_msgq_num_used_get(&uart_msgq) == 0) {
		uart_repeat_again(uart_state, is_timer);
	} else {
		uart_fetch_next_cmd_locked(uart_state);
	}
	k_spin_unlock(&uart_state->lock, key);
}

static void uart_timer_handler(struct k_timer *work)
{
	struct uart_state *uart_state =
		CONTAINER_OF(work, struct uart_state, uart_timer);
	uart_event_handler_common(uart_state, true);
}

static void uart_callback_handler(const struct device *dev,
				  struct uart_event *evt,
				  void *user_data)
{
	struct uart_state *uart_state = user_data;
	uart_event_handler_common(uart_state, false);
}

static void uart_init(struct uart_state *uart_state)
{
	int rc;

	uart_state->device = DEVICE_DT_GET(DT_NODELABEL(uart1));
	__ASSERT(uart_state->device, "Can't find uart1");

	rc = uart_callback_set(uart_state->device, uart_callback_handler, uart_state);
	__ASSERT(rc == 0, "Can't configure uart1 for async operation");

	k_timer_init(&uart_state->uart_timer, uart_timer_handler, NULL);
}

static void pwm_append_symbol(struct xmit_state *state, bool is_mark, int repeat)
{
	nrf_pwm_values_common_t val =
		is_mark ? state->mark_symbol : state->space_symbol;

	__ASSERT_NO_MSG(repeat > 0);
	for (int i = 0; i < repeat; i++) {
		__ASSERT_NO_MSG(state->pwm_value_count < MAX_XMIT_SYMBOLS);
		state->pwm_values[state->pwm_value_count++] = val;
	}
}

static void rhine_append_bit(struct xmit_state *state, bool bit)
{
	if (bit) {
		pwm_append_symbol(state, false, RHINE_SHORT_PULSE);
		pwm_append_symbol(state, true, RHINE_LONG_PULSE);
	} else {
		pwm_append_symbol(state, false, RHINE_LONG_PULSE);
		pwm_append_symbol(state, true, RHINE_SHORT_PULSE);
	}
}

static void rhine_build_data_seq(struct xmit_state *state,
	struct xmit_command *ic)
{
	state->command_repeat_interval = RHINE_COMMAND_MS;
	state->repeats_per_symbol = RHINE_REPEATS_PER_SYMBOL;

	pwm_append_symbol(state, true, RHINE_HEADER_PULSE);
	pwm_append_symbol(state, false, RHINE_SHORT_PULSE);
	pwm_append_symbol(state, true, RHINE_LONG_PULSE);

	// address in bits 11:8 (aligned for ease of use)
	// command in bits 6:0
	int bit;
	for (bit = 11; bit >= 0; bit--) {
		if (bit == 7)
			continue;
		rhine_append_bit(state, !!(ic->command & (1UL << bit)));
	}

	pwm_append_symbol(state, false, 1);
}

static void chungear_build_data_seq(struct xmit_state *state,
	struct xmit_command *ic, int bits)
{
	state->command_repeat_interval = CHUNGEAR_COMMAND_MS;
	state->repeats_per_symbol = CHUNGEAR_REPEATS_PER_SYMBOL;

	uint32_t data = ic->command & ((1UL << bits) - 1);
	// top bit (12 or 24) is the preamble (always 0)
	for (int bit = bits; bit >= 0; bit--) {
		if (data & (1UL << bit)) {
			pwm_append_symbol(state, true, 1);
			pwm_append_symbol(state, true, 1);
			pwm_append_symbol(state, false, 1);
		} else {
			pwm_append_symbol(state, false, 1);
			pwm_append_symbol(state, true, 1);
			pwm_append_symbol(state, false, 1);
		}
	}
	pwm_append_symbol(state, false, 1);
}

static zb_ret_t fan_build_data_seq(struct xmit_state *state,
				   struct xmit_command *ic)
{
	state->mark_symbol = FAN_MARK_SYMBOL;
	state->space_symbol = FAN_SPACE_SYMBOL;

	switch (ic->protocol) {
	case RF_PROTOCOL_RHINE:
		rhine_build_data_seq(state, ic);
		break;
	case RF_PROTOCOL_CHUNGEAR12:
		chungear_build_data_seq(state, ic, 12);
		break;
	case RF_PROTOCOL_CHUNGEAR24:
		chungear_build_data_seq(state, ic, 24);
		break;
	default:
		pwm_append_symbol(state, true, 1);
		return RET_ERROR;
	}
	return RET_OK;
}

static zb_ret_t ir_build_data_seq(struct xmit_state *state,
				  struct xmit_command *ic)
{
	state->mark_symbol = IR_MARK_SYMBOL;
	state->space_symbol = IR_SPACE_SYMBOL;
	state->command_repeat_interval = IR_COMMAND_MS;
	state->repeats_per_symbol = IR_REPEATS_PER_SYMBOL;

	switch (ic->protocol) {
	case IR_PROTOCOL_NEC1:
	case IR_PROTOCOL_NEC2:
		pwm_append_symbol(state, true, 16);
		pwm_append_symbol(state, false, 8);
		break;
	case IR_PROTOCOL_NECx1:
	case IR_PROTOCOL_NECx2:
		pwm_append_symbol(state, true, 8);
		pwm_append_symbol(state, false, 8);
		break;
	default:
		pwm_append_symbol(state, true, 1);
		return RET_ERROR;
	}

	for (unsigned int bit = 0; bit < 32; bit++) {
		// send bits 24..31, then 16..23, then 8..15, then 0..7
		unsigned int pos = (31 - bit) ^ 0x7;
		if (ic->command & (1UL << pos)) {
			pwm_append_symbol(state, true, 1);
			pwm_append_symbol(state, false, 3);
		} else {
			pwm_append_symbol(state, true, 1);
			pwm_append_symbol(state, false, 1);
		}
	}

	pwm_append_symbol(state, true, 1);
	pwm_append_symbol(state, false, 1);

	return RET_OK;
}

static void xmit_send_command(struct xmit_state *state, struct xmit_command *ic)
{
	nrf_pwm_sequence_t seq;

	state->pwm_value_count = 0;
	if (state->is_rf) {
		fan_build_data_seq(state, ic);
	} else {
		ir_build_data_seq(state, ic);
	}

	seq.repeats = state->repeats_per_symbol;
	seq.length = state->pwm_value_count;
	seq.values.p_common = state->pwm_values;
	seq.end_delay = 0;

	nrfx_pwm_simple_playback(&state->pwm, &seq, 1, NRFX_PWM_FLAG_STOP);

	k_timer_start(&state->xmit_timer,
		K_MSEC(state->command_repeat_interval),
		K_NO_WAIT);
	state->xmit_running = true;

	pulse_activity_led();
}

static void rf_freq_change_worker(struct k_work *work)
{
	struct xmit_state *state =
		CONTAINER_OF(work, struct xmit_state, freq_change_work);

	cc1101_set_freq(state->rf_freq);

	k_spinlock_key_t key = k_spin_lock(&state->lock);
	xmit_send_command(state, &state->current_command);
	k_spin_unlock(&state->lock, key);
}

static bool rf_schedule_freq_change(struct xmit_state *state,
				    struct xmit_command *ic)
{
	if (!state->is_rf || strap_value != STRAP_VALUE_CC1101) {
		return false;
	}

	float freq;

	switch (ic->protocol) {
	case RF_PROTOCOL_RHINE:
		freq = 303.825;
		break;
	case RF_PROTOCOL_CHUNGEAR12:
		freq = 303.738;
		break;
	case RF_PROTOCOL_CHUNGEAR24:
		freq = 304.3;
		break;
	default:
		return false;
	}

	if (state->rf_freq == freq) {
		return false;
	}

	state->rf_freq = freq;
	k_work_submit(&state->freq_change_work);
	return true;
}

static void xmit_fetch_next_cmd_locked(struct xmit_state *state)
{
	state->xmit_running = false;

next_command:
	if (k_msgq_get(state->msgq, &state->current_command, K_NO_WAIT) != 0) {
		return;
	}

	struct xmit_command *c = &state->current_command;
	switch (c->protocol) {
	case XMIT_PROTOCOL_PAUSE:
		k_timer_start(&state->xmit_timer, K_MSEC(c->command), K_NO_WAIT);
		break;
	case XMIT_PROTOCOL_NOP:
		goto next_command;
	default:
		if (!rf_schedule_freq_change(state, &state->current_command)) {
			xmit_send_command(state, &state->current_command);
		}
	}

	state->xmit_running = true;
	state->repeated_count = 0;
}

static void xmit_kick(struct xmit_state *state)
{
	k_spinlock_key_t key = k_spin_lock(&state->lock);
	if (!state->xmit_running)
		xmit_fetch_next_cmd_locked(state);
	k_spin_unlock(&state->lock, key);
}

static void xmit_timer_handler_locked(struct xmit_state *state)
{
	if (state->current_command.repeat == 0 &&
	    k_msgq_num_used_get(state->msgq) == 0) {
		xmit_send_command(state, &state->current_command);
		return;
	}

	state->repeated_count++;
	if (state->current_command.repeat > state->repeated_count) {
		xmit_send_command(state, &state->current_command);
		return;
	}

	xmit_fetch_next_cmd_locked(state);
}

static void xmit_timer_handler(struct k_timer *work)
{
	struct xmit_state *state =
		CONTAINER_OF(work, struct xmit_state, xmit_timer);
	k_spinlock_key_t key = k_spin_lock(&state->lock);
	xmit_timer_handler_locked(state);
	k_spin_unlock(&state->lock, key);
}

static void common_pwm_init(struct xmit_state *state,
			    nrfx_pwm_config_t *config)
{
	config->output_pins[0] = state->pwm_gpio_pin;

	nrfx_err_t status = nrfx_pwm_init(&state->pwm, config, NULL, NULL);
	__ASSERT_NO_MSG(status == NRFX_SUCCESS);

	k_timer_init(&state->xmit_timer, xmit_timer_handler, NULL);
}

#define EMPTY_PWM_CONFIG NRFX_PWM_DEFAULT_CONFIG( \
	NRFX_PWM_PIN_NOT_USED, \
	NRFX_PWM_PIN_NOT_USED, \
	NRFX_PWM_PIN_NOT_USED, \
	NRFX_PWM_PIN_NOT_USED)

static void ir_pwm_init(struct xmit_state *state)
{
	static nrfx_pwm_config_t config = EMPTY_PWM_CONFIG;
	config.base_clock = IR_PWMCLK;
	config.top_value = IR_PERIOD_TICKS;

	static const struct gpio_dt_spec gpio =
		GPIO_DT_SPEC_GET(DT_ALIAS(led_ir), gpios);

	state->msgq = &ir_msgq;
	state->pwm.p_registers = NRF_PWM0;
	state->pwm.drv_inst_idx = NRFX_PWM0_INST_IDX;

	state->gpio = &gpio;
	gpio_pin_configure_dt(state->gpio, GPIO_OUTPUT_INACTIVE);
	state->pwm_gpio_pin = RAW_GPIO_PIN(led_ir);

	common_pwm_init(state, &config);
}

static void rf_pwm_init(struct xmit_state *state)
{
	static nrfx_pwm_config_t config = EMPTY_PWM_CONFIG;
	config.base_clock = IR_PWMCLK;
	config.top_value = FAN_PERIOD_TICKS;

	state->msgq = &rf_msgq;
	state->pwm.p_registers = NRF_PWM1;
	state->pwm.drv_inst_idx = NRFX_PWM1_INST_IDX;

#if DT_NODE_EXISTS(DT_NODELABEL(rf_pin_cc1101))
	if (strap_value == STRAP_VALUE_CC1101) {
		static const struct gpio_dt_spec gpio =
			GPIO_DT_SPEC_GET(DT_NODELABEL(rf_pin_cc1101), gpios);
		state->gpio = &gpio;
		state->pwm_gpio_pin = RAW_GPIO_PIN(rf_pin_cc1101);
	} else {
#else
	{
#endif
		static const struct gpio_dt_spec gpio =
			GPIO_DT_SPEC_GET(DT_NODELABEL(rf_pin), gpios);
		state->gpio = &gpio;
		state->pwm_gpio_pin = RAW_GPIO_PIN(rf_pin);
	}
	gpio_pin_configure_dt(state->gpio, GPIO_OUTPUT_INACTIVE);

	state->is_rf = true;
	state->rf_freq = 0;
	k_work_init(&state->freq_change_work, rf_freq_change_worker);

	common_pwm_init(state, &config);
}

/*
 * We want to capture at least 2 full 50/60 Hz cycles = 40ms
 * Per the spec: Fsample < 1/(Tacq + Tconv)
 * Tacq is set by ADC_ACQ_TIME. Tconv is about 2us
 * If ADC_CC is 80 (fastest allowed), that sets the sampling rate to 200k/sec
 */
#define ADC_OVERSAMPLE		NRF_SAADC_OVERSAMPLE_16X
#define ADC_CC			80
#define ADC_N_SAMPLES		(40U * ((16000000 / ADC_CC) / 16) / 1000)
#define ADC_ACQ_TIME		NRF_SAADC_ACQTIME_3US

// Recalibrate every ~400s
#define CALIBRATION_INTERVAL	10000

struct adc_state {
	nrf_saadc_value_t adc_buffer[ADC_N_SAMPLES];
	struct k_work adc_work;
	int calibration_count;
	int prev_reading_mv;
};
static struct adc_state adc_state;

static void adc_event_handler(nrfx_saadc_evt_t const *evt_data);

static void adc_kickoff(struct adc_state *state)
{
	nrfx_err_t err;

	if (state->calibration_count == 0) {
		state->calibration_count = CALIBRATION_INTERVAL;
		err = nrfx_saadc_offset_calibrate(&adc_event_handler);
		__ASSERT_NO_MSG(err == NRFX_SUCCESS);
		return;
	} else {
		state->calibration_count--;
	}

	err = nrfx_saadc_buffer_set(state->adc_buffer, ADC_N_SAMPLES);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	err = nrfx_saadc_mode_trigger();
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);
}

static uint16_t adc_compute_peak2peak(struct adc_state *state, int start, int len)
{
	uint16_t min = 0xffff, max = 0x0;

	for (int i = start; i < (start+len); i++) {
		uint16_t sample = state->adc_buffer[i];
		// Negative values aren't supported here
		if (sample & 0x8000) {
			sample = 0;
		}
		if (sample > max) {
			max = sample;
		}
		if (sample < min) {
			min = sample;
		}
	}
	return max - min;
}

static void adc_update_attr(zb_uint8_t bufid, zb_uint16_t user_param)
{
	ZB_ZCL_SET_ATTRIBUTE(
		EP_CUSTOM,
		ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_RMSCURRENT_ID,
		(zb_uint8_t *)&user_param,
		ZB_FALSE);
}

static void adc_worker(struct k_work *work)
{
	struct adc_state *state =
		CONTAINER_OF(work, struct adc_state, adc_work);

	uint16_t p0 = adc_compute_peak2peak(state, 0, ADC_N_SAMPLES/2);
	uint16_t p1 = adc_compute_peak2peak(state, ADC_N_SAMPLES/2, ADC_N_SAMPLES/2);

	int delta;
	if (p0 >= p1) {
		delta = 100 * (p0 - p1) / p1;
	} else {
		delta = 100 * (p1 - p0) / p0;
	}

	// Vref is 0.6V, gain is 1/6, so the range is:
	//   0mV to 3600mV = 0x000 to 0xfff
	// Average both peak-to-peak readings, then convert to max amplitude,
	// then multiply by 0.707 (assuming a sine wave) for RMS millivolts
	uint16_t mv = (((p0 + p1) * 3600 / 0x1000) / 4) * 707 / 1000;

	// Discard as noise if the two samples are too far apart, or if
	// the previous reading is very close to the new reading
	if (delta <= 5) {
		int change_from_prev = (int)mv - state->prev_reading_mv;
		if (change_from_prev <= -10 || change_from_prev >= 10) {
			// 1000 mV RMS -> 20A
			const uint16_t ma = mv * 20;

			LOG_INF("ADC %d,%d (%d%%) -> %d mV -> %d mA",
				p0, p1, delta, mv, ma);

			state->prev_reading_mv = mv;
			ZB_SCHEDULE_APP_CALLBACK2(adc_update_attr,
				ZB_BUF_INVALID, ma);
		}
	}

	adc_kickoff(state);
}

static void adc_event_handler(nrfx_saadc_evt_t const *evt_data)
{
	if (evt_data->type == NRFX_SAADC_EVT_FINISHED) {
		k_work_submit(&adc_state.adc_work);
	} else if (evt_data->type == NRFX_SAADC_EVT_CALIBRATEDONE) {
		adc_kickoff(&adc_state);
	}
}

static void configure_adc(void)
{
	// https://devzone.nordicsemi.com/f/nordic-q-a/72487/how-to-use-nrfx-saadc-driver-on-zephyr-rtos

	nrfx_err_t err;

	k_work_init(&adc_state.adc_work, adc_worker);
	adc_state.prev_reading_mv = INT_MAX;

	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)),
		DT_IRQ(DT_NODELABEL(adc), priority),
		nrfx_saadc_irq_handler, NULL, 0);

	err = nrfx_saadc_init(NRFX_SAADC_DEFAULT_CONFIG_IRQ_PRIORITY);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	static nrfx_saadc_channel_t config =
		NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN0, 0);
	config.channel_config.acq_time = ADC_ACQ_TIME;

	err = nrfx_saadc_channel_config(&config);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	nrfx_saadc_adv_config_t saadc_adv_config =
		NRFX_SAADC_DEFAULT_ADV_CONFIG;
	saadc_adv_config.oversampling = ADC_OVERSAMPLE;
	saadc_adv_config.internal_timer_cc = ADC_CC;
	err = nrfx_saadc_advanced_mode_set(BIT(0) /* channel_mask */,
                                            NRF_SAADC_RESOLUTION_12BIT,
                                            &saadc_adv_config,
                                            adc_event_handler);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	adc_kickoff(&adc_state);
}

static nrfx_wdt_t wdt_instance = NRFX_WDT_INSTANCE(0);
static bool wdt_enabled = false;

static void configure_watchdog(void)
{
	static nrfx_wdt_channel_id channel;
	static nrfx_wdt_config_t conf = {
		.behaviour	= NRF_WDT_BEHAVIOUR_RUN_SLEEP,
		.reload_value	= 10000,
	};

	nrfx_err_t rc = nrfx_wdt_init(&wdt_instance, &conf, NULL);
	__ASSERT_NO_MSG(rc == NRFX_SUCCESS);

	rc = nrfx_wdt_channel_alloc(&wdt_instance, &channel);
	__ASSERT_NO_MSG(rc == NRFX_SUCCESS);

	nrfx_wdt_enable(&wdt_instance);
	nrfx_wdt_feed(&wdt_instance);
	wdt_enabled = true;
}

static void check_neighbors(zb_uint8_t bufid);

static void neighbor_check_cb(zb_uint8_t bufid)
{
	zb_nwk_nbr_iterator_params_t *args = ZB_BUF_GET_PARAM(bufid, zb_nwk_nbr_iterator_params_t);

	// FIXME: probably better not to count children of our router
	bool has_active_neighbors = args->index != ZB_NWK_NBR_ITERATOR_INDEX_EOT;

	k_spinlock_key_t key = k_spin_lock(&led_status.lock);
	led_status.is_connected = has_active_neighbors;
	k_spin_unlock(&led_status.lock, key);

	ZB_SCHEDULE_APP_ALARM(check_neighbors, bufid, ZB_MILLISECONDS_TO_BEACON_INTERVAL(1000));
}

static void check_neighbors(zb_uint8_t bufid)
{
	zb_ret_t error_code;

	if (bufid == ZB_BUF_INVALID) {
		error_code = zb_buf_get_out_delayed(check_neighbors);
		__ASSERT_NO_MSG(error_code == RET_OK);
		return;
	}

	// Pet the watchdog every time a ZBOSS buffer allocation succeeds
	if (wdt_enabled)
		nrfx_wdt_feed(&wdt_instance);

	zb_nwk_nbr_iterator_params_t *args = ZB_BUF_GET_PARAM(bufid, zb_nwk_nbr_iterator_params_t);
	args->update_count = 0;
	args->index = 0;

	error_code = zb_nwk_nbr_iterator_next(bufid, neighbor_check_cb);
	__ASSERT_NO_MSG(error_code == RET_OK);
}

static void schedule_neighbor_check(void)
{
	ZB_SCHEDULE_APP_CALLBACK(check_neighbors, ZB_BUF_INVALID);
}

#define IEEE_ADDR_BUF_SIZE       17

static void print_ieee_address(void)
{
	zb_ieee_addr_t device_addr;
	zb_osif_get_ieee_eui64(device_addr);

	char ieee_addr_buf[IEEE_ADDR_BUF_SIZE] = { 0 };
	int addr_len = ieee_addr_to_str(ieee_addr_buf,
				    sizeof(ieee_addr_buf),
				    device_addr);

	LOG_INF("Device is running: IEEE address 0x%s",
		addr_len ? ieee_addr_buf : "unknown");
}

void main(void)
{
	int err;

	LOG_INF("Starting multifunction A/V controller app...");

	/* Initialize */
	configure_watchdog();
	configure_straps();
	configure_gpio();
	configure_button();

	/* These use the same pin: P0.02 / AIN0 */
	if (strap_value != STRAP_VALUE_AC_SENSOR) {
		ir_pwm_init(&ir_state);
	}

	rf_pwm_init(&rf_state);
	uart_init(&uart_state);

	if (strap_value == STRAP_VALUE_CC1101) {
		cc1101_init();
	}

	err = settings_subsys_init();
	if (err) {
		LOG_ERR("settings initialization failed");
	}

	/* Register callback for handling ZCL commands. */
	ZB_ZCL_REGISTER_DEVICE_CB(zcl_device_cb);

#ifdef CONFIG_ZIGBEE_FOTA
	/* Initialize Zigbee FOTA download service. */
	zigbee_fota_init(ota_evt_handler);

	/* Mark the current firmware as valid. */
	confirm_image();
#endif /* CONFIG_ZIGBEE_FOTA */

	/* Register dimmer switch device context (endpoints). */
	ZB_AF_REGISTER_DEVICE_CTX(&zigbee_device_ctx);

	device_clusters_attr_init();

	/* Initialize ZCL scene table */
	zcl_scenes_init();

	/* Settings should be loaded after zcl_scenes_init */
	err = settings_load();
	if (err) {
		LOG_ERR("settings loading failed");
	}

	/* Start Zigbee default thread */
	zigbee_enable();

	start_led_update_loop();
	schedule_neighbor_check();

	k_sleep(K_SECONDS(1));
	print_ieee_address();

	/* Start the ADC loop after the Zigbee attributes are configured */
	if (strap_value == STRAP_VALUE_AC_SENSOR) {
		configure_adc();
	}
}
