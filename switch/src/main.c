/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 * @brief Dimmer switch for HA profile implementation.
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/reboot.h>

#include <ram_pwrdn.h>
#include <zboss_api.h>
#include <zboss_api_addons.h>
#include <zigbee/zigbee_app_utils.h>
#include <zigbee/zigbee_error_handler.h>
#include <zb_nrf_platform.h>
#include <zb_zcl_level_control.h>

#include "zb_mem_config_custom_max.h"

#define ZB_HA_DEFINE_DEVICE_DIMMER_SWITCH
#include "zb_ha_dimmer_switch.h"

#if CONFIG_ZIGBEE_FOTA
#include <zigbee/zigbee_fota.h>
#include <zephyr/dfu/mcuboot.h>

extern zb_af_endpoint_desc_t zigbee_fota_client_ep;
#endif /* CONFIG_ZIGBEE_FOTA */

#define MAX_INPUT_BUTTONS		6
#define BUTTON_ENDPOINT_BASE		10

// Hold down button0 for 8 seconds to reset the switch
#define RESET_BUTTON			0

/*
 * A lithium AAA cell is about 1200mAh = 4320C (Coulombs).
 * A CR123A is about 1500mAh = 5400C.
 * Steady state power consumption is ~1.5C per day.
 *
 * One single failed rejoin "cycle" consumes ~0.65C.  Repeating these
 * cycles over and over again would quickly drain the battery.
 *
 * So, to conserve battery life, zigbee_app_utils stops rejoin attempts
 * after ZB_DEV_REJOIN_TIMEOUT_MS if the network disappears (like during
 * a power outage).  It will not attempt another rejoin until
 * user_input_indicate() is called.
 *
 * Unfortunately that means that even many hours after the power outage is
 * over, the switch will be temporarily unresponsive to user input.
 * We can work around this by periodically making rejoin attempts, up
 * to a limit.
 */
#define REJOIN_INTERVAL_MS		(60 * 60 * 1000)
#define REJOIN_ATTEMPT_LIMIT		24

/* Do not erase NVRAM to save the network parameters after device reboot or
 * power-off. NOTE: If this option is set to ZB_TRUE then do full device erase
 * for all network devices before running other samples.
 */
#define ERASE_PERSISTENT_CONFIG    ZB_FALSE

struct sw {
	zb_uint8_t button_idx;
	const struct gpio_dt_spec *gpio;
	struct k_work button_pressed_work;
	struct k_work button_released_work;
	struct k_timer button_debounce_timer;
	struct k_timer button_press_timer;
	struct gpio_callback button_cb;
	uint64_t press_time;
};
static struct sw button_sw[MAX_INPUT_BUTTONS];

#if DT_NODE_EXISTS(DT_ALIAS(led0_green))
#define USE_LEDS		1
#else
#define USE_LEDS		0
#endif

#define LED_MASK_ACT		BIT(3)
#define LED_MASK_RED		BIT(2)
#define LED_MASK_GREEN		BIT(1)
#define LED_MASK_BLUE		BIT(0)

struct led_state {
	const struct gpio_dt_spec *act_gpio;
	const struct gpio_dt_spec *red_gpio;
	const struct gpio_dt_spec *green_gpio;
	const struct gpio_dt_spec *blue_gpio;

	struct k_timer led_timer;
};
static struct led_state led_state;
static bool is_joined = false;
static int rejoin_timeouts = 0;

static void led_pulse(struct led_state *state, zb_uint32_t mask);

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define BASIC_MANUF_NAME		"Nordic"
#define BASIC_MODEL_ID			"Light_Switch"

static zb_zcl_basic_attrs_ext_t basic_attr = {
	.zcl_version = ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
	.power_source = ZB_ZCL_BASIC_POWER_SOURCE_BATTERY,
};

static zb_uint16_t attr_identify_time;

/* Declare attribute list for Basic cluster. */
ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST_EXT(basic_attr_list,
	&basic_attr.zcl_version,
	&basic_attr.app_version,
	&basic_attr.stack_version,
	&basic_attr.hw_version,
	basic_attr.mf_name,
	basic_attr.model_id,
	basic_attr.date_code,
	&basic_attr.power_source,
	basic_attr.location_id,
	&basic_attr.ph_env,
	basic_attr.sw_ver);

/* Declare attribute list for Identify cluster. */
ZB_ZCL_DECLARE_IDENTIFY_ATTRIB_LIST(identify_attr_list, &attr_identify_time);

static zb_zcl_binary_input_attr_values_t binary_input_attr[MAX_INPUT_BUTTONS];

#define DECLARE_ONE_BUTTON_EP(id) \
	ZB_ZCL_DECLARE_BINARY_INPUT_ATTRIB_LIST(binary_input_attr_list_## id, \
		&binary_input_attr[id].out_of_service, \
		&binary_input_attr[id].present_value, \
		&binary_input_attr[id].status_flags); \
	ZB_HA_DECLARE_DIMMER_SWITCH_CLUSTER_LIST(dimmer_switch_clusters_## id, \
						 basic_attr_list, \
						 identify_attr_list, \
						 binary_input_attr_list_## id); \
	ZB_HA_DECLARE_DIMMER_SWITCH_EP(dimmer_switch_ep_## id, \
				       BUTTON_ENDPOINT_BASE + id, \
				       dimmer_switch_clusters_## id)

DECLARE_ONE_BUTTON_EP(0);

// HACK: defining two of the same endpoints causes a redefinition error
#undef ZB_DECLARE_SIMPLE_DESC
#define ZB_DECLARE_SIMPLE_DESC(...)

DECLARE_ONE_BUTTON_EP(1);
DECLARE_ONE_BUTTON_EP(2);
DECLARE_ONE_BUTTON_EP(3);
DECLARE_ONE_BUTTON_EP(4);
DECLARE_ONE_BUTTON_EP(5);

/*
 * Declare application's device context (list of registered endpoints)
 *
 * Expanded from the ZBOSS_DECLARE_DEVICE_CTX_4_EP macro in zboss_api_af.h
 * because we have more than 4 endpoints:
 */
ZB_AF_START_DECLARE_ENDPOINT_LIST(ep_list)
#ifdef CONFIG_ZIGBEE_FOTA
	&zigbee_fota_client_ep,
#endif
	&dimmer_switch_ep_0,
	&dimmer_switch_ep_1,
	&dimmer_switch_ep_2,
	&dimmer_switch_ep_3,
	&dimmer_switch_ep_4,
	&dimmer_switch_ep_5,
ZB_AF_FINISH_DECLARE_ENDPOINT_LIST;
ZBOSS_DECLARE_DEVICE_CTX(dimmer_switch_ctx, ep_list,
	(ZB_ZCL_ARRAY_SIZE(ep_list, zb_af_endpoint_desc_t*)));

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
	switch (evt->id) {
	case ZIGBEE_FOTA_EVT_PROGRESS:
		//dk_set_led(OTA_ACTIVITY_LED, evt->dl.progress % 2);
		break;

	case ZIGBEE_FOTA_EVT_FINISHED:
		LOG_INF("Reboot application.");
		sys_reboot(SYS_REBOOT_COLD);
		break;

	case ZIGBEE_FOTA_EVT_ERROR:
		LOG_ERR("OTA image transfer failed.");
		break;

	default:
		break;
	}
}
#endif /* CONFIG_ZIGBEE_FOTA */

static void device_clusters_attr_init(void)
{
	ZB_ZCL_SET_STRING_VAL(
		basic_attr.mf_name,
		BASIC_MANUF_NAME,
		ZB_ZCL_STRING_CONST_SIZE(BASIC_MANUF_NAME));

	ZB_ZCL_SET_STRING_VAL(
		basic_attr.model_id,
		BASIC_MODEL_ID,
		ZB_ZCL_STRING_CONST_SIZE(BASIC_MODEL_ID));

	ZB_ZCL_SET_STRING_VAL(
		basic_attr.date_code,
		CONFIG_DATE_CODE,
		ZB_ZCL_STRING_CONST_SIZE(CONFIG_DATE_CODE));

	for (int i = 0; i < MAX_INPUT_BUTTONS; i++) {
		binary_input_attr[i].out_of_service =
			ZB_ZCL_BINARY_INPUT_OUT_OF_SERVICE_DEFAULT_VALUE;
		binary_input_attr[i].present_value = ZB_FALSE;
		binary_input_attr[i].status_flags =
			ZB_ZCL_BINARY_INPUT_STATUS_FLAG_NORMAL;
	}

#ifdef CONFIG_ZIGBEE_FOTA
	ZB_ZCL_SET_STRING_VAL(
		basic_attr.sw_ver,
		CONFIG_MCUBOOT_IMAGE_VERSION,
		ZB_ZCL_STRING_CONST_SIZE(CONFIG_MCUBOOT_IMAGE_VERSION));

	LOG_INF("FW version: %s (%s)", CONFIG_MCUBOOT_IMAGE_VERSION, CONFIG_DATE_CODE);
#endif
}

/**@brief Zigbee stack event handler.
 *
 * @param[in]   bufid   Reference to the Zigbee stack buffer
 *                      used to pass signal.
 */
void zboss_signal_handler(zb_bufid_t bufid)
{
	zb_zdo_app_signal_hdr_t    *sig_hndler = NULL;
	zb_zdo_app_signal_type_t    sig = zb_get_app_signal(bufid, &sig_hndler);
	zb_ret_t                    status = ZB_GET_APP_SIGNAL_STATUS(bufid);

#ifdef CONFIG_ZIGBEE_FOTA
	/* Pass signal to the OTA client implementation. */
	zigbee_fota_signal_handler(bufid);
#endif /* CONFIG_ZIGBEE_FOTA */

	switch (sig) {
	case ZB_BDB_SIGNAL_DEVICE_REBOOT:
	/* fall-through */
	case ZB_BDB_SIGNAL_STEERING:
		/* Call default signal handler. */
		ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));

		if (status == RET_OK) {
			led_pulse(&led_state, LED_MASK_GREEN);
			is_joined = true;
			rejoin_timeouts = 0;
		} else {
			is_joined = false;
		}
		break;
	case ZB_BDB_SIGNAL_TC_REJOIN_DONE:
		ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
		if (status == RET_OK)
			break;
		if (rejoin_timeouts >= REJOIN_ATTEMPT_LIMIT)
			break;

		rejoin_timeouts++;
		ZB_ERROR_CHECK(ZB_SCHEDULE_APP_ALARM(
			(zb_callback_t)user_input_indicate, 0,
			ZB_MILLISECONDS_TO_BEACON_INTERVAL(
				REJOIN_INTERVAL_MS)));
		break;

	case ZB_ZDO_SIGNAL_LEAVE:
		/* Call default signal handler. */
		ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
		break;

	default:
		/* Call default signal handler. */
		ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
		break;
	}

	if (bufid) {
		zb_buf_free(bufid);
	}
}

#define FLAG_INITIAL_PRESS		0x0000
#define FLAG_LONG_PRESS			0x8000
#define FLAG_COLOR_TEMP			0x4000
#define FLAG_RELEASE			0x2000
#define FLAG_MASK			0xff00

#define NEUTRAL_MIRED			370

static void set_binary_input_attr(zb_uint16_t target_idx, zb_bool_t new_state)
{
	ZB_ZCL_SET_ATTRIBUTE(BUTTON_ENDPOINT_BASE + target_idx,
		ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
		(zb_uint8_t *)&new_state,
		ZB_FALSE);
}

static void invoke_light_switch_sender(zb_uint16_t idx_and_flags);

static void light_switch_send_on_off(zb_bufid_t bufid, zb_uint16_t idx_and_flags)
{
	// see https://devzone.nordicsemi.com/f/nordic-q-a/88820/zigbee-traces-use-the-wrong-log-level
	// "To send a packet to all matching devices in the binding table,
	// the correct approach would be..."
	const zb_uint16_t short_addr = 0;
	const zb_uint8_t endpoint = 0;
	const zb_uint8_t addr_mode = ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;

	zb_uint16_t target_idx = idx_and_flags & ~FLAG_MASK;

	if (idx_and_flags & FLAG_RELEASE) {
		set_binary_input_attr(target_idx, false);
		zb_buf_free(bufid);
		return;
	} else {
		set_binary_input_attr(target_idx, true);
	}

	if (idx_and_flags & FLAG_LONG_PRESS) {
		LOG_INF("button%d: send FULL BRIGHTNESS command", target_idx);

		ZB_ZCL_LEVEL_CONTROL_SEND_MOVE_TO_LEVEL_CMD(bufid,
			short_addr,
			addr_mode,
			endpoint,
			BUTTON_ENDPOINT_BASE + target_idx,
			ZB_AF_HA_PROFILE_ID,
			ZB_ZCL_DISABLE_DEFAULT_RESPONSE,
			NULL,
			ZB_ZCL_LEVEL_CONTROL_LEVEL_MAX_VALUE,
			0,
			ZB_ZCL_CMD_LEVEL_CONTROL_MOVE_TO_LEVEL_WITH_ON_OFF);

		// After setting full brightness, schedule another call
		// to switch to a neutral color temperature
		invoke_light_switch_sender(target_idx | FLAG_COLOR_TEMP);
	} else if (idx_and_flags & FLAG_COLOR_TEMP) {
		LOG_INF("button%d: send MOVE TO COLOR command", target_idx);

		ZB_ZCL_COLOR_CONTROL_SEND_MOVE_TO_COLOR_TEMPERATURE_REQ(bufid,
			short_addr,
			addr_mode,
			endpoint,
			BUTTON_ENDPOINT_BASE + target_idx,
			ZB_AF_HA_PROFILE_ID,
			ZB_ZCL_DISABLE_DEFAULT_RESPONSE,
			NULL,
			NEUTRAL_MIRED,
			0);
	} else {
		LOG_INF("button%d: send TOGGLE command", target_idx);

		ZB_ZCL_ON_OFF_SEND_TOGGLE_REQ(bufid,
			short_addr,
			addr_mode,
			endpoint,
			BUTTON_ENDPOINT_BASE + target_idx,
			ZB_AF_HA_PROFILE_ID,
			ZB_ZCL_DISABLE_DEFAULT_RESPONSE,
			NULL);
	}
	led_pulse(&led_state, LED_MASK_RED | LED_MASK_GREEN);
}

static void invoke_light_switch_sender(zb_uint16_t idx_and_flags)
{
	zb_ret_t zb_err_code = zb_buf_get_out_delayed_ext(
		light_switch_send_on_off,
		idx_and_flags,
		0);
	__ASSERT(zb_err_code == RET_OK, "error allocating ZBOSS buffer");
}

static void button_long_press_handler(struct k_timer *work)
{
	struct sw *sw = CONTAINER_OF(work, struct sw, button_press_timer);
	int64_t duration = k_uptime_get() - sw->press_time;

	if (duration >= 8000) {
		if (sw->button_idx == RESET_BUTTON) {
			sys_reboot(SYS_REBOOT_COLD);
		}
	} else if (duration >= 2000) {
		invoke_light_switch_sender(sw->button_idx | FLAG_LONG_PRESS);
		k_timer_start(&sw->button_press_timer, K_MSEC(6000), K_NO_WAIT);
	}
}

static void button_pressed_worker(struct k_work *work)
{
	struct sw *sw = CONTAINER_OF(work, struct sw, button_pressed_work);

	/* Inform default signal handler about user input at the device. */
	user_input_indicate();
	if (is_joined) {
		invoke_light_switch_sender(sw->button_idx | FLAG_INITIAL_PRESS);
	} else {
		led_pulse(&led_state, LED_MASK_RED);
	}
}

static void button_released_worker(struct k_work *work)
{
	struct sw *sw = CONTAINER_OF(work, struct sw, button_released_work);

	invoke_light_switch_sender(sw->button_idx | FLAG_RELEASE);
}

static void button_debounce_handler(struct k_timer *work)
{
	struct sw *sw = CONTAINER_OF(work, struct sw, button_debounce_timer);

	int val = gpio_pin_get_dt(sw->gpio);
	if (val) {
		sw->press_time = k_uptime_get();
		k_timer_start(&sw->button_press_timer, K_MSEC(2000), K_NO_WAIT);
		k_work_submit(&sw->button_pressed_work);
	} else {
		k_timer_stop(&sw->button_press_timer);
		k_work_submit(&sw->button_released_work);
	}
}

static void button_pressed(const struct device *dev, struct gpio_callback *cb,
			   uint32_t pin_pos)
{
	struct sw *sw = CONTAINER_OF(cb, struct sw, button_cb);

	// Wait a bit after the last transition, then sample the value to see
	// whether it was a press or release event
	k_timer_start(&sw->button_debounce_timer, K_MSEC(50), K_NO_WAIT);
}

static void configure_button(struct sw *sw,
			     zb_uint8_t button_idx,
			     const struct gpio_dt_spec *gpio)
{
	memset(sw, 0, sizeof(sw));

	sw->gpio = gpio;
	sw->button_idx = button_idx;

	k_work_init(&sw->button_pressed_work, button_pressed_worker);
	k_work_init(&sw->button_released_work, button_released_worker);
	k_timer_init(&sw->button_debounce_timer, button_debounce_handler, NULL);
	k_timer_init(&sw->button_press_timer, button_long_press_handler, NULL);

	gpio_pin_configure_dt(gpio, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(gpio, GPIO_INT_EDGE_BOTH);
	gpio_init_callback(&sw->button_cb, button_pressed, BIT(gpio->pin));
	gpio_add_callback(gpio->port, &sw->button_cb);
}

static void configure_buttons(void)
{
#if DT_NODE_EXISTS(DT_NODELABEL(usersw0))
	static const struct gpio_dt_spec usersw0 =
		GPIO_DT_SPEC_GET(DT_NODELABEL(usersw0), gpios);
	configure_button(&button_sw[0], 0, &usersw0);
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(usersw1))
	static const struct gpio_dt_spec usersw1 =
		GPIO_DT_SPEC_GET(DT_NODELABEL(usersw1), gpios);
	configure_button(&button_sw[1], 1, &usersw1);
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(usersw2))
	static const struct gpio_dt_spec usersw2 =
		GPIO_DT_SPEC_GET(DT_NODELABEL(usersw2), gpios);
	configure_button(&button_sw[2], 2, &usersw2);
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(usersw3))
	static const struct gpio_dt_spec usersw3 =
		GPIO_DT_SPEC_GET(DT_NODELABEL(usersw3), gpios);
	configure_button(&button_sw[3], 3, &usersw3);
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(usersw4))
	static const struct gpio_dt_spec usersw4 =
		GPIO_DT_SPEC_GET(DT_NODELABEL(usersw4), gpios);
	configure_button(&button_sw[4], 4, &usersw4);
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(usersw5))
	static const struct gpio_dt_spec usersw5 =
		GPIO_DT_SPEC_GET(DT_NODELABEL(usersw5), gpios);
	configure_button(&button_sw[5], 5, &usersw5);
#endif
}

static void led_pulse(struct led_state *state, zb_uint32_t mask)
{
#if USE_LEDS
	gpio_pin_set_dt(state->act_gpio, !!(mask & LED_MASK_ACT));
	gpio_pin_set_dt(state->red_gpio, !!(mask & LED_MASK_RED));
	gpio_pin_set_dt(state->green_gpio, !!(mask & LED_MASK_GREEN));
	gpio_pin_set_dt(state->blue_gpio, !!(mask & LED_MASK_BLUE));

	if (mask) {
		k_timer_start(&state->led_timer, K_MSEC(50), K_NO_WAIT);
	}
#endif
}

static void led_timeout(struct k_timer *work)
{
	struct led_state *state =
		CONTAINER_OF(work, struct led_state, led_timer);
	led_pulse(state, 0);
}

static void configure_leds(struct led_state *state)
{
	k_timer_init(&state->led_timer, led_timeout, NULL);

#if USE_LEDS
	static const struct gpio_dt_spec act_gpio =
		GPIO_DT_SPEC_GET(DT_ALIAS(led0_green), gpios);
	state->act_gpio = &act_gpio;
	gpio_pin_configure_dt(&act_gpio, GPIO_OUTPUT_INACTIVE);

	static const struct gpio_dt_spec red_gpio =
		GPIO_DT_SPEC_GET(DT_ALIAS(led1_red), gpios);
	state->red_gpio = &red_gpio;
	gpio_pin_configure_dt(&red_gpio, GPIO_OUTPUT_INACTIVE);

	static const struct gpio_dt_spec green_gpio =
		GPIO_DT_SPEC_GET(DT_ALIAS(led1_green), gpios);
	state->green_gpio = &green_gpio;
	gpio_pin_configure_dt(&green_gpio, GPIO_OUTPUT_INACTIVE);

	static const struct gpio_dt_spec blue_gpio =
		GPIO_DT_SPEC_GET(DT_ALIAS(led1_blue), gpios);
	state->blue_gpio = &blue_gpio;
	gpio_pin_configure_dt(&blue_gpio, GPIO_OUTPUT_INACTIVE);
#endif /* USE_LEDS */
}

#define SW_RESET_DURATION_MS			2000

static void reset_on_sw1(void)
{
#if USE_LEDS
	static const struct gpio_dt_spec sw0_gpio =
		GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
	gpio_pin_configure_dt(&sw0_gpio, GPIO_INPUT);

	static const struct gpio_dt_spec led_gpio =
		GPIO_DT_SPEC_GET(DT_ALIAS(led1_red), gpios);
	gpio_pin_configure_dt(&led_gpio, GPIO_OUTPUT_INACTIVE);

	// If the button is held down at boot time,
	// erase zb_nvram without starting Zigbee (in case NVRAM is
	// corrupt and causing ZBOSS assertions)
	k_sleep(K_MSEC(1));

	uint32_t start_time = k_uptime_get_32();
	while (gpio_pin_get_dt(&sw0_gpio)) {
		gpio_pin_set_dt(&led_gpio, 1);

		uint32_t now = k_uptime_get_32();
		if ((now - start_time) > SW_RESET_DURATION_MS) {
			zb_nvram_erase();
			gpio_pin_set_dt(&led_gpio, 0);
			k_sleep(K_SECONDS(2));
			sys_reboot(SYS_REBOOT_COLD);
		}
	}
	gpio_pin_set_dt(&led_gpio, 0);
#endif
}

void main(void)
{
	LOG_INF("Starting SED light switch");

	/* Initialize. */
	reset_on_sw1();
	configure_buttons();
	configure_leds(&led_state);

	zigbee_erase_persistent_storage(ERASE_PERSISTENT_CONFIG);
	zb_set_ed_timeout(ED_AGING_TIMEOUT_64MIN);

	// Defaults to 5s
	//zb_set_keepalive_timeout(ZB_MILLISECONDS_TO_BEACON_INTERVAL(3000));
	zigbee_configure_sleepy_behavior(true);
	power_down_unused_ram();

#ifdef CONFIG_ZIGBEE_FOTA
	/* Initialize Zigbee FOTA download service. */
	zigbee_fota_init(ota_evt_handler);

	/* Mark the current firmware as valid. */
	confirm_image();

	/* Register callback for handling ZCL commands. */
	ZB_ZCL_REGISTER_DEVICE_CB(zigbee_fota_zcl_cb);
#endif /* CONFIG_ZIGBEE_FOTA */

	/* Register dimmer switch device context (endpoints). */
	ZB_AF_REGISTER_DEVICE_CTX(&dimmer_switch_ctx);

	device_clusters_attr_init();

	/* Start Zigbee default thread. */
	zigbee_enable();

	LOG_INF("SED light switch started");

	while (1) {
		k_sleep(K_FOREVER);
	}
}
