#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/reboot.h>

#include <hal/nrf_ficr.h>
#include <hal/nrf_radio.h>
#include <nrfx_clock.h>

#define ACCESS_ADDRESS		0x8E89BED6UL
#define MAX_RX_BYTES		39

#define MAX_CMD_BYTES		64

#define TX_INTERVAL_MS		100

static const struct device *const usb_uart_dev =
	DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
K_MSGQ_DEFINE(cmd_msgq, MAX_CMD_BYTES, 4, 4);
static volatile bool uart_dtr;

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

static void radio_reset(void)
{
	nrf_radio_shorts_set(NRF_RADIO, 0);
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);

	nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_DISABLE);
	while (!nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_DISABLED));
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);

	nrf_radio_int_disable(NRF_RADIO, ~0);
	irq_disable(RADIO_IRQn);
}

uint32_t channel_to_freq(uint8_t channel)
{
	uint32_t freq;

	/* Special cases for the advertise channels */
	if(channel == 37)
		freq = 2;
	else if(channel == 38)
		freq = 26;
	else if(channel == 39)
		freq = 80;
	else
		freq = channel + (channel < 11 ? 2 : 3) * 2; // Spec Vol. 6, Part B, 1.4.1

	return 2400 + freq;
}

static uint8_t timebeacon_pkt[] = {
	// ADV_NONCONN_IND (0x02), length
	0x02, 0x18,
	// MAC address
	0x09, 0x09, 0x09, 0x09, 0x09, 0x09,
	// Service Data - 128 bit UUID (0x21)
	0x11, 0x07,
	// timestamp_ms in little endian format
	0x99, 0x88, 0x99, 0x88, 0x99, 0x88, 0x99, 0x88,
	// Our well-known UUID (hand-picked by /dev/random)
	0xfb, 0x2f, 0x8a, 0x92, 0x26, 0xea, 0x43, 0xe0,
};

static uint8_t rx_pkt_buf[MAX_RX_BYTES+1];
K_MSGQ_DEFINE(rx_msgq, MAX_RX_BYTES, 16, 4);

static uint32_t rx_dropped, rx_ok, rx_badcrc, rx_matches, tx_ok;

static bool radio_loop_running = false;
static int channel = 37;
static bool tx_mode;
static struct k_timer channel_change_timer;

static uint64_t last_timestamp_ms;
static int64_t last_timestamp_ktime;

/*
 * The loop should look like:
 *  - kickoff_tx():
 *    - configure RF channel
 *    - configure TX packet pointer
 *    - enable START_READY and END_DISABLE shortcuts
 *    - trigger TASK_TXEN
 *  - radio_interrupt() invoked on EVENT_DISABLED
 *    - call kickoff_rx():
 *      - configure RX packet pointer
 *      - enable START_READY shortcut
 *      - trigger TASK_RXEN
 *  - wait for EVENT_END (packet received)
 *          or EVENT_DISABLED (periodic channel_change timeout)
 *    -> radio_interrupt() again
 *    - if EVENT_END, process packet and trigger TASK_START
 *    - if EVENT_DISABLED, advance to the next channel and rerun kickoff_tx()
 */
static void kickoff_tx(void)
{
	tx_mode = true;

	nrf_radio_frequency_set(NRF_RADIO, channel_to_freq(channel));
	nrf_radio_datawhiteiv_set(NRF_RADIO, 0x40 | channel);

	nrf_radio_packetptr_set(NRF_RADIO, &timebeacon_pkt);
	nrf_radio_shorts_set(NRF_RADIO,
			     NRF_RADIO_SHORT_READY_START_MASK |
			     NRF_RADIO_SHORT_END_DISABLE_MASK);
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);

	uint64_t current_time = last_timestamp_ms;
	current_time += k_uptime_get() - last_timestamp_ktime;
	memcpy(&timebeacon_pkt[10], &current_time, 8);

	nrf_radio_int_disable(NRF_RADIO, ~0);
	nrf_radio_int_enable(NRF_RADIO, NRF_RADIO_INT_DISABLED_MASK);
	nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_TXEN);
	tx_ok++;
}

static void kickoff_rx(void)
{
	tx_mode = false;

	nrf_radio_packetptr_set(NRF_RADIO, &rx_pkt_buf);
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_END);
	nrf_radio_shorts_set(NRF_RADIO, NRF_RADIO_SHORT_READY_START_MASK);

	nrf_radio_int_disable(NRF_RADIO, ~0);
	nrf_radio_int_enable(NRF_RADIO, NRF_RADIO_INT_DISABLED_MASK |
					NRF_RADIO_INT_END_MASK);
	nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_RXEN);
}

static void process_rx_packet(void)
{
	__ASSERT_NO_MSG(rx_pkt_buf[MAX_RX_BYTES] == 0);
	if (nrf_radio_crc_status_check(NRF_RADIO)) {
		if (k_msgq_put(&rx_msgq, rx_pkt_buf, K_NO_WAIT) == 0) {
			rx_ok++;
		} else {
			rx_dropped++;
		}
	} else {
		rx_badcrc++;
	}
}

static void channel_change_handler(struct k_timer *work)
{
	// Periodically flip to a new channel and restart the cycle
	// The ISR will know what to do when it sees the DISABLED event
	nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_DISABLE);
}

static void radio_interrupt(void *unused)
{
	if (tx_mode) {
		if (nrf_radio_event_check(NRF_RADIO,
					  NRF_RADIO_EVENT_DISABLED)) {
			kickoff_rx();
		}
	} else {
		if (nrf_radio_event_check(NRF_RADIO,
					  NRF_RADIO_EVENT_DISABLED)) {
			if (++channel > 39) {
				channel = 37;
			}
			kickoff_tx();
			return;
		}
		if (nrf_radio_event_check(NRF_RADIO,
					  NRF_RADIO_EVENT_END)) {
			process_rx_packet();
			nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_END);
			nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_START);
		}
	}
}

static int radio_init(int channel)
{
	nrf_radio_packet_conf_t packet_conf;

	/* Turn off radio before configuring it */
	radio_reset();

	nrf_radio_txpower_set(NRF_RADIO, NRF_RADIO_TXPOWER_POS8DBM);
	nrf_radio_mode_set(NRF_RADIO, NRF_RADIO_MODE_BLE_1MBIT);

	/* Set the access address, address0/prefix0 used for both Rx and Tx
	 * address.
	 */
	nrf_radio_prefix0_set(NRF_RADIO, ACCESS_ADDRESS >> 24);
	nrf_radio_base0_set(NRF_RADIO, ACCESS_ADDRESS << 8);
	nrf_radio_rxaddresses_set(NRF_RADIO, RADIO_RXADDRESSES_ADDR0_Enabled);
	nrf_radio_txaddress_set(NRF_RADIO, 0x00);

	/* Configure CRC calculation. */
	nrf_radio_crcinit_set(NRF_RADIO, 0x00555555);
	nrf_radio_crc_configure(NRF_RADIO, RADIO_CRCCNF_LEN_Three,
				NRF_RADIO_CRC_ADDR_SKIP, 0x0000065B);

	memset(&packet_conf, 0, sizeof(packet_conf));

	packet_conf.s0len = 1;
	packet_conf.s1len = 0;
	packet_conf.lflen = 8;
	packet_conf.plen = NRF_RADIO_PREAMBLE_LENGTH_8BIT;
	packet_conf.whiteen = true;
	packet_conf.big_endian = false;
	packet_conf.balen = 3;
	packet_conf.statlen = 0;

	/* maxlen excludes S0/LENGTH/S1 fields */
	packet_conf.maxlen = MAX_RX_BYTES - 2;

	nrf_radio_packet_configure(NRF_RADIO, &packet_conf);

	nrf_radio_int_disable(NRF_RADIO, ~0);
	IRQ_CONNECT(RADIO_IRQn, IRQ_PRIO_LOWEST, radio_interrupt, NULL, 0);
	irq_enable(RADIO_IRQn);

	return 0;
}

struct led_status {
	struct k_timer activity_led_timer;

	const struct gpio_dt_spec *activity_gpio;
	const struct gpio_dt_spec *green_gpio;
	const struct gpio_dt_spec *red_gpio;
	const struct gpio_dt_spec *blue_gpio;
};
static struct led_status led_status;

static void activity_led_timeout(struct k_timer *work)
{
	gpio_pin_set_dt(led_status.activity_gpio, 0);
}

static void pulse_activity_led(void)
{
	if (led_status.activity_gpio) {
		gpio_pin_set_dt(led_status.activity_gpio, 1);
		k_timer_start(&led_status.activity_led_timer,
			      K_MSEC(100), K_NO_WAIT);
	}
}

static void set_stoplight_led(bool is_active, bool is_error)
{
	if (!led_status.green_gpio)
		return;

	if (is_error) {
		gpio_pin_set_dt(led_status.green_gpio, 1);
		gpio_pin_set_dt(led_status.red_gpio, 1);
	} else {
		gpio_pin_set_dt(led_status.green_gpio, !!is_active);
		gpio_pin_set_dt(led_status.red_gpio, !is_active);
	}
}

static void configure_leds(void)
{
	k_timer_init(&led_status.activity_led_timer,
		     activity_led_timeout, NULL);

#if DT_NODE_EXISTS(DT_ALIAS(led_activity))
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
#endif
}

static const uint8_t test_filter[] = { 0x03, 0x03, 0x88, 0xec }; // F030388EC
static volatile uint8_t rx_filter[MAX_RX_BYTES];

/*
 * Packet layout:
 *  00: PDU type
 *  01: length (doesn't include the first 2 bytes)
 *  02-07: advertising address
 *  08: advertising data
 *      (len, type, data...) tuples
 */
static bool match_pkt(uint8_t *p, uint8_t *filter)
{
	unsigned int bytes_left = p[1];

	if (bytes_left < 8 || bytes_left > (MAX_RX_BYTES - 2))
		return false;
	if (!filter[0])
		return false;

	bytes_left -= 6;
	uint8_t *cur = &p[8];

	// Try to match each payload element against the advertising data
	// in |filter|
	while (1)
	{
		if (bytes_left < 2)
			return false;

		unsigned int element_size = *cur + 1;
		if (element_size > bytes_left || element_size < 2)
			return false;

		// allow elements with more bytes than the filter, so
		// we can match Service Data elements like ours
		if (filter[0] > cur[0])
			return false;
		if (memcmp(cur+1, filter+1, filter[0]) == 0)
			return true;

		bytes_left -= element_size;
		cur += element_size;
	}
}

static void rx_processing_thread_fn(void)
{
	uint8_t rx_pkt[MAX_RX_BYTES];
	char rx_pkt_txt[MAX_RX_BYTES*2 + 3];

	while (1) {
		k_msgq_get(&rx_msgq, rx_pkt, K_FOREVER);
		if (match_pkt(rx_pkt, (uint8_t *)rx_filter)) {
			rx_matches++;

			int len = 1;
			rx_pkt_txt[0] = 'M';
			for (int i = 0; i < (rx_pkt[1] + 2); i++, len += 2) {
				sprintf(&rx_pkt_txt[1 + 2*i], "%02x",
					rx_pkt[i]);
			}
			rx_pkt_txt[len++] = '\r';
			rx_pkt_txt[len++] = '\n';

			if (uart_dtr) {
				uart_fifo_fill(usb_uart_dev, rx_pkt_txt, len);
				pulse_activity_led();
			}
		}
	}
}

K_THREAD_DEFINE(rx_processing_thread, 1024, rx_processing_thread_fn,
		NULL, NULL, NULL,
		K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);

static void stats_thread_fn(void)
{
	uint32_t last_tx_ok = 0;

	while (!radio_loop_running)
		k_sleep(K_SECONDS(1));

	while (1) {
		k_sleep(K_SECONDS(5));
		LOG_INF("rx_matches %d; rx_ok %d; tx_ok %d; rx_dropped %d; rx_badcrc %d",
			rx_matches, rx_ok, tx_ok, rx_dropped, rx_badcrc);

		// yellow LED if the radio loop froze
		if (tx_ok == last_tx_ok) {
			set_stoplight_led(true, true);
		}
		last_tx_ok = tx_ok;
	}
}

K_THREAD_DEFINE(stats_thread, 1024, stats_thread_fn,
		NULL, NULL, NULL,
		K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);

static uint8_t usb_uart_cmd[MAX_CMD_BYTES];
static int usb_uart_cursor;
static int64_t usb_uart_ktime;

static void usb_uart_rx_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		unsigned char byte;

		if (!uart_irq_rx_ready(dev)) {
			continue;
		}

		while (uart_fifo_read(dev, &byte, sizeof(byte))) {
			// ^U resets the buffer to a clean state
			if (!uart_dtr || byte == 0x15) {
				// discard all input
				usb_uart_cursor = 0;
				continue;
			}
			if (usb_uart_cursor == 0) {
				usb_uart_ktime = k_uptime_get();
			}
			if (byte == '\r' || byte == '\n') {
				if (usb_uart_cursor > 0 &&
				    usb_uart_cursor < MAX_CMD_BYTES)
				{
					int ret;
					ret = k_msgq_put(&cmd_msgq,
						usb_uart_cmd, K_NO_WAIT);
					__ASSERT_NO_MSG(ret == 0);
				}

				// ignore oversized or 0-byte commands
				memset(&usb_uart_cmd, 0, sizeof(usb_uart_cmd));
				usb_uart_cursor = 0;
			} else {
				if (usb_uart_cursor < MAX_CMD_BYTES) {
					usb_uart_cmd[usb_uart_cursor] = byte;
					usb_uart_cursor++;
				}
			}
		}
	}
}

static void start_radio_loop(void)
{
	if (radio_loop_running)
		return;

	radio_init(channel);
	kickoff_tx();
	k_timer_start(&channel_change_timer,
		K_MSEC(TX_INTERVAL_MS), K_MSEC(TX_INTERVAL_MS));
	LOG_INF("radio loop started on channel %d", channel);
	set_stoplight_led(true, false);
	radio_loop_running = true;
}

static int parse_hex_bytes(const char *in, uint8_t *out, int max_bytes)
{
	int len = 0;

	while (1) {
		if (!in[0])
			return len;
		if (!in[1])
			return -1;
		if (len >= max_bytes)
			return -1;

		char d[3] = { in[0], in[1], 0 };
		char *endp;
		unsigned long val = strtoul(d, &endp, 16);
		if (*endp != 0)
			return -1;

		*out = val;
		in += 2;
		out++;
		len++;
	}
}

static void error_response(void)
{
	char err[] = "E\r\n";
	uart_fifo_fill(usb_uart_dev, err, 3);
	set_stoplight_led(true, true);
}

static void update_timestamp(uint8_t *timestamp_bytes)
{
	uint64_t new_timestamp = 0;
	for (int i = 0; i < 8; i++) {
		new_timestamp <<= 8;
		new_timestamp |= timestamp_bytes[i];
	}

	unsigned int key = irq_lock();

	bool delta_valid = last_timestamp_ktime != 0;
	int64_t timestamp_delta = new_timestamp - last_timestamp_ms;
	int64_t ktime_delta = usb_uart_ktime - last_timestamp_ktime;

	last_timestamp_ms = new_timestamp;
	last_timestamp_ktime = usb_uart_ktime;

	irq_unlock(key);
	start_radio_loop();

	if (delta_valid) {
		LOG_INF("%s: delta is %lld", __func__,
			timestamp_delta - ktime_delta);
	}
}

static void process_usb_uart_cmd(const char *cmd)
{
	LOG_INF("%s: %s", __func__, cmd);

	int l = strlen(cmd);

	if (cmd[0] == 't') {
		// Test mode (for developers to enable the radio with a
		// simple command):
		// 1) Set a sensible default RX filter
		// 2) Mangle the TX UUID so it doesn't confuse "production"
		//    devices listening for the time
		memcpy((uint8_t *)rx_filter, test_filter, sizeof(test_filter));
		timebeacon_pkt[18] = 0x00;
		start_radio_loop();
		return;
	}

	if (cmd[0] == 'F') {
		if (parse_hex_bytes(&cmd[1],
				    (uint8_t *)rx_filter,
				    sizeof(rx_filter)) == -1) {
			error_response();
		}
		return;
	}

	if (cmd[0] == 'T') {
		// big endian 64bit millisecond timestamp
		// ex: T00000187307d3260
		uint8_t timestamp_bytes[8];

		if (l == 17 && parse_hex_bytes(&cmd[1],
				    timestamp_bytes,
				    sizeof(timestamp_bytes)) != -1) {
			update_timestamp(timestamp_bytes);
		} else {
			error_response();
		}
		return;
	}

	if (cmd[0] == 'R') {
		sys_reboot(SYS_REBOOT_COLD);
	}

	error_response();
}

static void uart_cmd_thread_fn(void)
{
	while (1) {
		uint8_t cmd[MAX_CMD_BYTES + 1] = {0};
		if (k_msgq_get(&cmd_msgq, cmd, K_FOREVER) == 0) {
			process_usb_uart_cmd(cmd);
		}
	}
}

K_THREAD_DEFINE(uart_cmd_thread, 1024, uart_cmd_thread_fn,
		NULL, NULL, NULL,
		K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);


// Based on https://devzone.nordicsemi.com/f/nordic-q-a/76344/retrieve-ble-mac-address-in-zephyr-environment
static void get_mac_address(uint8_t *mac)
{
	uint32_t device_addr_0 = NRF_FICR->DEVICEADDR[0];
	uint32_t device_addr_1 = NRF_FICR->DEVICEADDR[1];

	mac[0] = (device_addr_1 >> 8) | 0xc0;
	mac[1] = device_addr_1 >> 0;
	mac[2] = device_addr_0 >> 24;
	mac[3] = device_addr_0 >> 16;
	mac[4] = device_addr_0 >> 8;
	mac[5] = device_addr_0 >> 0;

	LOG_INF("BLE MAC: %02x:%02x:%02x:%02x:%02x:%02x",
		mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
}

void main(void)
{
	LOG_INF("Starting timebeacon app...");

	configure_leds();
	set_stoplight_led(false, false);

	nrfx_clock_start(NRF_CLOCK_DOMAIN_LFCLK);
	nrfx_clock_start(NRF_CLOCK_DOMAIN_HFCLK);

	get_mac_address(&timebeacon_pkt[2]);

	k_timer_init(&channel_change_timer, channel_change_handler, NULL);

	int ret;
	ret = device_is_ready(usb_uart_dev);
	__ASSERT_NO_MSG(ret == true);

	ret = usb_enable(NULL);
	__ASSERT_NO_MSG(ret == 0);

	// adapted from zephyr/samples/net/wpan_serial/src/main.c
	uart_irq_callback_set(usb_uart_dev, usb_uart_rx_handler);
	uart_irq_rx_enable(usb_uart_dev);

#if 1
	// FIXME: DTR detection is broken
	uart_dtr = true;
#else
	k_sleep(K_SECONDS(1));
	LOG_INF("Waiting for DTR");
	while (1) {
		uint32_t dtr = 0;
		uart_line_ctrl_get(usb_uart_dev, UART_LINE_CTRL_DTR, &dtr);

		if (dtr && !uart_dtr) {
			uart_dtr = true;
			LOG_INF("DTR detected");
		} else if (!dtr && uart_dtr) {
			set_stoplight_led(false, false);
			// DTR dropped -> stop UART TX/RX
			uart_dtr = false;
			rx_filter[0] = 0;
			LOG_INF("DTR dropped");
		} else {
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
	}
#endif
}
