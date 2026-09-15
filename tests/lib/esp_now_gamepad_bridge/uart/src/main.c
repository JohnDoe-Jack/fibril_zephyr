/* SPDX-License-Identifier: Apache-2.0 */
#include <string.h>
#include <zephyr/data/cobs.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/serial/uart_emul.h>
#include <zephyr/sys/crc.h>
#include <zephyr/ztest.h>
#include <esp_now_gamepad_bridge/gamepad_bridge.h>

static const struct device *const uart = DEVICE_DT_GET(DT_NODELABEL(gamepad_uart));
static uint8_t wire[16];
static size_t wire_len;
static uint8_t sequence;

static int encoded(const uint8_t *buf, size_t len, void *user_data)
{
	ARG_UNUSED(user_data);
	memcpy(wire + wire_len, buf, len);
	wire_len += len;
	return 0;
}

static void make_frame(uint8_t seq, bool bad_crc)
{
	uint8_t decoded[] = {0xa1, 0x80, 0x10, 0x81, 0x7f, 0, 0xff, 127, 255, seq, 0};
	struct cobs_encoder encoder;

	decoded[10] = crc8(decoded, 10, 0x07, 0, false) ^ (bad_crc ? 1 : 0);
	wire_len = 0;
	zassert_ok(cobs_encoder_init(&encoder, encoded, NULL, COBS_FLAG_TRAILING_DELIMITER));
	zassert_equal(cobs_encoder_write(&encoder, decoded, sizeof(decoded)), sizeof(decoded));
	zassert_ok(cobs_encoder_close(&encoder));
}

static void inject(const uint8_t *buf, size_t len)
{
	zassert_equal(uart_emul_put_rx_data(uart, buf, len), len);
}

static struct gamepad_state wait_for_seq(uint8_t seq)
{
	struct gamepad_state state;
	int64_t deadline = k_uptime_get() + 80;

	do {
		zassert_ok(gamepad_bridge_get_state(&state));
		if (state.connected && state.seq == seq) {
			return state;
		}
		k_sleep(K_MSEC(1));
	} while (k_uptime_get() < deadline);
	zassert_true(false, "sequence %u did not arrive", seq);
	return state;
}

static void *setup(void)
{
	struct gamepad_state state;
	struct gamepad_bridge_stats stats;

	zassert_false(gamepad_bridge_is_connected());
	zassert_equal(gamepad_bridge_start(), -EACCES);
	zassert_equal(gamepad_bridge_get_state(&state), -EACCES);
	zassert_equal(gamepad_bridge_get_stats(&stats), -EACCES);
	zassert_equal(gamepad_bridge_init(NULL), -ENODEV);
	zassert_ok(gamepad_bridge_init(uart));
	zassert_equal(gamepad_bridge_init(uart), -EALREADY);
	zassert_ok(gamepad_bridge_get_state(&state));
	zassert_false(state.connected);
	zassert_ok(gamepad_bridge_start());
	zassert_equal(gamepad_bridge_start(), -EALREADY);
	return NULL;
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	const uint8_t delimiter = 0;
	inject(&delimiter, 1);
	make_frame(++sequence, false);
	inject(wire, wire_len);
	(void)wait_for_seq(sequence);
}

ZTEST(gamepad_uart, test_uart_configuration_and_snapshot_fields)
{
	struct uart_config config;
	struct gamepad_state state;

	zassert_ok(uart_config_get(uart, &config));
	zassert_equal(config.baudrate, 115200);
	zassert_equal(config.data_bits, UART_CFG_DATA_BITS_8);
	zassert_equal(config.stop_bits, UART_CFG_STOP_BITS_1);
	zassert_equal(config.parity, UART_CFG_PARITY_NONE);
	zassert_equal(config.flow_ctrl, UART_CFG_FLOW_CTRL_NONE);
	zassert_ok(gamepad_bridge_get_state(&state));
	zassert_equal(state.buttons, GAMEPAD_BUTTON_A | GAMEPAD_BUTTON_VIEW);
	zassert_equal(state.lx, -127);
	zassert_equal(state.ly, 127);
	zassert_equal(state.rx, 0);
	zassert_equal(state.ry, -1);
	zassert_equal(state.lt, 127);
	zassert_equal(state.rt, 255);
	zassert_true(gamepad_bridge_is_connected());
	zassert_equal(gamepad_bridge_get_state(NULL), -EINVAL);
	zassert_equal(gamepad_bridge_get_stats(NULL), -EINVAL);
}

ZTEST(gamepad_uart, test_bytewise_reception_and_buffer_rollovers)
{
	make_frame(++sequence, false);
	for (size_t i = 0; i < wire_len; i++) {
		inject(wire + i, 1);
		k_sleep(K_MSEC(2));
	}
	(void)wait_for_seq(sequence);
	uint8_t burst[256];
	size_t len = 0;
	for (size_t i = 0; i < 16; i++) {
		make_frame(++sequence, false);
		memcpy(burst + len, wire, wire_len);
		len += wire_len;
	}
	inject(burst, len);
	(void)wait_for_seq(sequence);
}

ZTEST(gamepad_uart, test_bad_frames_resynchronize_without_updating_state)
{
	struct gamepad_bridge_stats before_stats, after_stats;
	zassert_ok(gamepad_bridge_get_stats(&before_stats));
	const uint8_t malformed[] = {10, 1, 0};
	inject(malformed, sizeof(malformed));
	make_frame(sequence + 1, true);
	inject(wire, wire_len);
	k_sleep(K_MSEC(10));
	struct gamepad_state state;
	zassert_ok(gamepad_bridge_get_state(&state));
	zassert_equal(state.seq, sequence);
	make_frame(++sequence, false);
	inject(wire, wire_len);
	(void)wait_for_seq(sequence);
	zassert_ok(gamepad_bridge_get_stats(&after_stats));
	zassert_equal(after_stats.cobs_errors, before_stats.cobs_errors + 1);
	zassert_equal(after_stats.crc_errors, before_stats.crc_errors + 1);
}

ZTEST(gamepad_uart, test_duplicate_stream_times_out_and_new_sequence_recovers)
{
	struct gamepad_bridge_stats initial, final;
	zassert_ok(gamepad_bridge_get_stats(&initial));
	make_frame(sequence, false);
	for (size_t i = 0; i < 6; i++) {
		k_sleep(K_MSEC(20));
		inject(wire, wire_len);
	}
	zassert_false(gamepad_bridge_is_connected());
	struct gamepad_state state;
	zassert_ok(gamepad_bridge_get_state(&state));
	zassert_equal(state.buttons, 0);
	zassert_equal(state.lx | state.ly | state.rx | state.ry | state.lt | state.rt, 0);
	zassert_ok(gamepad_bridge_get_stats(&final));
	zassert_equal(final.timeouts, initial.timeouts + 1);
	make_frame(++sequence, false);
	inject(wire, wire_len);
	(void)wait_for_seq(sequence);
}

ZTEST(gamepad_uart, test_timeout_runs_without_application_polling)
{
	struct gamepad_bridge_stats initial, final;
	zassert_ok(gamepad_bridge_get_stats(&initial));
	k_sleep(K_MSEC(120));
	zassert_ok(gamepad_bridge_get_stats(&final));
	zassert_equal(final.timeouts, initial.timeouts + 1);
	zassert_false(gamepad_bridge_is_connected());
}

ZTEST(gamepad_uart, test_read_only_stats_and_no_transmitted_bytes)
{
	struct gamepad_bridge_stats first, second;
	zassert_ok(gamepad_bridge_get_stats(&first));
	zassert_ok(gamepad_bridge_get_stats(&second));
	zassert_mem_equal(&first, &second, sizeof(first));
	uint8_t tx[16];
	zassert_equal(uart_emul_get_tx_data(uart, tx, sizeof(tx)), 0);
}

#ifdef CONFIG_ESP_NOW_GAMEPAD_BRIDGE_ASYNC
ZTEST(gamepad_uart, test_disabled_async_receiver_restarts_and_resynchronizes)
{
	struct gamepad_bridge_stats initial, current;
	zassert_ok(gamepad_bridge_get_stats(&initial));
	zassert_ok(uart_rx_disable(uart));
	int64_t deadline = k_uptime_get() + 80;
	do {
		k_sleep(K_MSEC(1));
		zassert_ok(gamepad_bridge_get_stats(&current));
	} while (current.rx_restarts == initial.rx_restarts && k_uptime_get() < deadline);
	zassert_equal(current.rx_restarts, initial.rx_restarts + 1);
	const uint8_t delimiter = 0;
	inject(&delimiter, 1);
	make_frame(++sequence, false);
	inject(wire, wire_len);
	(void)wait_for_seq(sequence);
}
#else
ZTEST(gamepad_uart, test_irq_overrun_discards_damaged_frame_and_recovers)
{
	struct gamepad_bridge_stats initial, current;
	zassert_ok(gamepad_bridge_get_stats(&initial));
	uart_emul_set_errors(uart, UART_ERROR_OVERRUN);
	make_frame(++sequence, false);
	inject(wire, wire_len);
	k_sleep(K_MSEC(10));
	zassert_ok(gamepad_bridge_get_stats(&current));
	zassert_equal(current.uart_errors, initial.uart_errors + 1);
	make_frame(++sequence, false);
	inject(wire, wire_len);
	(void)wait_for_seq(sequence);
}
#endif

ZTEST_SUITE(gamepad_uart, NULL, setup, before, NULL, NULL);
