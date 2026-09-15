/* SPDX-License-Identifier: Apache-2.0 */
#include <string.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/ztest.h>
#include <esp_now_gamepad_bridge/gamepad_bridge.h>

static uart_callback_t callback;
static void *callback_data;
static uint8_t *active_buffer;
static uint8_t *next_buffer;
static int configure_error;
static int callback_error;
static int response_error;
static atomic_t enable_error;
K_SEM_DEFINE(enable_attempt, 0, 10);

static int configure(const struct device *dev, const struct uart_config *config)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(config);
	return configure_error;
}

static int set_callback(const struct device *dev, uart_callback_t cb, void *data)
{
	ARG_UNUSED(dev);
	if (callback_error != 0) {
		return callback_error;
	}
	callback = cb;
	callback_data = data;
	return 0;
}

static int enable(const struct device *dev, uint8_t *buf, size_t len, int32_t timeout)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(len);
	ARG_UNUSED(timeout);
	int ret = atomic_get(&enable_error);

	if (ret == 0) {
		active_buffer = buf;
	}
	k_sem_give(&enable_attempt);
	return ret;
}

static int respond(const struct device *dev, uint8_t *buf, size_t len)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(len);
	if (response_error == 0) {
		next_buffer = buf;
	}
	return response_error;
}

static DEVICE_API(uart, fake_api) = {
	.configure = configure,
	.callback_set = set_callback,
	.rx_enable = enable,
	.rx_buf_rsp = respond,
};
DEVICE_DEFINE(fake_uart, "gamepad-fault-uart", NULL, NULL, NULL, NULL,
	      POST_KERNEL, 50, &fake_api);

static const struct device *const uart = DEVICE_GET(fake_uart);

static void event(enum uart_event_type type, uint8_t *buf)
{
	struct uart_event evt = {.type = type, .data.rx_buf.buf = buf};
	callback(uart, &evt, callback_data);
}

static struct gamepad_bridge_stats stats(void)
{
	struct gamepad_bridge_stats result;
	zassert_ok(gamepad_bridge_get_stats(&result));
	return result;
}

static void send_known_frame(void)
{
	const uint8_t wire[] = {7, 0xa1, 0x81, 0x40, 0x81, 0x7f, 0xff,
				1, 2, 0xff, 2, 0xae, 0};
	memcpy(active_buffer + 3, wire, sizeof(wire));
	struct uart_event evt = {
		.type = UART_RX_RDY,
		.data.rx = {.buf = active_buffer, .offset = 3, .len = sizeof(wire)},
	};
	callback(uart, &evt, callback_data);
}

ZTEST(gamepad_async_faults, test_api_failures_buffer_ownership_and_stopped_receiver_recovery)
{
	configure_error = -EINVAL;
	zassert_equal(gamepad_bridge_init(uart), -EINVAL);
	configure_error = 0;
	callback_error = -ENOSYS;
	zassert_equal(gamepad_bridge_init(uart), -ENOSYS);
	callback_error = 0;
	zassert_ok(gamepad_bridge_init(uart));
	atomic_set(&enable_error, -EIO);
	zassert_equal(gamepad_bridge_start(), -EIO);
	atomic_set(&enable_error, 0);
	zassert_ok(gamepad_bridge_start());
	zassert_equal(stats().uart_errors, 1);
	k_sem_reset(&enable_attempt);

	send_known_frame();
	zassert_true(gamepad_bridge_is_connected());
	zassert_equal(stats().frames_valid, 1);

	response_error = -ENOMEM;
	event(UART_RX_BUF_REQUEST, NULL);
	zassert_equal(stats().uart_errors, 2);
	response_error = 0;
	event(UART_RX_BUF_REQUEST, NULL);
	zassert_not_null(next_buffer);
	zassert_not_equal(next_buffer, active_buffer);
	uint8_t *released = active_buffer;
	event(UART_RX_BUF_RELEASED, released);
	active_buffer = next_buffer;
	event(UART_RX_BUF_REQUEST, NULL);
	zassert_equal(next_buffer, released);

	struct uart_event stopped = {
		.type = UART_RX_STOPPED,
		.data.rx_stop.reason = UART_ERROR_OVERRUN,
	};
	callback(uart, &stopped, callback_data);
	send_known_frame();
	zassert_equal(stats().frames_valid, 1);
	zassert_equal(stats().uart_errors, 3);
	event(UART_RX_BUF_RELEASED, active_buffer);
	event(UART_RX_BUF_RELEASED, next_buffer);
	atomic_set(&enable_error, -EIO);
	event(UART_RX_DISABLED, NULL);
	zassert_ok(k_sem_take(&enable_attempt, K_MSEC(80)));
	atomic_set(&enable_error, 0);
	zassert_ok(k_sem_take(&enable_attempt, K_MSEC(80)));
	int64_t deadline = k_uptime_get() + 80;
	while (stats().rx_restarts == 0 && k_uptime_get() < deadline) {
		k_sleep(K_MSEC(1));
	}
	zassert_equal(stats().rx_restarts, 1);
	zassert_true(stats().uart_errors >= 4);

	uint8_t delimiter = 0;
	active_buffer[0] = delimiter;
	struct uart_event boundary = {
		.type = UART_RX_RDY,
		.data.rx = {.buf = active_buffer, .offset = 0, .len = 1},
	};
	callback(uart, &boundary, callback_data);
	send_known_frame();
	zassert_equal(stats().frames_valid, 2);
	zassert_equal(stats().duplicate_frames, 1);
}

ZTEST_SUITE(gamepad_async_faults, NULL, NULL, NULL, NULL, NULL);
