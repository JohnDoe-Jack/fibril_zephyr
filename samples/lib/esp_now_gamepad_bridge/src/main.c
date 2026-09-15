/* SPDX-License-Identifier: Apache-2.0 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <esp_now_gamepad_bridge/gamepad_bridge.h>

LOG_MODULE_REGISTER(gamepad_sample, LOG_LEVEL_INF);

int main(void)
{
	const struct device *uart = DEVICE_DT_GET(DT_ALIAS(gamepad_uart));
	int ret = gamepad_bridge_init(uart);

	if (ret == 0) {
		ret = gamepad_bridge_start();
	}
	if (ret < 0) {
		LOG_ERR("Gamepad receiver startup failed (%d)", ret);
		return ret;
	}

	while (true) {
		struct gamepad_state state;
		struct gamepad_bridge_stats stats;

		ret = gamepad_bridge_get_state(&state);
		if (ret < 0) {
			return ret;
		}
		ret = gamepad_bridge_get_stats(&stats);
		if (ret < 0) {
			return ret;
		}
		LOG_INF("connected=%u seq=%u buttons=0x%04x L=(%d,%d) R=(%d,%d) LT=%u RT=%u",
			state.connected, state.seq, state.buttons, state.lx, state.ly,
			state.rx, state.ry, state.lt, state.rt);
		LOG_INF("valid=%u duplicate=%u dropped=%u timeout=%u UART errors=%u restarts=%u",
			stats.frames_valid, stats.duplicate_frames, stats.dropped_frames,
			stats.timeouts, stats.uart_errors, stats.rx_restarts);
		k_sleep(K_MSEC(200));
	}
}
