/* SPDX-License-Identifier: Apache-2.0 */
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/ztest.h>

LOG_MODULE_REGISTER(rp2350_can_board);

BUILD_ASSERT(!DT_HAS_CHOSEN(zephyr_console), "UART must remain available to applications");
BUILD_ASSERT(!DT_HAS_CHOSEN(zephyr_shell_uart), "Shell must not claim UART0");
BUILD_ASSERT(!IS_ENABLED(CONFIG_UART_CONSOLE), "UART console must be disabled");
BUILD_ASSERT(!IS_ENABLED(CONFIG_CAN_FD_MODE), "XL2515 supports Classic CAN only");
BUILD_ASSERT(IS_ENABLED(CONFIG_LOG_BACKEND_RTT), "Logs must use RTT");
BUILD_ASSERT(IS_ENABLED(CONFIG_BUILD_OUTPUT_UF2), "UF2 output must be enabled");
BUILD_ASSERT(DT_SAME_NODE(DT_ALIAS(can0), DT_CHOSEN(zephyr_canbus)),
	     "CAN alias and chosen must refer to the same controller");
BUILD_ASSERT(DT_REG_SIZE(DT_CHOSEN(zephyr_flash)) == 4 * 1024 * 1024,
	     "Board has 4 MiB flash");

CAN_MSGQ_DEFINE(rx_queue, 2);
K_SEM_DEFINE(tx_done, 0, 1);
static int tx_status;

static void tx_callback(const struct device *dev, int error, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);
	tx_status = error;
	k_sem_give(&tx_done);
}

ZTEST(rp2350_can, test_uart0_polling_tx_rx_with_gp0_gp1_jumper)
{
	const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
	unsigned char rx = 0;
	int ret;

	zassert_true(device_is_ready(uart), "UART0 is not ready");
	for (int i = 0; i < 64 && uart_poll_in(uart, &rx) == 0; i++) {
	}
	uart_poll_out(uart, 0xa5);
	int64_t deadline = k_uptime_get() + 100;

	do {
		ret = uart_poll_in(uart, &rx);
		if (ret != -1) {
			break;
		}
		k_sleep(K_MSEC(1));
	} while (k_uptime_get() < deadline);

	zassert_ok(ret, "UART RX timed out; connect GP0 to GP1");
	zassert_equal(rx, 0xa5, "UART loopback byte differs");
}

ZTEST(rp2350_can, test_application_overlay_pwm_channels)
{
	const struct pwm_dt_spec outputs[] = {
		PWM_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0),
		PWM_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1),
		PWM_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 2),
		PWM_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 3),
	};

	for (size_t i = 0; i < ARRAY_SIZE(outputs); i++) {
		zassert_true(pwm_is_ready_dt(&outputs[i]), "PWM is not ready");
		zassert_ok(pwm_set_dt(&outputs[i], outputs[i].period, PWM_USEC(1500)));
		k_sleep(K_MSEC(100));
		zassert_ok(pwm_set_dt(&outputs[i], outputs[i].period, 0));
	}
}

ZTEST(rp2350_can, test_classic_can_standard_and_extended_loopback_at_1mbps)
{
	const struct device *can = DEVICE_DT_GET(DT_ALIAS(can0));
	can_mode_t capabilities;

	zassert_true(device_is_ready(can), "XL2515 is not ready");
	zassert_ok(can_get_capabilities(can, &capabilities));
	zassert_false(capabilities & CAN_MODE_FD, "XL2515 cannot support CAN FD");
	zassert_ok(can_set_bitrate(can, 1000000));
	zassert_ok(can_set_mode(can, CAN_MODE_LOOPBACK));
	zassert_ok(can_start(can));

	for (int extended = 0; extended < 2; extended++) {
		struct can_frame tx = {
			.id = extended ? 0x1abcde : 0x123,
			.flags = extended ? CAN_FRAME_IDE : 0,
			.dlc = 8,
			.data = {0, 1, 2, 3, 4, 5, 6, 7},
		};
		const struct can_filter filter = {
			.id = tx.id,
			.mask = extended ? CAN_EXT_ID_MASK : CAN_STD_ID_MASK,
			.flags = extended ? CAN_FILTER_IDE : 0,
		};
		struct can_frame rx;
		int filter_id = can_add_rx_filter_msgq(can, &rx_queue, &filter);

		zassert_true(filter_id >= 0, "Cannot install RX filter: %d", filter_id);
		/* can_send's timeout covers mailbox acquisition, not completion. */
		zassert_ok(can_send(can, &tx, K_MSEC(100), tx_callback, NULL));
		int ret = k_sem_take(&tx_done, K_MSEC(100));

		if (ret != 0) {
			can_remove_rx_filter(can, filter_id);
			(void)can_stop(can);
		}
		zassert_ok(ret, "CAN TX completion timed out; check GP8 IRQ");
		zassert_ok(tx_status, "CAN TX failed");
		zassert_ok(k_msgq_get(&rx_queue, &rx, K_MSEC(100)));
		can_remove_rx_filter(can, filter_id);
		zassert_equal(rx.id, tx.id);
		zassert_equal(rx.flags & CAN_FRAME_IDE, tx.flags);
		zassert_equal(rx.dlc, tx.dlc);
		zassert_mem_equal(rx.data, tx.data, sizeof(tx.data));
	}

	zassert_ok(can_stop(can));
	LOG_INF("Classic CAN standard/extended loopback passed at 1 Mbps");
}

ZTEST_SUITE(rp2350_can, NULL, NULL, NULL, NULL, NULL);
