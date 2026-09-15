/* SPDX-License-Identifier: Apache-2.0 */
#include <string.h>
#include <zephyr/data/cobs.h>
#include <zephyr/sys/crc.h>
#include <zephyr/ztest.h>
#include "gamepad_protocol.h"

static struct gamepad_protocol protocol;
static uint8_t wire[64];
static size_t wire_len;

static int capture_encoded(const uint8_t *buf, size_t len, void *user_data)
{
	ARG_UNUSED(user_data);
	zassert_true(wire_len + len <= sizeof(wire));
	memcpy(wire + wire_len, buf, len);
	wire_len += len;
	return 0;
}

static void encode(const uint8_t *data, size_t len)
{
	struct cobs_encoder encoder;

	wire_len = 0;
	zassert_ok(cobs_encoder_init(&encoder, capture_encoded, NULL,
				    COBS_FLAG_TRAILING_DELIMITER));
	zassert_equal(cobs_encoder_write(&encoder, data, len), len);
	zassert_ok(cobs_encoder_close(&encoder));
}

static void make_frame(uint8_t seq)
{
	uint8_t decoded[] = {0xa1, 0x81, 0x40, 0x81, 0x7f, 0xff, 0, 0, 0xff, seq, 0};

	decoded[10] = crc8(decoded, 10, 0x07, 0, false);
	encode(decoded, sizeof(decoded));
}

static void feed_seq(uint8_t seq, int64_t now)
{
	make_frame(seq);
	gamepad_protocol_feed(&protocol, wire, wire_len, now);
}

static void assert_neutral(void)
{
	zassert_false(protocol.state.connected);
	zassert_equal(protocol.state.buttons, 0);
	zassert_equal(protocol.state.lx, 0);
	zassert_equal(protocol.state.ly, 0);
	zassert_equal(protocol.state.rx, 0);
	zassert_equal(protocol.state.ry, 0);
	zassert_equal(protocol.state.lt, 0);
	zassert_equal(protocol.state.rt, 0);
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	gamepad_protocol_init(&protocol, 100);
}

ZTEST(gamepad_protocol, test_initial_state_is_neutral_and_not_a_timeout)
{
	assert_neutral();
	gamepad_protocol_expire(&protocol, 1000);
	zassert_equal(protocol.stats.timeouts, 0);
}

ZTEST(gamepad_protocol, test_known_wire_vector_and_signed_axes)
{
	/* Fixed independently specified COBS/CRC vector, with zero fields. */
	const uint8_t frame[] = {7, 0xa1, 0x81, 0x40, 0x81, 0x7f, 0xff,
				 1, 2, 0xff, 2, 0xae, 0};
	gamepad_protocol_feed(&protocol, frame, sizeof(frame), 0);
	zassert_true(protocol.state.connected);
	zassert_equal(protocol.state.buttons, 0x4081);
	zassert_equal(protocol.state.lx, -127);
	zassert_equal(protocol.state.ly, 127);
	zassert_equal(protocol.state.rx, -1);
	zassert_equal(protocol.state.ry, 0);
	zassert_equal(protocol.state.lt, 0);
	zassert_equal(protocol.state.rt, 255);
	zassert_equal(protocol.state.seq, 0);
	zassert_equal(protocol.stats.frames_received, 1);
	zassert_equal(protocol.stats.frames_valid, 1);
}

ZTEST(gamepad_protocol, test_every_chunk_boundary_and_multiple_frames_per_chunk)
{
	make_frame(42);
	for (size_t split = 0; split <= wire_len; split++) {
		gamepad_protocol_init(&protocol, 100);
		gamepad_protocol_feed(&protocol, wire, split, 0);
		gamepad_protocol_feed(&protocol, wire + split, wire_len - split, 1);
		zassert_true(protocol.state.connected);
		zassert_equal(protocol.state.seq, 42);
	}
	uint8_t combined[64];
	make_frame(43);
	size_t first_len = wire_len;
	memcpy(combined, wire, first_len);
	make_frame(44);
	memcpy(combined + first_len, wire, wire_len);
	gamepad_protocol_feed(&protocol, combined, first_len + wire_len, 2);
	zassert_equal(protocol.state.seq, 44);
	zassert_equal(protocol.stats.frames_valid, 3);
}

ZTEST(gamepad_protocol, test_duplicates_do_not_update_state_or_watchdog)
{
	feed_seq(7, 0);
	uint8_t altered[] = {0xa1, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0};
	altered[10] = crc8(altered, 10, 0x07, 0, false);
	encode(altered, sizeof(altered));
	for (int64_t now = 20; now <= 100; now += 20) {
		gamepad_protocol_feed(&protocol, wire, wire_len, now);
	}
	gamepad_protocol_expire(&protocol, 100);
	assert_neutral();
	zassert_equal(protocol.stats.duplicate_frames, 5);
	zassert_equal(protocol.stats.timeouts, 1);
	zassert_equal(protocol.last_frame_ms, 0);
	feed_seq(7, 120);
	assert_neutral();
	feed_seq(8, 140);
	zassert_true(protocol.state.connected);
}

ZTEST(gamepad_protocol, test_duplicate_preserves_snapshot_before_timeout)
{
	feed_seq(7, 0);
	struct gamepad_state previous = protocol.state;
	feed_seq(7, 20);
	zassert_mem_equal(&protocol.state, &previous, sizeof(previous));
	zassert_equal(protocol.last_frame_ms, 0);
}

ZTEST(gamepad_protocol, test_wrap_and_modulo_drop_accounting)
{
	feed_seq(254, 0);
	feed_seq(255, 1);
	feed_seq(0, 2);
	zassert_equal(protocol.stats.dropped_frames, 0);
	feed_seq(3, 3);
	zassert_equal(protocol.stats.dropped_frames, 2);
	feed_seq(2, 4);
	zassert_equal(protocol.stats.dropped_frames, 256);
	zassert_equal(protocol.state.seq, 2);
}

ZTEST(gamepad_protocol, test_50hz_frames_keep_link_alive_then_timeout_exactly_once)
{
	for (uint8_t seq = 0; seq < 20; seq++) {
		feed_seq(seq, seq * 20);
		gamepad_protocol_expire(&protocol, seq * 20 + 19);
		zassert_true(protocol.state.connected);
	}
	gamepad_protocol_expire(&protocol, 479);
	zassert_true(protocol.state.connected);
	gamepad_protocol_expire(&protocol, 480);
	assert_neutral();
	gamepad_protocol_expire(&protocol, 1000);
	zassert_equal(protocol.stats.timeouts, 1);
	feed_seq(20, 1001);
	zassert_true(protocol.state.connected);
}

ZTEST(gamepad_protocol, test_crc_errors_never_refresh_watchdog)
{
	feed_seq(1, 0);
	uint8_t bad_crc[] = {0xa1, 1, 2, 3, 4, 5, 6, 7, 8, 2, 0};
	bad_crc[10] = crc8(bad_crc, 10, 0x07, 0, false) ^ 1;
	encode(bad_crc, sizeof(bad_crc));
	for (int64_t now = 20; now <= 120; now += 20) {
		gamepad_protocol_feed(&protocol, wire, wire_len, now);
	}
	assert_neutral();
	zassert_equal(protocol.stats.crc_errors, 6);
	zassert_equal(protocol.stats.frames_valid, 1);
	zassert_equal(protocol.stats.timeouts, 1);
}

ZTEST(gamepad_protocol, test_wrong_kind_and_lengths_are_rejected)
{
	uint8_t decoded[12] = {0xa2};
	decoded[10] = crc8(decoded, 10, 0x07, 0, false);
	encode(decoded, 11);
	gamepad_protocol_feed(&protocol, wire, wire_len, 0);
	encode(decoded, 10);
	gamepad_protocol_feed(&protocol, wire, wire_len, 1);
	encode(decoded, 12);
	gamepad_protocol_feed(&protocol, wire, wire_len, 2);
	assert_neutral();
	zassert_equal(protocol.stats.kind_errors, 1);
	zassert_equal(protocol.stats.length_errors, 2);
	zassert_equal(protocol.stats.frames_received, 3);
}

ZTEST(gamepad_protocol, test_malformed_cobs_and_overflow_resynchronize_at_delimiter)
{
	const uint8_t malformed[] = {5, 0xa1, 0};
	gamepad_protocol_feed(&protocol, malformed, sizeof(malformed), 0);
	zassert_equal(protocol.stats.cobs_errors, 1);
	feed_seq(1, 1);
	zassert_equal(protocol.state.seq, 1);
	uint8_t oversized[80];
	memset(oversized, 1, sizeof(oversized));
	gamepad_protocol_feed(&protocol, oversized, sizeof(oversized), 2);
	const uint8_t delimiter = 0;
	gamepad_protocol_feed(&protocol, &delimiter, 1, 2);
	feed_seq(2, 3);
	zassert_equal(protocol.state.seq, 2);
	zassert_equal(protocol.stats.length_errors, 1);
	zassert_equal(protocol.stats.frames_received, 4);
}

ZTEST(gamepad_protocol, test_uart_loss_discards_frame_tail_until_next_boundary)
{
	make_frame(1);
	gamepad_protocol_feed(&protocol, wire, 4, 0);
	gamepad_protocol_resync(&protocol);
	gamepad_protocol_feed(&protocol, wire + 4, wire_len - 4, 1);
	assert_neutral();
	feed_seq(2, 2);
	zassert_true(protocol.state.connected);
	zassert_equal(protocol.state.seq, 2);
}

ZTEST(gamepad_protocol, test_axis_bytes_are_preserved_without_normalization)
{
	uint8_t decoded[] = {0xa1, 0, 0, 0x80, 0x80, 0x80, 0x80, 0, 0, 1, 0};
	decoded[10] = crc8(decoded, 10, 0x07, 0, false);
	encode(decoded, sizeof(decoded));
	gamepad_protocol_feed(&protocol, wire, wire_len, 0);
	zassert_equal(protocol.state.lx, -128);
	zassert_equal(protocol.state.ly, -128);
	zassert_equal(protocol.state.rx, -128);
	zassert_equal(protocol.state.ry, -128);
}

ZTEST_SUITE(gamepad_protocol, NULL, NULL, before, NULL, NULL);
