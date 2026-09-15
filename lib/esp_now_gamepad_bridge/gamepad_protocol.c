/* SPDX-License-Identifier: Apache-2.0 */
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include "gamepad_protocol.h"

static int decoded_data(const uint8_t * buf, size_t len, void * user_data)
{
  struct gamepad_protocol * protocol = user_data;

  if (buf != NULL) {
    if (len > sizeof(protocol->decoded) - protocol->decoded_len) {
      return -ENOMEM;
    }
    memcpy(protocol->decoded + protocol->decoded_len, buf, len);
    protocol->decoded_len += len;
    return 0;
  }

  const uint8_t * payload = protocol->decoded;

  if (protocol->decoded_len != GAMEPAD_DECODED_SIZE) {
    protocol->stats.length_errors++;
    return 0;
  }
  if (crc8(payload, GAMEPAD_DECODED_SIZE - 1, 0x07, 0x00, false) != payload[10]) {
    protocol->stats.crc_errors++;
    return 0;
  }
  if (payload[0] != 0xa1) {
    protocol->stats.kind_errors++;
    return 0;
  }

  protocol->stats.frames_valid++;
  uint8_t delta = (uint8_t)(payload[9] - protocol->state.seq);

  if (protocol->have_seq && delta == 0) {
    protocol->stats.duplicate_frames++;
    return 0;
  }
  if (protocol->have_seq && delta > 1) {
    protocol->stats.dropped_frames += delta - 1U;
  }

  protocol->state = (struct gamepad_state){
    .buttons = sys_get_le16(&payload[1]),
    .lx = (int8_t)payload[3],
    .ly = (int8_t)payload[4],
    .rx = (int8_t)payload[5],
    .ry = (int8_t)payload[6],
    .lt = payload[7],
    .rt = payload[8],
    .seq = payload[9],
    .connected = true,
  };
  protocol->last_frame_ms = protocol->now_ms;
  protocol->have_seq = true;
  protocol->accepted = true;
  return 0;
}

static void reset_decoder(struct gamepad_protocol * protocol)
{
  protocol->decoded_len = 0;
  (void)cobs_decoder_init(&protocol->decoder, decoded_data, protocol,
                          COBS_FLAG_TRAILING_DELIMITER);
}

void gamepad_protocol_init(struct gamepad_protocol * protocol, uint32_t timeout_ms)
{
  memset(protocol, 0, sizeof(*protocol));
  protocol->timeout_ms = timeout_ms;
  reset_decoder(protocol);
}

void gamepad_protocol_expire(struct gamepad_protocol * protocol, int64_t now_ms)
{
  if (protocol->state.connected &&
      now_ms - protocol->last_frame_ms >= protocol->timeout_ms) {
    uint8_t seq = protocol->state.seq;

    protocol->state = (struct gamepad_state){.seq = seq};
    protocol->stats.timeouts++;
  }
}

void gamepad_protocol_resync(struct gamepad_protocol * protocol)
{
  reset_decoder(protocol);
  protocol->discarding = true;
}

bool gamepad_protocol_feed(struct gamepad_protocol * protocol, const uint8_t * data,
                          size_t len, int64_t now_ms)
{
  gamepad_protocol_expire(protocol, now_ms);
  protocol->now_ms = now_ms;
  protocol->accepted = false;

  /* One-byte writes locate the exact delimiter even when the COBS API aborts
   * with an error without reporting how many input bytes it consumed.
   */
  for (size_t i = 0; i < len; i++) {
    if (!protocol->discarding) {
      int ret = cobs_decoder_write(&protocol->decoder, &data[i], 1);

      if (ret < 0) {
        if (ret == -ENOMEM) {
          protocol->stats.length_errors++;
        } else {
          protocol->stats.cobs_errors++;
        }
        protocol->discarding = true;
      }
    }
    if (data[i] == 0) {
      protocol->stats.frames_received++;
      protocol->discarding = false;
      reset_decoder(protocol);
    }
  }
  return protocol->accepted;
}
