/* SPDX-License-Identifier: Apache-2.0 */
#ifndef FIBRIL_GAMEPAD_PROTOCOL_H_
#define FIBRIL_GAMEPAD_PROTOCOL_H_

#include <zephyr/data/cobs.h>
#include <esp_now_gamepad_bridge/gamepad_bridge.h>

#define GAMEPAD_DECODED_SIZE 11

/* Internal state machine. The caller serializes all access and supplies time. */
struct gamepad_protocol
{
  struct cobs_decoder decoder;
  struct gamepad_state state;
  struct gamepad_bridge_stats stats;
  uint8_t decoded[GAMEPAD_DECODED_SIZE];
  size_t decoded_len;
  bool discarding;
  bool have_seq;
  bool accepted;
  int64_t now_ms;
  int64_t last_frame_ms;
  uint32_t timeout_ms;
};

void gamepad_protocol_init(struct gamepad_protocol * protocol, uint32_t timeout_ms);
bool gamepad_protocol_feed(struct gamepad_protocol * protocol, const uint8_t * data,
                          size_t len, int64_t now_ms);
void gamepad_protocol_expire(struct gamepad_protocol * protocol, int64_t now_ms);
void gamepad_protocol_resync(struct gamepad_protocol * protocol);

#endif
