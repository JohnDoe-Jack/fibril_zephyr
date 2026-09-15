/* SPDX-License-Identifier: Apache-2.0 */
#ifndef FIBRIL_ESP_NOW_GAMEPAD_BRIDGE_H_
#define FIBRIL_ESP_NOW_GAMEPAD_BRIDGE_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file
 * @brief Receive-only ESP-NOW gamepad UART bridge.
 * @defgroup esp_now_gamepad_bridge ESP-NOW gamepad UART bridge
 * @ingroup lib
 * @{
 */

/** @brief Button bits in the little-endian wire mask. */
enum gamepad_button {
  GAMEPAD_BUTTON_DPAD_LEFT = 1U << 0, /**< D-pad left. */
  GAMEPAD_BUTTON_DPAD_UP = 1U << 1, /**< D-pad up. */
  GAMEPAD_BUTTON_DPAD_RIGHT = 1U << 2, /**< D-pad right. */
  GAMEPAD_BUTTON_DPAD_DOWN = 1U << 3, /**< D-pad down. */
  GAMEPAD_BUTTON_X = 1U << 4, /**< X. */
  GAMEPAD_BUTTON_Y = 1U << 5, /**< Y. */
  GAMEPAD_BUTTON_B = 1U << 6, /**< B. */
  GAMEPAD_BUTTON_A = 1U << 7, /**< A. */
  GAMEPAD_BUTTON_LB = 1U << 8, /**< Left bumper. */
  GAMEPAD_BUTTON_RB = 1U << 9, /**< Right bumper. */
  GAMEPAD_BUTTON_L3 = 1U << 10, /**< Left stick click. */
  GAMEPAD_BUTTON_R3 = 1U << 11, /**< Right stick click. */
  GAMEPAD_BUTTON_VIEW = 1U << 12, /**< View. */
  GAMEPAD_BUTTON_MENU = 1U << 13, /**< Menu. */
  GAMEPAD_BUTTON_HOME = 1U << 14, /**< Home. */
};

/** @brief Atomic snapshot; raw axes use +X right and +Y down. */
struct gamepad_state
{
  uint16_t buttons; /**< Button mask; bit 15 is reserved by the sender. */
  int8_t lx; /**< Left X, sender range -127..127. */
  int8_t ly; /**< Left Y, sender range -127..127. */
  int8_t rx; /**< Right X, sender range -127..127. */
  int8_t ry; /**< Right Y, sender range -127..127. */
  uint8_t lt; /**< Left trigger, 0..255. */
  uint8_t rt; /**< Right trigger, 0..255. */
  uint8_t seq; /**< Last accepted sequence; retained when disconnected. */
  bool connected; /**< A new valid frame was received within the timeout. */
};

/** @brief Read-only counters, wrapping modulo 2^32. */
struct gamepad_bridge_stats
{
  uint32_t frames_received; /**< Delimiters seen, including rejected/empty frames. */
  uint32_t frames_valid; /**< Correct length, CRC and kind, including duplicates. */
  uint32_t cobs_errors; /**< Malformed COBS frames. */
  uint32_t crc_errors; /**< CRC mismatches. */
  uint32_t length_errors; /**< Short/long frames, including bounded-buffer overflow. */
  uint32_t kind_errors; /**< Unexpected payload kind. */
  uint32_t duplicate_frames; /**< Valid frames with the previous sequence. */
  uint32_t dropped_frames; /**< Sum of modulo-256 sequence gaps minus one. */
  uint32_t timeouts; /**< Connected-to-disconnected timeout transitions. */
  uint32_t uart_errors; /**< UART receive errors or failed receive API operations. */
  uint32_t rx_restarts; /**< Successful async RX restarts after RX_DISABLED. */
};

/**
 * @brief Bind the singleton bridge to a dedicated UART and configure 115200 8N1.
 *
 * Call from thread context before start. No bytes are transmitted. The UART
 * must not be used by console, shell, or another receiver. State starts neutral.
 * A successful init binds the UART for the lifetime of the application.
 *
 * @param uart Ready UART device supporting the selected receive backend.
 * @return 0 on success, -ENODEV if not ready, -EBUSY for a console/shell UART,
 *         -EALREADY if initialized, or a negative UART API error.
 */
int gamepad_bridge_init(const struct device * uart);

/**
 * @brief Start receiving. Thread context only; retry is allowed after failure.
 * @return 0 on success, -EACCES before init, -EALREADY if already started,
 *         or a negative UART API error.
 */
int gamepad_bridge_start(void);

/**
 * @brief Copy a coherent state snapshot, enforcing timeout before returning.
 *
 * This and other getters are callable from threads or ordinary ISRs. No UART
 * traffic or timeout does not make this API fail: inspect connected instead.
 *
 * @param state Destination snapshot.
 * @return 0 on success, -EINVAL for NULL, -EACCES before init.
 */
int gamepad_bridge_get_state(struct gamepad_state * state);

/** @brief Return false before init or after the link timeout. */
bool gamepad_bridge_is_connected(void);

/**
 * @brief Copy a coherent statistics snapshot without clearing counters.
 * @param stats Destination counters.
 * @return 0 on success, -EINVAL for NULL, -EACCES before init.
 */
int gamepad_bridge_get_stats(struct gamepad_bridge_stats * stats);

/** @} */
#ifdef __cplusplus
}
#endif
#endif
