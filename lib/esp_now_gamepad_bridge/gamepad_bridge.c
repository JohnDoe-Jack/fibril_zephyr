/* SPDX-License-Identifier: Apache-2.0 */
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <esp_now_gamepad_bridge/gamepad_bridge.h>
#include "gamepad_protocol.h"

#define RX_BUFFER_SIZE 64
#define RX_IDLE_TIMEOUT_US 1000
#define RX_RETRY_DELAY K_MSEC(10)

static struct gamepad_protocol protocol;
static struct k_spinlock state_lock;
static const struct device * bridge_uart;
static bool initialized;
static bool started;
K_MUTEX_DEFINE(lifecycle_lock);

static void watchdog_expired(struct k_timer * timer)
{
  ARG_UNUSED(timer);
  k_spinlock_key_t key = k_spin_lock(&state_lock);

  gamepad_protocol_expire(&protocol, k_uptime_get());
  k_spin_unlock(&state_lock, key);
}
K_TIMER_DEFINE(watchdog, watchdog_expired, NULL);

static void receive_bytes(const uint8_t * data, size_t len)
{
  k_spinlock_key_t key = k_spin_lock(&state_lock);

  if (gamepad_protocol_feed(&protocol, data, len, k_uptime_get())) {
    k_timer_start(&watchdog, K_MSEC(CONFIG_ESP_NOW_GAMEPAD_BRIDGE_TIMEOUT_MS), K_NO_WAIT);
  }
  k_spin_unlock(&state_lock, key);
}

static void receive_error(void)
{
  k_spinlock_key_t key = k_spin_lock(&state_lock);

  protocol.stats.uart_errors++;
  gamepad_protocol_resync(&protocol);
  k_spin_unlock(&state_lock, key);
}

#ifdef CONFIG_ESP_NOW_GAMEPAD_BRIDGE_ASYNC
static uint8_t rx_buffers[2][RX_BUFFER_SIZE] __aligned(4);
static bool buffer_owned[2];
static bool rx_active;
static bool rx_stopped;

static int enable_async_rx(void)
{
  k_spinlock_key_t key = k_spin_lock(&state_lock);

  if (rx_active) {
    k_spin_unlock(&state_lock, key);
    return -EALREADY;
  }
  buffer_owned[0] = true;
  rx_active = true;
  rx_stopped = false;
  k_spin_unlock(&state_lock, key);

  /* UART calls are outside state_lock: some drivers call back synchronously. */
  int ret = uart_rx_enable(bridge_uart, rx_buffers[0], RX_BUFFER_SIZE, RX_IDLE_TIMEOUT_US);

  if (ret < 0) {
    key = k_spin_lock(&state_lock);
    buffer_owned[0] = false;
    buffer_owned[1] = false;
    rx_active = false;
    protocol.stats.uart_errors++;
    k_spin_unlock(&state_lock, key);
  }
  return ret;
}

static void restart_rx(struct k_work * work);
K_WORK_DELAYABLE_DEFINE(restart_work, restart_rx);

static void restart_rx(struct k_work * work)
{
  ARG_UNUSED(work);
  int ret = enable_async_rx();

  if (ret == 0) {
    k_spinlock_key_t key = k_spin_lock(&state_lock);

    protocol.stats.rx_restarts++;
    k_spin_unlock(&state_lock, key);
  } else if (ret != -EALREADY) {
    (void)k_work_reschedule(&restart_work, RX_RETRY_DELAY);
  }
}

static void provide_buffer(const struct device * uart)
{
  int index = -1;
  k_spinlock_key_t key = k_spin_lock(&state_lock);

  for (int i = 0; i < ARRAY_SIZE(rx_buffers); i++) {
    if (!buffer_owned[i]) {
      buffer_owned[i] = true;
      index = i;
      break;
    }
  }
  k_spin_unlock(&state_lock, key);

  if (index < 0) {
    receive_error();
    return;
  }
  int ret = uart_rx_buf_rsp(uart, rx_buffers[index], RX_BUFFER_SIZE);

  if (ret < 0) {
    key = k_spin_lock(&state_lock);
    buffer_owned[index] = false;
    k_spin_unlock(&state_lock, key);
    receive_error();
  }
}

static void async_callback(const struct device * uart, struct uart_event * event, void * user_data)
{
  ARG_UNUSED(user_data);
  k_spinlock_key_t key;

  switch (event->type) {
  case UART_RX_RDY:
    key = k_spin_lock(&state_lock);
    bool stopped = rx_stopped;
    k_spin_unlock(&state_lock, key);
    if (!stopped) {
      receive_bytes(event->data.rx.buf + event->data.rx.offset, event->data.rx.len);
    }
    break;
  case UART_RX_BUF_REQUEST:
    provide_buffer(uart);
    break;
  case UART_RX_BUF_RELEASED:
    key = k_spin_lock(&state_lock);
    for (size_t i = 0; i < ARRAY_SIZE(rx_buffers); i++) {
      if (event->data.rx_buf.buf == rx_buffers[i]) {
        buffer_owned[i] = false;
      }
    }
    k_spin_unlock(&state_lock, key);
    break;
  case UART_RX_STOPPED:
    key = k_spin_lock(&state_lock);
    rx_stopped = true;
    protocol.stats.uart_errors++;
    gamepad_protocol_resync(&protocol);
    k_spin_unlock(&state_lock, key);
    break;
  case UART_RX_DISABLED:
    key = k_spin_lock(&state_lock);
    /* RX_DISABLED follows release of all driver-owned buffers. */
    buffer_owned[0] = false;
    buffer_owned[1] = false;
    rx_active = false;
    gamepad_protocol_resync(&protocol);
    k_spin_unlock(&state_lock, key);
    (void)k_work_reschedule(&restart_work, K_NO_WAIT);
    break;
  default:
    break;
  }
}
#else
static void irq_callback(const struct device * uart, void * user_data)
{
  ARG_UNUSED(user_data);
  uint8_t data[RX_BUFFER_SIZE];

  while (uart_irq_update(uart) > 0) {
    if (uart_err_check(uart) > 0) {
      receive_error();
    }
    if (uart_irq_rx_ready(uart) <= 0) {
      break;
    }
    int len = uart_fifo_read(uart, data, sizeof(data));

    if (len <= 0) {
      if (len < 0) {
        receive_error();
      }
      break;
    }
    receive_bytes(data, len);
  }
}
#endif

static bool is_console_uart(const struct device * uart)
{
#if DT_HAS_CHOSEN(zephyr_console) && defined(CONFIG_UART_CONSOLE)
  if (uart == DEVICE_DT_GET(DT_CHOSEN(zephyr_console))) {
    return true;
  }
#endif
#if DT_HAS_CHOSEN(zephyr_shell_uart) && defined(CONFIG_SHELL_BACKEND_SERIAL)
  if (uart == DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart))) {
    return true;
  }
#endif
  ARG_UNUSED(uart);
  return false;
}

int gamepad_bridge_init(const struct device * uart)
{
  int ret;
  k_mutex_lock(&lifecycle_lock, K_FOREVER);

  if (initialized) {
    ret = -EALREADY;
    goto out;
  }
  if (!device_is_ready(uart)) {
    ret = -ENODEV;
    goto out;
  }
  if (is_console_uart(uart)) {
    ret = -EBUSY;
    goto out;
  }

  const struct uart_config config = {
    .baudrate = 115200,
    .parity = UART_CFG_PARITY_NONE,
    .stop_bits = UART_CFG_STOP_BITS_1,
    .data_bits = UART_CFG_DATA_BITS_8,
    .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
  };
  ret = uart_configure(uart, &config);
  if (ret < 0) {
    goto out;
  }
#ifdef CONFIG_ESP_NOW_GAMEPAD_BRIDGE_ASYNC
  ret = uart_callback_set(uart, async_callback, NULL);
#else
  ret = uart_irq_callback_user_data_set(uart, irq_callback, NULL);
#endif
  if (ret == 0) {
    k_spinlock_key_t key = k_spin_lock(&state_lock);

    gamepad_protocol_init(&protocol, CONFIG_ESP_NOW_GAMEPAD_BRIDGE_TIMEOUT_MS);
    bridge_uart = uart;
    initialized = true;
    k_spin_unlock(&state_lock, key);
  }
out:
  k_mutex_unlock(&lifecycle_lock);
  return ret;
}

int gamepad_bridge_start(void)
{
  int ret;
  k_mutex_lock(&lifecycle_lock, K_FOREVER);

  if (!initialized) {
    ret = -EACCES;
    goto out;
  }
  if (started) {
    ret = -EALREADY;
    goto out;
  }
#ifdef CONFIG_ESP_NOW_GAMEPAD_BRIDGE_ASYNC
  ret = enable_async_rx();
#else
  uart_irq_err_enable(bridge_uart);
  uart_irq_rx_enable(bridge_uart);
  ret = 0;
#endif
  if (ret == 0) {
    started = true;
  }
out:
  k_mutex_unlock(&lifecycle_lock);
  return ret;
}

int gamepad_bridge_get_state(struct gamepad_state * state)
{
  if (state == NULL) {
    return -EINVAL;
  }
  k_spinlock_key_t key = k_spin_lock(&state_lock);

  if (!initialized) {
    k_spin_unlock(&state_lock, key);
    return -EACCES;
  }
  gamepad_protocol_expire(&protocol, k_uptime_get());
  *state = protocol.state;
  k_spin_unlock(&state_lock, key);
  return 0;
}

bool gamepad_bridge_is_connected(void)
{
  struct gamepad_state state;

  return gamepad_bridge_get_state(&state) == 0 && state.connected;
}

int gamepad_bridge_get_stats(struct gamepad_bridge_stats * stats)
{
  if (stats == NULL) {
    return -EINVAL;
  }
  k_spinlock_key_t key = k_spin_lock(&state_lock);

  if (!initialized) {
    k_spin_unlock(&state_lock, key);
    return -EACCES;
  }
  *stats = protocol.stats;
  k_spin_unlock(&state_lock, key);
  return 0;
}
