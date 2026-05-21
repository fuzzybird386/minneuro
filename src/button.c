#include "inc/button.h"

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "inc/minneuro_mode.h"

LOG_MODULE_REGISTER(button, LOG_LEVEL_INF);

#define BUTTON_GPIO_NODE DT_NODELABEL(gpio1)
#define BUTTON_HOME_PIN 13u

#define BUTTON_DEBOUNCE_MS 120u

static const struct device *button_gpio;
static struct gpio_callback button_cb;

static struct k_work button_home_work;

static uint32_t button_home_last_ms;

static bool button_is_debounced(uint32_t *last_tick_ms)
{
  uint32_t now_ms = k_uptime_get_32();

  if ((now_ms - *last_tick_ms) < BUTTON_DEBOUNCE_MS) {
    return false;
  }

  *last_tick_ms = now_ms;
  return true;
}

static int button_execute_home(void)
{
#if MINNEURO_DEBUG_MODE
  LOG_INF("button_home ignored in debug mode");
  return 0;
#else
  LOG_INF("button_home has no action in normal mode");
  return 0;
#endif
}

static void button_home_work_handler(struct k_work *work)
{
  ARG_UNUSED(work);
  (void)button_execute_home();
}

static void button_irq_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
  ARG_UNUSED(dev);
  ARG_UNUSED(cb);

  if ((pins & BIT(BUTTON_HOME_PIN)) != 0u) {
    if (button_is_debounced(&button_home_last_ms)) {
      (void)k_work_submit(&button_home_work);
    }
  }
}

int button_init(void)
{
  int err;

  button_gpio = DEVICE_DT_GET(BUTTON_GPIO_NODE);
  if (!device_is_ready(button_gpio)) {
    return -ENODEV;
  }

  k_work_init(&button_home_work, button_home_work_handler);

  err = gpio_pin_configure(button_gpio, BUTTON_HOME_PIN, GPIO_INPUT | GPIO_PULL_DOWN | GPIO_ACTIVE_HIGH);
  if (err) {
    return err;
  }

  err = gpio_pin_interrupt_configure(button_gpio, BUTTON_HOME_PIN, GPIO_INT_EDGE_TO_ACTIVE);
  if (err) {
    return err;
  }

  gpio_init_callback(&button_cb, button_irq_handler, BIT(BUTTON_HOME_PIN));
  err = gpio_add_callback(button_gpio, &button_cb);
  if (err) {
    return err;
  }

  LOG_INF("Button ready: home=P1.13 (active high)");
  return 0;
}
