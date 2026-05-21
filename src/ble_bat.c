#include "inc/ble_bat.h"

#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "driver/battery.h"

LOG_MODULE_REGISTER(ble_bat, LOG_LEVEL_INF);

enum {
  BLE_BAT_REFRESH_INTERVAL_MS = 1000,
};

struct ble_bat_runtime {
  bool initialized;
  bool has_level;
  uint8_t level;
};

static struct ble_bat_runtime s_ble_bat;

static void ble_bat_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(s_ble_bat_work, ble_bat_work_handler);

static void ble_bat_refresh_once(void)
{
  battery_measurement_t measurement;
  int err;

  err = battery_refresh(&measurement);
  if (err) {
    LOG_WRN("battery refresh failed: %d", err);
    return;
  }

  if (!s_ble_bat.has_level || s_ble_bat.level != measurement.level_pct) {
    err = bt_bas_set_battery_level(measurement.level_pct);
    if (err) {
      LOG_WRN("battery level publish failed: %d", err);
      return;
    }

    s_ble_bat.level = measurement.level_pct;
    s_ble_bat.has_level = true;
  }
}

static void ble_bat_work_handler(struct k_work *work)
{
  ARG_UNUSED(work);

  ble_bat_refresh_once();
  (void)k_work_reschedule(&s_ble_bat_work, K_MSEC(BLE_BAT_REFRESH_INTERVAL_MS));
}

int ble_bat_init(void)
{
  int err;

  if (s_ble_bat.initialized) {
    return 0;
  }

  err = battery_init();
  if (err) {
    return err;
  }

  ble_bat_refresh_once();
  (void)k_work_schedule(&s_ble_bat_work, K_MSEC(BLE_BAT_REFRESH_INTERVAL_MS));

  s_ble_bat.initialized = true;
  return 0;
}

uint8_t ble_bat_get_level(void)
{
  return s_ble_bat.level;
}
