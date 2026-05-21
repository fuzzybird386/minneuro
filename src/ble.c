#include "inc/ble.h"

#include "inc/minneuro_mode.h"

#include <zephyr/sys/util.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

#include "inc/ble_conn.h"
#include "inc/ble_bat.h"
#include "inc/ble_debug.h"
#include "inc/ble_aaaa.h"

LOG_MODULE_REGISTER(ble, LOG_LEVEL_INF);

void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	printk("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
}
static struct bt_gatt_cb gatt_callbacks = {.att_mtu_updated = mtu_updated};

int ble_init()
{
  int err = 0;
  
  // Bluetooth
  err = bt_enable(NULL);
  if (err) {
    return err;
  }
  bt_gatt_cb_register(&gatt_callbacks);
  // // // LOG_INF("Bluetooth init success.");
  // // if (IS_ENABLED(CONFIG_BT_SETTINGS))
  // // {
  // //   settings_load();
  // //   // LOG_INF("Settings init success.");
  // // }

  err = ble_aaaa_init();
  if (err) {
    return err;
  }

#if MINNEURO_DEBUG_MODE && !MINNEURO_STIM_TEST_MODE
  err = ble_debug_init();
  if (err) {
    LOG_WRN("debug BLE service init failed (%d), continuing without debug service", err);
  }
#endif

  err = ble_bat_init();
  if (err) {
    err = 0;
  }
  
  err = ble_adv_init();
  if (err) {
    return err;
  }
  
  err = ble_adv_start();
  if (err) {
    return err;
  }

  return 0;
}

int ble_uninit()
{
  return 0;
}
