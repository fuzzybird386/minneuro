#ifndef BLE_AAAA_AAE_IMU_C__
#define BLE_AAAA_AAE_IMU_C__

#include "inc/ble_aaaa_priv.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(AAE, LOG_LEVEL_INF);

#define STATE_CODE_LEN 20
#define STREAM_BUFFER_MAX_LEN 512

typedef struct {
  uint8_t state_codes[STATE_CODE_LEN];
  uint8_t stream_buffer[STREAM_BUFFER_MAX_LEN];
  uint16_t stream_buffer_len;
  volatile bool stream_ready;
  // Notifications:
  volatile bool aae0_notify_enabled;
  volatile bool aaef_notify_enabled;
} aae_t;

static volatile aae_t state_aae;

void ble_ccc_cfg_changed_aae0(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	state_aae.aae0_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Notification[aae0] %s", state_aae.aae0_notify_enabled ? "enabled" : "disabled");
}

void ble_ccc_cfg_changed_aaef(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	state_aae.aaef_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Notification[aaeF] %s", state_aae.aaef_notify_enabled ? "enabled" : "disabled");
}

ssize_t ble_on_aae0_read_request(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
  // const char *value = attr->user_data; // NULL
  const char *value = (const char *)state_aae.state_codes;
  return bt_gatt_attr_read(conn, attr, buf, len, offset, value, STATE_CODE_LEN);
}

ssize_t ble_on_aae0_write_request(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{

  LOG_INF("payload len: %d", len);
  if (len == 1) { // debug mode
    int err = 0; // xx_verify();
    LOG_INF("faca: debug verify: %02x", err);
    return 0;
  }
  /**
   * 
   * Request Payload Parser & Resp.
   * 
  */
  uint8_t * data = (uint8_t *)buf;
  uint16_t request_id = (uint16_t)(data[0] << 8) | (data[1] & 0xFF);
  uint8_t method = data[2] & 0xFF;

  return len;
}

int ble_aaef_notify_commit(uint8_t * data, uint8_t len)
{
  // copy:
  for (int i = 0; i < len; i++) {
    state_aae.stream_buffer[i] = data[i];
  }
  state_aae.stream_buffer_len = len;
  state_aae.stream_ready = true;
  return 0;
}

int ble_aaef_notify_commit_mpu(uint32_t seq, mpu_imu_data_t * data, uint8_t len, uint8_t ch)
{
  // printk("Notify IMU: seq=%u, len=%u, ch=%u\n", seq, len, ch);
  uint16_t index = 0;
  // spec code:
  state_aae.stream_buffer[index++] = 0x72;
  // seq:
  state_aae.stream_buffer[index++] = (seq >> 24) & 0xFF;
  state_aae.stream_buffer[index++] = (seq >> 16) & 0xFF;
  state_aae.stream_buffer[index++] = (seq >> 8) & 0xFF;
  state_aae.stream_buffer[index++] = (seq) & 0xFF;
  // len:
  state_aae.stream_buffer[index++] = len;                           // frame length.

  for (uint8_t i = 0; i < len; i++) {
    state_aae.stream_buffer[index++] = (data[i].temperature >> 8) & 0xFF; // temperature h.
    state_aae.stream_buffer[index++] = (data[i].temperature & 0xFF);      // temperature l.
    state_aae.stream_buffer[index++] = (data[i].ax >> 8) & 0xFF;          // ax h.
    state_aae.stream_buffer[index++] = (data[i].ax & 0xFF);               // ax l.
    state_aae.stream_buffer[index++] = (data[i].ay >> 8) & 0xFF;          // ay h.
    state_aae.stream_buffer[index++] = (data[i].ay & 0xFF);               // ay l.
    state_aae.stream_buffer[index++] = (data[i].az >> 8) & 0xFF;          // az h.
    state_aae.stream_buffer[index++] = (data[i].az & 0xFF);               // az l.
    state_aae.stream_buffer[index++] = (data[i].gx >> 8) & 0xFF;          // gx h.
    state_aae.stream_buffer[index++] = (data[i].gx & 0xFF);               // gx l.
    state_aae.stream_buffer[index++] = (data[i].gy >> 8) & 0xFF;          // gy h.
    state_aae.stream_buffer[index++] = (data[i].gy & 0xFF);               // gy l.
    state_aae.stream_buffer[index++] = (data[i].gz >> 8) & 0xFF;          // gz h.
    state_aae.stream_buffer[index++] = (data[i].gz & 0xFF);               // gz l.
    state_aae.stream_buffer[index++] = (data[i].mx >> 8) & 0xFF;          // mx h.
    state_aae.stream_buffer[index++] = (data[i].mx & 0xFF);               // mx l.
    state_aae.stream_buffer[index++] = (data[i].my >> 8) & 0xFF;          // my h.
    state_aae.stream_buffer[index++] = (data[i].my & 0xFF);               // my l.
    state_aae.stream_buffer[index++] = (data[i].mz >> 8) & 0xFF;          // mz h.
    state_aae.stream_buffer[index++] = (data[i].mz & 0xFF);               // mz l.
    if (ch >= 13) {
      state_aae.stream_buffer[index++] = (data[i].yaw >> 8) & 0xFF;       // yaw h.
      state_aae.stream_buffer[index++] = (data[i].yaw & 0xFF);            // yaw l.
      state_aae.stream_buffer[index++] = (data[i].pitch >> 8) & 0xFF;     // pitch h.
      state_aae.stream_buffer[index++] = (data[i].pitch & 0xFF);          // pitch l.
      state_aae.stream_buffer[index++] = (data[i].roll >> 8) & 0xFF;      // roll h.
      state_aae.stream_buffer[index++] = (data[i].roll & 0xFF);           // roll l.
    }
  }
  state_aae.stream_buffer_len = index;
  state_aae.stream_ready = true;
  return 0;
}

int ble_aaef_loop()
{
  // if ready
  if (!state_aae.aaef_notify_enabled) {
    return -1;
  }
  if (!state_aae.stream_ready) {
    return -2;
  }
  // printk("len: %u\n", state_aae.stream_buffer_len);
  int err = bt_gatt_notify(NULL, &aaaa_service.attrs[IMU_AAEF_INDEX], (uint8_t*)&(state_aae.stream_buffer), state_aae.stream_buffer_len);
  if (err) {
    printk("Failed to notify IMU data: %d\n", err);
    return err;
  }
  return 0;
}

#endif
