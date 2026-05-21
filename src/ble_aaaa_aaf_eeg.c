#ifndef BLE_AAAA_AAE_IMU_C__
#define BLE_AAAA_AAE_IMU_C__

#include "inc/ble_aaaa_priv.h"

#include "neuro/inc/eeg_manager.h"
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(AAF, LOG_LEVEL_INF);

// #define STATE_CODE_LEN 20
#define STREAM_BUFFER_MAX_LEN 512

typedef struct {
  // uint8_t state_codes[STATE_CODE_LEN];
  uint8_t stream_buffer[STREAM_BUFFER_MAX_LEN];
  uint16_t stream_buffer_len;
  volatile bool stream_ready;
  // Notifications:
  volatile bool aaf0_notify_enabled;
  volatile bool aaff_notify_enabled;
} aaf_t;

static volatile aaf_t state_aaf;
static struct k_work_delayable aaff_ctrl_work;
static volatile bool aaff_target_eeg_running;
static volatile bool aaff_eeg_running;
static eeg_manager_config_t aaff_pending_cfg;

// config status
typedef struct {
  volatile uint8_t  status; // 0: stop, 1: start
  volatile uint16_t sample_rate_hz; // e.g. 1000Hz;
  volatile uint16_t channel_size; // e.g. 16
  volatile uint8_t  resolution_bits; // e.g. 16
  volatile float     scale_factor; // e.g. 0.192f uV/bit 
} aaf_config_t;

static aaf_config_t aaf_config = {
  .status = 0,
  .sample_rate_hz = TASK_EEG_DEFAULT_SAMPLE_RATE_HZ,
  .channel_size = TASK_EEG_CHANNEL_SIZE,
  .resolution_bits = 16,
  .scale_factor = 0.192f,
};

#define AAF0_CONFIG_WIRE_LEN 10u

static inline void put_be16(uint8_t *dst, uint16_t v)
{
  dst[0] = (uint8_t)((v >> 8) & 0xFFu);
  dst[1] = (uint8_t)(v & 0xFFu);
}

static inline void put_be32(uint8_t *dst, uint32_t v)
{
  dst[0] = (uint8_t)((v >> 24) & 0xFFu);
  dst[1] = (uint8_t)((v >> 16) & 0xFFu);
  dst[2] = (uint8_t)((v >> 8) & 0xFFu);
  dst[3] = (uint8_t)(v & 0xFFu);
}

static uint16_t build_aaf0_config_wire(uint8_t *out)
{
  uint32_t scale_raw;

  if (!out) {
    return 0u;
  }

  out[0] = (uint8_t)aaf_config.status;
  put_be16(&out[1], (uint16_t)aaf_config.sample_rate_hz);
  put_be16(&out[3], (uint16_t)aaf_config.channel_size);
  out[5] = (uint8_t)aaf_config.resolution_bits;

  memcpy(&scale_raw, (const void *)&aaf_config.scale_factor, sizeof(scale_raw));
  put_be32(&out[6], scale_raw);

  return AAF0_CONFIG_WIRE_LEN;
}

static void ble_aaff_ctrl_work_handler(struct k_work *work)
{
  int err;

  ARG_UNUSED(work);

  if (aaff_target_eeg_running == aaff_eeg_running) {
    return;
  }

  if (aaff_target_eeg_running) {
    err = eeg_manager_start(aaff_pending_cfg);
    if (err) {
      LOG_ERR("eeg_manager_start failed: %d", err);
      aaff_eeg_running = false;
      return;
    }

    aaff_eeg_running = true;
    aaf_config.sample_rate_hz = (uint16_t)aaff_pending_cfg.sample_rate_hz;
    aaf_config.status = 1;
    LOG_INF("AAFF EEG started: sample_rate=%u", aaff_pending_cfg.sample_rate_hz);
    return;
  }

  err = eeg_manager_stop();
  if (err) {
    LOG_ERR("eeg_manager_stop failed: %d", err);
  }
  aaff_eeg_running = false;
  aaf_config.status = 0;
  state_aaf.stream_ready = false;
  LOG_INF("AAFF EEG stopped");
}

#define AAFF_CTRL_DEBOUNCE_MS 50
static inline void trigger_task_start(eeg_manager_config_t cfg) {
  aaff_pending_cfg.sample_rate_hz = cfg.sample_rate_hz;
  aaff_target_eeg_running = true;
  (void)k_work_reschedule(&aaff_ctrl_work, K_MSEC(AAFF_CTRL_DEBOUNCE_MS));
}

static inline void trigger_task_stop() {
  aaff_target_eeg_running = false;
  (void)k_work_reschedule(&aaff_ctrl_work, K_MSEC(AAFF_CTRL_DEBOUNCE_MS));
}
/// BLE:

void ble_ccc_cfg_changed_aaf0(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	state_aaf.aaf0_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Notification[AAF0] %s", state_aaf.aaf0_notify_enabled ? "enabled" : "disabled");
}

void ble_ccc_cfg_changed_aaff(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

  state_aaf.aaff_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  if (!state_aaf.aaff_notify_enabled) {
    state_aaf.stream_ready = false;
  }

  LOG_INF("Notification[AAFF] %s", state_aaf.aaff_notify_enabled ? "enabled" : "disabled");
}

ssize_t ble_on_aaf0_read_request(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
  // const char *value = (const char *)state_aaf.state_codes;
  // return bt_gatt_attr_read(conn, attr, buf, len, offset, value, STATE_CODE_LEN);

  // TEST CODE (Read from SPI):

  uint8_t wire[AAF0_CONFIG_WIRE_LEN];
  uint16_t wire_len = build_aaf0_config_wire(wire);

  // const char *value = (const char *)state_aaf.state_codes;
  return bt_gatt_attr_read(conn, attr, buf, len, offset, wire, wire_len);
}

#define AAFF_METHOD_START_WITH_CONFIG 0x00
#define AAFF_METHOD_STOP              0x01

ssize_t ble_on_aaf0_write_request(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
  ARG_UNUSED(conn);
  ARG_UNUSED(attr);
  ARG_UNUSED(offset);
  ARG_UNUSED(flags);

  LOG_INF("payload len: %d", len);
  if (len == 1) { // debug mode
    int err = eeg_manager_verify();
    LOG_INF("faca: debug verify: %02x", err);
    return 0;
  }
  /**
   * 
   * Request Payload Parser & Resp.
   * 
  */
  if (len < 3u) {
    LOG_WRN("AAFF write payload too short: %u", len);
    return len;
  }

  const uint8_t * data = (const uint8_t *)buf;
  uint16_t request_id = (uint16_t)(data[0] << 8) | (data[1] & 0xFF);
  uint8_t method = data[2] & 0xFF;

  if (method == AAFF_METHOD_START_WITH_CONFIG) {
    eeg_manager_config_t cfg = {
      .sample_rate_hz = aaf_config.sample_rate_hz,
    };

    if (len >= 5u) {
      cfg.sample_rate_hz = ((uint16_t)data[3] << 8) | data[4];
    }
    trigger_task_start(cfg);
    LOG_INF("AAFF cmd start(cfg), req=%u, sample_rate=%u", request_id, cfg.sample_rate_hz);
    return len;
  }

  if (method == AAFF_METHOD_STOP) {
    trigger_task_stop();
    LOG_INF("AAFF cmd stop, req=%u (debounced)", request_id);
    return len;
  }

  LOG_WRN("AAFF cmd unsupported: 0x%02x, req=%u", method, request_id);

  return len;
}

int ble_aaff_notify_commit(uint8_t * data, uint8_t len)
{
  // copy:
  for (int i = 0; i < len; i++) {
    state_aaf.stream_buffer[i] = data[i];
  }

  state_aaf.stream_buffer_len = len;
  state_aaf.stream_ready = true;
  return 0;
}

int ble_aaff_notify_commit_mpu(uint32_t seq, mpu_eeg_data_t * data, uint8_t len, uint16_t channel_size)
{
  uint16_t payload_len;

  if ((len == 0u) || (channel_size == 0u)) {
    return -EINVAL;
  }

  if (state_aaf.stream_ready) {
    return -EBUSY;
  }

  payload_len = (uint16_t)(8u + ((uint16_t)len * channel_size * 2u));
  if (payload_len > STREAM_BUFFER_MAX_LEN) {
    return -EMSGSIZE;
  }

  uint16_t index = 0;
  // spec code:
  state_aaf.stream_buffer[index++] = 0x73;
  // seq:
  state_aaf.stream_buffer[index++] = (seq >> 24) & 0xFF;
  state_aaf.stream_buffer[index++] = (seq >> 16) & 0xFF;
  state_aaf.stream_buffer[index++] = (seq >> 8) & 0xFF;
  state_aaf.stream_buffer[index++] = (seq) & 0xFF;
  // len:
  state_aaf.stream_buffer[index++] = len;                           // frame length.
  // ch:
  state_aaf.stream_buffer[index++] = (channel_size >> 8) & 0xFF;    // channel size l.
  state_aaf.stream_buffer[index++] = (channel_size & 0xFF);         // channel size r.

  uint16_t temp = 0;
  for (uint16_t i = 0; i < len; i++) {
    for (uint16_t j = 0; j < channel_size; j++) {
      temp = data[i].volt[j];
      state_aaf.stream_buffer[index++] = (temp >> 8) & 0xFF; // volt l.
      state_aaf.stream_buffer[index++] = (temp & 0xFF);      // volt r.
    }
  }
  state_aaf.stream_buffer_len = index;
  state_aaf.stream_ready = true;
  return 0;
}

int ble_aaff_loop()
{
  // if ready
  if (!state_aaf.aaff_notify_enabled) {
    return -1;
  }
  if (!state_aaf.stream_ready) {
    return -2;
  }
  // printk("len: %u\n", state_aaf.stream_buffer_len);
  int err = bt_gatt_notify(NULL, &aaaa_service.attrs[EEG_AAFF_INDEX], (uint8_t*)&(state_aaf.stream_buffer), state_aaf.stream_buffer_len);
  if (err) {
    // retry once:
    // printk("Failed to notify EEG data: %d\n", err);
    err = bt_gatt_notify(NULL, &aaaa_service.attrs[EEG_AAFF_INDEX], (uint8_t*)&(state_aaf.stream_buffer), state_aaf.stream_buffer_len);
    if (err) {
      return err;
    }
  }
  state_aaf.stream_ready = false;
  return 0;
}

bool ble_aaff_can_accept_frame(void)
{
  return state_aaf.aaff_notify_enabled && !state_aaf.stream_ready;
}

int ble_aaff_init()
{
  k_work_init_delayable(&aaff_ctrl_work, ble_aaff_ctrl_work_handler);
  return 0;
}

#endif
