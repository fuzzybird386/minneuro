#include "inc/ble_debug.h"

#include "inc/minneuro_mode.h"

#if MINNEURO_DEBUG_MODE

#include <errno.h>
#include <string.h>

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "driver/ks1092.h"
#include "neuro/inc/neuro_stim_bank.h"
#include "neuro/inc/wave_ctrl.h"

LOG_MODULE_REGISTER(ble_debug, LOG_LEVEL_INF);

enum {
  DEBUG_SERVICE_INDEX = 0,
  DEBUG_WAVE_CTRL_INDEX = 2,
  DEBUG_WAVE_CTRL_CCC_INDEX = 3,
  DEBUG_EEG_CTRL_INDEX = 5,
  DEBUG_EEG_CTRL_CCC_INDEX = 6,
  DEBUG_EEG_STREAM_INDEX = 8,
  DEBUG_EEG_STREAM_CCC_INDEX = 9,
};

enum {
  DEBUG_EEG_CHANNEL_COUNT = 2,
  DEBUG_EEG_RESOLUTION_BITS = 16,
  DEBUG_EEG_MAX_FRAME_SAMPLES = 12,
  DEBUG_EEG_THREAD_STACK_SIZE = 1024,
  DEBUG_EEG_THREAD_PRIORITY = 5,
  DEBUG_STREAM_BUFFER_MAX_LEN = 128,
  DEBUG_WAVE_CTRL_WIRE_LEN = 9,
  DEBUG_EEG_CTRL_WIRE_LEN = 6,
  DEBUG_WAVE_POLARITY_POSITIVE = 0,
  DEBUG_WAVE_POLARITY_NEGATIVE = 1,
};

typedef enum {
  DEBUG_WAVE_TYPE_SINE = 0,
  DEBUG_WAVE_TYPE_SQUARE = 1,
  DEBUG_WAVE_TYPE_TRIANGLE = 2,
  DEBUG_WAVE_TYPE_DC = 3,
} debug_wave_type_t;

typedef struct {
  uint8_t running;
  uint8_t wave_type;
  uint16_t frequency_hz;
  uint16_t duration_ms;
  uint16_t amplitude_ua;
  uint8_t polarity;
} debug_wave_ctrl_wire_t;

typedef struct {
  uint8_t running;
  uint16_t sample_rate_hz;
  uint8_t frame_samples;
  uint8_t channel_count;
  uint8_t resolution_bits;
} debug_eeg_ctrl_wire_t;

struct debug_eeg_runtime {
  struct k_timer timer;
  struct k_sem tick_sem;
  struct k_thread thread;
  bool available;
  bool initialized;
  bool running;
  bool stream_notify_enabled;
  bool ctrl_notify_enabled;
  uint16_t sample_rate_hz;
  uint8_t frame_samples;
  uint32_t seq;
  uint8_t sample_index;
  int16_t frame[DEBUG_EEG_MAX_FRAME_SAMPLES][DEBUG_EEG_CHANNEL_COUNT];
};

struct debug_runtime {
  struct k_mutex lock;
  bool initialized;
  bool wave_available;
  bool wave_ctrl_notify_enabled;
  wave_ctrl_config_t wave_cfg;
  wave_ctrl_pattern_code_t wave_pattern[128];
  neuro_stim_request_t wave_request;
  struct debug_eeg_runtime eeg;
};

static struct debug_runtime s_debug;
K_THREAD_STACK_DEFINE(s_debug_eeg_stack, DEBUG_EEG_THREAD_STACK_SIZE);

static struct bt_uuid_128 s_debug_service_uuid =
  BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0000AAAB, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb));
static struct bt_uuid_128 s_debug_wave_ctrl_uuid =
  BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0000AA01, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb));
static struct bt_uuid_128 s_debug_eeg_ctrl_uuid =
  BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0000AA02, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb));
static struct bt_uuid_128 s_debug_eeg_stream_uuid =
  BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0000AA03, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb));

static void debug_wave_ctrl_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);
static void debug_eeg_ctrl_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);
static void debug_eeg_stream_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);
static ssize_t debug_wave_ctrl_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    void *buf, uint16_t len, uint16_t offset);
static ssize_t debug_wave_ctrl_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
static ssize_t debug_eeg_ctrl_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset);
static ssize_t debug_eeg_ctrl_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    const void *buf, uint16_t len, uint16_t offset, uint8_t flags);

BT_GATT_SERVICE_DEFINE(debug_service,
  BT_GATT_PRIMARY_SERVICE(&s_debug_service_uuid),
  BT_GATT_CHARACTERISTIC(&s_debug_wave_ctrl_uuid.uuid,
                         BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                         BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                         debug_wave_ctrl_read, debug_wave_ctrl_write, NULL),
  BT_GATT_CCC(debug_wave_ctrl_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
  BT_GATT_CHARACTERISTIC(&s_debug_eeg_ctrl_uuid.uuid,
                         BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                         BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                         debug_eeg_ctrl_read, debug_eeg_ctrl_write, NULL),
  BT_GATT_CCC(debug_eeg_ctrl_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
  BT_GATT_CHARACTERISTIC(&s_debug_eeg_stream_uuid.uuid,
                         BT_GATT_CHRC_NOTIFY,
                         BT_GATT_PERM_NONE,
                         NULL, NULL, NULL),
  BT_GATT_CCC(debug_eeg_stream_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static void put_be16(uint8_t *dst, uint16_t value)
{
  dst[0] = (uint8_t)((value >> 8) & 0xffu);
  dst[1] = (uint8_t)(value & 0xffu);
}

static uint16_t get_be16(const uint8_t *src)
{
  return (uint16_t)(((uint16_t)src[0] << 8) | src[1]);
}

static uint8_t debug_default_frame_samples(uint16_t sample_rate_hz)
{
  uint32_t frame_samples = (sample_rate_hz + 99u) / 100u;

  if (frame_samples == 0u) {
    frame_samples = 1u;
  }
  if (frame_samples > DEBUG_EEG_MAX_FRAME_SAMPLES) {
    frame_samples = DEBUG_EEG_MAX_FRAME_SAMPLES;
  }

  return (uint8_t)frame_samples;
}

static int16_t debug_quantize_uv(float value_uv)
{
  int32_t rounded = (int32_t)(value_uv >= 0.0f ? (value_uv + 0.5f) : (value_uv - 0.5f));

  if (rounded > INT16_MAX) {
    return INT16_MAX;
  }
  if (rounded < INT16_MIN) {
    return INT16_MIN;
  }

  return (int16_t)rounded;
}

static neuro_stim_waveform_t debug_wave_type_to_waveform(uint8_t wave_type)
{
  switch (wave_type) {
  case DEBUG_WAVE_TYPE_SQUARE:
    return NEURO_STIM_WAVEFORM_SQUARE;
  case DEBUG_WAVE_TYPE_TRIANGLE:
    return NEURO_STIM_WAVEFORM_TRIANGLE;
  case DEBUG_WAVE_TYPE_DC:
    return NEURO_STIM_WAVEFORM_DC;
  case DEBUG_WAVE_TYPE_SINE:
  default:
    return NEURO_STIM_WAVEFORM_SINE;
  }
}

static uint8_t debug_waveform_to_wave_type(neuro_stim_waveform_t waveform)
{
  switch (waveform) {
  case NEURO_STIM_WAVEFORM_SQUARE:
    return DEBUG_WAVE_TYPE_SQUARE;
  case NEURO_STIM_WAVEFORM_TRIANGLE:
    return DEBUG_WAVE_TYPE_TRIANGLE;
  case NEURO_STIM_WAVEFORM_DC:
    return DEBUG_WAVE_TYPE_DC;
  case NEURO_STIM_WAVEFORM_SINE:
  default:
    return DEBUG_WAVE_TYPE_SINE;
  }
}

static uint16_t debug_build_wave_ctrl_wire(uint8_t *out)
{
  neuro_stim_request_t request;

  if (out == NULL) {
    return 0u;
  }

  k_mutex_lock(&s_debug.lock, K_FOREVER);
  request = s_debug.wave_request;
  k_mutex_unlock(&s_debug.lock);

  out[0] = wave_ctrl_is_running() ? 1u : 0u;
  out[1] = debug_waveform_to_wave_type(request.waveform);
  put_be16(&out[2], request.frequency_hz);
  put_be16(&out[4], request.duration_ms);
  put_be16(&out[6], request.amplitude_ua);
  out[8] = (request.polarity < 0) ? DEBUG_WAVE_POLARITY_NEGATIVE : DEBUG_WAVE_POLARITY_POSITIVE;
  return DEBUG_WAVE_CTRL_WIRE_LEN;
}

static uint16_t debug_build_eeg_ctrl_wire(uint8_t *out)
{
  struct debug_eeg_runtime *eeg = &s_debug.eeg;

  if (out == NULL) {
    return 0u;
  }

  out[0] = (eeg->available && eeg->running) ? 1u : 0u;
  put_be16(&out[1], eeg->sample_rate_hz);
  out[3] = eeg->frame_samples;
  out[4] = DEBUG_EEG_CHANNEL_COUNT;
  out[5] = DEBUG_EEG_RESOLUTION_BITS;
  return DEBUG_EEG_CTRL_WIRE_LEN;
}

static void debug_notify_wave_ctrl(void)
{
  uint8_t wire[DEBUG_WAVE_CTRL_WIRE_LEN];
  uint16_t wire_len;

  if (!s_debug.wave_ctrl_notify_enabled) {
    return;
  }

  wire_len = debug_build_wave_ctrl_wire(wire);
  (void)bt_gatt_notify(NULL, &debug_service.attrs[DEBUG_WAVE_CTRL_INDEX], wire, wire_len);
}

static void debug_notify_eeg_ctrl(void)
{
  uint8_t wire[DEBUG_EEG_CTRL_WIRE_LEN];
  uint16_t wire_len;

  if (!s_debug.eeg.ctrl_notify_enabled) {
    return;
  }

  wire_len = debug_build_eeg_ctrl_wire(wire);
  (void)bt_gatt_notify(NULL, &debug_service.attrs[DEBUG_EEG_CTRL_INDEX], wire, wire_len);
}

static void debug_eeg_timer_handler(struct k_timer *timer)
{
  ARG_UNUSED(timer);
  k_sem_give(&s_debug.eeg.tick_sem);
}

static void debug_eeg_publish_frame(struct debug_eeg_runtime *eeg)
{
  uint8_t packet[DEBUG_STREAM_BUFFER_MAX_LEN];
  uint16_t index = 0u;

  if (!eeg->stream_notify_enabled || eeg->sample_index == 0u) {
    return;
  }

  packet[index++] = 0x73u;
  packet[index++] = (uint8_t)((eeg->seq >> 24) & 0xffu);
  packet[index++] = (uint8_t)((eeg->seq >> 16) & 0xffu);
  packet[index++] = (uint8_t)((eeg->seq >> 8) & 0xffu);
  packet[index++] = (uint8_t)(eeg->seq & 0xffu);
  packet[index++] = eeg->sample_index;
  put_be16(&packet[index], DEBUG_EEG_CHANNEL_COUNT);
  index += 2u;

  for (uint8_t i = 0u; i < eeg->sample_index; ++i) {
    for (uint8_t ch = 0u; ch < DEBUG_EEG_CHANNEL_COUNT; ++ch) {
      put_be16(&packet[index], (uint16_t)eeg->frame[i][ch]);
      index += 2u;
    }
  }

  (void)bt_gatt_notify(NULL, &debug_service.attrs[DEBUG_EEG_STREAM_INDEX], packet, index);
  eeg->seq++;
}

static void debug_eeg_thread_main(void *arg1, void *arg2, void *arg3)
{
  struct debug_eeg_runtime *eeg = &s_debug.eeg;
  ks1092_eeg_data_t sample;

  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  while (true) {
    (void)k_sem_take(&eeg->tick_sem, K_FOREVER);

    if (!eeg->running) {
      continue;
    }

    if (!eeg->available) {
      continue;
    }

    if (ks1092_read_eeg(&sample) != 0) {
      continue;
    }

    eeg->frame[eeg->sample_index][0] = debug_quantize_uv(sample.ch1_uv);
    eeg->frame[eeg->sample_index][1] = debug_quantize_uv(sample.ch2_uv);
    eeg->sample_index++;

    if (eeg->sample_index >= eeg->frame_samples) {
      debug_eeg_publish_frame(eeg);
      eeg->sample_index = 0u;
    }
  }
}

static int debug_eeg_start(uint16_t sample_rate_hz, uint8_t frame_samples)
{
  uint32_t period_us;

  if ((sample_rate_hz == 0u) || (frame_samples == 0u) ||
      (frame_samples > DEBUG_EEG_MAX_FRAME_SAMPLES)) {
    return -EINVAL;
  }

  period_us = 1000000u / sample_rate_hz;
  if (period_us == 0u) {
    return -EINVAL;
  }

  s_debug.eeg.sample_rate_hz = sample_rate_hz;
  s_debug.eeg.frame_samples = frame_samples;
  s_debug.eeg.sample_index = 0u;
  s_debug.eeg.running = true;
  k_timer_start(&s_debug.eeg.timer, K_USEC(period_us), K_USEC(period_us));
  debug_notify_eeg_ctrl();
  return 0;
}

static int debug_eeg_stop(void)
{
  k_timer_stop(&s_debug.eeg.timer);
  s_debug.eeg.running = false;
  s_debug.eeg.sample_index = 0u;
  debug_notify_eeg_ctrl();
  return 0;
}

static int debug_wave_apply(const debug_wave_ctrl_wire_t *wire)
{
  neuro_stim_buffer_t stim_buffer;
  neuro_stim_plan_t stim_plan;
  neuro_stim_request_t request;
  int err;

  if (wire == NULL) {
    return -EINVAL;
  }

  if (!s_debug.wave_available) {
    return -ENODEV;
  }

  if (wire->running == 0u) {
    err = wave_ctrl_stop();
    if (err == 0) {
      debug_notify_wave_ctrl();
    }
    return err;
  }

  request.waveform = debug_wave_type_to_waveform(wire->wave_type);
  request.frequency_hz = wire->frequency_hz;
  request.duration_ms = wire->duration_ms;
  request.amplitude_ua = wire->amplitude_ua;
  request.polarity = (wire->polarity == DEBUG_WAVE_POLARITY_NEGATIVE) ? -1 : 1;
  request.level_10 = 1u;

  stim_buffer.pattern = s_debug.wave_pattern;
  stim_buffer.pattern_capacity = ARRAY_SIZE(s_debug.wave_pattern);

  err = neuro_stim_bank_generate(&request, &s_debug.wave_cfg, &stim_buffer, &stim_plan);
  if (err) {
    return err;
  }

  err = wave_ctrl_start(&stim_plan.wave_cfg);
  if (err) {
    return err;
  }

  k_mutex_lock(&s_debug.lock, K_FOREVER);
  s_debug.wave_request = request;
  k_mutex_unlock(&s_debug.lock);
  debug_notify_wave_ctrl();
  return 0;
}

static void debug_wave_ctrl_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
  ARG_UNUSED(attr);
  s_debug.wave_ctrl_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static void debug_eeg_ctrl_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
  ARG_UNUSED(attr);
  s_debug.eeg.ctrl_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static void debug_eeg_stream_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
  ARG_UNUSED(attr);
  s_debug.eeg.stream_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static ssize_t debug_wave_ctrl_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    void *buf, uint16_t len, uint16_t offset)
{
  uint8_t wire[DEBUG_WAVE_CTRL_WIRE_LEN];
  uint16_t wire_len = debug_build_wave_ctrl_wire(wire);

  return bt_gatt_attr_read(conn, attr, buf, len, offset, wire, wire_len);
}

static ssize_t debug_wave_ctrl_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
  const uint8_t *data = buf;
  debug_wave_ctrl_wire_t wire;
  int err;

  ARG_UNUSED(conn);
  ARG_UNUSED(attr);
  ARG_UNUSED(offset);
  ARG_UNUSED(flags);

  if (len < DEBUG_WAVE_CTRL_WIRE_LEN) {
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
  }

  if (!s_debug.wave_available) {
    return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
  }

  wire.running = data[0];
  wire.wave_type = data[1];
  wire.frequency_hz = get_be16(&data[2]);
  wire.duration_ms = get_be16(&data[4]);
  wire.amplitude_ua = get_be16(&data[6]);
  wire.polarity = data[8];

  err = debug_wave_apply(&wire);
  if (err) {
    LOG_WRN("debug wave apply failed: %d", err);
    return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
  }

  return len;
}

static ssize_t debug_eeg_ctrl_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset)
{
  uint8_t wire[DEBUG_EEG_CTRL_WIRE_LEN];
  uint16_t wire_len = debug_build_eeg_ctrl_wire(wire);

  return bt_gatt_attr_read(conn, attr, buf, len, offset, wire, wire_len);
}

static ssize_t debug_eeg_ctrl_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
  const uint8_t *data = buf;
  uint16_t sample_rate_hz;
  uint8_t frame_samples;
  int err;

  ARG_UNUSED(conn);
  ARG_UNUSED(attr);
  ARG_UNUSED(offset);
  ARG_UNUSED(flags);

  if (len < DEBUG_EEG_CTRL_WIRE_LEN) {
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
  }

  if (!s_debug.eeg.available) {
    return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
  }

  sample_rate_hz = get_be16(&data[1]);
  frame_samples = data[3];

  if (data[0] == 0u) {
    err = debug_eeg_stop();
  } else {
    if (sample_rate_hz == 0u) {
      sample_rate_hz = s_debug.eeg.sample_rate_hz;
    }
    if (frame_samples == 0u) {
      frame_samples = debug_default_frame_samples(sample_rate_hz);
    }
    err = debug_eeg_start(sample_rate_hz, frame_samples);
  }

  if (err) {
    LOG_WRN("debug eeg ctrl failed: %d", err);
    return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
  }

  return len;
}

int ble_debug_init(void)
{
  int err;

  if (s_debug.initialized) {
    return 0;
  }

  memset(&s_debug, 0, sizeof(s_debug));
  k_mutex_init(&s_debug.lock);

  s_debug.wave_cfg.dac_cfg.reference_source = DAC80501_REFERENCE_INTERNAL;
  s_debug.wave_cfg.dac_cfg.output_range = DAC80501_OUTPUT_RANGE_2V50;
  s_debug.wave_cfg.dac_cfg.update_mode = DAC80501_UPDATE_ASYNC;
  s_debug.wave_cfg.dac_cfg.reference_mv = 2500u;
  s_debug.wave_cfg.dac_cfg.initial_code = 0x8000u;
  s_debug.wave_cfg.dac_cfg.enable_on_init = true;
  s_debug.wave_cfg.current_lsb_ua = 20u;
  s_debug.wave_cfg.full_scale_current_ua = 1250u;
  s_debug.wave_cfg.idle_code = 0;

  s_debug.wave_request.waveform = NEURO_STIM_WAVEFORM_SINE;
  s_debug.wave_request.frequency_hz = 8u;
  s_debug.wave_request.duration_ms = 500u;
  s_debug.wave_request.amplitude_ua = 200u;
  s_debug.wave_request.polarity = 1;
  s_debug.wave_request.level_10 = 1u;

  err = wave_ctrl_reset(&s_debug.wave_cfg);
  if (err) {
    LOG_WRN("wave control unavailable (%d), debug wave output disabled", err);
    s_debug.wave_available = false;
  } else {
    s_debug.wave_available = true;
  }

  err = ks1092_reset();
  if (err) {
    LOG_WRN("ks1092 unavailable (%d), debug EEG stream disabled", err);
    s_debug.eeg.available = false;
  } else {
    s_debug.eeg.available = true;
  }

  k_timer_init(&s_debug.eeg.timer, debug_eeg_timer_handler, NULL);
  k_sem_init(&s_debug.eeg.tick_sem, 0, UINT_MAX);
  s_debug.eeg.sample_rate_hz = 250u;
  s_debug.eeg.frame_samples = debug_default_frame_samples(s_debug.eeg.sample_rate_hz);

  k_thread_create(&s_debug.eeg.thread,
                  s_debug_eeg_stack,
                  K_THREAD_STACK_SIZEOF(s_debug_eeg_stack),
                  debug_eeg_thread_main,
                  NULL,
                  NULL,
                  NULL,
                  DEBUG_EEG_THREAD_PRIORITY,
                  0,
                  K_NO_WAIT);
  k_thread_name_set(&s_debug.eeg.thread, "dbg_eeg");

  s_debug.initialized = true;
  LOG_INF("BLE debug service ready (AAAB), wave=%s eeg=%s",
          s_debug.wave_available ? "on" : "off",
          s_debug.eeg.available ? "on" : "off");
  return 0;
}

#else

int ble_debug_init(void)
{
  return 0;
}

#endif
