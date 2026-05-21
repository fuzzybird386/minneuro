#include "neuro/inc/eeg_ctrl.h"

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>

enum {
  EEG_CTRL_DEFAULT_RATE_HZ = EEG_CTRL_RATE_500HZ,
};

struct eeg_ctrl_runtime {
  struct k_mutex lock;
  bool initialized;
  bool running;
  uint32_t sample_rate_hz;
  uint16_t samples_per_frame;
};

static struct eeg_ctrl_runtime s_eeg_ctrl;

static bool eeg_ctrl_rate_supported(uint32_t sample_rate_hz)
{
  return (sample_rate_hz == EEG_CTRL_RATE_250HZ) ||
         (sample_rate_hz == EEG_CTRL_RATE_500HZ) ||
         (sample_rate_hz == EEG_CTRL_RATE_1000HZ);
}

static uint32_t eeg_ctrl_period_us(uint32_t sample_rate_hz)
{
  if (!eeg_ctrl_rate_supported(sample_rate_hz)) {
    return 0u;
  }

  return 1000000u / sample_rate_hz;
}

static uint16_t eeg_ctrl_frame_samples(uint32_t sample_rate_hz)
{
  if (sample_rate_hz > EEG_CTRL_MAX_SAMPLES_PER_FRAME) {
    return EEG_CTRL_MAX_SAMPLES_PER_FRAME;
  }

  return (uint16_t)sample_rate_hz;
}

static int16_t eeg_ctrl_quantize_uv(float value_uv)
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

int eeg_ctrl_init(void)
{
  if (s_eeg_ctrl.initialized) {
    return 0;
  }

  memset(&s_eeg_ctrl, 0, sizeof(s_eeg_ctrl));
  k_mutex_init(&s_eeg_ctrl.lock);
  s_eeg_ctrl.sample_rate_hz = EEG_CTRL_DEFAULT_RATE_HZ;
  s_eeg_ctrl.samples_per_frame = eeg_ctrl_frame_samples(s_eeg_ctrl.sample_rate_hz);
  s_eeg_ctrl.initialized = true;
  return ks1092_init();
}

int eeg_ctrl_reset(const eeg_ctrl_config_t *cfg)
{
  uint32_t sample_rate_hz = EEG_CTRL_DEFAULT_RATE_HZ;
  int err;

  err = eeg_ctrl_init();
  if (err) {
    return err;
  }

  if (cfg != NULL && cfg->sample_rate_hz != 0u) {
    sample_rate_hz = cfg->sample_rate_hz;
  }
  if (!eeg_ctrl_rate_supported(sample_rate_hz)) {
    return -EINVAL;
  }

  k_mutex_lock(&s_eeg_ctrl.lock, K_FOREVER);
  s_eeg_ctrl.running = false;
  s_eeg_ctrl.sample_rate_hz = sample_rate_hz;
  s_eeg_ctrl.samples_per_frame = eeg_ctrl_frame_samples(sample_rate_hz);
  k_mutex_unlock(&s_eeg_ctrl.lock);

  return ks1092_reset();
}

int eeg_ctrl_start(uint32_t sample_rate_hz)
{
  int err;

  if (!eeg_ctrl_rate_supported(sample_rate_hz)) {
    return -EINVAL;
  }

  err = eeg_ctrl_init();
  if (err) {
    return err;
  }

  k_mutex_lock(&s_eeg_ctrl.lock, K_FOREVER);
  s_eeg_ctrl.sample_rate_hz = sample_rate_hz;
  s_eeg_ctrl.samples_per_frame = eeg_ctrl_frame_samples(sample_rate_hz);
  s_eeg_ctrl.running = true;
  k_mutex_unlock(&s_eeg_ctrl.lock);

  return 0;
}

int eeg_ctrl_stop(void)
{
  if (!s_eeg_ctrl.initialized) {
    return -EACCES;
  }

  k_mutex_lock(&s_eeg_ctrl.lock, K_FOREVER);
  s_eeg_ctrl.running = false;
  k_mutex_unlock(&s_eeg_ctrl.lock);
  return 0;
}

int eeg_ctrl_read_frame(eeg_ctrl_frame_t *frame)
{
  ks1092_eeg_data_t sample;
  uint32_t sample_rate_hz;
  uint32_t period_us;
  uint16_t sample_count;

  if (!s_eeg_ctrl.initialized) {
    return -EACCES;
  }
  if (frame == NULL) {
    return -EINVAL;
  }

  k_mutex_lock(&s_eeg_ctrl.lock, K_FOREVER);
  if (!s_eeg_ctrl.running) {
    k_mutex_unlock(&s_eeg_ctrl.lock);
    return -EAGAIN;
  }
  sample_rate_hz = s_eeg_ctrl.sample_rate_hz;
  sample_count = s_eeg_ctrl.samples_per_frame;
  k_mutex_unlock(&s_eeg_ctrl.lock);

  period_us = eeg_ctrl_period_us(sample_rate_hz);
  if (period_us == 0u || sample_count == 0u) {
    return -EINVAL;
  }

  frame->sample_rate_hz = sample_rate_hz;
  frame->sample_count = sample_count;

  for (uint16_t i = 0u; i < sample_count; ++i) {
    if (ks1092_read_eeg(&sample) != 0) {
      frame->sample_count = i;
      return -EIO;
    }

    frame->samples[i].ch1_uv = eeg_ctrl_quantize_uv(sample.ch1_uv);
    frame->samples[i].ch2_uv = eeg_ctrl_quantize_uv(sample.ch2_uv);

    if ((i + 1u) < sample_count) {
      k_busy_wait(period_us);
    }
  }

  return 0;
}

bool eeg_ctrl_is_running(void)
{
  bool running;

  if (!s_eeg_ctrl.initialized) {
    return false;
  }

  k_mutex_lock(&s_eeg_ctrl.lock, K_FOREVER);
  running = s_eeg_ctrl.running;
  k_mutex_unlock(&s_eeg_ctrl.lock);
  return running;
}
