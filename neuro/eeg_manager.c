#include "neuro/inc/eeg_manager.h"

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(eeg_manager, LOG_LEVEL_INF);


static struct k_timer eeg_timer;
static struct k_mutex eeg_lock;
static bool eeg_lock_initialized;

static eeg_manager_config_t eeg_config = {
  .sample_rate_hz = TASK_EEG_DEFAULT_SAMPLE_RATE_HZ,
};

static eeg_manager_frame_t eeg_frames[2];
static uint8_t eeg_write_frame_index;
static uint8_t eeg_write_sample_index;
static uint8_t eeg_frame_size = 1u;
static uint8_t eeg_stream_interval_ms = 10u;
static atomic_t eeg_ready_frame_index = ATOMIC_INIT(-1);
static atomic_t eeg_sampling_in_progress = ATOMIC_INIT(0);
static bool eeg_initialized;
static bool eeg_running;

static uint32_t eeg_ceil_div_u32(uint32_t numerator, uint32_t denominator)
{
  if (denominator == 0u) {
    return 0u;
  }

  return (numerator + denominator - 1u) / denominator;
}

static uint32_t eeg_period_us(uint32_t sample_rate_hz)
{
  if ((sample_rate_hz == 0u) || (sample_rate_hz > 1000000u)) {
    return 0u;
  }

  return 1000000u / sample_rate_hz;
}

static void eeg_ensure_lock_initialized(void)
{
  if (!eeg_lock_initialized) {
    k_mutex_init(&eeg_lock);
    eeg_lock_initialized = true;
  }
}

static void eeg_reset_buffers(void)
{
  memset(eeg_frames, 0, sizeof(eeg_frames));
  eeg_write_frame_index = 0u;
  eeg_write_sample_index = 0u;
  atomic_set(&eeg_ready_frame_index, -1);
}

static void eeg_update_frame_timing_locked(void)
{
  const uint32_t sample_rate_hz = eeg_config.sample_rate_hz;
  const uint32_t target_interval_ms = 10u;
  uint32_t min_samples;
  uint32_t max_samples;
  uint32_t target_samples;
  uint32_t selected_samples;
  uint32_t interval_ms;

  if (sample_rate_hz == 0u) {
    eeg_frame_size = 1u;
    eeg_stream_interval_ms = 10u;
    return;
  }

  min_samples = eeg_ceil_div_u32(sample_rate_hz * 7u, 1000u);
  max_samples = (sample_rate_hz * 15u) / 1000u;
  target_samples = (sample_rate_hz * target_interval_ms + 500u) / 1000u;

  if (min_samples == 0u) {
    min_samples = 1u;
  }
  if (max_samples == 0u) {
    max_samples = 1u;
  }
  if (target_samples == 0u) {
    target_samples = 1u;
  }

  if (min_samples > TASK_EEG_FRAME_SIZE) {
    min_samples = TASK_EEG_FRAME_SIZE;
  }
  if (max_samples > TASK_EEG_FRAME_SIZE) {
    max_samples = TASK_EEG_FRAME_SIZE;
  }
  if (max_samples < min_samples) {
    max_samples = min_samples;
  }

  selected_samples = target_samples;
  if (selected_samples < min_samples) {
    selected_samples = min_samples;
  }
  if (selected_samples > max_samples) {
    selected_samples = max_samples;
  }

  interval_ms = (selected_samples * 1000u + (sample_rate_hz / 2u)) / sample_rate_hz;
  if (interval_ms == 0u) {
    interval_ms = 1u;
  }
  if (interval_ms > 255u) {
    interval_ms = 255u;
  }

  eeg_frame_size = (uint8_t)selected_samples;
  eeg_stream_interval_ms = (uint8_t)interval_ms;

  LOG_INF("EEG frame size=%u, stream interval=%u ms @ %u Hz",
          eeg_frame_size,
          eeg_stream_interval_ms,
          sample_rate_hz);
}

static int eeg_wait_sampling_idle_locked(void)
{
  for (uint8_t wait_i = 0; wait_i < 20u; ++wait_i) {
    if (atomic_get(&eeg_sampling_in_progress) == 0) {
      break;
    }
    k_busy_wait(200);
  }

  atomic_set(&eeg_sampling_in_progress, 0);
  return 0;
}

static int eeg_stop_sampling_locked(void)
{
  if (!eeg_running) {
    return 0;
  }

  eeg_running = false;
  k_timer_stop(&eeg_timer);
  (void)eeg_wait_sampling_idle_locked();
  return 0;
}

static int eeg_start_sampling_locked(void)
{
  uint32_t period_us = eeg_period_us(eeg_config.sample_rate_hz);

  if (period_us == 0u) {
    return -EINVAL;
  }

  eeg_running = true;
  k_timer_start(&eeg_timer, K_USEC(period_us), K_USEC(period_us));
  return 0;
}

static void eeg_timer_handler(struct k_timer *timer)
{
  ARG_UNUSED(timer);

  if (!eeg_running) {
    return;
  }

  atomic_set(&eeg_sampling_in_progress, 1);
  if (!eeg_running) {
    atomic_set(&eeg_sampling_in_progress, 0);
    return;
  }

  eeg_write_sample_index++;
  if (eeg_write_sample_index < eeg_frame_size) {
    atomic_set(&eeg_sampling_in_progress, 0);
    return;
  }

  eeg_frames[eeg_write_frame_index].sample_count = eeg_frame_size;
  atomic_set(&eeg_ready_frame_index, (atomic_val_t)eeg_write_frame_index);
  eeg_write_frame_index ^= 1u;
  eeg_write_sample_index = 0u;
  atomic_set(&eeg_sampling_in_progress, 0);
}

int eeg_manager_init(void)
{
  eeg_ensure_lock_initialized();

  k_mutex_lock(&eeg_lock, K_FOREVER);

  if (!eeg_initialized) {
    k_timer_init(&eeg_timer, eeg_timer_handler, NULL);
    eeg_initialized = true;
  }

  eeg_reset_buffers();
  eeg_running = false;
  atomic_set(&eeg_sampling_in_progress, 0);

  k_mutex_unlock(&eeg_lock);
  return 0;
}

int eeg_manager_start(eeg_manager_config_t cfg)
{
  int err;

  if (eeg_period_us(cfg.sample_rate_hz) == 0u) {
    return -EINVAL;
  }

  k_mutex_lock(&eeg_lock, K_FOREVER);

  (void)eeg_stop_sampling_locked();

  eeg_config = cfg;
  eeg_update_frame_timing_locked();
  eeg_reset_buffers();

  err = eeg_start_sampling_locked();

  k_mutex_unlock(&eeg_lock);
  return err;
}

int eeg_manager_stop(void)
{
  int err;

  k_mutex_lock(&eeg_lock, K_FOREVER);
  err = eeg_stop_sampling_locked();
  k_mutex_unlock(&eeg_lock);

  return err;
}

int eeg_manager_verify(void)
{
  return 0;
}

int eeg_manager_test_convert_once(void)
{
  return 0;
}

int eeg_manager_read_frame(eeg_manager_frame_t *frame)
{
  atomic_val_t ready_index;

  if (frame == NULL) {
    return -EINVAL;
  }

  ready_index = atomic_get(&eeg_ready_frame_index);
  if (ready_index < 0) {
    return -EAGAIN;
  }

  memcpy(frame, &eeg_frames[ready_index], sizeof(*frame));
  (void)atomic_cas(&eeg_ready_frame_index, ready_index, -1);
  return 0;
}
