#include "neuro/inc/wave_ctrl.h"

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>

enum {
  WAVE_CTRL_THREAD_STACK_SIZE = 1024,
  WAVE_CTRL_THREAD_PRIORITY = 5,
  WAVE_CTRL_DEFAULT_CURRENT_LSB_UA = 20u,
  WAVE_CTRL_DEFAULT_FULL_SCALE_CURRENT_UA = 1250u,
};

struct wave_ctrl_runtime {
  struct k_mutex lock;
  struct k_timer timer;
  struct k_sem tick_sem;
  struct k_work done_work;
  struct k_thread thread;

  bool initialized;
  bool running;

  wave_ctrl_config_t config;
  wave_ctrl_wave_cfg_t wave;

  size_t next_index;
  uint32_t repeats_remaining;
  wave_ctrl_done_reason_t done_reason;
};

static struct wave_ctrl_runtime s_wave_ctrl;
K_THREAD_STACK_DEFINE(s_wave_ctrl_stack, WAVE_CTRL_THREAD_STACK_SIZE);

static uint16_t wave_ctrl_pattern_to_dac_code_cfg(const wave_ctrl_config_t *cfg,
                                                  wave_ctrl_pattern_code_t pattern_code)
{
  int32_t current_ua;
  int32_t full_scale_ua;
  int64_t numerator;
  int64_t denominator;
  int64_t shifted;

  full_scale_ua = (int32_t)cfg->full_scale_current_ua;
  if (full_scale_ua <= 0) {
    return 0u;
  }

  current_ua = (int32_t)pattern_code * (int32_t)cfg->current_lsb_ua;

  if (current_ua > full_scale_ua) {
    current_ua = full_scale_ua;
  } else if (current_ua < -full_scale_ua) {
    current_ua = -full_scale_ua;
  }

  shifted = (int64_t)current_ua + (int64_t)full_scale_ua;
  numerator = shifted * 65535ll;
  denominator = (int64_t)full_scale_ua * 2ll;

  return (uint16_t)((numerator + (denominator / 2ll)) / denominator);
}

static uint16_t wave_ctrl_pattern_to_dac_code(wave_ctrl_pattern_code_t pattern_code)
{
  return wave_ctrl_pattern_to_dac_code_cfg(&s_wave_ctrl.config, pattern_code);
}

static void wave_ctrl_drain_ticks(void)
{
  while (k_sem_take(&s_wave_ctrl.tick_sem, K_NO_WAIT) == 0) {
  }
}

static void wave_ctrl_done_work_handler(struct k_work *work)
{
  wave_ctrl_on_done_t on_done;
  void *user_data;
  wave_ctrl_done_reason_t reason;

  ARG_UNUSED(work);

  k_mutex_lock(&s_wave_ctrl.lock, K_FOREVER);
  on_done = s_wave_ctrl.wave.on_done;
  user_data = s_wave_ctrl.wave.user_data;
  reason = s_wave_ctrl.done_reason;
  k_mutex_unlock(&s_wave_ctrl.lock);

  if (on_done != NULL) {
    on_done(reason, user_data);
  }
}

static void wave_ctrl_timer_handler(struct k_timer *timer)
{
  ARG_UNUSED(timer);
  k_sem_give(&s_wave_ctrl.tick_sem);
}

static void wave_ctrl_thread_main(void *arg1, void *arg2, void *arg3)
{
  wave_ctrl_pattern_code_t sample;
  wave_ctrl_done_reason_t done_reason;
  bool notify_done;
  bool running;
  int err;

  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  while (true) {
    k_sem_take(&s_wave_ctrl.tick_sem, K_FOREVER);

    sample = 0;
    notify_done = false;
    done_reason = WAVE_CTRL_DONE_COMPLETED;

    k_mutex_lock(&s_wave_ctrl.lock, K_FOREVER);

    running = s_wave_ctrl.running &&
              (s_wave_ctrl.wave.pattern != NULL) &&
              (s_wave_ctrl.wave.pattern_len > 0u);

    if (!running) {
      k_mutex_unlock(&s_wave_ctrl.lock);
      continue;
    }

    sample = s_wave_ctrl.wave.pattern[s_wave_ctrl.next_index];
    s_wave_ctrl.next_index++;

    if (s_wave_ctrl.next_index >= s_wave_ctrl.wave.pattern_len) {
      s_wave_ctrl.next_index = 0u;

      if (s_wave_ctrl.repeats_remaining > 0u) {
        s_wave_ctrl.repeats_remaining--;
      }

      if (s_wave_ctrl.repeats_remaining == 0u) {
        s_wave_ctrl.running = false;
        done_reason = WAVE_CTRL_DONE_COMPLETED;
        s_wave_ctrl.done_reason = done_reason;
        notify_done = true;
      }
    }

    k_mutex_unlock(&s_wave_ctrl.lock);

    err = dac80501_set_code(wave_ctrl_pattern_to_dac_code(sample));
    if (err != 0) {
      k_timer_stop(&s_wave_ctrl.timer);
      (void)dac80501_set_code(wave_ctrl_pattern_to_dac_code(s_wave_ctrl.config.idle_code));

      k_mutex_lock(&s_wave_ctrl.lock, K_FOREVER);
      s_wave_ctrl.running = false;
      s_wave_ctrl.done_reason = WAVE_CTRL_DONE_ERROR;
      notify_done = true;
      k_mutex_unlock(&s_wave_ctrl.lock);
    }

    if (notify_done) {
      k_timer_stop(&s_wave_ctrl.timer);
      (void)dac80501_set_code(wave_ctrl_pattern_to_dac_code(s_wave_ctrl.config.idle_code));
      k_work_submit(&s_wave_ctrl.done_work);
    }
  }
}

int wave_ctrl_init(void)
{
  if (s_wave_ctrl.initialized) {
    return 0;
  }

  memset(&s_wave_ctrl, 0, sizeof(s_wave_ctrl));

  k_mutex_init(&s_wave_ctrl.lock);
  k_timer_init(&s_wave_ctrl.timer, wave_ctrl_timer_handler, NULL);
  k_sem_init(&s_wave_ctrl.tick_sem, 0, UINT_MAX);
  k_work_init(&s_wave_ctrl.done_work, wave_ctrl_done_work_handler);

  k_thread_create(&s_wave_ctrl.thread,
                  s_wave_ctrl_stack,
                  K_THREAD_STACK_SIZEOF(s_wave_ctrl_stack),
                  wave_ctrl_thread_main,
                  NULL,
                  NULL,
                  NULL,
                  WAVE_CTRL_THREAD_PRIORITY,
                  0,
                  K_NO_WAIT);
  k_thread_name_set(&s_wave_ctrl.thread, "wave_ctrl");

  s_wave_ctrl.initialized = true;
  return 0;
}

int wave_ctrl_reset(const wave_ctrl_config_t *cfg)
{
  wave_ctrl_config_t local_cfg;
  int err;

  err = wave_ctrl_init();
  if (err) {
    return err;
  }

  local_cfg.dac_cfg.reference_source = DAC80501_REFERENCE_INTERNAL;
  local_cfg.dac_cfg.output_range = DAC80501_OUTPUT_RANGE_2V50;
  local_cfg.dac_cfg.update_mode = DAC80501_UPDATE_ASYNC;
  local_cfg.dac_cfg.reference_mv = 2500u;
  local_cfg.dac_cfg.initial_code = 0x8000u;
  local_cfg.dac_cfg.enable_on_init = true;
  local_cfg.current_lsb_ua = WAVE_CTRL_DEFAULT_CURRENT_LSB_UA;
  local_cfg.full_scale_current_ua = WAVE_CTRL_DEFAULT_FULL_SCALE_CURRENT_UA;
  local_cfg.idle_code = 0;

  if (cfg != NULL) {
    local_cfg = *cfg;
  }

  if (local_cfg.current_lsb_ua == 0u) {
    local_cfg.current_lsb_ua = WAVE_CTRL_DEFAULT_CURRENT_LSB_UA;
  }
  if (local_cfg.full_scale_current_ua == 0u) {
    local_cfg.full_scale_current_ua = WAVE_CTRL_DEFAULT_FULL_SCALE_CURRENT_UA;
  }

  local_cfg.dac_cfg.initial_code =
    wave_ctrl_pattern_to_dac_code_cfg(&local_cfg, local_cfg.idle_code);

  k_timer_stop(&s_wave_ctrl.timer);
  wave_ctrl_drain_ticks();

  k_mutex_lock(&s_wave_ctrl.lock, K_FOREVER);
  s_wave_ctrl.running = false;
  memset(&s_wave_ctrl.wave, 0, sizeof(s_wave_ctrl.wave));
  s_wave_ctrl.next_index = 0u;
  s_wave_ctrl.repeats_remaining = 0u;
  s_wave_ctrl.done_reason = WAVE_CTRL_DONE_COMPLETED;
  s_wave_ctrl.config = local_cfg;
  k_mutex_unlock(&s_wave_ctrl.lock);

  err = dac80501_init(&local_cfg.dac_cfg);
  if (err) {
    return err;
  }

  return 0;
}

int wave_ctrl_start(const wave_ctrl_wave_cfg_t *cfg)
{
  if (!s_wave_ctrl.initialized) {
    return -EACCES;
  }
  if (cfg == NULL || cfg->pattern == NULL) {
    return -EINVAL;
  }
  if (cfg->pattern_len == 0u || cfg->period_us == 0u || cfg->repeat_count == 0u) {
    return -EINVAL;
  }

  k_timer_stop(&s_wave_ctrl.timer);
  wave_ctrl_drain_ticks();

  k_mutex_lock(&s_wave_ctrl.lock, K_FOREVER);
  s_wave_ctrl.wave = *cfg;
  s_wave_ctrl.next_index = 0u;
  s_wave_ctrl.repeats_remaining = cfg->repeat_count;
  s_wave_ctrl.done_reason = WAVE_CTRL_DONE_COMPLETED;
  s_wave_ctrl.running = true;
  k_mutex_unlock(&s_wave_ctrl.lock);

  k_timer_start(&s_wave_ctrl.timer, K_USEC(cfg->period_us), K_USEC(cfg->period_us));
  return 0;
}

int wave_ctrl_stop(void)
{
  int err;

  if (!s_wave_ctrl.initialized) {
    return -EACCES;
  }

  k_timer_stop(&s_wave_ctrl.timer);
  wave_ctrl_drain_ticks();

  k_mutex_lock(&s_wave_ctrl.lock, K_FOREVER);
  s_wave_ctrl.running = false;
  s_wave_ctrl.done_reason = WAVE_CTRL_DONE_STOPPED;
  k_mutex_unlock(&s_wave_ctrl.lock);

  err = dac80501_set_code(wave_ctrl_pattern_to_dac_code(s_wave_ctrl.config.idle_code));
  if (err) {
    return err;
  }

  return 0;
}

bool wave_ctrl_is_running(void)
{
  bool running;

  if (!s_wave_ctrl.initialized) {
    return false;
  }

  k_mutex_lock(&s_wave_ctrl.lock, K_FOREVER);
  running = s_wave_ctrl.running;
  k_mutex_unlock(&s_wave_ctrl.lock);

  return running;
}
