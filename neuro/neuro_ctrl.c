#include "neuro/inc/neuro_ctrl.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <hal/nrf_gpio.h>
#include <zephyr/kernel.h>

#include "neuro/inc/eeg_ctrl.h"
#include "neuro/inc/wave_ctrl.h"
#include "src/inc/bsp.h"

enum {
  NEURO_CTRL_THREAD_STACK_SIZE = 3072,
  NEURO_CTRL_THREAD_PRIORITY = 5,
  NEURO_CTRL_DEFAULT_EEG_RATE_HZ = EEG_CTRL_RATE_500HZ,
  NEURO_CTRL_SWITCH_LEVEL_EEG = 0,
  NEURO_CTRL_SWITCH_LEVEL_STIM = 1,
  NEURO_CTRL_POLL_INTERVAL_MS = 20,
  NEURO_CTRL_MAX_STIM_PATTERN_LEN = 128,
};

struct neuro_ctrl_runtime {
  struct k_mutex lock;
  struct k_sem wave_done_sem;
  struct k_thread thread;

  bool initialized;
  bool running;
  bool stop_requested;
  uint32_t cycle_index;

  eeg_algo_context_t eeg_algo_ctx;
  sleep_fusion_context_t fusion_ctx;
  wave_ctrl_config_t wave_cfg;
  wave_ctrl_pattern_code_t stim_pattern[NEURO_CTRL_MAX_STIM_PATTERN_LEN];
  neuro_ctrl_state_t state;
};

static struct neuro_ctrl_runtime s_neuro_ctrl;
K_THREAD_STACK_DEFINE(s_neuro_ctrl_stack, NEURO_CTRL_THREAD_STACK_SIZE);

static void neuro_ctrl_set_phase(neuro_ctrl_phase_t phase)
{
  k_mutex_lock(&s_neuro_ctrl.lock, K_FOREVER);
  s_neuro_ctrl.state.phase = phase;
  s_neuro_ctrl.state.running = s_neuro_ctrl.running;
  s_neuro_ctrl.state.cycle_index = s_neuro_ctrl.cycle_index;
  k_mutex_unlock(&s_neuro_ctrl.lock);
}

//eeg与stim切换开关和原理图描述相反，原理图中：stim---0  eeg----1
//暂时引出switch函数，用于stim_test.c中
void neuro_ctrl_set_switch(bool eeg_mode)
{
  nrf_gpio_pin_write(PIN_EEGSTIM_SWITCH,
                     eeg_mode ? NEURO_CTRL_SWITCH_LEVEL_EEG : NEURO_CTRL_SWITCH_LEVEL_STIM);
}

static void neuro_ctrl_wave_done(wave_ctrl_done_reason_t reason, void *user_data)
{
  ARG_UNUSED(reason);
  ARG_UNUSED(user_data);
  k_sem_give(&s_neuro_ctrl.wave_done_sem);
}

static bool neuro_ctrl_should_stop(void)
{
  bool stop_requested;

  k_mutex_lock(&s_neuro_ctrl.lock, K_FOREVER);
  stop_requested = s_neuro_ctrl.stop_requested;
  k_mutex_unlock(&s_neuro_ctrl.lock);

  return stop_requested;
}

static int neuro_ctrl_run_cycle(void)
{
  eeg_ctrl_frame_t frame;
  eeg_algo_result_t eeg_result;
  sleep_fusion_inputs_t fusion_inputs;
  sleep_fusion_result_t fusion_result;
  neuro_stim_request_t stim_request;
  neuro_stim_buffer_t stim_buffer;
  neuro_stim_plan_t stim_plan;
  int err;

  neuro_ctrl_set_phase(NEURO_CTRL_PHASE_EEG_ACQUIRE);
  neuro_ctrl_set_switch(true);

  err = eeg_ctrl_start(NEURO_CTRL_DEFAULT_EEG_RATE_HZ);
  if (err) {
    return err;
  }

  while (!neuro_ctrl_should_stop()) {
    err = eeg_ctrl_read_frame(&frame);
    if (err == 0) {
      break;
    }
    if (err != -EAGAIN) {
      (void)eeg_ctrl_stop();
      return err;
    }
    k_msleep(NEURO_CTRL_POLL_INTERVAL_MS);
  }

  (void)eeg_ctrl_stop();
  if (neuro_ctrl_should_stop()) {
    return 0;
  }

  neuro_ctrl_set_phase(NEURO_CTRL_PHASE_ANALYZE);
  err = eeg_algo_analyze_frame(&s_neuro_ctrl.eeg_algo_ctx, &frame, &eeg_result);
  if (err) {
    return err;
  }

  fusion_inputs.eeg = &eeg_result;
  fusion_inputs.ppg = NULL;
  fusion_inputs.imu = NULL;
  err = sleep_fusion_analyze(&s_neuro_ctrl.fusion_ctx, &fusion_inputs, &fusion_result);
  if (err) {
    return err;
  }

  k_mutex_lock(&s_neuro_ctrl.lock, K_FOREVER);
  s_neuro_ctrl.state.last_eeg_result = eeg_result;
  s_neuro_ctrl.state.last_fusion_result = fusion_result;
  s_neuro_ctrl.state.last_cycle_stimulated = false;
  memset(&s_neuro_ctrl.state.last_stim_request, 0, sizeof(s_neuro_ctrl.state.last_stim_request));
  k_mutex_unlock(&s_neuro_ctrl.lock);

  err = neuro_stim_bank_match(&fusion_result, &stim_request);
  if (err != 0) {
    neuro_ctrl_set_phase(NEURO_CTRL_PHASE_SKIP_STIM);
    return 0;
  }

  stim_buffer.pattern = s_neuro_ctrl.stim_pattern;
  stim_buffer.pattern_capacity = ARRAY_SIZE(s_neuro_ctrl.stim_pattern);
  err = neuro_stim_bank_generate(&stim_request, &s_neuro_ctrl.wave_cfg, &stim_buffer, &stim_plan);
  if (err) {
    neuro_ctrl_set_phase(NEURO_CTRL_PHASE_SKIP_STIM);
    return 0;
  }

  stim_plan.wave_cfg.on_done = neuro_ctrl_wave_done;
  stim_plan.wave_cfg.user_data = NULL;

  k_mutex_lock(&s_neuro_ctrl.lock, K_FOREVER);
  s_neuro_ctrl.state.last_stim_request = stim_request;
  s_neuro_ctrl.state.last_cycle_stimulated = true;
  k_mutex_unlock(&s_neuro_ctrl.lock);

  neuro_ctrl_set_phase(NEURO_CTRL_PHASE_STIMULATE);
  neuro_ctrl_set_switch(false);
  k_sem_reset(&s_neuro_ctrl.wave_done_sem);

  err = wave_ctrl_start(&stim_plan.wave_cfg);
  if (err) {
    neuro_ctrl_set_switch(true);
    return err;
  }

  while (!neuro_ctrl_should_stop()) {
    if (k_sem_take(&s_neuro_ctrl.wave_done_sem, K_MSEC(NEURO_CTRL_POLL_INTERVAL_MS)) == 0) {
      break;
    }
  }

  (void)wave_ctrl_stop();
  neuro_ctrl_set_switch(true);
  return 0;
}

static void neuro_ctrl_thread_main(void *arg1, void *arg2, void *arg3)
{
  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  while (true) {
    if (!s_neuro_ctrl.running) {
      k_msleep(50);
      continue;
    }

    if (neuro_ctrl_run_cycle() == 0) {
      k_mutex_lock(&s_neuro_ctrl.lock, K_FOREVER);
      s_neuro_ctrl.cycle_index++;
      s_neuro_ctrl.state.cycle_index = s_neuro_ctrl.cycle_index;
      k_mutex_unlock(&s_neuro_ctrl.lock);
      continue;
    }

    k_msleep(100);
  }
}

int neuro_ctrl_init(void)
{
  int err;

  if (s_neuro_ctrl.initialized) {
    return 0;
  }

  memset(&s_neuro_ctrl, 0, sizeof(s_neuro_ctrl));

  k_mutex_init(&s_neuro_ctrl.lock);
  k_sem_init(&s_neuro_ctrl.wave_done_sem, 0, 1);
  eeg_algo_context_init(&s_neuro_ctrl.eeg_algo_ctx);
  sleep_fusion_context_init(&s_neuro_ctrl.fusion_ctx);

  nrf_gpio_cfg_output(PIN_EEGSTIM_SWITCH);
  neuro_ctrl_set_switch(true);

  err = eeg_ctrl_reset(NULL);
  if (err) {
    return err;
  }

  s_neuro_ctrl.wave_cfg.dac_cfg.reference_source = DAC80501_REFERENCE_INTERNAL;
  s_neuro_ctrl.wave_cfg.dac_cfg.output_range = DAC80501_OUTPUT_RANGE_2V50;
  s_neuro_ctrl.wave_cfg.dac_cfg.update_mode = DAC80501_UPDATE_ASYNC;
  s_neuro_ctrl.wave_cfg.dac_cfg.reference_mv = 2500u;
  s_neuro_ctrl.wave_cfg.dac_cfg.enable_on_init = true;
  s_neuro_ctrl.wave_cfg.current_lsb_ua = 20u;
  s_neuro_ctrl.wave_cfg.full_scale_current_ua = 1250u;
  s_neuro_ctrl.wave_cfg.idle_code = 0;

  err = wave_ctrl_reset(&s_neuro_ctrl.wave_cfg);
  if (err) {
    return err;
  }

  k_thread_create(&s_neuro_ctrl.thread,
                  s_neuro_ctrl_stack,
                  K_THREAD_STACK_SIZEOF(s_neuro_ctrl_stack),
                  neuro_ctrl_thread_main,
                  NULL,
                  NULL,
                  NULL,
                  NEURO_CTRL_THREAD_PRIORITY,
                  0,
                  K_NO_WAIT);
  k_thread_name_set(&s_neuro_ctrl.thread, "neuro_ctrl");

  s_neuro_ctrl.state.phase = NEURO_CTRL_PHASE_IDLE;
  s_neuro_ctrl.initialized = true;
  return 0;
}

int neuro_ctrl_start(void)
{
  int err = neuro_ctrl_init();

  if (err) {
    return err;
  }

  k_mutex_lock(&s_neuro_ctrl.lock, K_FOREVER);
  s_neuro_ctrl.stop_requested = false;
  s_neuro_ctrl.running = true;
  s_neuro_ctrl.state.running = true;
  k_mutex_unlock(&s_neuro_ctrl.lock);

  return 0;
}

int neuro_ctrl_stop(void)
{
  if (!s_neuro_ctrl.initialized) {
    return -EACCES;
  }

  k_mutex_lock(&s_neuro_ctrl.lock, K_FOREVER);
  s_neuro_ctrl.stop_requested = true;
  s_neuro_ctrl.running = false;
  s_neuro_ctrl.state.running = false;
  s_neuro_ctrl.state.phase = NEURO_CTRL_PHASE_IDLE;
  k_mutex_unlock(&s_neuro_ctrl.lock);

  (void)eeg_ctrl_stop();
  (void)wave_ctrl_stop();
  neuro_ctrl_set_switch(true);
  return 0;
}

int neuro_ctrl_get_state(neuro_ctrl_state_t *state)
{
  if (!s_neuro_ctrl.initialized) {
    return -EACCES;
  }
  if (state == NULL) {
    return -EINVAL;
  }

  k_mutex_lock(&s_neuro_ctrl.lock, K_FOREVER);
  *state = s_neuro_ctrl.state;
  k_mutex_unlock(&s_neuro_ctrl.lock);
  return 0;
}
