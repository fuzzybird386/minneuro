#include "inc/stim_test.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <hal/nrf_gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "inc/bsp.h"
#include "driver/dac80501.h"

#include "neuro/inc/neuro_ctrl.h"

LOG_MODULE_REGISTER(stim_test, LOG_LEVEL_INF);

/*
 * DAC resolution: determined at init by reading DEVICE_ID.
 * 16-bit for DAC80501, 14-bit for DAC70501, 12-bit for DAC60501.
 * We default to 16 for stim_test; adjusted in stim_test_start().
 */
#define STIM_TEST_DAC_RESOLUTION 16
#define STIM_TEST_DAC_MAX_CODE   ((1U << STIM_TEST_DAC_RESOLUTION) - 1U)

enum {
  STIM_TEST_THREAD_STACK_SIZE = 4096,
  STIM_TEST_THREAD_PRIORITY = 6,
  STIM_TEST_SWITCH_LEVEL_STIM = 1,
  STIM_TEST_WAVE_SAMPLES = 64,
  STIM_TEST_FULL_SCALE_CURRENT_UA = 1250,
  STIM_TEST_IDLE_CURRENT_UA = 0,
  STIM_TEST_INTER_STEP_PAUSE_MS = 120,
  STIM_TEST_STARTUP_SWEEP_MS = 1250,
  STIM_TEST_Q15_SCALE = 32767,
};

typedef enum {
  STIM_TEST_WAVE_SINE = 0,
  STIM_TEST_WAVE_SQUARE,
  STIM_TEST_WAVE_TRIANGLE,
  STIM_TEST_WAVE_DC,
} stim_test_waveform_t;

typedef struct {
  stim_test_waveform_t waveform;
  uint16_t frequency_hz;
  uint16_t duration_ms;
  uint16_t amplitude_ua;
  int8_t polarity;
} stim_test_step_t;

struct stim_test_runtime {
  struct k_thread thread;
  bool started;
};

static struct stim_test_runtime s_stim_test;
K_THREAD_STACK_DEFINE(s_stim_test_stack, STIM_TEST_THREAD_STACK_SIZE);

static const stim_test_step_t s_stim_test_sequence[] = {
  {
    .waveform = STIM_TEST_WAVE_SINE,
    .frequency_hz = 100u,
    .duration_ms = 2000u,
    .amplitude_ua = 1000u,
    .polarity = 1,
  },
  {
    .waveform = STIM_TEST_WAVE_SQUARE,
    .frequency_hz = 80u,
    .duration_ms = 2000u,
    .amplitude_ua = 800u,
    .polarity = 1,
  },
  {
    .waveform = STIM_TEST_WAVE_TRIANGLE,
    .frequency_hz = 60u,
    .duration_ms = 2000u,
    .amplitude_ua = 1000u,
    .polarity = 1,
  },
  {
    .waveform = STIM_TEST_WAVE_DC,
    .frequency_hz = 1u,
    .duration_ms = 2000u,
    .amplitude_ua = 500u,
    .polarity = 1,
  },
};

static const int16_t s_stim_test_sine_q15[STIM_TEST_WAVE_SAMPLES] = {
  0, 3212, 6393, 9512, 12539, 15446, 18204, 20787,
  23170, 25329, 27245, 28898, 30273, 31356, 32137, 32609,
  32767, 32609, 32137, 31356, 30273, 28898, 27245, 25329,
  23170, 20787, 18204, 15446, 12539, 9512, 6393, 3212,
  0, -3212, -6393, -9512, -12539, -15446, -18204, -20787,
  -23170, -25329, -27245, -28898, -30273, -31356, -32137, -32609,
  -32767, -32609, -32137, -31356, -30273, -28898, -27245, -25329,
  -23170, -20787, -18204, -15446, -12539, -9512, -6393, -3212,
};

static const int16_t s_stim_test_triangle_q15[STIM_TEST_WAVE_SAMPLES] = {
  -32767, -30719, -28671, -26623, -24575, -22527, -20479, -18431,
  -16384, -14336, -12288, -10240, -8192, -6144, -4096, -2048,
  0, 2048, 4096, 6144, 8192, 10240, 12288, 14336,
  16384, 18431, 20479, 22527, 24575, 26623, 28671, 30719,
  32767, 30719, 28671, 26623, 24575, 22527, 20479, 18431,
  16384, 14336, 12288, 10240, 8192, 6144, 4096, 2048,
  0, -2048, -4096, -6144, -8192, -10240, -12288, -14336,
  -16384, -18431, -20479, -22527, -24575, -26623, -28671, -30719,
};

static const char *stim_test_waveform_name(stim_test_waveform_t waveform)
{
  switch (waveform) {
  case STIM_TEST_WAVE_SQUARE:
    return "square";
  case STIM_TEST_WAVE_TRIANGLE:
    return "triangle";
  case STIM_TEST_WAVE_DC:
    return "dc";
  case STIM_TEST_WAVE_SINE:
  default:
    return "sine";
  }
}

static void stim_test_select_output_path(void)
{
  nrf_gpio_cfg_output(PIN_EEGSTIM_SWITCH);
  nrf_gpio_pin_write(PIN_EEGSTIM_SWITCH, STIM_TEST_SWITCH_LEVEL_STIM);
}

static void stim_test_enable_output(void)
{
  nrf_gpio_cfg_output(PIN_STIM_ENABLE);
  nrf_gpio_pin_set(PIN_STIM_ENABLE);
}

static int32_t stim_test_clamp_current_ua(int32_t current_ua)
{
  if (current_ua > STIM_TEST_FULL_SCALE_CURRENT_UA) {
    return STIM_TEST_FULL_SCALE_CURRENT_UA;
  }
  if (current_ua < -STIM_TEST_FULL_SCALE_CURRENT_UA) {
    return -STIM_TEST_FULL_SCALE_CURRENT_UA;
  }

  return current_ua;
}

//电流换算为DAC数码
static uint16_t stim_test_current_to_dac_code(int32_t current_ua)
{
  int64_t shifted;
  int64_t numerator;
  int64_t denominator;

  current_ua = stim_test_clamp_current_ua(current_ua);
  shifted = (int64_t)current_ua + (int64_t)STIM_TEST_FULL_SCALE_CURRENT_UA;
  numerator = shifted * (int64_t)STIM_TEST_DAC_MAX_CODE;
  denominator = (int64_t)STIM_TEST_FULL_SCALE_CURRENT_UA * 2ll;

  return (uint16_t)((numerator + (denominator / 2ll)) / denominator);
}

static int stim_test_write_current_ua(int32_t current_ua)
{
  return dac80501_set_code(stim_test_current_to_dac_code(current_ua));
}

static int stim_test_hold_code(uint16_t code, uint32_t hold_ms)
{
  int err = dac80501_set_code(code);

  if (err) {
    LOG_ERR("stim test set code 0x%04x failed (%d)", code, err);
    return err;
  }

  k_msleep(hold_ms);
  return 0;
}

static int16_t stim_test_wave_sample_q15(stim_test_waveform_t waveform, size_t sample_index)
{
  switch (waveform) {
  case STIM_TEST_WAVE_SQUARE:
    return (sample_index < (STIM_TEST_WAVE_SAMPLES / 2u)) ? STIM_TEST_Q15_SCALE : -STIM_TEST_Q15_SCALE;
  case STIM_TEST_WAVE_TRIANGLE:
    return s_stim_test_triangle_q15[sample_index % STIM_TEST_WAVE_SAMPLES];
  case STIM_TEST_WAVE_DC:
    return STIM_TEST_Q15_SCALE;
  case STIM_TEST_WAVE_SINE:
  default:
    return s_stim_test_sine_q15[sample_index % STIM_TEST_WAVE_SAMPLES];
  }
}

static int stim_test_play_dc(const stim_test_step_t *step)
{
  int32_t current_ua;
  int err;

  current_ua = (step->polarity >= 0) ? (int32_t)step->amplitude_ua : -(int32_t)step->amplitude_ua;
  err = stim_test_write_current_ua(current_ua);
  if (err) {
    return err;
  }

  k_msleep(step->duration_ms);
  return stim_test_write_current_ua(STIM_TEST_IDLE_CURRENT_UA);
}

static int stim_test_play_periodic(const stim_test_step_t *step)
{
  uint32_t period_us;
  uint32_t cycle_count;
  int32_t signed_amplitude_ua;

  if (step->frequency_hz == 0u) {
    return -EINVAL;
  }

  period_us = 1000000u / ((uint32_t)step->frequency_hz * STIM_TEST_WAVE_SAMPLES);
  if (period_us == 0u) {
    period_us = 1u;
  }

  cycle_count = ((uint32_t)step->duration_ms * (uint32_t)step->frequency_hz + 999u) / 1000u;
  if (cycle_count == 0u) {
    cycle_count = 1u;
  }

  signed_amplitude_ua =
    (step->polarity >= 0) ? (int32_t)step->amplitude_ua : -(int32_t)step->amplitude_ua;

  for (uint32_t cycle = 0u; cycle < cycle_count; ++cycle) {
    for (size_t i = 0u; i < STIM_TEST_WAVE_SAMPLES; ++i) {
      int16_t sample_q15 = stim_test_wave_sample_q15(step->waveform, i);
      int32_t current_ua =
        (int32_t)(((int64_t)sample_q15 * (int64_t)signed_amplitude_ua) / (int64_t)STIM_TEST_Q15_SCALE);
      int err = stim_test_write_current_ua(current_ua);

      if (err) {
        return err;
      }

      if (!(cycle == (cycle_count - 1u) && i == (STIM_TEST_WAVE_SAMPLES - 1u))) {
        k_busy_wait(period_us);
      }
    }
  }

  return stim_test_write_current_ua(STIM_TEST_IDLE_CURRENT_UA);
}

static int stim_test_play(const stim_test_step_t *step)
{
  if (step == NULL) {
    return -EINVAL;
  }

  if (step->waveform == STIM_TEST_WAVE_DC) {
    return stim_test_play_dc(step);
  }

  return stim_test_play_periodic(step);
}

static void stim_test_thread_main(void *arg1, void *arg2, void *arg3)
{
  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  LOG_INF("stim test loop started");

  while (true) {
    //for (size_t i = 0u; i < ARRAY_SIZE(s_stim_test_sequence); ++i) {
      const stim_test_step_t *step = &s_stim_test_sequence[1];
      int err;

      LOG_INF("stim test %s: %u Hz, %u ms, %u uA",
              stim_test_waveform_name(step->waveform),
              step->frequency_hz,
              step->duration_ms,
              step->amplitude_ua);

      err = stim_test_play(step);
      if (err) {
        LOG_ERR("stim test step %u failed (%d)",1, err);
        (void)stim_test_write_current_ua(STIM_TEST_IDLE_CURRENT_UA);
        k_msleep(200);
        break;
      }
      
      k_msleep(STIM_TEST_INTER_STEP_PAUSE_MS);
    //}
  }
}

int stim_test_start(void)
{
  int err;

  if (s_stim_test.started) {
    return 0;
  }

  memset(&s_stim_test, 0, sizeof(s_stim_test));

  /* Initialize DAC using HAL driver */
  err = dac80501_init(NULL);
  if (err) {
    LOG_ERR("DAC80501 init failed (%d)", err);
    return err;
  }

  err = neuro_ctrl_init();
  if (err) {
    LOG_ERR("neuro ctrl init failed (%d)", err);
    return err;
  }
  neuro_ctrl_set_switch(false);

  stim_test_select_output_path();
  stim_test_enable_output();

  err = stim_test_hold_code(0x0000u, STIM_TEST_STARTUP_SWEEP_MS);
  if (err) {
    return err;
  }
  err = stim_test_hold_code(0x8000u, STIM_TEST_STARTUP_SWEEP_MS);
  if (err) {
    return err;
  }
  err = stim_test_hold_code(0xFFFFu, STIM_TEST_STARTUP_SWEEP_MS);
  if (err) {
    return err;
  }
  err = stim_test_hold_code(stim_test_current_to_dac_code(STIM_TEST_IDLE_CURRENT_UA),
                            STIM_TEST_STARTUP_SWEEP_MS);
  if (err) {
    return err;
  }

  k_thread_create(&s_stim_test.thread,
                  s_stim_test_stack,
                  K_THREAD_STACK_SIZEOF(s_stim_test_stack),
                  stim_test_thread_main,
                  NULL,
                  NULL,
                  NULL,
                  STIM_TEST_THREAD_PRIORITY,
                  0,
                  K_NO_WAIT);
  k_thread_name_set(&s_stim_test.thread, "stim_test");

  s_stim_test.started = true;
  return 0;
}
