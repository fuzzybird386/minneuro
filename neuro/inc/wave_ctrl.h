#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/dac80501.h"

/*
 * Wave controller
 * ---------------
 * The output path is:
 *   DAC80501: 0.0 V .. 2.5 V
 *   External subtractor: -1.25 V bias
 *   Effective output/current command: -1.25 mA .. +1.25 mA
 *
 * Pattern samples are represented as signed "current codes":
 *   physical_current_uA = sample_code * current_lsb_ua
 *
 * With the default configuration:
 *   current_lsb_ua = 20
 * so one pattern step represents 20 uA.
 */

typedef int16_t wave_ctrl_pattern_code_t;

typedef enum {
  WAVE_CTRL_DONE_COMPLETED = 0,
  WAVE_CTRL_DONE_STOPPED,
  WAVE_CTRL_DONE_ERROR,
} wave_ctrl_done_reason_t;

typedef void (*wave_ctrl_on_done_t)(wave_ctrl_done_reason_t reason, void *user_data);

typedef struct {
  dac80501_config_t dac_cfg;
  uint16_t current_lsb_ua;
  uint16_t full_scale_current_ua;
  wave_ctrl_pattern_code_t idle_code;
} wave_ctrl_config_t;

typedef struct {
  const wave_ctrl_pattern_code_t *pattern;
  size_t pattern_len;
  uint32_t period_us;
  /* The whole pattern is played repeat_count times. */
  uint32_t repeat_count;
  wave_ctrl_on_done_t on_done;
  void *user_data;
} wave_ctrl_wave_cfg_t;

int wave_ctrl_init(void);
int wave_ctrl_reset(const wave_ctrl_config_t *cfg);
int wave_ctrl_start(const wave_ctrl_wave_cfg_t *cfg);
int wave_ctrl_stop(void);

bool wave_ctrl_is_running(void);
