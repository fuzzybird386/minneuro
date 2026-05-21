#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "neuro/inc/sleep_fusion.h"
#include "neuro/inc/wave_ctrl.h"

typedef enum {
  NEURO_STIM_WAVEFORM_SINE = 0,
  NEURO_STIM_WAVEFORM_SQUARE,
  NEURO_STIM_WAVEFORM_TRIANGLE,
  NEURO_STIM_WAVEFORM_DC,
} neuro_stim_waveform_t;

typedef struct {
  neuro_stim_waveform_t waveform;
  uint16_t frequency_hz;
  uint16_t duration_ms;
  uint16_t amplitude_ua;
  int8_t polarity;
  uint8_t level_10;
} neuro_stim_request_t;

typedef struct {
  wave_ctrl_pattern_code_t *pattern;
  size_t pattern_capacity;
} neuro_stim_buffer_t;

typedef struct {
  bool valid;
  neuro_stim_request_t request;
  wave_ctrl_wave_cfg_t wave_cfg;
} neuro_stim_plan_t;

int neuro_stim_bank_match(const sleep_fusion_result_t *fusion, neuro_stim_request_t *request);
int neuro_stim_bank_generate(const neuro_stim_request_t *request,
                             const wave_ctrl_config_t *wave_cfg,
                             neuro_stim_buffer_t *buffer,
                             neuro_stim_plan_t *plan);

