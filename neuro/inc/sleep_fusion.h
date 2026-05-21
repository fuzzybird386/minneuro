#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "neuro/inc/eeg_algo.h"
#include "neuro/inc/sleep_types.h"

typedef struct {
  bool initialized;
  uint32_t fused_frame_count;
  float rolling_sleep_quality_score;
  float rolling_suitability_score;
  sleep_stage_t previous_stage;
  float previous_stim_level;
} sleep_fusion_context_t;

typedef struct {
  const eeg_algo_result_t *eeg;
  const void *ppg;
  const void *imu;
} sleep_fusion_inputs_t;

typedef struct {
  sleep_stage_result_t stage;
  float sleep_quality_score;
  float stimulation_suitability_score;
  float residual_score;
  float deep_sleep_score;
  float arousal_risk_score;
  float stability_score;
  bool recommend_stim;
  uint8_t stim_level_10;
} sleep_fusion_result_t;

void sleep_fusion_context_init(sleep_fusion_context_t *ctx);
int sleep_fusion_analyze(sleep_fusion_context_t *ctx,
                         const sleep_fusion_inputs_t *inputs,
                         sleep_fusion_result_t *result);

