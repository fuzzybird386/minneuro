#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "neuro/inc/eeg_ctrl.h"
#include "neuro/inc/sleep_types.h"

enum {
  EEG_ALGO_MAX_FFT_SIZE = 1024,
};

typedef struct {
  bool initialized;
  uint32_t analyzed_frame_count;
  float long_term_median_frequency_hz;
  float long_term_sigma_ratio;
  float long_term_delta_ratio;
} eeg_algo_context_t;

typedef struct {
  float delta_power;
  float theta_power;
  float alpha_power;
  float sigma_power;
  float beta_power;

  float delta_ratio;
  float theta_ratio;
  float alpha_ratio;
  float sigma_ratio;
  float beta_ratio;

  float rms_uv;
  float variance_uv2;
  float peak_to_peak_uv;
  float envelope_mean_uv;
  float envelope_std_uv;
  float line_length_uv;
  float zero_cross_rate_hz;

  float spectral_centroid_hz;
  float spectral_entropy;
  float peak_frequency_hz;
  float median_frequency_hz;
  float long_term_median_frequency_hz;

  float slow_wave_score;
  float spindle_score;
  float k_complex_score;

  uint16_t slow_wave_count;
  uint16_t spindle_like_count;
  uint16_t k_complex_like_count;

  float clipping_ratio;
  float baseline_drift_uv;
  float high_freq_ratio;
  float signal_quality_score;
  float sleep_depth_score;
  float arousal_risk_score;
  float sleep_continuity_score;

  sleep_stage_result_t stage;
} eeg_algo_result_t;

void eeg_algo_context_init(eeg_algo_context_t *ctx);
int eeg_algo_analyze_frame(eeg_algo_context_t *ctx,
                           const eeg_ctrl_frame_t *frame,
                           eeg_algo_result_t *result);

