#include "neuro/inc/neuro_stim_bank.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#include <zephyr/sys/util.h>

#ifndef NEURO_STIM_PI
#define NEURO_STIM_PI 3.14159265358979323846f
#endif

typedef struct {
  sleep_stage_t preferred_stage;
  neuro_stim_waveform_t waveform;
  uint16_t frequency_hz;
  uint16_t duration_ms;
  uint8_t base_level_10;
  float min_suitability;
} neuro_stim_profile_t;

static const uint16_t s_level_to_amplitude_ua[10] = {
  100u, 150u, 220u, 300u, 380u,
  480u, 600u, 720u, 860u, 1000u,
};

static const neuro_stim_profile_t s_profiles[] = {
  { SLEEP_STAGE_N1,  NEURO_STIM_WAVEFORM_SINE,     8u,  400u, 3u, 0.58f },
  { SLEEP_STAGE_N1,  NEURO_STIM_WAVEFORM_SQUARE,  10u,  300u, 3u, 0.60f },
  { SLEEP_STAGE_N2,  NEURO_STIM_WAVEFORM_SINE,     1u, 1000u, 4u, 0.55f },
  { SLEEP_STAGE_N2,  NEURO_STIM_WAVEFORM_TRIANGLE, 2u,  800u, 4u, 0.56f },
  { SLEEP_STAGE_N2,  NEURO_STIM_WAVEFORM_SQUARE,   5u,  300u, 5u, 0.60f },
  { SLEEP_STAGE_N2,  NEURO_STIM_WAVEFORM_SINE,    12u,  400u, 4u, 0.62f },
  { SLEEP_STAGE_N3,  NEURO_STIM_WAVEFORM_SINE,     1u, 1000u, 5u, 0.52f },
  { SLEEP_STAGE_N3,  NEURO_STIM_WAVEFORM_TRIANGLE, 1u,  900u, 5u, 0.54f },
  { SLEEP_STAGE_N3,  NEURO_STIM_WAVEFORM_DC,       1u,  200u, 2u, 0.65f },
  { SLEEP_STAGE_REM, NEURO_STIM_WAVEFORM_SINE,     6u,  200u, 2u, 0.75f },
  { SLEEP_STAGE_W,   NEURO_STIM_WAVEFORM_SINE,     4u,  200u, 1u, 0.85f },
};

static float neuro_stim_clampf(float v, float lo, float hi)
{
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

static uint8_t neuro_stim_apply_level_bounds(uint8_t level_10)
{
  if (level_10 < 1u) {
    return 1u;
  }
  if (level_10 > 10u) {
    return 10u;
  }
  return level_10;
}

int neuro_stim_bank_match(const sleep_fusion_result_t *fusion, neuro_stim_request_t *request)
{
  const neuro_stim_profile_t *best = NULL;
  float best_score = -1000.0f;

  if (fusion == NULL || request == NULL) {
    return -EINVAL;
  }

  memset(request, 0, sizeof(*request));

  if (!fusion->recommend_stim) {
    return -EAGAIN;
  }

  for (size_t i = 0u; i < ARRAY_SIZE(s_profiles); ++i) {
    const neuro_stim_profile_t *profile = &s_profiles[i];
    float score = fusion->stimulation_suitability_score - profile->min_suitability;

    if (profile->preferred_stage == fusion->stage.stage) {
      score += 0.25f;
    }
    if (fusion->stage.stage == SLEEP_STAGE_N3 &&
        profile->waveform == NEURO_STIM_WAVEFORM_SINE &&
        profile->frequency_hz == 1u) {
      score += 0.10f;
    }
    score += 0.10f * fusion->deep_sleep_score;
    score -= 0.12f * fusion->arousal_risk_score;
    score += 0.05f * fusion->residual_score;

    if (score > best_score) {
      best_score = score;
      best = profile;
    }
  }

  if (best == NULL || best_score < 0.0f) {
    return -EAGAIN;
  }

  request->waveform = best->waveform;
  request->frequency_hz = best->frequency_hz;
  request->duration_ms = best->duration_ms;
  request->level_10 = neuro_stim_apply_level_bounds((uint8_t)((best->base_level_10 + fusion->stim_level_10) / 2u));
  request->amplitude_ua = s_level_to_amplitude_ua[request->level_10 - 1u];
  request->polarity = 1;
  return 0;
}

int neuro_stim_bank_generate(const neuro_stim_request_t *request,
                             const wave_ctrl_config_t *wave_cfg,
                             neuro_stim_buffer_t *buffer,
                             neuro_stim_plan_t *plan)
{
  size_t samples_per_cycle;
  uint32_t period_us;
  uint32_t repeat_count;
  int32_t amplitude_code;

  if (request == NULL || wave_cfg == NULL || buffer == NULL || plan == NULL ||
      buffer->pattern == NULL || buffer->pattern_capacity == 0u) {
    return -EINVAL;
  }
  if (request->frequency_hz == 0u || wave_cfg->current_lsb_ua == 0u) {
    return -EINVAL;
  }

  memset(plan, 0, sizeof(*plan));

  if (request->waveform == NEURO_STIM_WAVEFORM_DC) {
    samples_per_cycle = 1u;
    period_us = 1000u;
    repeat_count = request->duration_ms;
  } else {
    samples_per_cycle = 64u;
    if (samples_per_cycle > buffer->pattern_capacity) {
      return -ENOSPC;
    }
    period_us = 1000000u / ((uint32_t)request->frequency_hz * (uint32_t)samples_per_cycle);
    if (period_us == 0u) {
      period_us = 1u;
    }
    repeat_count = ((uint32_t)request->duration_ms * (uint32_t)request->frequency_hz + 999u) / 1000u;
    if (repeat_count == 0u) {
      repeat_count = 1u;
    }
  }

  amplitude_code = (int32_t)(request->amplitude_ua / wave_cfg->current_lsb_ua);
  if (amplitude_code <= 0) {
    amplitude_code = 1;
  }

  if (request->waveform == NEURO_STIM_WAVEFORM_DC) {
    buffer->pattern[0] = (wave_ctrl_pattern_code_t)(request->polarity >= 0 ? amplitude_code : -amplitude_code);
  } else {
    for (size_t i = 0u; i < samples_per_cycle; ++i) {
      float phase = (2.0f * NEURO_STIM_PI * (float)i) / (float)samples_per_cycle;
      float sample = 0.0f;

      switch (request->waveform) {
      case NEURO_STIM_WAVEFORM_SINE:
        sample = sinf(phase);
        break;
      case NEURO_STIM_WAVEFORM_SQUARE:
        sample = (i < (samples_per_cycle / 2u)) ? 1.0f : -1.0f;
        break;
      case NEURO_STIM_WAVEFORM_TRIANGLE:
      {
        float x = (float)i / (float)samples_per_cycle;
        sample = 4.0f * fabsf(x - 0.5f) - 1.0f;
        sample = -sample;
        break;
      }
      default:
        sample = 0.0f;
        break;
      }

      sample *= (float)(request->polarity >= 0 ? amplitude_code : -amplitude_code);
      buffer->pattern[i] = (wave_ctrl_pattern_code_t)lroundf(sample);
    }
  }

  plan->valid = true;
  plan->request = *request;
  plan->wave_cfg.pattern = buffer->pattern;
  plan->wave_cfg.pattern_len = samples_per_cycle;
  plan->wave_cfg.period_us = period_us;
  plan->wave_cfg.repeat_count = repeat_count;
  plan->wave_cfg.on_done = NULL;
  plan->wave_cfg.user_data = NULL;
  return 0;
}
