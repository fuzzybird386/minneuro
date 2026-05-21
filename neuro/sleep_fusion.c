#include "neuro/inc/sleep_fusion.h"

#include <errno.h>
#include <string.h>

static float sleep_fusion_clampf(float v, float lo, float hi)
{
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

void sleep_fusion_context_init(sleep_fusion_context_t *ctx)
{
  if (ctx == NULL) {
    return;
  }

  memset(ctx, 0, sizeof(*ctx));
  ctx->initialized = true;
  ctx->previous_stage = SLEEP_STAGE_UNKNOWN;
}

int sleep_fusion_analyze(sleep_fusion_context_t *ctx,
                         const sleep_fusion_inputs_t *inputs,
                         sleep_fusion_result_t *result)
{
  const eeg_algo_result_t *eeg;
  float stage_bonus = 0.0f;
  float residual = 0.0f;

  if (ctx == NULL || inputs == NULL || result == NULL || inputs->eeg == NULL) {
    return -EINVAL;
  }
  if (!ctx->initialized) {
    sleep_fusion_context_init(ctx);
  }

  eeg = inputs->eeg;
  memset(result, 0, sizeof(*result));

  result->stage = eeg->stage;
  result->deep_sleep_score = eeg->sleep_depth_score;
  result->arousal_risk_score = eeg->arousal_risk_score;
  result->stability_score =
    sleep_fusion_clampf((0.50f * eeg->sleep_continuity_score) +
                        (0.25f * eeg->signal_quality_score) +
                        (0.25f * (1.0f - eeg->arousal_risk_score)),
                        0.0f,
                        1.0f);

  switch (eeg->stage.stage) {
  case SLEEP_STAGE_N2:
    stage_bonus = 0.10f;
    break;
  case SLEEP_STAGE_N3:
    stage_bonus = 0.20f;
    break;
  case SLEEP_STAGE_REM:
    stage_bonus = -0.10f;
    break;
  case SLEEP_STAGE_W:
    stage_bonus = -0.20f;
    break;
  case SLEEP_STAGE_N1:
    stage_bonus = -0.05f;
    break;
  default:
    stage_bonus = -0.15f;
    break;
  }

  result->sleep_quality_score =
    sleep_fusion_clampf((0.30f * eeg->signal_quality_score) +
                        (0.25f * eeg->sleep_continuity_score) +
                        (0.25f * eeg->sleep_depth_score) +
                        (0.20f * (1.0f - eeg->arousal_risk_score)) +
                        stage_bonus,
                        0.0f,
                        1.0f);

  residual = (ctx->fused_frame_count > 0u) ?
             (result->sleep_quality_score - ctx->rolling_sleep_quality_score) : 0.0f;
  result->residual_score = residual;

  result->stimulation_suitability_score =
    sleep_fusion_clampf((0.40f * result->sleep_quality_score) +
                        (0.25f * eeg->signal_quality_score) +
                        (0.20f * eeg->stage.confidence) +
                        (0.15f * (1.0f - eeg->arousal_risk_score)) -
                        (0.10f * (eeg->stage.stage == SLEEP_STAGE_W)) -
                        (0.15f * (eeg->stage.stage == SLEEP_STAGE_REM)) +
                        (0.05f * residual),
                        0.0f,
                        1.0f);

  result->recommend_stim =
    (result->stimulation_suitability_score >= 0.52f) &&
    (eeg->signal_quality_score >= 0.35f) &&
    (eeg->stage.stage != SLEEP_STAGE_W) &&
    (eeg->stage.stage != SLEEP_STAGE_REM);

  result->stim_level_10 =
    (uint8_t)(1u + (uint8_t)(sleep_fusion_clampf(result->stimulation_suitability_score, 0.0f, 0.999f) * 9.0f));

  ctx->fused_frame_count++;
  if (ctx->fused_frame_count == 1u) {
    ctx->rolling_sleep_quality_score = result->sleep_quality_score;
    ctx->rolling_suitability_score = result->stimulation_suitability_score;
  } else {
    ctx->rolling_sleep_quality_score =
      0.85f * ctx->rolling_sleep_quality_score + 0.15f * result->sleep_quality_score;
    ctx->rolling_suitability_score =
      0.85f * ctx->rolling_suitability_score + 0.15f * result->stimulation_suitability_score;
  }
  ctx->previous_stage = result->stage.stage;
  ctx->previous_stim_level = (float)result->stim_level_10;

  return 0;
}

