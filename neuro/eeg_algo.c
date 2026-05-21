#include "neuro/inc/eeg_algo.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

enum {
  EEG_ALGO_MIN_FRAME_SAMPLES = 200,
};

#ifndef EEG_ALGO_PI
#define EEG_ALGO_PI 3.14159265358979323846f
#endif

typedef struct {
  float real[EEG_ALGO_MAX_FFT_SIZE];
  float imag[EEG_ALGO_MAX_FFT_SIZE];
} eeg_algo_workspace_t;

static eeg_algo_workspace_t s_ws;

static float eeg_algo_clampf(float value, float min_value, float max_value)
{
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

static uint16_t eeg_algo_next_pow2(uint16_t value)
{
  uint16_t n = 1u;

  while (n < value && n < EEG_ALGO_MAX_FFT_SIZE) {
    n <<= 1;
  }

  return n;
}

static float eeg_algo_signal_mean(const float *x, uint16_t len)
{
  float sum = 0.0f;

  for (uint16_t i = 0; i < len; ++i) {
    sum += x[i];
  }

  return (len > 0u) ? (sum / (float)len) : 0.0f;
}

static float eeg_algo_signal_variance(const float *x, uint16_t len, float mean)
{
  float acc = 0.0f;

  for (uint16_t i = 0; i < len; ++i) {
    float d = x[i] - mean;
    acc += d * d;
  }

  return (len > 0u) ? (acc / (float)len) : 0.0f;
}

static void eeg_algo_prepare_signal(const eeg_ctrl_frame_t *frame, float *dst, uint16_t len)
{
  float mean;

  for (uint16_t i = 0; i < len; ++i) {
    dst[i] = 0.5f * ((float)frame->samples[i].ch1_uv + (float)frame->samples[i].ch2_uv);
  }

  mean = eeg_algo_signal_mean(dst, len);
  for (uint16_t i = 0; i < len; ++i) {
    dst[i] -= mean;
  }
}

static void eeg_algo_compute_time_features(const float *signal,
                                           uint16_t len,
                                           uint32_t sample_rate_hz,
                                           eeg_algo_result_t *result)
{
  float rms_acc = 0.0f;
  float abs_sum = 0.0f;
  float abs_sq_acc = 0.0f;
  float max_value = signal[0];
  float min_value = signal[0];
  float line_length = 0.0f;
  float mean_abs;
  float prev = signal[0];
  uint16_t zero_crosses = 0u;

  for (uint16_t i = 0; i < len; ++i) {
    float v = signal[i];
    float av = fabsf(v);

    rms_acc += v * v;
    abs_sum += av;
    abs_sq_acc += av * av;

    if (v > max_value) {
      max_value = v;
    }
    if (v < min_value) {
      min_value = v;
    }

    if (i > 0u) {
      line_length += fabsf(v - prev);
      if (((v >= 0.0f) && (prev < 0.0f)) || ((v < 0.0f) && (prev >= 0.0f))) {
        zero_crosses++;
      }
      prev = v;
    }
  }

  result->rms_uv = sqrtf(rms_acc / (float)len);
  result->variance_uv2 = eeg_algo_signal_variance(signal, len, 0.0f);
  result->peak_to_peak_uv = max_value - min_value;
  result->line_length_uv = line_length / (float)len;
  result->zero_cross_rate_hz = ((float)zero_crosses * (float)sample_rate_hz) / (float)len;

  mean_abs = abs_sum / (float)len;
  result->envelope_mean_uv = mean_abs;
  result->envelope_std_uv = sqrtf(eeg_algo_clampf((abs_sq_acc / (float)len) - (mean_abs * mean_abs),
                                                  0.0f,
                                                  1.0e12f));
}

static void eeg_algo_fft_bit_reverse(float *real, float *imag, uint16_t n)
{
  uint16_t j = 0u;

  for (uint16_t i = 1u; i < n; ++i) {
    uint16_t bit = n >> 1;

    while (j & bit) {
      j ^= bit;
      bit >>= 1;
    }
    j ^= bit;

    if (i < j) {
      float tr = real[i];
      float ti = imag[i];
      real[i] = real[j];
      imag[i] = imag[j];
      real[j] = tr;
      imag[j] = ti;
    }
  }
}

static void eeg_algo_fft(float *real, float *imag, uint16_t n)
{
  eeg_algo_fft_bit_reverse(real, imag, n);

  for (uint16_t len = 2u; len <= n; len <<= 1) {
    float ang = -2.0f * EEG_ALGO_PI / (float)len;
    float wlen_r = cosf(ang);
    float wlen_i = sinf(ang);

    for (uint16_t i = 0u; i < n; i += len) {
      float wr = 1.0f;
      float wi = 0.0f;
      uint16_t half = len >> 1;

      for (uint16_t j = 0u; j < half; ++j) {
        uint16_t u = i + j;
        uint16_t v = u + half;
        float vr = real[v] * wr - imag[v] * wi;
        float vi = real[v] * wi + imag[v] * wr;
        float next_wr = wr * wlen_r - wi * wlen_i;
        float next_wi = wr * wlen_i + wi * wlen_r;

        real[v] = real[u] - vr;
        imag[v] = imag[u] - vi;
        real[u] += vr;
        imag[u] += vi;

        wr = next_wr;
        wi = next_wi;
      }
    }
  }
}

static void eeg_algo_compute_spectrum(const float *signal,
                                      uint16_t len,
                                      uint32_t sample_rate_hz,
                                      eeg_algo_result_t *result)
{
  uint16_t fft_len = eeg_algo_next_pow2(len);
  float total_power = 0.0f;
  float weighted_freq_sum = 0.0f;
  float cumulative_power = 0.0f;
  float peak_power = -1.0f;
  uint16_t peak_bin = 0u;
  uint16_t median_bin = 0u;

  memset(s_ws.imag, 0, sizeof(s_ws.imag));

  for (uint16_t i = 0u; i < fft_len; ++i) {
    if (i < len) {
      float w = 0.5f - 0.5f * cosf((2.0f * EEG_ALGO_PI * (float)i) / (float)(len - 1u));
      s_ws.real[i] = signal[i] * w;
    } else {
      s_ws.real[i] = 0.0f;
    }
  }

  eeg_algo_fft(s_ws.real, s_ws.imag, fft_len);

  for (uint16_t bin = 1u; bin < (fft_len / 2u); ++bin) {
    float power = s_ws.real[bin] * s_ws.real[bin] + s_ws.imag[bin] * s_ws.imag[bin];
    float freq_hz = ((float)bin * (float)sample_rate_hz) / (float)fft_len;

    total_power += power;
    weighted_freq_sum += power * freq_hz;

    if (power > peak_power) {
      peak_power = power;
      peak_bin = bin;
    }
  }

  if (total_power <= 0.0f) {
    return;
  }

  result->peak_frequency_hz = ((float)peak_bin * (float)sample_rate_hz) / (float)fft_len;
  result->spectral_centroid_hz = weighted_freq_sum / total_power;

  for (uint16_t bin = 1u; bin < (fft_len / 2u); ++bin) {
    float power = s_ws.real[bin] * s_ws.real[bin] + s_ws.imag[bin] * s_ws.imag[bin];
    float p = power / total_power;
    float freq_hz = ((float)bin * (float)sample_rate_hz) / (float)fft_len;

    if (p > 1.0e-9f) {
      result->spectral_entropy -= p * log2f(p);
    }

    cumulative_power += power;
    if ((median_bin == 0u) && (cumulative_power >= (0.5f * total_power))) {
      median_bin = bin;
      result->median_frequency_hz = freq_hz;
    }
  }

  for (uint16_t bin = 1u; bin < (fft_len / 2u); ++bin) {
    float freq_hz = ((float)bin * (float)sample_rate_hz) / (float)fft_len;
    float power = s_ws.real[bin] * s_ws.real[bin] + s_ws.imag[bin] * s_ws.imag[bin];

    if (freq_hz >= 0.5f && freq_hz < 4.0f) {
      result->delta_power += power;
    } else if (freq_hz >= 4.0f && freq_hz < 8.0f) {
      result->theta_power += power;
    } else if (freq_hz >= 8.0f && freq_hz < 12.0f) {
      result->alpha_power += power;
    } else if (freq_hz >= 12.0f && freq_hz < 16.0f) {
      result->sigma_power += power;
    } else if (freq_hz >= 16.0f && freq_hz < 30.0f) {
      result->beta_power += power;
    }
  }

  result->delta_ratio = result->delta_power / total_power;
  result->theta_ratio = result->theta_power / total_power;
  result->alpha_ratio = result->alpha_power / total_power;
  result->sigma_ratio = result->sigma_power / total_power;
  result->beta_ratio = result->beta_power / total_power;
  result->high_freq_ratio = eeg_algo_clampf(1.0f - (result->delta_ratio + result->theta_ratio +
                                                    result->alpha_ratio + result->sigma_ratio),
                                            0.0f,
                                            1.0f);
}

static uint16_t eeg_algo_count_peaks(const float *x,
                                     uint16_t len,
                                     float threshold,
                                     uint16_t refractory)
{
  uint16_t count = 0u;
  uint16_t cooldown = 0u;

  if (len < 3u) {
    return 0u;
  }

  for (uint16_t i = 1u; i < (len - 1u); ++i) {
    float v = x[i];

    if (cooldown > 0u) {
      cooldown--;
      continue;
    }

    if ((v > threshold) && (v >= x[i - 1u]) && (v > x[i + 1u])) {
      count++;
      cooldown = refractory;
    }
  }

  return count;
}

static void eeg_algo_detect_events(const float *signal,
                                   uint16_t len,
                                   uint32_t sample_rate_hz,
                                   eeg_algo_result_t *result)
{
  uint16_t sw_refractory = (uint16_t)(sample_rate_hz / 3u);
  uint16_t kcx_refractory = (uint16_t)(sample_rate_hz / 4u);
  float slow_wave_threshold = result->rms_uv * 1.8f;
  float k_complex_threshold = result->rms_uv * 2.5f;

  for (uint16_t i = 0u; i < len; ++i) {
    float prev = (i > 0u) ? signal[i - 1u] : signal[i];
    float next = (i + 1u < len) ? signal[i + 1u] : signal[i];
    s_ws.imag[i] = (prev + signal[i] + next) / 3.0f;
  }

  result->slow_wave_count = eeg_algo_count_peaks(s_ws.imag,
                                                 len,
                                                 slow_wave_threshold,
                                                 sw_refractory);
  result->k_complex_like_count = eeg_algo_count_peaks(s_ws.imag,
                                                      len,
                                                      k_complex_threshold,
                                                      kcx_refractory);

  result->spindle_like_count = (uint16_t)lroundf(result->sigma_ratio * 100.0f);
  result->slow_wave_score = eeg_algo_clampf(result->delta_ratio * 4.0f +
                                            ((float)result->slow_wave_count / 12.0f),
                                            0.0f,
                                            1.0f);
  result->spindle_score = eeg_algo_clampf(result->sigma_ratio * 6.0f +
                                          ((float)result->spindle_like_count / 20.0f),
                                          0.0f,
                                          1.0f);
  result->k_complex_score = eeg_algo_clampf(((float)result->k_complex_like_count / 8.0f),
                                            0.0f,
                                            1.0f);
}

static void eeg_algo_measure_signal_quality(const float *signal, uint16_t len, eeg_algo_result_t *result)
{
  uint16_t clipped = 0u;

  for (uint16_t i = 0u; i < len; ++i) {
    if (fabsf(signal[i]) > 3000.0f) {
      clipped++;
    }
  }

  result->clipping_ratio = (float)clipped / (float)len;
  result->baseline_drift_uv = fabsf(eeg_algo_signal_mean(signal, len));
}

static void eeg_algo_finalize_quality_score(eeg_algo_result_t *result)
{
  float abs_mean = result->envelope_mean_uv;

  result->signal_quality_score =
    eeg_algo_clampf(1.0f -
                    (result->clipping_ratio * 2.5f) -
                    eeg_algo_clampf(result->high_freq_ratio - 0.35f, 0.0f, 0.5f) -
                    eeg_algo_clampf((result->baseline_drift_uv / 500.0f), 0.0f, 0.5f) -
                    eeg_algo_clampf(abs_mean < 5.0f ? 0.3f : 0.0f, 0.0f, 0.3f),
                    0.0f,
                    1.0f);
}

static sleep_stage_result_t eeg_algo_classify_stage(const eeg_algo_result_t *result)
{
  sleep_stage_result_t out = {
    .stage = SLEEP_STAGE_UNKNOWN,
    .confidence = 0.25f,
  };

  if (result->alpha_ratio > 0.24f && result->beta_ratio > 0.10f) {
    out.stage = SLEEP_STAGE_W;
    out.confidence = eeg_algo_clampf(0.45f + result->alpha_ratio + result->beta_ratio, 0.0f, 0.98f);
    return out;
  }

  if ((result->delta_ratio > 0.42f) &&
      (result->slow_wave_score > 0.45f) &&
      (result->signal_quality_score > 0.30f)) {
    out.stage = SLEEP_STAGE_N3;
    out.confidence = eeg_algo_clampf(0.40f + result->delta_ratio + 0.3f * result->slow_wave_score,
                                     0.0f,
                                     0.99f);
    return out;
  }

  if ((result->sigma_ratio > 0.10f) &&
      ((result->spindle_score > 0.35f) || (result->k_complex_score > 0.20f))) {
    out.stage = SLEEP_STAGE_N2;
    out.confidence = eeg_algo_clampf(0.35f + result->sigma_ratio + 0.25f * result->k_complex_score,
                                     0.0f,
                                     0.95f);
    return out;
  }

  if ((result->theta_ratio > 0.20f) &&
      (result->alpha_ratio < 0.18f) &&
      (result->delta_ratio < 0.35f)) {
    out.stage = SLEEP_STAGE_REM;
    out.confidence = eeg_algo_clampf(0.25f + result->theta_ratio + 0.15f * (1.0f - result->delta_ratio),
                                     0.0f,
                                     0.85f);
    return out;
  }

  out.stage = SLEEP_STAGE_N1;
  out.confidence = eeg_algo_clampf(0.25f + result->theta_ratio + 0.1f * (1.0f - result->alpha_ratio),
                                   0.0f,
                                   0.75f);
  return out;
}

void eeg_algo_context_init(eeg_algo_context_t *ctx)
{
  if (ctx == NULL) {
    return;
  }

  memset(ctx, 0, sizeof(*ctx));
  ctx->initialized = true;
}

int eeg_algo_analyze_frame(eeg_algo_context_t *ctx,
                           const eeg_ctrl_frame_t *frame,
                           eeg_algo_result_t *result)
{
  float *signal = s_ws.real;
  float alpha = 0.12f;

  if (ctx == NULL || frame == NULL || result == NULL) {
    return -EINVAL;
  }
  if (frame->sample_count < EEG_ALGO_MIN_FRAME_SAMPLES) {
    return -EINVAL;
  }
  if (!ctx->initialized) {
    eeg_algo_context_init(ctx);
  }

  memset(result, 0, sizeof(*result));
  eeg_algo_prepare_signal(frame, signal, frame->sample_count);
  eeg_algo_compute_time_features(signal, frame->sample_count, frame->sample_rate_hz, result);
  eeg_algo_detect_events(signal, frame->sample_count, frame->sample_rate_hz, result);
  eeg_algo_measure_signal_quality(signal, frame->sample_count, result);
  eeg_algo_compute_spectrum(signal, frame->sample_count, frame->sample_rate_hz, result);
  eeg_algo_finalize_quality_score(result);

  ctx->analyzed_frame_count++;
  if (ctx->analyzed_frame_count == 1u) {
    ctx->long_term_median_frequency_hz = result->median_frequency_hz;
    ctx->long_term_sigma_ratio = result->sigma_ratio;
    ctx->long_term_delta_ratio = result->delta_ratio;
  } else {
    ctx->long_term_median_frequency_hz =
      (1.0f - alpha) * ctx->long_term_median_frequency_hz + alpha * result->median_frequency_hz;
    ctx->long_term_sigma_ratio =
      (1.0f - alpha) * ctx->long_term_sigma_ratio + alpha * result->sigma_ratio;
    ctx->long_term_delta_ratio =
      (1.0f - alpha) * ctx->long_term_delta_ratio + alpha * result->delta_ratio;
  }

  result->long_term_median_frequency_hz = ctx->long_term_median_frequency_hz;
  result->sleep_depth_score =
    eeg_algo_clampf((0.65f * result->delta_ratio) +
                    (0.20f * result->slow_wave_score) +
                    (0.15f * ctx->long_term_delta_ratio),
                    0.0f,
                    1.0f);
  result->arousal_risk_score =
    eeg_algo_clampf((0.40f * result->beta_ratio) +
                    (0.20f * result->alpha_ratio) +
                    (0.20f * result->high_freq_ratio) +
                    (0.20f * (1.0f - result->signal_quality_score)),
                    0.0f,
                    1.0f);
  result->sleep_continuity_score =
    eeg_algo_clampf((0.45f * result->spindle_score) +
                    (0.20f * result->k_complex_score) +
                    (0.20f * result->signal_quality_score) +
                    (0.15f * (1.0f - result->arousal_risk_score)),
                    0.0f,
                    1.0f);

  result->stage = eeg_algo_classify_stage(result);
  return 0;
}
