#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/ks1092.h"

enum {
  EEG_CTRL_RATE_250HZ = 250u,
  EEG_CTRL_RATE_500HZ = 500u,
  EEG_CTRL_RATE_1000HZ = 1000u,
  EEG_CTRL_MAX_SAMPLES_PER_FRAME = EEG_CTRL_RATE_500HZ,
};

typedef struct {
  uint32_t sample_rate_hz;
} eeg_ctrl_config_t;

typedef struct {
  int16_t ch1_uv;
  int16_t ch2_uv;
} eeg_ctrl_sample_t;

typedef struct {
  uint32_t sample_rate_hz;
  uint16_t sample_count;
  eeg_ctrl_sample_t samples[EEG_CTRL_MAX_SAMPLES_PER_FRAME];
} eeg_ctrl_frame_t;

int eeg_ctrl_init(void);
int eeg_ctrl_reset(const eeg_ctrl_config_t *cfg);
int eeg_ctrl_start(uint32_t sample_rate_hz);
int eeg_ctrl_stop(void);
int eeg_ctrl_read_frame(eeg_ctrl_frame_t *frame);

bool eeg_ctrl_is_running(void);
