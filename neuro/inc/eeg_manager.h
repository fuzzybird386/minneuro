#pragma once

#include <stdbool.h>
#include <stdint.h>

#define TASK_EEG_DEFAULT_SAMPLE_RATE_HZ 500u
#define TASK_EEG_FRAME_SIZE 14u // Max samples per BLE frame, runtime frame size is computed on start.
#define TASK_EEG_CHANNEL_SIZE 16

typedef struct {
  uint32_t sample_rate_hz;
} eeg_manager_config_t;

typedef struct {
  uint8_t sample_count;
  int16_t volts[TASK_EEG_FRAME_SIZE][TASK_EEG_CHANNEL_SIZE];
} eeg_manager_frame_t;

int eeg_manager_init(void);

int eeg_manager_start(eeg_manager_config_t cfg);

int eeg_manager_stop(void);
int eeg_manager_verify(void);

// Timer Helper:
int eeg_manager_read_frame(eeg_manager_frame_t *frame);
