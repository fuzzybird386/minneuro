#pragma once

#include <stdint.h>

typedef struct {
  int16_t eeg1_raw;
  int16_t eeg2_raw;
  int16_t battery_raw;
} board_adc_sample_t;

int board_adc_init(void);
int board_adc_read(board_adc_sample_t *sample);
