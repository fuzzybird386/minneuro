#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * KS1092
 * ------
 * Dual-channel EEG analog front-end configured over SPI, with analog outputs
 * sampled by the nRF SAADC.
 *
 * Public API:
 * - ks1092_init(): initialize GPIO state and SAADC.
 * - ks1092_reset(): apply the default channel register configuration.
 * - ks1092_read_eeg(): read both EEG output voltages in microvolts.
 */

typedef enum {
  KS1092_REG_CH1SET = 0x00,
  KS1092_REG_CH2SET = 0x01,
} ks1092_register_t;

typedef enum {
  KS1092_STAGE1_GAIN_9  = 0x00,
  KS1092_STAGE1_GAIN_17 = 0x02,
} ks1092_stage1_gain_t;

typedef enum {
  KS1092_STAGE2_GAIN_40  = 0x00,
  KS1092_STAGE2_GAIN_60  = 0x04,
  KS1092_STAGE2_GAIN_80  = 0x05,
  KS1092_STAGE2_GAIN_120 = 0x06,
  KS1092_STAGE2_GAIN_160 = 0x07,
} ks1092_stage2_gain_t;

typedef struct {
  bool enabled;
  ks1092_stage1_gain_t stage1_gain;
  ks1092_stage2_gain_t stage2_gain;
} ks1092_channel_cfg_t;

typedef struct {
  ks1092_channel_cfg_t ch1;
  ks1092_channel_cfg_t ch2;
} ks1092_config_t;

typedef struct {
  float ch1_uv;
  float ch2_uv;
} ks1092_eeg_data_t;

int ks1092_init(void);
int ks1092_reset(void);
int ks1092_read_eeg(ks1092_eeg_data_t *data);
