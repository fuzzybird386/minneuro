#include "board_adc.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nrfx_saadc.h>
#include <zephyr/kernel.h>

#include "src/inc/bsp.h"

enum {
  BOARD_ADC_RESOLUTION_BITS = 12,
  BOARD_ADC_CHANNEL_COUNT = 3,
  BOARD_ADC_EEG1_CHANNEL_INDEX = 0,
  BOARD_ADC_EEG2_CHANNEL_INDEX = 1,
  BOARD_ADC_BATTERY_CHANNEL_INDEX = 2,
  BOARD_ADC_CHANNEL_MASK =
    BIT(BOARD_ADC_EEG1_CHANNEL_INDEX) |
    BIT(BOARD_ADC_EEG2_CHANNEL_INDEX) |
    BIT(BOARD_ADC_BATTERY_CHANNEL_INDEX),
};

struct board_adc_runtime {
  struct k_mutex lock;
  bool initialized;
  nrf_saadc_value_t samples[BOARD_ADC_CHANNEL_COUNT];
};

static struct board_adc_runtime s_board_adc;

static nrfx_saadc_channel_t board_adc_battery_channel_config(void)
{
  nrfx_saadc_channel_t channel = NRFX_SAADC_DEFAULT_CHANNEL_SE(PIN_ADC_BATTERY,
                                                               BOARD_ADC_BATTERY_CHANNEL_INDEX);

  channel.channel_config.gain = NRF_SAADC_GAIN1_4;
  return channel;
}

int board_adc_init(void)
{
  nrfx_saadc_channel_t channels[BOARD_ADC_CHANNEL_COUNT] = {
    NRFX_SAADC_DEFAULT_CHANNEL_SE(PIN_ADC_EEG1, BOARD_ADC_EEG1_CHANNEL_INDEX),
    NRFX_SAADC_DEFAULT_CHANNEL_SE(PIN_ADC_EEG2, BOARD_ADC_EEG2_CHANNEL_INDEX),
    board_adc_battery_channel_config(),
  };
  int err;

  if (s_board_adc.initialized) {
    return 0;
  }

  k_mutex_init(&s_board_adc.lock);

  err = nrfx_saadc_init(0);
  if (err != 0 && err != -EALREADY) {
    return err;
  }

  err = nrfx_saadc_channels_config(channels, ARRAY_SIZE(channels));
  if (err) {
    return err;
  }

  err = nrfx_saadc_simple_mode_set(BOARD_ADC_CHANNEL_MASK,
                                   NRF_SAADC_RESOLUTION_12BIT,
                                   NRF_SAADC_OVERSAMPLE_DISABLED,
                                   NULL);
  if (err) {
    return err;
  }

  err = nrfx_saadc_offset_calibrate(NULL);
  if (err && err != -EBUSY) {
    return err;
  }

  s_board_adc.initialized = true;
  return 0;
}

int board_adc_read(board_adc_sample_t *sample)
{
  int err;

  if (sample == NULL) {
    return -EINVAL;
  }

  err = board_adc_init();
  if (err) {
    return err;
  }

  k_mutex_lock(&s_board_adc.lock, K_FOREVER);

  err = nrfx_saadc_buffer_set(s_board_adc.samples, ARRAY_SIZE(s_board_adc.samples));
  if (!err) {
    err = nrfx_saadc_mode_trigger();
  }

  if (!err) {
    sample->eeg1_raw = s_board_adc.samples[BOARD_ADC_EEG1_CHANNEL_INDEX];
    sample->eeg2_raw = s_board_adc.samples[BOARD_ADC_EEG2_CHANNEL_INDEX];
    sample->battery_raw = s_board_adc.samples[BOARD_ADC_BATTERY_CHANNEL_INDEX];
  } else {
    memset(sample, 0, sizeof(*sample));
  }

  k_mutex_unlock(&s_board_adc.lock);
  return err;
}
