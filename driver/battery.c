#include "battery.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nrfx_saadc.h>
#include <zephyr/kernel.h>

#include "board_adc.h"

enum {
  BATTERY_ADC_RESOLUTION_BITS = 12,
  BATTERY_ADC_REF_MV = NRFX_SAADC_REF_INTERNAL_VALUE,
  BATTERY_ADC_GAIN_NUM = 4,
  BATTERY_DIVIDER_TOP_KOHM = 820,
  BATTERY_DIVIDER_BOTTOM_KOHM = 1000,
  BATTERY_SAMPLE_AVG_COUNT = 8,
  BATTERY_HISTORY_DEPTH = 10,
  BATTERY_EMPTY_MV = 3300,
  BATTERY_FULL_MV = 4200,
};

struct battery_runtime {
  struct k_mutex lock;
  bool initialized;
  bool has_measurement;
  uint16_t history_mv[BATTERY_HISTORY_DEPTH];
  uint8_t history_count;
  uint8_t history_head;
  battery_measurement_t latest;
};

static struct battery_runtime s_battery;

static uint16_t battery_raw_to_sense_mv(int16_t raw)
{
  uint64_t numerator;

  if (raw <= 0) {
    return 0u;
  }

  numerator = (uint64_t)(uint16_t)raw * BATTERY_ADC_REF_MV * BATTERY_ADC_GAIN_NUM;
  numerator += (1u << (BATTERY_ADC_RESOLUTION_BITS - 1));
  return (uint16_t)(numerator >> BATTERY_ADC_RESOLUTION_BITS);
}

static uint16_t battery_sense_to_battery_mv(uint16_t sense_mv)
{
  uint64_t numerator = (uint64_t)sense_mv *
                       (BATTERY_DIVIDER_TOP_KOHM + BATTERY_DIVIDER_BOTTOM_KOHM);

  numerator += BATTERY_DIVIDER_BOTTOM_KOHM / 2u;
  return (uint16_t)(numerator / BATTERY_DIVIDER_BOTTOM_KOHM);
}

static uint16_t battery_average_history_mv(void)
{
  uint32_t sum = 0u;

  for (uint8_t i = 0u; i < s_battery.history_count; ++i) {
    sum += s_battery.history_mv[i];
  }

  if (s_battery.history_count == 0u) {
    return 0u;
  }

  return (uint16_t)(sum / s_battery.history_count);
}

static uint8_t battery_mv_to_percent(uint16_t battery_mv)
{
  uint32_t scaled;

  if (battery_mv <= BATTERY_EMPTY_MV) {
    return 0u;
  }
  if (battery_mv >= BATTERY_FULL_MV) {
    return 100u;
  }

  scaled = (uint32_t)(battery_mv - BATTERY_EMPTY_MV) * 100u;
  return (uint8_t)(scaled / (BATTERY_FULL_MV - BATTERY_EMPTY_MV));
}

int battery_init(void)
{
  int err;

  if (s_battery.initialized) {
    return 0;
  }

  memset(&s_battery, 0, sizeof(s_battery));
  k_mutex_init(&s_battery.lock);

  err = board_adc_init();
  if (err) {
    return err;
  }

  s_battery.initialized = true;
  return 0;
}

int battery_refresh(battery_measurement_t *measurement)
{
  board_adc_sample_t adc_sample;
  uint32_t instant_sum_mv = 0u;
  uint16_t instant_avg_mv;
  battery_measurement_t latest;
  int err;

  err = battery_init();
  if (err) {
    return err;
  }

  for (uint8_t i = 0u; i < BATTERY_SAMPLE_AVG_COUNT; ++i) {
    err = board_adc_read(&adc_sample);
    if (err) {
      return err;
    }

    instant_sum_mv += battery_sense_to_battery_mv(
      battery_raw_to_sense_mv(adc_sample.battery_raw));
  }

  instant_avg_mv = (uint16_t)(instant_sum_mv / BATTERY_SAMPLE_AVG_COUNT);

  k_mutex_lock(&s_battery.lock, K_FOREVER);

  s_battery.history_mv[s_battery.history_head] = instant_avg_mv;
  s_battery.history_head = (uint8_t)((s_battery.history_head + 1u) % BATTERY_HISTORY_DEPTH);
  if (s_battery.history_count < BATTERY_HISTORY_DEPTH) {
    s_battery.history_count++;
  }

  latest.battery_mv = battery_average_history_mv();
  latest.sense_mv = (uint16_t)(((uint32_t)latest.battery_mv * BATTERY_DIVIDER_BOTTOM_KOHM) /
                               (BATTERY_DIVIDER_TOP_KOHM + BATTERY_DIVIDER_BOTTOM_KOHM));
  latest.level_pct = battery_mv_to_percent(latest.battery_mv);

  s_battery.latest = latest;
  s_battery.has_measurement = true;

  if (measurement != NULL) {
    *measurement = latest;
  }

  k_mutex_unlock(&s_battery.lock);
  return 0;
}

int battery_get_latest(battery_measurement_t *measurement)
{
  if (measurement == NULL) {
    return -EINVAL;
  }

  if (!s_battery.initialized) {
    return -EACCES;
  }

  k_mutex_lock(&s_battery.lock, K_FOREVER);
  if (!s_battery.has_measurement) {
    k_mutex_unlock(&s_battery.lock);
    return -EAGAIN;
  }

  *measurement = s_battery.latest;
  k_mutex_unlock(&s_battery.lock);
  return 0;
}
