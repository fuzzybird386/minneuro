#pragma once

#include <stdint.h>

typedef struct {
  uint16_t sense_mv;
  uint16_t battery_mv;
  uint8_t level_pct;
} battery_measurement_t;

int battery_init(void);
int battery_refresh(battery_measurement_t *measurement);
int battery_get_latest(battery_measurement_t *measurement);
