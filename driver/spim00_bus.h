#pragma once

#include <stddef.h>
#include <stdint.h>

#include <hal/nrf_spim.h>

typedef struct {
  uint32_t frequency_hz;
  nrf_spim_mode_t mode;
  uint8_t orc;
} spim00_bus_config_t;

int spim00_bus_init(void);
int spim00_bus_acquire(const spim00_bus_config_t *cfg);
void spim00_bus_release(void);
int spim00_bus_xfer(const uint8_t *tx, uint8_t *rx, size_t len);

