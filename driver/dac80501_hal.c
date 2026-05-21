#include "dac80501_hal.h"

#include <errno.h>
#include <stddef.h>

#include <hal/nrf_gpio.h>

#include "spim00_bus.h"
#include "src/inc/bsp.h"

/*
 * Board assumptions:
 * - SPIM00 is shared onto the common SPI wires in bsp.h.
 * - DAC80501 uses PIN_SPI_CS_STIM_DAC as chip select.
 * - PIN_STIM_ENABLE is treated as an active-high hardware enable.
 *
 * DAC80501 SPI timing:
 * - Data are clocked into the DAC on each falling SCLK edge.
 * - CPOL = 0, CPHA = 1 => SPI mode 1.
 */

#define DAC80501_SPI_CLK_HZ NRFX_MHZ_TO_HZ(8)

static const spim00_bus_config_t s_dac80501_spi_cfg = {
  .frequency_hz = DAC80501_SPI_CLK_HZ,
  .mode = NRF_SPIM_MODE_1,
  .orc = 0x00u,
};

static bool s_ready;

static void dac80501_hal_select(void)
{
  nrf_gpio_pin_clear(PIN_SPI_CS_STIM_DAC);
}

static void dac80501_hal_deselect(void)
{
  nrf_gpio_pin_set(PIN_SPI_CS_STIM_DAC);
}

int dac80501_hal_init(void)
{
  int err;

  err = spim00_bus_init();
  if (err != 0) {
    return err;
  }

  nrf_gpio_cfg_output(PIN_SPI_CS_STIM_DAC);
  dac80501_hal_deselect();

  nrf_gpio_cfg_output(PIN_STIM_ENABLE);
  nrf_gpio_pin_clear(PIN_STIM_ENABLE);

  s_ready = true;
  return 0;
}

int dac80501_hal_enable(bool enable)
{
  if (!s_ready) {
    return -EACCES;
  }

  if (enable) {
    nrf_gpio_pin_set(PIN_STIM_ENABLE);
  } else {
    nrf_gpio_pin_clear(PIN_STIM_ENABLE);
  }

  return 0;
}

int dac80501_hal_write(const uint8_t *tx, size_t len)
{
  int err;

  if (!s_ready) {
    return -EACCES;
  }
  if (tx == NULL || len == 0u) {
    return -EINVAL;
  }

  err = spim00_bus_acquire(&s_dac80501_spi_cfg);
  if (err) {
    return err;
  }

  dac80501_hal_select();
  err = spim00_bus_xfer(tx, NULL, len);
  dac80501_hal_deselect();
  spim00_bus_release();

  return err;
}
