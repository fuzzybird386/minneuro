/*
 * file_nrf54_hal.c — nRF54 platform HAL for SD card SPI
 *
 * SD shares SPIM00 with DAC80501 (dac80501_hal.c) and KS1092 (ks1092.c).
 * All transfers go through spim00_bus (single k_mutex + per-device SPIM config),
 * so SD must not use Zephyr spi_transceive on the same instance.
 *
 * Protocol note (file.c):
 *   Between file_cs_select() and file_cs_deselect() the HAL holds the bus mutex
 *   and file_xfer() performs only nrfx transfers (no nested acquire). Dummy clocks
 *   with CS high (sd_idle_clocks) use file_xfer() without an active CS session,
 *   so each byte acquires/releases in MODE0 — safe to interleave with DAC/KS1092.
 *
 * SD SPI mode 0 (CPOL=0 CPHA=0). Frequency: see SD_SPI_*_HZ (prescaler SoCs).
 */

#include "file.h"
#include "spim00_bus.h"
#include "src/inc/bsp.h"

#include <string.h>

#include <errno.h>

#include <hal/nrf_gpio.h>
#include <zephyr/kernel.h>

#define SPI_CS     PIN_SPI_CS_SDCARD
#define INT_SDCARD PIN_INT_SDCARD

#define SD_SPI_SLOW_HZ 2000000
#define SD_SPI_FAST_HZ 8000000

static spim00_bus_config_t s_sd_spi_cfg = {
    .frequency_hz = SD_SPI_SLOW_HZ,
    .mode = NRF_SPIM_MODE_0,
    .orc = 0xFFu,
};

static uint8_t s_scratch_tx[512];
static uint8_t s_scratch_rx[512];

/* True while file_cs_select() held the bus lock (multi-xfer SD transaction). */
static bool s_sd_cs_session;

/* nrfx_spim_xfer / spim00_bus_xfer return 0 on success. */
static int file_map_nrfx_xfer(int nrf_err)
{
  if (nrf_err == 0) {
    return 0;
  }
  return -EIO;
}

static int file_do_xfer_locked(const uint8_t *tx, uint8_t *rx, uint32_t len)
{
  return file_map_nrfx_xfer(spim00_bus_xfer(tx, rx, len));
}

int file_dev_init(void)
{
  int err = spim00_bus_init();

  if (err != 0) {
    return err;
  }

  nrf_gpio_cfg_input(INT_SDCARD, NRF_GPIO_PIN_PULLUP);

  nrf_gpio_cfg_output(SPI_CS);
  nrf_gpio_pin_set(SPI_CS);

  return 0;
}

void file_spi_set_slow(void)
{
  s_sd_spi_cfg.frequency_hz = SD_SPI_SLOW_HZ;
}

void file_spi_set_fast(void)
{
  s_sd_spi_cfg.frequency_hz = SD_SPI_FAST_HZ;
}

void file_cs_select(void)
{
  (void)spim00_bus_acquire(&s_sd_spi_cfg);
  s_sd_cs_session = true;
  nrf_gpio_pin_clear(SPI_CS);
}

void file_cs_deselect(void)
{
  nrf_gpio_pin_set(SPI_CS);
  s_sd_cs_session = false;
  spim00_bus_release();
}

int file_xfer(const uint8_t *tx, uint8_t *rx, uint32_t len)
{
  int err;

  if (len == 0U) {
    return 0;
  }
  if (len > sizeof(s_scratch_tx)) {
    return -EINVAL;
  }

  if (s_sd_cs_session) {
    if ((tx != NULL) && (rx != NULL)) {
      err = file_do_xfer_locked(tx, rx, len);
    } else if ((tx != NULL) && (rx == NULL)) {
      err = file_do_xfer_locked(tx, NULL, len);
    } else if ((tx == NULL) && (rx != NULL)) {
      memset(s_scratch_tx, 0xFF, len);
      err = file_do_xfer_locked(s_scratch_tx, rx, len);
    } else {
      memset(s_scratch_tx, 0xFF, len);
      err = file_do_xfer_locked(s_scratch_tx, s_scratch_rx, len);
    }
    return err;
  }

  err = spim00_bus_acquire(&s_sd_spi_cfg);
  if (err != 0) {
    return err;
  }

  if ((tx != NULL) && (rx != NULL)) {
    err = file_do_xfer_locked(tx, rx, len);
  } else if ((tx != NULL) && (rx == NULL)) {
    err = file_do_xfer_locked(tx, NULL, len);
  } else if ((tx == NULL) && (rx != NULL)) {
    memset(s_scratch_tx, 0xFF, len);
    err = file_do_xfer_locked(s_scratch_tx, rx, len);
  } else {
    memset(s_scratch_tx, 0xFF, len);
    err = file_do_xfer_locked(s_scratch_tx, s_scratch_rx, len);
  }

  spim00_bus_release();
  return err;
}

int file_is_card_inserted(void)
{
  return (nrf_gpio_pin_read(INT_SDCARD) == 0U) ? 1 : 0;
}
