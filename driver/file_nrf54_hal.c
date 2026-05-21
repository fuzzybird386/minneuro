/*
 * file_nrf54_hal.c — nRF54 platform HAL for SD card SPI
 *
 * SD shares SPIM00 with the Zephyr SPI controller node (&spi00). On this board,
 * DAC80501 may attach under &spi00 as ti,dac80501-spi (disabled in DTS during SD
 * bring-up). Mixing Zephyr spi_nrfx here with a parallel nrfx/spim00_bus layer on
 * the same peripheral corrupts driver state and typically hangs inside xfer.
 *
 * Transfers therefore go through Zephyr spi_transceive(); SD chip-select stays
 * manual on PIN_SPI_CS_SDCARD (DAC keeps controller cs-gpios on another pin).
 *
 * Frequency caveat (nRF54L15): CMSIS sets NRF_SPIM_HAS_PRESCALER=1 and
 * NRF_SPIM_HAS_FREQUENCY=0. nrfx then validates the rate via the SPIM prescaler
 * (divisor must be even and ≤ instance max, e.g. 126 on SPIM00 @ 128 MHz core).
 * That makes very low clocks such as 125 kHz impossible (128 MHz / 125 kHz ≫ 126),
 * so nrfx_spim_init returns -EINVAL (-22). Zephyr still allows requesting 125 kHz
 * in spi_config, but the nrfx layer rejects it — use an achievable rate (see below).
 * (SD Physical Layer prefers ≤400 kHz until the card leaves idle; many cards still
 * work a bit above that during bring-up.)
 */

#include "file.h"
#include "src/inc/bsp.h"

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>

#include <hal/nrf_gpio.h>

#define SPI_CS     PIN_SPI_CS_SDCARD
#define INT_SDCARD PIN_INT_SDCARD

#define SPI_BUS_NODE DT_NODELABEL(spi00)

#if !DT_NODE_HAS_STATUS(SPI_BUS_NODE, okay)
#error SPI HAL requires spi00 status okay — SD uses the Zephyr SPI controller on spi00
#endif

static const struct device *const s_spi_bus = DEVICE_DT_GET(SPI_BUS_NODE);

/*
 * "Slow" SPI for SD: spec wants ≤400 kHz before switching speed; on nRF54L15 SPI
 * master this HW can only hit certain divisors — too-slow requests fail nrfx init
 * (-EINVAL / log "Failed to initialize nrfx driver: -22"). 2 MHz is a safe round
 * rate that passes prescaler checks on SPIM00 (128 MHz / 2 MHz = 64).
 */
#define SD_SPI_SLOW_HZ 2000000
#define SD_SPI_FAST_HZ 8000000

/* SD SPI mode 0 (CPOL=0 CPHA=0); DAC uses mode 1 via its own spi_dt_spec. */
static struct spi_config s_spi_sd_cfg = {
    .frequency = SD_SPI_SLOW_HZ,
    .operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
    .slave = 0,
    .cs = {0},
    .word_delay = 0,
};

static uint8_t s_scratch_tx[512];
static uint8_t s_scratch_rx[512];

int file_dev_init(void)
{
  if (!device_is_ready(s_spi_bus)) {
    return -ENODEV;
  }

  nrf_gpio_cfg_input(INT_SDCARD, NRF_GPIO_PIN_PULLUP);

  nrf_gpio_cfg_output(SPI_CS);
  nrf_gpio_pin_set(SPI_CS);

  return 0;
}

void file_spi_set_slow(void)
{
  s_spi_sd_cfg.frequency = SD_SPI_SLOW_HZ;
}

void file_spi_set_fast(void)
{
  s_spi_sd_cfg.frequency = SD_SPI_FAST_HZ;
}

void file_cs_select(void)
{
  nrf_gpio_pin_clear(SPI_CS);
}

void file_cs_deselect(void)
{
  nrf_gpio_pin_set(SPI_CS);
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

  if (tx != NULL && rx != NULL) {
    struct spi_buf tx_buf = {.buf = (void *)tx, .len = len};
    struct spi_buf rx_buf = {.buf = rx, .len = len};
    const struct spi_buf_set tx_bufs = {.buffers = &tx_buf, .count = 1};
    const struct spi_buf_set rx_bufs = {.buffers = &rx_buf, .count = 1};
    err = spi_transceive(s_spi_bus, &s_spi_sd_cfg, &tx_bufs, &rx_bufs);
  } else if (tx != NULL && rx == NULL) {
    struct spi_buf tx_buf = {.buf = (void *)tx, .len = len};
    const struct spi_buf_set tx_bufs = {.buffers = &tx_buf, .count = 1};
    err = spi_write(s_spi_bus, &s_spi_sd_cfg, &tx_bufs);
  } else if (tx == NULL && rx != NULL) {
    memset(s_scratch_tx, 0xFF, len);
    struct spi_buf tx_buf = {.buf = s_scratch_tx, .len = len};
    struct spi_buf rx_buf = {.buf = rx, .len = len};
    const struct spi_buf_set tx_bufs = {.buffers = &tx_buf, .count = 1};
    const struct spi_buf_set rx_bufs = {.buffers = &rx_buf, .count = 1};
    err = spi_transceive(s_spi_bus, &s_spi_sd_cfg, &tx_bufs, &rx_bufs);
  } else {
    memset(s_scratch_tx, 0xFF, len);
    struct spi_buf tx_buf = {.buf = s_scratch_tx, .len = len};
    struct spi_buf rx_buf = {.buf = s_scratch_rx, .len = len};
    const struct spi_buf_set tx_bufs = {.buffers = &tx_buf, .count = 1};
    const struct spi_buf_set rx_bufs = {.buffers = &rx_buf, .count = 1};
    err = spi_transceive(s_spi_bus, &s_spi_sd_cfg, &tx_bufs, &rx_bufs);
  }

  return err;
}

int file_is_card_inserted(void)
{
  return (nrf_gpio_pin_read(INT_SDCARD) == 0U) ? 1 : 0;
}
