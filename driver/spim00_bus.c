#include "spim00_bus.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <hal/nrf_spim.h>
#include <haly/nrfy_spim.h>
#include <nrfx_spim.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "src/inc/bsp.h"

static nrfx_spim_t s_spim = NRFX_SPIM_INSTANCE(NRF_SPIM00);
static struct k_mutex s_lock;
static bool s_initialized;
static bool s_config_valid;
static spim00_bus_config_t s_active_cfg;

static void spim00_bus_apply_frequency_hz(uint32_t frequency_hz)
{
#if NRF_SPIM_HAS_FREQUENCY
  nrf_spim_frequency_t frequency;

  switch (frequency_hz) {
  case NRFX_KHZ_TO_HZ(250):
    frequency = NRF_SPIM_FREQ_250K;
    break;
  case NRFX_MHZ_TO_HZ(8):
    frequency = NRF_SPIM_FREQ_8M;
    break;
  default:
    frequency = NRF_SPIM_FREQ_4M;
    break;
  }

  nrf_spim_frequency_set(s_spim.p_reg, frequency);
#elif NRF_SPIM_HAS_PRESCALER
  nrf_spim_prescaler_set(s_spim.p_reg,
                         NRF_SPIM_PRESCALER_CALCULATE(s_spim.p_reg, frequency_hz));
#else
#error "Unsupported SPIM frequency configuration for this SoC"
#endif
}

static void spim00_bus_apply_config(const spim00_bus_config_t *cfg)
{
  spim00_bus_apply_frequency_hz(cfg->frequency_hz);
  nrf_spim_configure(s_spim.p_reg, cfg->mode, NRF_SPIM_BIT_ORDER_MSB_FIRST);
  nrf_spim_orc_set(s_spim.p_reg, cfg->orc);
  s_active_cfg = *cfg;
  s_config_valid = true;
}

int spim00_bus_init(void)
{
  nrfx_spim_config_t cfg = NRFX_SPIM_DEFAULT_CONFIG(PIN_SPI_CLK,
                                                    PIN_SPI_MOSI,
                                                    PIN_SPI_MISO,
                                                    NRF_SPIM_PIN_NOT_CONNECTED);
  int err;

  if (s_initialized) {
    return 0;
  }

  k_mutex_init(&s_lock);

  cfg.frequency = NRFX_MHZ_TO_HZ(4);
  cfg.mode = NRF_SPIM_MODE_0;
  cfg.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;
  cfg.orc = 0xFFu;

  err = nrfx_spim_init(&s_spim, &cfg, NULL, NULL);
  if (err != 0 && err != -EALREADY) {
    printk("spim00: nrfx_spim_init failed (%d)\n", err);
    return err;
  }

  s_initialized = true;
  s_config_valid = false;
  memset(&s_active_cfg, 0, sizeof(s_active_cfg));
  return 0;
}

int spim00_bus_acquire(const spim00_bus_config_t *cfg)
{
  if (!s_initialized) {
    return -EACCES;
  }
  if (cfg == NULL) {
    return -EINVAL;
  }

  k_mutex_lock(&s_lock, K_FOREVER);

  if (!s_config_valid ||
      memcmp(&s_active_cfg, cfg, sizeof(*cfg)) != 0) {
    spim00_bus_apply_config(cfg);
  }

  return 0;
}

void spim00_bus_release(void)
{
  if (!s_initialized) {
    return;
  }

  k_mutex_unlock(&s_lock);
}

int spim00_bus_xfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
  nrfx_spim_xfer_desc_t xfer = {
    .p_tx_buffer = tx,
    .tx_length = (tx != NULL) ? len : 0u,
    .p_rx_buffer = rx,
    .rx_length = (rx != NULL) ? len : 0u,
  };
  int err;

  if (!s_initialized) {
    return -EACCES;
  }
  if (len == 0u) {
    return 0;
  }

  err = nrfx_spim_xfer(&s_spim, &xfer, 0);
  return err;
}
