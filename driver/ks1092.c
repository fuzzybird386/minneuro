#include "ks1092.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <hal/nrf_gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "board_adc.h"
#include "spim00_bus.h"
#include "src/inc/bsp.h"
#include <zephyr/logging/log.h>

#if !defined(KS1092_SPI_PROG)
#define KS1092_SPI_PROG 0
#endif

#if KS1092_SPI_PROG
#include <hal/nrf_spim.h>
#endif

LOG_MODULE_REGISTER(ks1092, LOG_LEVEL_INF);

enum {
#if KS1092_SPI_PROG
  KS1092_SPI_CLK_HZ = NRFX_MHZ_TO_HZ(4),
  KS1092_OPCODE_RESET = 0xFE,
  KS1092_OPCODE_RREG_BASE = 0x10,
  KS1092_OPCODE_WREG_BASE = 0x20,
  KS1092_CHANNEL_PD_BIT = 5,
  KS1092_STAGE2_SHIFT = 2,
  KS1092_STAGE2_MASK = 0x1Cu,
  KS1092_STAGE1_MASK = 0x03u,
  KS1092_SPI_INTERFRAME_US = 10,
#endif
  KS1092_CHANNEL_COUNT = 2,
  KS1092_ADC_RESOLUTION_BITS = 12,
  KS1092_POWER_SETTLE_MS = 5,
  KS1092_RESET_PULSE_MS = 1,
  KS1092_RESET_RECOVERY_MS = 2,
};

/*
 * Board assumption:
 * - CHLEN is hard-wired on the PCB and is not MCU-controlled.
 * - Default to CHLEN = 0. To support boards wired with CHLEN = 1,
 *   only this constant needs to be changed.
 *
 * Channel enable truth table from the KS1092 datasheet:
 * - CH1 enabled when CH1PD XOR  CHLEN = 1
 * - CH2 enabled when CH2PD XNOR CHLEN = 1
 */
enum {
  KS1092_CHLEN_LEVEL = 0,
};

#if KS1092_SPI_PROG
static const spim00_bus_config_t s_ks1092_spi_cfg = {
  .frequency_hz = KS1092_SPI_CLK_HZ,
  .mode = NRF_SPIM_MODE_1,
  .orc = 0x00u,
};
#endif

static bool s_initialized;
static ks1092_config_t s_cfg;

static const ks1092_config_t s_default_cfg = {
  .ch1 = {
    .enabled = true,
    .stage1_gain = KS1092_STAGE1_GAIN_9,
    .stage2_gain = KS1092_STAGE2_GAIN_40,
  },
  .ch2 = {
    .enabled = true,
    .stage1_gain = KS1092_STAGE1_GAIN_9,
    .stage2_gain = KS1092_STAGE2_GAIN_40,
  },
};

static void ks1092_deselect(void)
{
  nrf_gpio_pin_set(PIN_SPI_CS_EEG);
}

#if KS1092_SPI_PROG
static void ks1092_select(void)
{
  nrf_gpio_pin_clear(PIN_SPI_CS_EEG);
}

static int ks1092_spi_write(const uint8_t *tx, size_t len)
{
  int err;

  if (tx == NULL || len == 0u) {
    return -EINVAL;
  }

  err = spim00_bus_acquire(&s_ks1092_spi_cfg);
  if (err) {
    return err;
  }

  ks1092_select();
  err = spim00_bus_xfer(tx, NULL, len);
  ks1092_deselect();
  spim00_bus_release();

  return err;
}

static int ks1092_spi_read(const uint8_t *tx, uint8_t *rx, size_t len)
{
  int err;

  if (tx == NULL || rx == NULL || len == 0u) {
    return -EINVAL;
  }

  err = spim00_bus_acquire(&s_ks1092_spi_cfg);
  if (err) {
    return err;
  }

  ks1092_select();
  err = spim00_bus_xfer(tx, rx, len);
  ks1092_deselect();
  spim00_bus_release();

  return err;
}

static int ks1092_write_registers(ks1092_register_t start_reg,
                                  const uint8_t *data,
                                  size_t count)
{
  uint8_t frame[2 + KS1092_CHANNEL_COUNT];

  if (data == NULL || count == 0u || count > KS1092_CHANNEL_COUNT) {
    return -EINVAL;
  }

  frame[0] = (uint8_t)(KS1092_OPCODE_WREG_BASE | ((uint8_t)start_reg & 0x0Fu));
  frame[1] = (uint8_t)(count - 1u);
  for (size_t i = 0; i < count; ++i) {
    frame[2 + i] = data[i];
  }

  return ks1092_spi_write(frame, count + 2u);
}

static int ks1092_read_registers(ks1092_register_t start_reg,
                                 uint8_t *data,
                                 size_t count)
{
  uint8_t tx[2 + KS1092_CHANNEL_COUNT] = {0};
  uint8_t rx[sizeof(tx)] = {0};
  int err;

  if (data == NULL || count == 0u || count > KS1092_CHANNEL_COUNT) {
    return -EINVAL;
  }

  tx[0] = (uint8_t)(KS1092_OPCODE_RREG_BASE | ((uint8_t)start_reg & 0x0Fu));
  tx[1] = (uint8_t)(count - 1u);

  err = ks1092_spi_read(tx, rx, count + 2u);
  if (err) {
    return err;
  }

  for (size_t i = 0; i < count; ++i) {
    data[i] = rx[2 + i];
  }

  return 0;
}

static uint8_t ks1092_encode_channel(const ks1092_channel_cfg_t *cfg, bool is_ch2)
{
  uint8_t reg = 0u;
  bool pd_bit;

  if (cfg == NULL) {
    return 0u;
  }

  if (cfg->enabled) {
    pd_bit = is_ch2 ? (KS1092_CHLEN_LEVEL != 0) : (KS1092_CHLEN_LEVEL == 0);
  } else {
    pd_bit = is_ch2 ? (KS1092_CHLEN_LEVEL == 0) : (KS1092_CHLEN_LEVEL != 0);
  }

  if (pd_bit) {
    reg |= (uint8_t)(1u << KS1092_CHANNEL_PD_BIT);
  }

  reg |= (uint8_t)(((uint8_t)cfg->stage2_gain << KS1092_STAGE2_SHIFT) &
                   KS1092_STAGE2_MASK);
  reg |= (uint8_t)((uint8_t)cfg->stage1_gain & KS1092_STAGE1_MASK);
  return reg;
}

static int ks1092_apply_config(const ks1092_config_t *cfg)
{
  uint8_t regs[KS1092_CHANNEL_COUNT];
  const uint8_t exp0 = ks1092_encode_channel(&cfg->ch1, false);
  const uint8_t exp1 = ks1092_encode_channel(&cfg->ch2, true);
  int err;

  regs[0] = exp0;
  regs[1] = exp1;

  err = ks1092_write_registers(KS1092_REG_CH1SET, regs, KS1092_CHANNEL_COUNT);
  if (err) {
    LOG_ERR("ks1092_write_registers: ks1092_write_registers failed (%d)", err);
    return err;
  }

  k_busy_wait(KS1092_SPI_INTERFRAME_US);

  err = ks1092_read_registers(KS1092_REG_CH1SET, regs, KS1092_CHANNEL_COUNT);
  if (err) {
    LOG_ERR("ks1092_read_registers: ks1092_read_registers failed (%d)", err);
    return err;
  }

  if (regs[0] != exp0 || regs[1] != exp1) {
    printk(
        "ks1092: CHxSET readback mismatch rb=%02x %02x expect %02x %02x "
        "(SPI/CS/power or KS1092_CHLEN_LEVEL vs board wiring?)\n",
        regs[0], regs[1], exp0, exp1);
    return -EIO;
  }

  return 0;
}
#endif /* KS1092_SPI_PROG */

static float ks1092_saadc_raw_to_uv(nrf_saadc_value_t raw)
{
  float ref_uv = (float)(NRFX_SAADC_REF_INTERNAL_VALUE * 1000);
  float scale = (float)(1u << KS1092_ADC_RESOLUTION_BITS);

  return ((float)raw * ref_uv) / scale;
}

int ks1092_init(void)
{
  int err;

  nrf_gpio_cfg_output(PIN_SPI_CS_EEG);
  ks1092_deselect();

  nrf_gpio_cfg_output(PIN_EEG_ENABLE);
  nrf_gpio_pin_clear(PIN_EEG_ENABLE);

  err = spim00_bus_init();
  if (err != 0) {
    return err;
  }

  err = board_adc_init();
  if (err) {
    return err;
  }

  s_initialized = true;
  return 0;
}

int ks1092_reset(void)
{
  int err;

  err = ks1092_init();
  if (err) {
    return err;
  }
  nrf_gpio_pin_clear(PIN_EEG_ENABLE);
  k_msleep(KS1092_RESET_PULSE_MS);
  nrf_gpio_pin_set(PIN_EEG_ENABLE);
  k_msleep(KS1092_POWER_SETTLE_MS);

#if KS1092_SPI_PROG
  {
    uint8_t cmd = KS1092_OPCODE_RESET;

    err = ks1092_spi_write(&cmd, sizeof(cmd));
    if (err) {
      LOG_ERR("ks1092_reset: ks1092_spi_write failed (%d)", err);
      return err;
    }

    k_msleep(KS1092_RESET_RECOVERY_MS);

    err = ks1092_apply_config(&s_default_cfg);
    if (err) {
      LOG_ERR("ks1092_reset: ks1092_apply_config failed (%d)", err);
      return err;
    }
  }
#else
  k_msleep(KS1092_RESET_RECOVERY_MS);
#endif

  s_cfg = s_default_cfg;
  return 0;
}

int ks1092_read_eeg(ks1092_eeg_data_t *data)
{
  board_adc_sample_t sample;
  int err;

  if (!s_initialized) {
    return -EACCES;
  }
  if (data == NULL) {
    return -EINVAL;
  }

  err = board_adc_read(&sample);
  if (err) {
    return err;
  }

  data->ch1_uv = ks1092_saadc_raw_to_uv(sample.eeg1_raw);
  data->ch2_uv = ks1092_saadc_raw_to_uv(sample.eeg2_raw);
  return 0;
}
