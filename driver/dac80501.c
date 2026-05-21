#include "dac80501.h"

#include <errno.h>
#include <stddef.h>

#include <zephyr/kernel.h>

#include "dac80501_hal.h"

enum {
  DAC80501_CODE_MAX = 0xFFFFu,
  DAC80501_DEFAULT_REFERENCE_MV = 2500u,
  DAC80501_RESET_RECOVERY_US = 300u,
};

static dac80501_state_t s_dac80501;

static uint16_t dac80501_compute_full_scale_mv(uint16_t reference_mv,
                                               bool reference_divide_by_2,
                                               dac80501_buffer_gain_t buffer_gain)
{
  uint32_t full_scale_mv = reference_mv;

  if (reference_divide_by_2) {
    full_scale_mv /= 2u;
  }

  if (buffer_gain == DAC80501_BUFFER_GAIN_2X) {
    full_scale_mv *= 2u;
  }

  return (uint16_t)full_scale_mv;
}

static void dac80501_apply_range_to_gain_regs(dac80501_output_range_t range,
                                              bool *reference_divide_by_2,
                                              dac80501_buffer_gain_t *buffer_gain)
{
  switch (range) {
  case DAC80501_OUTPUT_RANGE_1V25:
    *reference_divide_by_2 = true;
    *buffer_gain = DAC80501_BUFFER_GAIN_1X;
    break;
  case DAC80501_OUTPUT_RANGE_2V50:
    /*
     * Use the 1.25-V internal operating reference with a 2x output buffer.
     * This keeps the DAC out of REF-ALARM on 3.3-V rails while still giving
     * a 0 V .. 2.5 V output range.
     */
    *reference_divide_by_2 = true;
    *buffer_gain = DAC80501_BUFFER_GAIN_2X;
    break;
  case DAC80501_OUTPUT_RANGE_5V00:
  default:
    *reference_divide_by_2 = false;
    *buffer_gain = DAC80501_BUFFER_GAIN_2X;
    break;
  }
}

static int dac80501_write_register(dac80501_register_t reg, uint16_t value)
{
  uint8_t frame[3] = {
    (uint8_t)reg,
    (uint8_t)(value >> 8),
    (uint8_t)(value & 0xFFu),
  };

  return dac80501_hal_write(frame, sizeof(frame));
}

static int dac80501_write_config_registers(void)
{
  int err;

  err = dac80501_write_register(DAC80501_REG_SYNC, s_dac80501.sync_reg);
  if (err) {
    return err;
  }

  err = dac80501_write_register(DAC80501_REG_CONFIG, s_dac80501.config_reg);
  if (err) {
    return err;
  }

  err = dac80501_write_register(DAC80501_REG_GAIN, s_dac80501.gain_reg);
  if (err) {
    return err;
  }

  return 0;
}

static int dac80501_refresh_output(void)
{
  int err;

  err = dac80501_write_register(DAC80501_REG_DAC, s_dac80501.dac_code);
  if (err) {
    return err;
  }

  if (s_dac80501.update_mode == DAC80501_UPDATE_SYNC) {
    err = dac80501_latch();
    if (err) {
      return err;
    }
  }

  return 0;
}

uint16_t dac80501_clamp_code(uint32_t code)
{
  if (code > DAC80501_CODE_MAX) {
    return DAC80501_CODE_MAX;
  }

  return (uint16_t)code;
}

uint16_t dac80501_voltage_to_code(uint16_t mv, uint16_t full_scale_mv)
{
  uint32_t code;

  if (full_scale_mv == 0u) {
    return 0u;
  }

  if (mv >= full_scale_mv) {
    return DAC80501_CODE_MAX;
  }

  code = ((uint32_t)mv * DAC80501_CODE_MAX + (full_scale_mv / 2u)) / full_scale_mv;
  return dac80501_clamp_code(code);
}

int dac80501_init(const dac80501_config_t *cfg)
{
  dac80501_config_t local_cfg;
  int err;

  local_cfg.reference_source = DAC80501_REFERENCE_INTERNAL;
  local_cfg.output_range = DAC80501_OUTPUT_RANGE_2V50;
  local_cfg.update_mode = DAC80501_UPDATE_ASYNC;
  local_cfg.reference_mv = DAC80501_DEFAULT_REFERENCE_MV;
  local_cfg.initial_code = 0u;
  local_cfg.enable_on_init = true;

  if (cfg != NULL) {
    local_cfg = *cfg;
  }

  if (local_cfg.reference_mv == 0u) {
    local_cfg.reference_mv = DAC80501_DEFAULT_REFERENCE_MV;
  }

  err = dac80501_hal_init();
  if (err) {
    return err;
  }

  err = dac80501_hal_enable(true);
  if (err) {
    return err;
  }

  s_dac80501.initialized = true;
  s_dac80501.enabled = true;
  s_dac80501.reference_source = local_cfg.reference_source;
  s_dac80501.update_mode = local_cfg.update_mode;
  s_dac80501.reference_mv = local_cfg.reference_mv;
  s_dac80501.dac_code = local_cfg.initial_code;

  dac80501_apply_range_to_gain_regs(local_cfg.output_range,
                                    &s_dac80501.reference_divide_by_2,
                                    &s_dac80501.buffer_gain);

  s_dac80501.sync_reg =
    (local_cfg.update_mode == DAC80501_UPDATE_SYNC) ? DAC80501_SYNC_EN_MASK : 0u;

  s_dac80501.config_reg = 0u;
  if (local_cfg.reference_source == DAC80501_REFERENCE_EXTERNAL) {
    s_dac80501.config_reg |= DAC80501_CONFIG_REF_PWDWN_MASK;
  }

  s_dac80501.gain_reg = 0u;
  if (s_dac80501.reference_divide_by_2) {
    s_dac80501.gain_reg |= DAC80501_GAIN_REF_DIV_MASK;
  }
  if (s_dac80501.buffer_gain == DAC80501_BUFFER_GAIN_2X) {
    s_dac80501.gain_reg |= DAC80501_GAIN_BUFF_GAIN_MASK;
  }

  s_dac80501.full_scale_mv =
    dac80501_compute_full_scale_mv(s_dac80501.reference_mv,
                                   s_dac80501.reference_divide_by_2,
                                   s_dac80501.buffer_gain);

  err = dac80501_software_reset();
  if (err) {
    return err;
  }

  /*
   * Software reset initiates a POR event. Wait for the reset recovery window
   * before programming the configuration registers again.
   */
  k_busy_wait(DAC80501_RESET_RECOVERY_US);

  err = dac80501_write_config_registers();
  if (err) {
    return err;
  }

  err = dac80501_refresh_output();
  if (err) {
    return err;
  }

  if (!local_cfg.enable_on_init) {
    err = dac80501_disable();
    if (err) {
      return err;
    }
  }

  return 0;
}

int dac80501_enable(void)
{
  int err;

  if (!s_dac80501.initialized) {
    return -EACCES;
  }

  err = dac80501_hal_enable(true);
  if (err) {
    return err;
  }

  s_dac80501.config_reg &= (uint16_t)~DAC80501_CONFIG_DAC_PWDWN_MASK;

  err = dac80501_write_register(DAC80501_REG_CONFIG, s_dac80501.config_reg);
  if (err) {
    return err;
  }

  err = dac80501_write_register(DAC80501_REG_GAIN, s_dac80501.gain_reg);
  if (err) {
    return err;
  }

  err = dac80501_refresh_output();
  if (err) {
    return err;
  }

  s_dac80501.enabled = true;
  return 0;
}

int dac80501_disable(void)
{
  int err;

  if (!s_dac80501.initialized) {
    return -EACCES;
  }

  s_dac80501.config_reg |= DAC80501_CONFIG_DAC_PWDWN_MASK;

  err = dac80501_write_register(DAC80501_REG_CONFIG, s_dac80501.config_reg);
  if (err) {
    return err;
  }

  err = dac80501_hal_enable(false);
  if (err) {
    return err;
  }

  s_dac80501.enabled = false;
  return 0;
}

int dac80501_set_code(uint16_t code)
{
  if (!s_dac80501.initialized) {
    return -EACCES;
  }

  s_dac80501.dac_code = code;
  return dac80501_refresh_output();
}

int dac80501_set_voltage_mv(uint16_t mv)
{
  return dac80501_set_code(dac80501_voltage_to_code(mv, s_dac80501.full_scale_mv));
}

int dac80501_set_output_range(dac80501_output_range_t range)
{
  int err;

  if (!s_dac80501.initialized) {
    return -EACCES;
  }

  dac80501_apply_range_to_gain_regs(range,
                                    &s_dac80501.reference_divide_by_2,
                                    &s_dac80501.buffer_gain);

  s_dac80501.gain_reg = 0u;
  if (s_dac80501.reference_divide_by_2) {
    s_dac80501.gain_reg |= DAC80501_GAIN_REF_DIV_MASK;
  }
  if (s_dac80501.buffer_gain == DAC80501_BUFFER_GAIN_2X) {
    s_dac80501.gain_reg |= DAC80501_GAIN_BUFF_GAIN_MASK;
  }

  s_dac80501.full_scale_mv =
    dac80501_compute_full_scale_mv(s_dac80501.reference_mv,
                                   s_dac80501.reference_divide_by_2,
                                   s_dac80501.buffer_gain);

  err = dac80501_write_register(DAC80501_REG_GAIN, s_dac80501.gain_reg);
  if (err) {
    return err;
  }

  return dac80501_refresh_output();
}

int dac80501_set_update_mode(dac80501_update_mode_t mode)
{
  int err;

  if (!s_dac80501.initialized) {
    return -EACCES;
  }

  s_dac80501.update_mode = mode;
  s_dac80501.sync_reg = (mode == DAC80501_UPDATE_SYNC) ? DAC80501_SYNC_EN_MASK : 0u;

  err = dac80501_write_register(DAC80501_REG_SYNC, s_dac80501.sync_reg);
  if (err) {
    return err;
  }

  if (mode == DAC80501_UPDATE_ASYNC) {
    err = dac80501_refresh_output();
    if (err) {
      return err;
    }
  }

  return 0;
}

int dac80501_latch(void)
{
  if (!s_dac80501.initialized) {
    return -EACCES;
  }

  return dac80501_write_register(DAC80501_REG_TRIGGER, DAC80501_TRIGGER_LDAC_MASK);
}

int dac80501_software_reset(void)
{
  if (!s_dac80501.initialized) {
    return -EACCES;
  }

  return dac80501_write_register(DAC80501_REG_TRIGGER, DAC80501_TRIGGER_SOFT_RESET);
}

const dac80501_state_t *dac80501_state_get(void)
{
  return &s_dac80501;
}
