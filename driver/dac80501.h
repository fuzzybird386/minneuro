#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * TI DAC80501
 * -----------
 * SPI frame format is 24 bits:
 *   [23:16] command/register address
 *   [15:0]  register payload
 *
 * Register map and bit definitions are taken from:
 * TI DACx0501 datasheet, Rev. E, August 2023.
 */

typedef enum {
  DAC80501_REG_NOOP    = 0x00,
  DAC80501_REG_DEVID   = 0x01,
  DAC80501_REG_SYNC    = 0x02,
  DAC80501_REG_CONFIG  = 0x03,
  DAC80501_REG_GAIN    = 0x04,
  DAC80501_REG_TRIGGER = 0x05,
  DAC80501_REG_STATUS  = 0x07,
  DAC80501_REG_DAC     = 0x08,
} dac80501_register_t;

typedef enum {
  DAC80501_RESOLUTION_16BIT = 0u,
  DAC80501_RESOLUTION_14BIT = 1u,
  DAC80501_RESOLUTION_12BIT = 2u,
} dac80501_resolution_t;

typedef enum {
  DAC80501_REFERENCE_INTERNAL = 0,
  DAC80501_REFERENCE_EXTERNAL = 1,
} dac80501_reference_source_t;

typedef enum {
  DAC80501_UPDATE_ASYNC = 0,
  DAC80501_UPDATE_SYNC  = 1,
} dac80501_update_mode_t;

typedef enum {
  DAC80501_BUFFER_GAIN_1X = 0,
  DAC80501_BUFFER_GAIN_2X = 1,
} dac80501_buffer_gain_t;

/*
 * These ranges are the common output ranges used with the DAC80501
 * when the reference presented to the DAC core is 2.5 V.
 *
 * Effective full-scale:
 *   VFS = VREF * (BUFF_GAIN ? 2 : 1) / (REF_DIV ? 2 : 1)
 */
typedef enum {
  DAC80501_OUTPUT_RANGE_1V25 = 0,
  DAC80501_OUTPUT_RANGE_2V50 = 1,
  DAC80501_OUTPUT_RANGE_5V00 = 2,
} dac80501_output_range_t;

typedef struct {
  dac80501_reference_source_t reference_source;
  dac80501_output_range_t output_range;
  dac80501_update_mode_t update_mode;
  uint16_t reference_mv;
  uint16_t initial_code;
  bool enable_on_init;
} dac80501_config_t;

typedef struct {
  bool initialized;
  bool enabled;
  dac80501_reference_source_t reference_source;
  dac80501_update_mode_t update_mode;
  dac80501_buffer_gain_t buffer_gain;
  bool reference_divide_by_2;
  uint16_t reference_mv;
  uint16_t full_scale_mv;
  uint16_t dac_code;
  uint16_t sync_reg;
  uint16_t config_reg;
  uint16_t gain_reg;
} dac80501_state_t;

enum {
  DAC80501_DEVID_RESOLUTION_SHIFT = 12,
  DAC80501_DEVID_RESOLUTION_MASK  = 0x7000,
  DAC80501_DEVID_RSTSEL_MASK      = 0x0080,

  DAC80501_SYNC_EN_MASK           = 0x0001,

  DAC80501_CONFIG_REF_PWDWN_MASK  = 0x0100,
  DAC80501_CONFIG_DAC_PWDWN_MASK  = 0x0001,

  DAC80501_GAIN_REF_DIV_MASK      = 0x0100,
  DAC80501_GAIN_BUFF_GAIN_MASK    = 0x0001,

  DAC80501_TRIGGER_LDAC_MASK      = 0x0010,
  DAC80501_TRIGGER_SOFT_RESET     = 0x000A,

  DAC80501_STATUS_REF_ALARM_MASK  = 0x0001,
};

int dac80501_init(const dac80501_config_t *cfg);
int dac80501_enable(void);
int dac80501_disable(void);
int dac80501_set_code(uint16_t code);
int dac80501_set_voltage_mv(uint16_t mv);
int dac80501_set_output_range(dac80501_output_range_t range);
int dac80501_set_update_mode(dac80501_update_mode_t mode);
int dac80501_latch(void);
int dac80501_software_reset(void);

uint16_t dac80501_clamp_code(uint32_t code);
uint16_t dac80501_voltage_to_code(uint16_t mv, uint16_t full_scale_mv);
const dac80501_state_t *dac80501_state_get(void);
