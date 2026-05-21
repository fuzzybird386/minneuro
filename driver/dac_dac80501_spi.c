/*
 * Copyright (c) 2025 MinNeuro
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr DAC driver for Texas Instruments DAC80501 (SPI variant).
 *
 * SPI frame: 24-bit = [7:0] register address + [15:0] data (big-endian).
 * CPOL = 0, CPHA = 1  →  SPI mode 1.
 *
 * Reference: TI DACx0501 datasheet, Rev. E, August 2023.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(dac_dac80501_spi, CONFIG_DAC_LOG_LEVEL);

/* ── Register addresses ─────────────────────────────────────────────── */
#define DAC80501_REG_NOOP    0x00U
#define DAC80501_REG_DEVID   0x01U
#define DAC80501_REG_SYNC    0x02U
#define DAC80501_REG_CONFIG  0x03U
#define DAC80501_REG_GAIN    0x04U
#define DAC80501_REG_TRIGGER 0x05U
#define DAC80501_REG_STATUS  0x07U
#define DAC80501_REG_DAC     0x08U

/* ── Bit masks ──────────────────────────────────────────────────────── */
#define DAC80501_DEVID_RES_MASK       GENMASK(14, 12)
#define DAC80501_CONFIG_REF_PWDWN     BIT(8)
#define DAC80501_CONFIG_DAC_PWDWN     BIT(0)
#define DAC80501_GAIN_REFDIV_EN       BIT(8)
#define DAC80501_GAIN_BUFF_GAIN       BIT(0)
#define DAC80501_TRIGGER_SOFT_RESET   (BIT(1) | BIT(3))  /* 0x000A */

#define DAC80501_POR_DELAY_US         300

/* ── Output-gain enumeration (maps to DTS property) ─────────────────── */
enum dac80501_output_gain {
	DAC80501_GAIN_MUL2 = 0,    /* VREF × 2   (buff_gain=2x)            */
	DAC80501_GAIN_MUL1,        /* VREF × 1                              */
	DAC80501_GAIN_DIV2,        /* VREF / 2   (ref_div=on)               */
	DAC80501_GAIN_DIV2_X2,     /* VREF/2 × 2 (ref_div + buff_gain=2x)   */
};

enum dac80501_ref_source {
	DAC80501_REF_INTERNAL = 0,
	DAC80501_REF_EXTERNAL,
};

/* ── Per-instance config / data ─────────────────────────────────────── */
struct dac80501_spi_config {
	struct spi_dt_spec spi;
	enum dac80501_ref_source voltage_reference;
	enum dac80501_output_gain output_gain;
};

struct dac80501_spi_data {
	uint8_t resolution;   /* 12, 14 or 16 */
	bool    configured;
};

/* ── Low-level SPI helpers ──────────────────────────────────────────── */
static int dac80501_spi_write_reg(const struct device *dev,
				  uint8_t reg, uint16_t value)
{
	const struct dac80501_spi_config *cfg = dev->config;
	uint8_t frame[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFFU) };

	const struct spi_buf tx_buf = { .buf = frame, .len = sizeof(frame) };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

	return spi_write_dt(&cfg->spi, &tx);
}

static int dac80501_spi_read_reg(const struct device *dev,
				 uint8_t reg, uint16_t *value)
{
	const struct dac80501_spi_config *cfg = dev->config;
	int ret;

	/* DAC80501-SPI read: send register address, then clock out NOP to get data. */
	uint8_t tx_addr[3] = { reg, 0x00, 0x00 };
	const struct spi_buf tx_buf = { .buf = tx_addr, .len = sizeof(tx_addr) };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

	ret = spi_write_dt(&cfg->spi, &tx);
	if (ret) {
		return ret;
	}

	/* Second transaction: send NOP and read back the data. */
	uint8_t tx_nop[3] = { DAC80501_REG_NOOP, 0x00, 0x00 };
	uint8_t rx_data[3] = { 0 };

	const struct spi_buf tx_buf2 = { .buf = tx_nop, .len = sizeof(tx_nop) };
	const struct spi_buf_set tx2 = { .buffers = &tx_buf2, .count = 1 };
	const struct spi_buf rx_buf = { .buf = rx_data, .len = sizeof(rx_data) };
	const struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1 };

	ret = spi_transceive_dt(&cfg->spi, &tx2, &rx);
	if (ret) {
		return ret;
	}

	*value = sys_get_be16(&rx_data[1]);
	return 0;
}

/* ── Zephyr DAC API ─────────────────────────────────────────────────── */
static int dac80501_spi_channel_setup(const struct device *dev,
				      const struct dac_channel_cfg *channel_cfg)
{
	struct dac80501_spi_data *data = dev->data;

	if (channel_cfg->channel_id != 0) {
		LOG_ERR("Unsupported channel %d (only channel 0)", channel_cfg->channel_id);
		return -ENOTSUP;
	}

	if (channel_cfg->resolution != data->resolution) {
		LOG_ERR("Unsupported resolution %d, device has %d",
			channel_cfg->resolution, data->resolution);
		return -ENOTSUP;
	}

	if (channel_cfg->internal) {
		LOG_ERR("Internal channels not supported");
		return -ENOTSUP;
	}

	data->configured = true;
	return 0;
}

static int dac80501_spi_write_value(const struct device *dev,
				    uint8_t channel, uint32_t value)
{
	struct dac80501_spi_data *data = dev->data;

	if (channel != 0) {
		LOG_ERR("Unsupported channel %d", channel);
		return -ENOTSUP;
	}

	if (!data->configured) {
		LOG_ERR("Channel not configured");
		return -EINVAL;
	}

	if (value >= (1U << data->resolution)) {
		LOG_ERR("Value %u out of range for %u-bit DAC", value, data->resolution);
		return -EINVAL;
	}

	/* Left-align the value in the 16-bit DAC register. */
	uint16_t reg_value = (uint16_t)(value << (16U - data->resolution));

	return dac80501_spi_write_reg(dev, DAC80501_REG_DAC, reg_value);
}

/* ── Initialisation ─────────────────────────────────────────────────── */
static int dac80501_spi_init(const struct device *dev)
{
	const struct dac80501_spi_config *cfg = dev->config;
	struct dac80501_spi_data *data = dev->data;
	uint16_t devid;
	int ret;

	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus %s not ready", cfg->spi.bus->name);
		return -ENODEV;
	}

	/* Soft reset */
	ret = dac80501_spi_write_reg(dev, DAC80501_REG_TRIGGER,
				     DAC80501_TRIGGER_SOFT_RESET);
	if (ret) {
		LOG_ERR("Soft-reset failed (%d)", ret);
		return ret;
	}
	k_usleep(DAC80501_POR_DELAY_US);

	/* Read DEVICE_ID to determine resolution */
	ret = dac80501_spi_read_reg(dev, DAC80501_REG_DEVID, &devid);
	if (ret) {
		LOG_WRN("DEVICE_ID read failed (%d), assuming 16-bit", ret);
		data->resolution = 16;
	} else {
		uint8_t res_field = (uint8_t)FIELD_GET(DAC80501_DEVID_RES_MASK, devid);
		data->resolution = 16U - 2U * res_field;
		LOG_INF("DAC80501 detected: %u-bit resolution (DEVID=0x%04x)",
			data->resolution, devid);
	}

	/* Configure reference */
	uint16_t config_reg = 0;
	if (cfg->voltage_reference == DAC80501_REF_EXTERNAL) {
		config_reg |= DAC80501_CONFIG_REF_PWDWN;
	}
	ret = dac80501_spi_write_reg(dev, DAC80501_REG_CONFIG, config_reg);
	if (ret) {
		LOG_ERR("CONFIG write failed (%d)", ret);
		return ret;
	}

	/* Configure gain */
	uint16_t gain_reg = 0;
	switch (cfg->output_gain) {
	case DAC80501_GAIN_MUL2:
		gain_reg |= DAC80501_GAIN_BUFF_GAIN;
		break;
	case DAC80501_GAIN_DIV2:
		gain_reg |= DAC80501_GAIN_REFDIV_EN;
		break;
	case DAC80501_GAIN_DIV2_X2:
		gain_reg |= DAC80501_GAIN_REFDIV_EN | DAC80501_GAIN_BUFF_GAIN;
		break;
	case DAC80501_GAIN_MUL1:
	default:
		break;
	}
	ret = dac80501_spi_write_reg(dev, DAC80501_REG_GAIN, gain_reg);
	if (ret) {
		LOG_ERR("GAIN write failed (%d)", ret);
		return ret;
	}

	/* Start with DAC output at zero */
	ret = dac80501_spi_write_reg(dev, DAC80501_REG_DAC, 0x0000);
	if (ret) {
		LOG_ERR("DAC register init failed (%d)", ret);
		return ret;
	}

	data->configured = false;
	LOG_INF("DAC80501-SPI initialised (ref=%s, gain=%s)",
		cfg->voltage_reference == DAC80501_REF_INTERNAL ? "internal" : "external",
		cfg->output_gain == DAC80501_GAIN_MUL2 ? "mul2" :
		cfg->output_gain == DAC80501_GAIN_DIV2 ? "div2" :
		cfg->output_gain == DAC80501_GAIN_DIV2_X2 ? "div2-x2" : "mul1");

	return 0;
}

/* ── Driver API ─────────────────────────────────────────────────────── */
static DEVICE_API(dac, dac80501_spi_driver_api) = {
	.channel_setup = dac80501_spi_channel_setup,
	.write_value   = dac80501_spi_write_value,
};

/* ── Instantiation ──────────────────────────────────────────────────── */
#define DT_DRV_COMPAT ti_dac80501_spi

#define DAC80501_SPI_DEFINE(n)                                                  \
	static struct dac80501_spi_data dac80501_spi_data_##n = {};             \
	static const struct dac80501_spi_config dac80501_spi_config_##n = {     \
		.spi = SPI_DT_SPEC_INST_GET(n,                                 \
			SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |                \
			SPI_WORD_SET(8) | SPI_MODE_CPHA,                       \
			0),                                                     \
		.voltage_reference =                                            \
			_CONCAT(DAC80501_REF_,                                  \
				DT_STRING_UPPER_TOKEN(DT_DRV_INST(n),          \
						      voltage_reference)),      \
		.output_gain =                                                  \
			_CONCAT(DAC80501_GAIN_,                                 \
				DT_STRING_UPPER_TOKEN(DT_DRV_INST(n),          \
						      output_gain)),            \
	};                                                                      \
	DEVICE_DT_INST_DEFINE(n, &dac80501_spi_init, NULL,                     \
			      &dac80501_spi_data_##n,                           \
			      &dac80501_spi_config_##n,                         \
			      POST_KERNEL, CONFIG_DAC_INIT_PRIORITY,            \
			      &dac80501_spi_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DAC80501_SPI_DEFINE)
