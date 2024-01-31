/*
 * Copyright (c) 2024 CATIE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ams_as621x

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "ams_as621x.h"

LOG_MODULE_REGISTER(AS621X, CONFIG_SENSOR_LOG_LEVEL);

struct as621x_cfg {
	struct i2c_dt_spec i2c;
};

struct as621x_data {
	uint16_t temp;
};

static int as621x_set_conversion_rate(const struct device *dev, uint8_t rate)
{
	const struct as621x_cfg *cfg = dev->config;
	uint16_t config;

	int ret = i2c_burst_read_dt(&cfg->i2c, AS621X_REG_CONFIG, (uint8_t *)&config, 2);

	if (ret) {
		return -EIO;
	}

	config &= ~(0x3 << 6);
	config |= (rate << 6);

	return i2c_burst_write_dt(&cfg->i2c, AS621X_REG_CONFIG, (uint8_t *)&config, 2);
}

static int as621x_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct as621x_data *dev_data = dev->data;
	const struct as621x_cfg *dev_cfg = dev->config;

	uint16_t value;
	int ret;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	ret = i2c_burst_read_dt(&dev_cfg->i2c, AS621X_REG_TVAL, (uint8_t *)&value, 2);
	if (!ret) {
		value = (value >> 8) | (value << 8);
		dev_data->temp = value;
	}

	return ret;
}

static int as621x_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct as621x_data *dev_data = dev->data;

	if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	// One LSB corresponds to 0.0078125°C (1/128 °C)
	val->val1 = dev_data->temp >> 7;
	val->val2 = (dev_data->temp % 128) * 78125;

	return 0;
}

static int as621x_init(const struct device *dev)
{
	const struct as621x_cfg *cfg = dev->config;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus %s not ready", cfg->i2c.bus->name);
		return -ENODEV;
	}

	if (as621x_set_conversion_rate(dev, AS621X_CONVERSION_RATE)) {
		LOG_ERR("Failed to set conversion rate");
		return -EIO;
	}

	return 0;
}

static const struct sensor_driver_api as621x_driver_api = {
	.sample_fetch = as621x_sample_fetch,
	.channel_get = as621x_channel_get,
};

#define AS621X_INIT(n)                                                                             \
	static struct as621x_cfg as621x_config_##n = {                                             \
		.i2c = I2C_DT_SPEC_INST_GET(n),                                                    \
	};                                                                                         \
	static struct as621x_data as621x_data_##n;                                                 \
	DEVICE_DT_INST_DEFINE(n, as621x_init, NULL, &as621x_data_##n, &as621x_config_##n,          \
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &as621x_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AS621X_INIT)
