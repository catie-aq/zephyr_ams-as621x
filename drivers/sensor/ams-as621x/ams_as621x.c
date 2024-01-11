/*
 * Copyright (c) 2024 CATIE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ams_as621x

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ams_as621x, CONFIG_SENSOR_LOG_LEVEL);

#define TVAL   0x0
#define CONFIG 0x1
#define TLOW   0x2
#define THIGH  0x3

struct as621x_cfg {
	struct i2c_dt_spec i2c;
};

struct as621x_data {
	uint16_t temp;
};

static int as621x_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct as621x_data *dev_data = dev->data;
	const struct as621x_cfg *dev_cfg = dev->config;

	uint16_t value;
	int ret;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	ret = i2c_burst_read_dt(&dev_cfg->i2c, TVAL, (uint8_t *)&value, 2);
	if (!ret) {
		value = (value >> 8) | (value << 8);
		dev_data->temp = value;
	}

	return ret;
}

static int as621x_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	struct as621x_data *dev_data = dev->data;

	if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	// One LSB corresponds to 0.0078125°C (1/128 °C)
	val->val1 = dev_data->temp / 128;
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

	return 0;
}

static const struct sensor_driver_api as621x_driver_api = {
	.sample_fetch = as621x_fetch,
	.channel_get = as621x_get,
};

#define AS621X_INIT(n)                                                                             \
	static struct as621x_cfg as621x_config_##n = {                                             \
		.i2c = I2C_DT_SPEC_INST_GET(n),                                                    \
	};                                                                                         \
	static struct as621x_data as621x_data_##n;                                                 \
	DEVICE_DT_INST_DEFINE(n, as621x_init, NULL, &as621x_data_##n, &as621x_config_##n,          \
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &as621x_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AS621X_INIT)
