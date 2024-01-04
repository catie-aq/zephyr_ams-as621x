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

struct as621x_dev_cfg {
	struct i2c_dt_spec i2c_port;
};

static int as621x_fetch(const struct device *dev, enum sensor_channel chan)
{
	return 0;
}

static int as621x_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	return 0;
}

static const struct sensor_driver_api as5600_driver_api = {
	.sample_fetch = as621x_fetch,
	.channel_get = as621x_get,
};

#define AS621X_INIT(n)                                                                             \
	static struct as621x_dev_cfg as621x_config_##n = {                                         \
		.i2c_port = I2C_DT_SPEC_INST_GET(n),                                               \
	};                                                                                         \
	static struct as621x_data as621x_data_##n;                                                 \
	DEVICE_DT_INST_DEFINE(n, as621x_init, NULL, &as621x_data_##n, &as621x_config_##n,          \
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &as5600_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AS621X_INIT)
