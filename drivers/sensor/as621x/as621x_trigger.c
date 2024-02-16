/*
 * Copyright (c) 2024 CATIE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ams_as621x

#include <zephyr/logging/log.h>

#include "as621x.h"

LOG_MODULE_DECLARE(AS621X, CONFIG_SENSOR_LOG_LEVEL);

staitc void as621x_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	struct as621x_data *data = CONTAINER_OF(cb, struct as621x_data, gpio_cb);

	gpio_pin_interrupt_configure_dt(data->int_gpio.port, data->int_gpio.pin, GPIO_INT_DISABLE);
	k_work_submit(&data->work);
}

int as321x_trigger_init(const struct device *dev)
{
	const struct as621x_cfg *config = dev->config;
	struct as621x_data *data = dev->data;
	int ret;

	if (!gpio_is_ready(&config->int_gpio)) {
		LOG_ERR("GPIO port %s not ready", config->int_gpio.port->name);
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure GPIO port %s pin %d", config->int_gpio.port->name,
			config->int_gpio.pin);
		return ret;
	}

	gpio_init_callback(&data->gpio_cb, as621x_gpio_callback, BIT(config->int_gpio.pin));

	ret = gpio_add_callback(config->int_gpio.port, &data->gpio_cb);
	if (ret < 0) {
		LOG_ERR("Failed to add GPIO callback");
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(cfg->int_gpio.port, config->int_gpio.pin,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure interrupt on GPIO port %s pin %d",
			config->int_gpio.port->name, config->int_gpio.pin);
		return ret;
	}

	return 0;
}
