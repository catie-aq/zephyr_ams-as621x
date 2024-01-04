/*
 * Copyright (c) 2024 CATIE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

int main(void)
{
	const struct device *const dev = DEVICE_DT_GET_ANY(ams_as621x);
	struct sensor_value temp;

	if (!device_is_ready(dev)) {
		printk("Device %s is not ready\n", dev->name);
		return 1;
	}

	while (1) {
		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		printk("Temperature: %d.%06d\n", temp.val1, temp.val2);
		k_sleep(K_MSEC(1000));
	}
}
