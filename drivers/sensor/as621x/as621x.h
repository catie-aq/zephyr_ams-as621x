/*
 * Copyright (c) 2024 CATIE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_AS621X_AS621X_H_
#define ZEPHYR_DRIVERS_SENSOR_AS621X_AS621X_H_

#define AS621X_REG_TVAL   0x0
#define AS621X_REG_CONFIG 0x1
#define AS621X_REG_TLOW   0x2
#define AS621X_REG_THIGH  0x3

#define AS621X_CR_SHIFT 6

#define ASX621X_CONFIG_ALERT BIT(5)
#define ASX621X_CONFIG_CR    BIT_MASK(2) << AS621X_CR_SHIFT
#define ASX621X_CONFIG_SM    BIT(8)
#define ASX621X_CONFIG_IM    BIT(9)
#define ASX621X_CONFIG_POL   BIT(10)
#define ASX621X_CONFIG_CF    BIT_MASK(2) << 11
#define ASX621X_CONFIG_SS    BIT(15)

#if defined CONFIG_AS621X_CONVERSION_RATE_0_25HZ
#define AS621X_CONVERSION_RATE 0x0
#elif defined CONFIG_AS621X_CONVERSION_RATE_1HZ
#define AS621X_CONVERSION_RATE 0x1
#elif defined CONFIG_AS621X_CONVERSION_RATE_4HZ
#define AS621X_CONVERSION_RATE 0x2
#elif defined CONFIG_AS621X_CONVERSION_RATE_8HZ
#define AS621X_CONVERSION_RATE 0x3
#endif

#if CONFIG_AS61X_TRIGGER
int as621x_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler);
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_AS621X_AS621X_H_ */
