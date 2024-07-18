/*
 * Copyright (c) 2024 ITE Corporation. All Rights Reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ite_it8801_mfd

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/mfd/mfd_ite_it8801.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mfd_ite_it8801, CONFIG_MFD_LOG_LEVEL);

struct mfd_it8801_config {
	const struct i2c_dt_spec i2c_dev;
};

const struct i2c_dt_spec *mfd_it8801_get_i2c_dt_spec(const struct device *dev)
{
	const struct mfd_it8801_config *config = dev->config;

	return &config->i2c_dev;
}

static int it8801_check_vendor_id(const struct device *dev)
{
	const struct mfd_it8801_config *config = dev->config;
	uint8_t val;
	int i, ret;

	/*  Verify vendor ID registers(16-bits). */
	for (i = 0; i < ARRAY_SIZE(it8801_id_verify); i++) {
		ret = i2c_reg_read_byte_dt(&config->i2c_dev,
					   it8801_id_verify[i].reg, &val);

		if (!ret ) {
			return ret;
		}

		if (val != it8801_id_verify[i].chip_id)
			return ret;
	}

	return ret;
}

static int mfd_it8801_init(const struct device *dev)
{
	const struct mfd_it8801_config *config = dev->config;
	int ret;

	if (!device_is_ready(config->i2c_dev.bus)) {
		LOG_ERR("I2C bus %s is not ready", config->i2c_dev.bus->name);
		return -ENODEV;
	}

	/*  Verify Vendor ID registers. */
	ret = it8801_check_vendor_id(dev);
	if (ret) {
		LOG_ERR("Failed to read IT8801 vendor id %x", ret);
		return ret;
	}

	return 0;
}

#define MFD_IT8801_DEFINE(inst)                                          \
	static const struct mfd_it8801_config it8801_cfg_##inst = {      \
		.i2c_dev = I2C_DT_SPEC_INST_GET(inst),                   \
	};                                                               \
                                                                         \
	DEVICE_DT_INST_DEFINE(inst,                                      \
			      mfd_it8801_init,                           \
			      NULL,                                      \
			      NULL,                                      \
			      &it8801_cfg_##inst,                        \
			      POST_KERNEL,                               \
			      CONFIG_MFD_INIT_PRIORITY,                  \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(MFD_IT8801_DEFINE)
