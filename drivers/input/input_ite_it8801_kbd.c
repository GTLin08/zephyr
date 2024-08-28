/*
 * Copyright (c) 2024 ITE Corporation. All Rights Reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ite_it8801_kbd

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/mfd/mfd_ite_it8801.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/input/input.h>
#include <zephyr/input/input_kbd_matrix.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(input_ite_it8801_kbd, CONFIG_INPUT_LOG_LEVEL);

struct it8801_mfd_input_altctrl_cfg {
	/* GPIO control device structure */
	const struct device *gpiocr;
	/* GPIO control pin */
	uint8_t pin;
	/* GPIO function select */
	uint8_t alt_func;
};

struct kbd_it8801_config {
	struct input_kbd_matrix_common_config common;
	/* IT8801 controller dev */
	const struct device *mfd;
	/* Alert GPIO pin */
	const struct gpio_dt_spec irq_gpios;
	/* KSO alternate configuration */
	const struct it8801_mfd_input_altctrl_cfg *altctrl;
	uint8_t kso_mapping[DT_INST_PROP(0, col_size)];
	/* Keyboard scan out mode control register */
	uint8_t reg_ksomcr;
	/* Keyboard scan in data register */
	uint8_t reg_ksidr;
	/* Keyboard scan in edge event register */
	uint8_t reg_ksieer;
	/* Keyboard scan in interrupt enable register */
	uint8_t reg_ksiier;
};

struct kbd_it8801_data {
	struct input_kbd_matrix_common_data common;
	/* Alert pin callback */
	struct gpio_callback gpio_pin_cb;
	const struct device *kbd_dev;
	/* I2C device for the MFD parent */
	const struct i2c_dt_spec *i2c_dev;
};

INPUT_KBD_STRUCT_CHECK(struct kbd_it8801_config, struct kbd_it8801_data);

static void kbd_it8801_drive_column(const struct device *dev, int col)
{
	const struct kbd_it8801_config *config = dev->config;
	struct kbd_it8801_data *data = dev->data;
	int ret;
	uint8_t kso_val;

	/* Tri-state all outputs */
	if (col == INPUT_KBD_MATRIX_COLUMN_DRIVE_NONE) {
		/* KSO[22:11, 6:0] output high */
		kso_val = IT8801_REG_MASK_KSOSDIC | IT8801_REG_MASK_AKSOSC;
	}
	/* Assert all outputs */
	else if (col == INPUT_KBD_MATRIX_COLUMN_DRIVE_ALL) {
		/* KSO[22:11, 6:0] output low */
		kso_val = IT8801_REG_MASK_AKSOSC;
	} else {
		/*
		 * Selected KSO[22:11, 6:0] output low,
		 * all others KSO output high
		 */
		kso_val = config->kso_mapping[col];
	}

	ret = i2c_reg_write_byte_dt(data->i2c_dev, config->reg_ksomcr, kso_val);
	if (ret != 0) {
		LOG_ERR("Failed to drive column (ret %d)", ret);
		return;
	}
}

static kbd_row_t kbd_it8801_read_row(const struct device *dev)
{
	const struct kbd_it8801_config *const config = dev->config;
	struct kbd_it8801_data *data = dev->data;
	int ret;
	uint8_t value, ksieer = 0;

	ret = i2c_reg_read_byte_dt(data->i2c_dev, config->reg_ksidr, &value);

	/* This register needs to write clear after reading data */
	ret = i2c_reg_update_byte_dt(data->i2c_dev, config->reg_ksieer, ksieer,
				     ksieer);
	if (ret != 0) {
		LOG_ERR("Failed to read row (ret %d)", ret);
	}

	/* Bits are active-low, so invert returned levels */
	return (~value) & 0xff;
}

void it8801_kbd_isr(const struct device *gpio_port, struct gpio_callback *cb,
		    uint32_t pins)
{
	struct kbd_it8801_data *data =
		CONTAINER_OF(cb, struct kbd_it8801_data, gpio_pin_cb);

	gpio_pin_interrupt_configure(gpio_port, (find_msb_set(pins) - 1),
				     GPIO_INT_DISABLE);

	input_kbd_matrix_poll_start(data->kbd_dev);
}

static void it8801_muxed_kbd_gpio_intr_enable(const struct device *dev)
{
	const struct kbd_it8801_config *const config = dev->config;
	int ret;

	ret = gpio_pin_interrupt_configure_dt(&config->irq_gpios,
				GPIO_INT_MODE_EDGE | GPIO_INT_TRIG_LOW);
	if (ret != 0) {
		LOG_ERR("Failed to configure irq_gpios (ret %d)", ret);
		return;
	}
}

static void kbd_it8801_set_detect_mode(const struct device *dev, bool enable)
{
	const struct kbd_it8801_config *const config = dev->config;
	struct kbd_it8801_data *data = dev->data;
	int ret;

	if (enable) {
		/* Clear pending iterrupts */
		ret = i2c_reg_write_byte_dt(data->i2c_dev, config->reg_ksieer,
					    GENMASK(7, 0));
		/* Enable GPIO interrupt */
		it8801_muxed_kbd_gpio_intr_enable(dev);
		/* Enable KSI falling edge event trigger interrupt */
		ret = i2c_reg_write_byte_dt(data->i2c_dev, config->reg_ksiier,
					    GENMASK(7, 0));
	} else {
		/* Disable KSI falling edge event trigger interrupt */
		ret = i2c_reg_write_byte_dt(data->i2c_dev, config->reg_ksiier,
					    0x00);
	}

	if (ret != 0) {
		LOG_ERR("Failed to set detect mode (ret %d)", ret);
		return;
	}
}

static int kbd_it8801_init(const struct device *dev)
{
	const struct kbd_it8801_config *const config = dev->config;
	struct kbd_it8801_data *data = dev->data;
	int ret;

	/* Verify multi-function parent is ready */
	if (!device_is_ready(config->mfd)) {
		LOG_ERR("(input)%s is not ready", config->mfd->name);
		return -ENODEV;
	}

#if MFDCTRL_ALT_COUNT
	int status;

	for (int i = 0; i < MFDCTRL_ALT_COUNT; i++) {
		/* Switching the pin to KSO alternate function (KSO[21:18]) */
		status = mfd_it8801_configure_pins(config->altctrl[i].gpiocr,
						   config->altctrl[i].pin,
						   config->altctrl[i].alt_func);
		if (status != 0) {
			LOG_ERR("Failed to configure KSO[21:18] pins");
			return status;
		}
	}
#endif
	data->kbd_dev = dev;
	data->i2c_dev = mfd_it8801_get_i2c_dt_spec(config->mfd);

	/* Disable wakeup and interrupt of KSI pins before configuring */
	kbd_it8801_set_detect_mode(dev, false);

	/* Start with KEYBOARD_COLUMN_ALL, KSO[22:11, 6:0] output low */
	ret = i2c_reg_write_byte_dt(data->i2c_dev, config->reg_ksomcr,
				    IT8801_REG_MASK_AKSOSC);
	/* Keyboard scan in interrupt enable register */
	ret = i2c_reg_write_byte_dt(data->i2c_dev, config->reg_ksiier,
				    GENMASK(7, 0));
	/* Gather KSI interrupt enable */
	ret = i2c_reg_write_byte_dt(data->i2c_dev, IT8801_REG_GIECR,
				    IT8801_REG_MASK_GKSIIE);
	/* Alert response enable */
	ret = i2c_reg_write_byte_dt(data->i2c_dev, IT8801_REG_SMBCR,
				    IT8801_REG_MASK_ARE);
	if (ret != 0) {
		LOG_ERR("Failed to initialization setting (ret %d)", ret);
		return ret;
	}

	/* Initialize GPIO interrupt callback */
	gpio_init_callback(&data->gpio_pin_cb, it8801_kbd_isr,
			   BIT(config->irq_gpios.pin));

	ret = gpio_add_callback(config->irq_gpios.port, &data->gpio_pin_cb);
	if (ret != 0) {
		LOG_ERR("Failed to add INT callback: %d",ret);
		return ret;
	}

	return input_kbd_matrix_common_init(dev);
}

#if MFDCTRL_ALT_COUNT
	static const struct it8801_mfd_input_altctrl_cfg
		it8801_input_altctrl[IT8801_DT_INST_MFCCTRL_LEN(0)] =
			IT8801_DT_MFD_ITEMS_LIST(0);
#endif

INPUT_KBD_MATRIX_DT_INST_DEFINE(0);

static const struct input_kbd_matrix_api kbd_it8801_api = {
	.drive_column = kbd_it8801_drive_column,
	.read_row = kbd_it8801_read_row,
	.set_detect_mode = kbd_it8801_set_detect_mode,
};

static const struct kbd_it8801_config kbd_it8801_cfg_0 = {
	.common = INPUT_KBD_MATRIX_DT_INST_COMMON_CONFIG_INIT(0, &kbd_it8801_api),
	.mfd = DEVICE_DT_GET(DT_INST_PARENT(0)),
	.irq_gpios = GPIO_DT_SPEC_GET(DT_INST_PARENT(0), irq_gpios),
#if MFDCTRL_ALT_COUNT
	.altctrl = it8801_input_altctrl,
#endif
	.kso_mapping = DT_INST_PROP(0, kso_mapping),
	.reg_ksomcr = DT_INST_REG_ADDR_BY_IDX(0, 0),
	.reg_ksidr = DT_INST_REG_ADDR_BY_IDX(0, 1),
	.reg_ksieer = DT_INST_REG_ADDR_BY_IDX(0, 2),
	.reg_ksiier = DT_INST_REG_ADDR_BY_IDX(0, 3),
};

static struct kbd_it8801_data kbd_it8801_data_0;
	DEVICE_DT_INST_DEFINE(0,
			      &kbd_it8801_init,
			      NULL,
			      &kbd_it8801_data_0,
			      &kbd_it8801_cfg_0,
			      POST_KERNEL,
			      CONFIG_INPUT_INIT_PRIORITY,
			      NULL);

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
	     "only one ite,it8801-kbd compatible node can be supported");
BUILD_ASSERT(IN_RANGE(DT_INST_PROP(0, row_size), 1, 8), "invalid row-size");
BUILD_ASSERT(IN_RANGE(DT_INST_PROP(0, col_size), 1, 19), "invalid col-size");
