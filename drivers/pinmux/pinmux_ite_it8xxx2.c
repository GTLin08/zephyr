/*
 * Copyright (c) 2020 ITE Corporation. All Rights Reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief PINMUX driver for the IT8xxx2
 */

#include <errno.h>
#include <device.h>
#include <drivers/pinmux.h>
#include <soc.h>

#define DT_DRV_COMPAT		ite_it8xxx2_pinmux
#define PIN_REG_OFFSET		8

struct pinmux_it8xxx2_config {
	uintptr_t base;
};

#define DEV_CFG(dev)					\
	((const struct pinmux_it8xxx2_config * const)	\
	 (dev)->config)

static int pinmux_it8xxx2_set(const struct device *dev,
		uint32_t pin, uint32_t func)
{
	const struct pinmux_it8xxx2_config *config = DEV_CFG(dev);
	uint32_t reg;
	uint8_t val;

	printk("dev=%p\n",dev);
	printk("config->base=%lx, pin=%d\n",config->base ,pin);

	if (func > IT8XXX2_PINMUX_IOF1 || pin >= IT8XXX2_PINMUX_PINS) {
		return -EINVAL;
	}

	reg = config->base + pin;
	val = sys_read8(reg);
	if (func == IT8XXX2_PINMUX_IOF0) {
		sys_write8((val &= ~(GPCR_PORT_PIN_MODE_INPUT |
			GPCR_PORT_PIN_MODE_OUTPUT)), reg);
	} else if (func == IT8XXX2_PINMUX_IOF1){
		sys_write8((val | GPCR_PORT_PIN_MODE_INPUT) &
			~GPCR_PORT_PIN_MODE_OUTPUT, reg);
	}

	return 0;
}

static int pinmux_it8xxx2_get(const struct device *dev,
		uint32_t pin, uint32_t *func)
{
	const struct pinmux_it8xxx2_config *config = DEV_CFG(dev);
	uint32_t reg;
	uint8_t val;

	if (pin >= IT8XXX2_PINMUX_PINS || func == NULL) {
		return -EINVAL;
	}

	reg = config->base + pin;
	val = sys_read8(reg);

	*func = (val & GPCR_PORT_PIN_MODE_INPUT) ?
		IT8XXX2_PINMUX_IOF1 : IT8XXX2_PINMUX_IOF0;	

	return 0;
}

static int pinmux_it8xxx2_pullup(const struct device *dev,
		uint32_t pin, uint8_t func)
{
	return -ENOTSUP;
}

static int pinmux_it8xxx2_input(const struct device *dev,
		uint32_t pin, uint8_t func)
{
	return -ENOTSUP;
}

static int pinmux_it8xxx2_init(const struct device *dev)
{
	const struct pinmux_it8xxx2_config *config = DEV_CFG(dev);

	printk("pinmux_it8xxx2_init-base=%lx\n",config->base);

	return 0;
}

static const struct pinmux_driver_api pinmux_it8xxx2_driver_api = {
	.set = pinmux_it8xxx2_set,
	.get = pinmux_it8xxx2_get,
	.pullup = pinmux_it8xxx2_pullup,
	.input = pinmux_it8xxx2_input,
};

#define PINMUX_ITE_INIT(inst)						\
	static const struct pinmux_it8xxx2_config pinmux_it8xxx2_cfg_##inst = {\
		.base = DT_INST_REG_ADDR(inst),		\
	};								\
									\
	DEVICE_DT_INST_DEFINE(inst,					\
			    &pinmux_it8xxx2_init,				\
			    device_pm_control_nop,			\
			    NULL, &pinmux_it8xxx2_cfg_##inst,		\
			    POST_KERNEL,				\
			    CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,	\
			    &pinmux_it8xxx2_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PINMUX_ITE_INIT)
