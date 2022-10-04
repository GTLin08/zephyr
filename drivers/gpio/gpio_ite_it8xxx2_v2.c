/*
 * Copyright (c) 2022 ITE Corporation. All Rights Reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#include <errno.h>
#include <soc_dt.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/interrupt_controller/wuc_ite_it8xxx2.h>
#include <zephyr/dt-bindings/gpio/ite-it8xxx2-gpio.h>
#include <zephyr/dt-bindings/interrupt-controller/ite-intc.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>

#include "gpio_utils.h"

#define DT_DRV_COMPAT ite_it8xxx2_gpio_v2

struct gpio_wui_maps_cfg {
	/* WUC control device structure */
	const struct device *wucs;
	/* WUC pin mask */
	uint8_t mask;
};

/*
 * Structure gpio_ite_cfg is about the setting of gpio
 * this config will be used at initial time
 */
struct gpio_ite_cfg {
	/* Mapping table between gpio bits and wui */
	const struct gpio_wui_maps_cfg wui_maps[8];
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	/* gpio port data register (bit mapping to pin) */
	uintptr_t reg_gpdr;
	/* gpio port control register (byte mapping to pin) */
	uintptr_t reg_gpcr;
	/* gpio port data mirror register (bit mapping to pin) */
	uintptr_t reg_gpdmr;
	/* gpio port output type register (bit mapping to pin) */
	uintptr_t reg_gpotr;
	/* Port 1.8V select control. */
	uintptr_t reg_p18sc;
	/* gpio's irq */
	uint8_t gpio_irq[8];
	/* Index in gpio_1p8v for voltage level control register element. */
	uint8_t index;
	/* Number of pins per group of gpio. */
	uint8_t num_pins;
};

/* Structure gpio_ite_data is about callback function */
struct gpio_ite_data {
	struct gpio_driver_data common;
	sys_slist_t callbacks;
};

/* 1.8v gpio group a, b, c, d, e, f, g, h, i, j, k, l, and m */
#define GPIO_GROUP_COUNT 13

/**
 * Driver functions
 */
static int gpio_ite_configure(const struct device *dev,
			      gpio_pin_t pin,
			      gpio_flags_t flags)
{
	const struct gpio_ite_cfg *gpio_config = dev->config;
	volatile uint8_t *reg_gpdr = (uint8_t *)gpio_config->reg_gpdr;
	volatile uint8_t *reg_gpcr = (uint8_t *)(gpio_config->reg_gpcr + pin);
	volatile uint8_t *reg_gpotr = (uint8_t *)gpio_config->reg_gpotr;
	volatile uint8_t *reg_1p8v = (uint8_t *)gpio_config->reg_p18sc;
	uint8_t mask = BIT(pin);

	__ASSERT(gpio_config->index < GPIO_GROUP_COUNT,
		"Invalid GPIO group index");

	/* Don't support "open source" mode */
	if (((flags & GPIO_SINGLE_ENDED) != 0) &&
	    ((flags & GPIO_LINE_OPEN_DRAIN) == 0)) {
		return -ENOTSUP;
	}

	/*
	 * Select open drain first, so that we don't glitch the signal
	 * when changing the line to an output.
	 */
	if (flags & GPIO_OPEN_DRAIN) {
		*reg_gpotr |= mask;
	} else {
		*reg_gpotr &= ~mask;
	}

	/* 1.8V or 3.3V */
	gpio_flags_t volt = flags & IT8XXX2_GPIO_VOLTAGE_MASK;

	if (volt == IT8XXX2_GPIO_VOLTAGE_1P8) {
		__ASSERT(!(flags & GPIO_PULL_UP),
		"Don't enable internal pullup if 1.8V voltage is used");
		*reg_1p8v |= mask;
	} else if (volt == IT8XXX2_GPIO_VOLTAGE_3P3 ||
		   volt == IT8XXX2_GPIO_VOLTAGE_DEFAULT) {
		*reg_1p8v &= ~mask;
	} else {
		return -EINVAL;
	}

	/* If output, set level before changing type to an output. */
	if (flags & GPIO_OUTPUT) {
		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			*reg_gpdr |= mask;
		} else if (flags & GPIO_OUTPUT_INIT_LOW) {
			*reg_gpdr &= ~mask;
		}
	}

	/* Set input or output. */
	if (flags & GPIO_OUTPUT)
		*reg_gpcr = (*reg_gpcr | GPCR_PORT_PIN_MODE_OUTPUT) &
				~GPCR_PORT_PIN_MODE_INPUT;
	else
		*reg_gpcr = (*reg_gpcr | GPCR_PORT_PIN_MODE_INPUT) &
				~GPCR_PORT_PIN_MODE_OUTPUT;

	/* Handle pullup / pulldown */
	if (flags & GPIO_PULL_UP) {
		*reg_gpcr = (*reg_gpcr | GPCR_PORT_PIN_MODE_PULLUP) &
				~GPCR_PORT_PIN_MODE_PULLDOWN;
	} else if (flags & GPIO_PULL_DOWN) {
		*reg_gpcr = (*reg_gpcr | GPCR_PORT_PIN_MODE_PULLDOWN) &
				~GPCR_PORT_PIN_MODE_PULLUP;
	} else {
		/* No pull up/down */
		*reg_gpcr &= ~(GPCR_PORT_PIN_MODE_PULLUP |
				GPCR_PORT_PIN_MODE_PULLDOWN);
	}

	return 0;
}

#ifdef CONFIG_GPIO_GET_CONFIG
static int gpio_ite_get_config(const struct device *dev,
			       gpio_pin_t pin,
			       gpio_flags_t *out_flags)
{
	const struct gpio_ite_cfg *gpio_config = dev->config;
	volatile uint8_t *reg_gpdr = (uint8_t *)gpio_config->reg_gpdr;
	volatile uint8_t *reg_gpcr = (uint8_t *)(gpio_config->reg_gpcr + pin);
	volatile uint8_t *reg_gpotr = (uint8_t *)gpio_config->reg_gpotr;
	volatile uint8_t *reg_1p8v = (uint8_t *)gpio_config->reg_p18sc;
	uint8_t mask = BIT(pin);
	gpio_flags_t flags = 0;

	__ASSERT(gpio_config->index < GPIO_GROUP_COUNT,
		"Invalid GPIO group index");

	/* push-pull or open-drain */
	if (*reg_gpotr & mask)
		flags |= GPIO_OPEN_DRAIN;

	/* 1.8V or 3.3V */
	if (*reg_1p8v & mask) {
		flags |= GPIO_VOLTAGE_1P8;
	} else {
		flags |= GPIO_VOLTAGE_3P3;
	}

	/* set input or output. */
	if (*reg_gpcr & GPCR_PORT_PIN_MODE_OUTPUT) {
		flags |= GPIO_OUTPUT;

		/* set level */
		if (*reg_gpdr & mask) {
			flags |= GPIO_OUTPUT_HIGH;
		} else {
			flags |= GPIO_OUTPUT_LOW;
		}
	}

	if (*reg_gpcr & GPCR_PORT_PIN_MODE_INPUT) {
		flags |= GPIO_INPUT;

		/* pullup / pulldown */
		if (*reg_gpcr & GPCR_PORT_PIN_MODE_PULLUP) {
			flags |= GPIO_PULL_UP;
		}

		if (*reg_gpcr & GPCR_PORT_PIN_MODE_PULLDOWN) {
			flags |= GPIO_PULL_DOWN;
		}
	}

	*out_flags = flags;

	return 0;
}
#endif

static int gpio_ite_port_get_raw(const struct device *dev,
				 gpio_port_value_t *value)
{
	const struct gpio_ite_cfg *gpio_config = dev->config;
	volatile uint8_t *reg_gpdmr = (uint8_t *)gpio_config->reg_gpdmr;

	/* Get raw bits of GPIO mirror register */
	*value = *reg_gpdmr;

	return 0;
}

static int gpio_ite_port_set_masked_raw(const struct device *dev,
					gpio_port_pins_t mask,
					gpio_port_value_t value)
{
	const struct gpio_ite_cfg *gpio_config = dev->config;
	volatile uint8_t *reg_gpdr = (uint8_t *)gpio_config->reg_gpdr;
	uint8_t out = *reg_gpdr;

	*reg_gpdr = ((out & ~mask) | (value & mask));

	return 0;
}

static int gpio_ite_port_set_bits_raw(const struct device *dev,
				      gpio_port_pins_t pins)
{
	const struct gpio_ite_cfg *gpio_config = dev->config;
	volatile uint8_t *reg_gpdr = (uint8_t *)gpio_config->reg_gpdr;

	/* Set raw bits of GPIO data register */
	*reg_gpdr |= pins;

	return 0;
}

static int gpio_ite_port_clear_bits_raw(const struct device *dev,
				        gpio_port_pins_t pins)
{
	const struct gpio_ite_cfg *gpio_config = dev->config;
	volatile uint8_t *reg_gpdr = (uint8_t *)gpio_config->reg_gpdr;

	/* Clear raw bits of GPIO data register */
	*reg_gpdr &= ~pins;

	return 0;
}

static int gpio_ite_port_toggle_bits(const struct device *dev,
				     gpio_port_pins_t pins)
{
	const struct gpio_ite_cfg *gpio_config = dev->config;
	volatile uint8_t *reg_gpdr = (uint8_t *)gpio_config->reg_gpdr;

	/* Toggle raw bits of GPIO data register */
	*reg_gpdr ^= pins;

	return 0;
}

static int gpio_ite_manage_callback(const struct device *dev,
				    struct gpio_callback *callback,
				    bool set)
{
	struct gpio_ite_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

static void gpio_ite_isr(const void *arg)
{
	const struct device *dev = arg;
	const struct gpio_ite_cfg *gpio_config = dev->config;
	struct gpio_ite_data *data = dev->data;
	uint8_t irq = ite_intc_get_irq_num();
	uint8_t num_pins = gpio_config->num_pins;
	uint8_t pin;

	for (pin = 0; pin <= num_pins; pin++) {
		if (irq == gpio_config->gpio_irq[pin]) {
			const struct device *wucs = gpio_config->wui_maps[pin].wucs;
			const struct it8xxx2_wuc_cfg *wuc_config = wucs->config;
			volatile uint8_t *reg_wuesr = wuc_config->reg_wuesr;
			uint8_t wuc_mask = gpio_config->wui_maps[pin].mask;

			printk("isr=====pin=%d\n",pin);
			printk("isr=====reg_wuesr=%p\n",reg_wuesr);
			printk("isr=====wuc_mask=%d\n",wuc_mask);

			/* Clear the WUC status register. */
			*reg_wuesr = wuc_mask;
			gpio_fire_callbacks(&data->callbacks, dev, BIT(pin));

			break;
		}
	}
}

static int gpio_ite_pin_interrupt_configure(const struct device *dev,
					    gpio_pin_t pin,
					    enum gpio_int_mode mode,
					    enum gpio_int_trig trig)
{
	const struct gpio_ite_cfg *gpio_config = dev->config;
	const struct device *wucs = gpio_config->wui_maps[pin].wucs;
	const struct it8xxx2_wuc_cfg *wuc_config = wucs->config;
	uint8_t gpio_irq = gpio_config->gpio_irq[pin];

	printk("===============pin=%d\n",pin);
	printk("gpio_irq=%d\n",gpio_irq);

	if (mode == GPIO_INT_MODE_DISABLED) {
		/* Disable GPIO interrupt */
		irq_disable(gpio_irq);
		return 0;
	}

	if (mode == GPIO_INT_MODE_LEVEL) {
		printk("Level trigger mode not supported.\r\n");
		return -ENOTSUP;
	}

	/* Disable irq before configuring it */
	irq_disable(gpio_irq);

	if (trig & GPIO_INT_TRIG_BOTH) {
		volatile uint8_t *reg_wuemr = wuc_config->reg_wuemr;
		volatile uint8_t *reg_wuesr = wuc_config->reg_wuesr;
		volatile uint8_t *reg_wubemr = wuc_config->reg_wubemr;
		uint8_t wuc_mask = gpio_config->wui_maps[pin].mask;

		printk("reg_wuemr=%p\n",wuc_config->reg_wuemr);
		printk("wuc_mask=%d\n",wuc_mask);

		/* Set both edges interrupt. */
		if ((trig & GPIO_INT_TRIG_BOTH) == GPIO_INT_TRIG_BOTH) {
			*reg_wubemr |= wuc_mask;
		} else {
			*reg_wubemr &= ~wuc_mask;
		}

		if (trig & GPIO_INT_TRIG_LOW) {
			*reg_wuemr |= wuc_mask;
		} else {
			*reg_wuemr &= ~wuc_mask;
		}
		/*
		 * Always write 1 to clear the WUC status register after
		 * modifying edge mode selection register (WUBEMR and WUEMR).
		 */
		*reg_wuesr = wuc_mask;
	}

	/* Enable GPIO interrupt */
	irq_connect_dynamic(gpio_irq, 0, gpio_ite_isr, dev, 0);
	irq_enable(gpio_irq);

	return 0;
}

static const struct gpio_driver_api gpio_ite_driver_api = {
	.pin_configure = gpio_ite_configure,
#ifdef CONFIG_GPIO_GET_CONFIG
	.pin_get_config = gpio_ite_get_config,
#endif
	.port_get_raw = gpio_ite_port_get_raw,
	.port_set_masked_raw = gpio_ite_port_set_masked_raw,
	.port_set_bits_raw = gpio_ite_port_set_bits_raw,
	.port_clear_bits_raw = gpio_ite_port_clear_bits_raw,
	.port_toggle_bits = gpio_ite_port_toggle_bits,
	.pin_interrupt_configure = gpio_ite_pin_interrupt_configure,
	.manage_callback = gpio_ite_manage_callback,
};

static int gpio_ite_init(const struct device *dev)
{
	const struct gpio_ite_cfg *gpio_config = dev->config;
	uint8_t gpio_irq = gpio_config->gpio_irq[0];

	volatile uint8_t *reg_1p8v = (uint8_t *)gpio_config->reg_p18sc;

	printk("gpio_irq0=%d\n",gpio_irq);
	printk("reg_1p8v=%p\n",reg_1p8v);

	return 0;
}

#define GPIO_ITE_DEV_CFG_DATA(inst)                                \
static struct gpio_ite_data gpio_ite_data_##inst;                  \
static const struct gpio_ite_cfg gpio_ite_cfg_##inst = {           \
	.wui_maps = IT8XXX2_DT_WUC_ITEMS_LIST(inst),               \
	.common = {                                                \
		.port_pin_mask =                                   \
			GPIO_PORT_PIN_MASK_FROM_DT_INST(inst)      \
	},                                                         \
	.reg_gpdr = DT_INST_REG_ADDR_BY_IDX(inst, 0),              \
	.reg_gpcr = DT_INST_REG_ADDR_BY_IDX(inst, 1),              \
	.reg_gpdmr = DT_INST_REG_ADDR_BY_IDX(inst, 2),             \
	.reg_gpotr = DT_INST_REG_ADDR_BY_IDX(inst, 3),             \
	.reg_p18sc = DT_INST_REG_ADDR_BY_IDX(inst, 4),             \
	.gpio_irq = IT8XXX2_DT_GPIO_IRQ_LIST(inst),                \
	.index = (uint8_t)(DT_INST_REG_ADDR(inst) -                \
			   DT_REG_ADDR(DT_NODELABEL(gpioa))),      \
	.num_pins = DT_INST_PROP(inst, ngpios),                    \
	};                                                         \
DEVICE_DT_INST_DEFINE(inst,                                        \
		gpio_ite_init,                                     \
		NULL,                                              \
		&gpio_ite_data_##inst,                             \
		&gpio_ite_cfg_##inst,                              \
		PRE_KERNEL_1,                                      \
		65,                         \
		&gpio_ite_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_ITE_DEV_CFG_DATA)

static int gpio_it8xxx2_init_set(const struct device *arg)
{
	ARG_UNUSED(arg);

	if (IS_ENABLED(CONFIG_SOC_IT8XXX2_GPIO_GROUP_K_L_DEFAULT_PULL_DOWN)) {
		const struct device *const gpiok = DEVICE_DT_GET(DT_NODELABEL(gpiok));
		const struct device *const gpiol = DEVICE_DT_GET(DT_NODELABEL(gpiol));

		for (int i = 0; i < 8; i++) {
			gpio_pin_configure(gpiok, i, GPIO_INPUT | GPIO_PULL_DOWN);
			gpio_pin_configure(gpiol, i, GPIO_INPUT | GPIO_PULL_DOWN);
		}
	}

	if (IS_ENABLED(CONFIG_SOC_IT8XXX2_GPIO_H7_DEFAULT_OUTPUT_LOW)) {
		const struct device *const gpioh = DEVICE_DT_GET(DT_NODELABEL(gpioh));

		gpio_pin_configure(gpioh, 7, GPIO_OUTPUT_LOW);
	}

	return 0;
}
SYS_INIT(gpio_it8xxx2_init_set, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY);
