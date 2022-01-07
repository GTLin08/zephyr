/*
 * Copyright (c) 2022 ITE Corporation. All Rights Reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ite_it8xxx2_pinctrl_func

#include <drivers/gpio.h>
#include <drivers/pinctrl.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(pinctrl_ite_it8xxx2, LOG_LEVEL_ERR);

#define GPIO_IT8XXX2_REG_BASE \
	((struct gpio_it8xxx2_regs *)DT_REG_ADDR(DT_NODELABEL(gpiogcr)))

struct pinctrl_it8xxx2_config {
	/* gpio port control register (byte mapping to pin) */
	uint8_t *reg_gpcr;
	/* function 3 general control register */
	uintptr_t func3_gcr[8];
	/* function 3 enable mask */
	uint8_t func3_en_mask[8];
	/* function 4 general control register */
	uintptr_t func4_gcr[8];
	/* function 4 enable mask */
	uint8_t func4_en_mask[8];
	/* GPIO cells */
	const struct gpio_dt_spec gpio_dev;
};

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
			   uintptr_t reg)
{
	ARG_UNUSED(reg);

	uint8_t pin;
	volatile uint8_t *reg_gpcr = NULL;
	volatile uint8_t *reg_func3_gcr = NULL;
	volatile uint8_t *reg_func4_gcr = NULL;
	const struct device *gpio_device;

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		const struct pinctrl_it8xxx2_config *pinctrl_config =
			pins[i].pinctrls->config;
		pin = pins[i].pin;
		reg_gpcr = (uint8_t *)(pinctrl_config->reg_gpcr + pin);
		reg_func3_gcr = (uint8_t *)(pinctrl_config->func3_gcr[pin]);
		reg_func4_gcr = (uint8_t *)(pinctrl_config->func4_gcr[pin]);
		gpio_device = pinctrl_config->gpio_dev.port;

		/* Handle tri-state mode. */
		if (IT8XXX2_DT_PINCFG_IMPEDANCE(pins[i].pincfg)) {
			*reg_gpcr |= (GPCR_PORT_PIN_MODE_PULLUP |
				      GPCR_PORT_PIN_MODE_PULLDOWN);
		}

		/* Handle GPIO pin configuration. */
		if (pins[i].pincfg) {
			gpio_pin_configure(gpio_device, pin, pins[i].pincfg);
		}

		/* Common settings for alternate function. */
		*reg_gpcr &= ~(GPCR_PORT_PIN_MODE_INPUT |
			       GPCR_PORT_PIN_MODE_OUTPUT);

		/* Handle alternate function. */
		switch (pins[i].alt_fun) {
		case IT8XXX2_PINMUX_FUNC_1:
			/* Func1: Alternate function has been set above. */
			break;
		case IT8XXX2_PINMUX_FUNC_2:
			/* Func2: WUI function: turn the pin into an input */
			*reg_gpcr |= GPCR_PORT_PIN_MODE_INPUT;
			break;
		case IT8XXX2_PINMUX_FUNC_3:
			/*
			 * Func3: In addition to the alternate setting above,
			 *        Func3 also need to set the general control.
			 */
			*reg_func3_gcr |= pinctrl_config->func3_en_mask[pin];
			break;
		case IT8XXX2_PINMUX_FUNC_4:
			/*
			 * Func4: In addition to the alternate setting above,
			 *        Func4 also need to set the general control.
			 */
			*reg_func4_gcr |= pinctrl_config->func4_en_mask[pin];
			break;
		case IT8XXX2_PINMUX_DEFAULT:
			*reg_gpcr |= GPCR_PORT_PIN_MODE_INPUT;
			*reg_func3_gcr &= ~pinctrl_config->func3_en_mask[pin];
			*reg_func4_gcr &= ~pinctrl_config->func4_en_mask[pin];
			break;
		default:
			LOG_ERR("This function is not supported.");
			return -EINVAL;
		}

	}

	return 0;
}

static int pinctrl_it8xxx2_init(const struct device *dev)
{
	struct gpio_it8xxx2_regs *const gpio_base = GPIO_IT8XXX2_REG_BASE;

	/*
	 * The default value of LPCRSTEN is bit2:1 = 10b(GPD2) in GCR.
	 * If LPC reset is enabled on GPB7, we have to clear bit2:1
	 * to 00b.
	 */
	gpio_base->GPIO_GCR &= ~IT8XXX2_GPIO_LPCRSTEN;

	/*
	 * TODO: If UART2 swaps from bit2:1 to bit6:5 in H group, we
	 * have to set UART1PSEL = 1 in UART1PMR register.
	 */

	return 0;
}

#define PINCTRL_ITE_INIT(inst)                                                      \
	static const struct pinctrl_it8xxx2_config pinctrl_it8xxx2_cfg_##inst = {   \
		.reg_gpcr = (uint8_t *)DT_INST_REG_ADDR(inst),                      \
		.func3_gcr = DT_INST_PROP(inst, func3_gcr),                         \
		.func3_en_mask = DT_INST_PROP(inst, func3_en_mask),                 \
		.func4_gcr = DT_INST_PROP(inst, func4_gcr),                         \
		.func4_en_mask = DT_INST_PROP(inst, func4_en_mask),                 \
		.gpio_dev = GPIO_DT_SPEC_INST_GET(inst, gpios),                     \
	};                                                                          \
                                                                                    \
	DEVICE_DT_INST_DEFINE(inst, &pinctrl_it8xxx2_init,                          \
			      NULL,                                                 \
			      NULL,                                                 \
			      &pinctrl_it8xxx2_cfg_##inst,                          \
			      PRE_KERNEL_1,                                         \
			      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                  \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(PINCTRL_ITE_INIT)

