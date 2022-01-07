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

#define PINCTRL_GROUP_COUNT	13

/* Array containing pointers to each Pinctrl port. */
const struct pinctrl_it8xxx2_config *pinctrl_config[PINCTRL_GROUP_COUNT];

struct pinctrl_it8xxx2_config {
	/* gpio port control register (byte mapping to pin) */
	uint8_t *reg_gpcr;
	/* gpio port data register (bit mapping to pin) */
	uint8_t *reg_gpdr;
	/* gpio port output type register (bit mapping to pin) */
	uint8_t *reg_gpotr;
	/* function 3 general control register */
	uintptr_t func3_gcr[8];
	/* function 3 enable mask */
	uint8_t func3_en_mask[8];
	/* function 4 general control register */
	uintptr_t func4_gcr[8];
	/* function 4 enable mask */
	uint8_t func4_en_mask[8];
	/* Input voltage selection */
	uintptr_t volt_sel[8];
	/* Input voltage selection mask */
	uint8_t volt_sel_mask[8];
	uint8_t instance;
};

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
			   uintptr_t reg)
{
	ARG_UNUSED(reg);

	uint8_t pin, port;
	volatile uint8_t *reg_gpcr = NULL;
	volatile uint8_t *reg_gpdr = NULL;
	volatile uint8_t *reg_gpotr = NULL;
	volatile uint8_t *reg_func3_gcr = NULL;
	volatile uint8_t *reg_func4_gcr = NULL;
	volatile uint8_t *reg_volt_sel = NULL;

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		port = IT8XXX2_DT_PINMUX_PORT(pins[i].pinmux);
		pin = IT8XXX2_DT_PINMUX_PIN(pins[i].pinmux);
		reg_gpcr = (uint8_t *)pinctrl_config[port]->reg_gpcr + pin;
		reg_gpdr = (uint8_t *)pinctrl_config[port]->reg_gpdr;
		reg_gpotr = (uint8_t *)pinctrl_config[port]->reg_gpotr;
		reg_func3_gcr = (uint8_t *)(pinctrl_config[port]->func3_gcr[pin]);
		reg_func4_gcr = (uint8_t *)(pinctrl_config[port]->func4_gcr[pin]);
		reg_volt_sel = (uint8_t *)(pinctrl_config[port]->volt_sel[pin]);

		/* Common settings for alternate function. */
		*reg_gpcr &= ~(GPCR_PORT_PIN_MODE_INPUT |
			       GPCR_PORT_PIN_MODE_OUTPUT);

		/* Handle alternate function. */
		switch (IT8XXX2_DT_PINMUX_FUNC(pins[i].pinmux)) {
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
			*reg_func3_gcr |= pinctrl_config[port]->func3_en_mask[pin];
			break;
		case IT8XXX2_PINMUX_FUNC_4:
			/*
			 * Func4: In addition to the alternate setting above,
			 *        Func4 also need to set the general control.
			 */
			*reg_func4_gcr |= pinctrl_config[port]->func4_en_mask[pin];
			break;
		case IT8XXX2_PINMUX_DEFAULT:
			*reg_gpcr |= GPCR_PORT_PIN_MODE_INPUT;
			*reg_func3_gcr &= ~pinctrl_config[port]->func3_en_mask[pin];
			*reg_func4_gcr &= ~pinctrl_config[port]->func4_en_mask[pin];
			break;
		default:
			LOG_ERR("This function is not supported.");
			return -EINVAL;
		}

		/* Handle pull-up or pull-down. */
		switch (IT8XXX2_DT_PINCFG_PUPDR(pins[i].pincfg)) {
		case IT8XXX2_NO_PULL:
			/* No pull-up or pull-down */
			*reg_gpcr &= ~(GPCR_PORT_PIN_MODE_PULLUP |
				       GPCR_PORT_PIN_MODE_PULLDOWN);
			break;
		case IT8XXX2_PULL_UP:
			*reg_gpcr = (*reg_gpcr | GPCR_PORT_PIN_MODE_PULLUP) &
				     ~GPCR_PORT_PIN_MODE_PULLDOWN;
			break;
		case IT8XXX2_PULL_DOWN:
			*reg_gpcr = (*reg_gpcr | GPCR_PORT_PIN_MODE_PULLDOWN) &
				     ~GPCR_PORT_PIN_MODE_PULLUP;
			break;
		default:
			LOG_ERR("This pull level is not supported.");
			return -EINVAL;
		}

		/* Handle output type of push-pull or open-down. */
		switch (IT8XXX2_DT_PINCFG_OTYPER(pins[i].pincfg)) {
		case IT8XXX2_PUSH_PULL:
			*reg_gpotr &= ~BIT(pin);
			break;
		case IT8XXX2_OPEN_DRAIN:
			*reg_gpotr |= BIT(pin);
			break;
		default:
			LOG_ERR("The output type is not supported.");
			return -EINVAL;
		}

		/* Handle output high or low. */
		if (IT8XXX2_DT_PINCFG_OVAL(pins[i].pincfg)) {
			switch (IT8XXX2_DT_PINCFG_OVAL(pins[i].pincfg)) {
			case IT8XXX2_OVAL_HIGH:
				*reg_gpdr |= BIT(pin);
				break;
			case IT8XXX2_OVAL_LOW:
				*reg_gpdr &= ~BIT(pin);
				break;
			default:
				LOG_ERR("The output level is not supported.");
				return -EINVAL;
			}
			/* Set pin to output. */
			*reg_gpcr = (*reg_gpcr | GPCR_PORT_PIN_MODE_OUTPUT) &
				     ~GPCR_PORT_PIN_MODE_INPUT;
		}

		/* Handle voltage 3.3V or 1.8V. */
		switch (IT8XXX2_DT_PINCFG_VOLTAGE(pins[i].pincfg)) {
		case IT8XXX2_VOLTAGE_DEFAULT:
			break;
		case IT8XXX2_VOLTAGE_3P3:
			/* Input voltage selection 3.3V. */
			*reg_volt_sel &= ~pinctrl_config[port]->volt_sel_mask[pin];
			break;
		case IT8XXX2_VOLTAGE_1P8:
			__ASSERT(!(IT8XXX2_DT_PINCFG_PUPDR(pins[i].pincfg)
				   == IT8XXX2_PULL_UP),
			"Don't enable internal pullup if 1.8V voltage is used");
			/* Input voltage selection 1.8V. */
			*reg_volt_sel |= pinctrl_config[port]->volt_sel_mask[pin];
			break;
		default:
			LOG_ERR("The voltage selection is not supported");
			return -EINVAL;
		}

	}

	return 0;
}

static int pinctrl_it8xxx2_init(const struct device *dev)
{
	struct gpio_it8xxx2_regs *const gpio_base = GPIO_IT8XXX2_REG_BASE;
	const struct pinctrl_it8xxx2_config *const config = dev->config;

	pinctrl_config[config->instance] = dev->config;

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
		.reg_gpcr = (uint8_t *)DT_INST_REG_ADDR_BY_IDX(inst, 0),            \
		.reg_gpdr = (uint8_t *)DT_INST_REG_ADDR_BY_IDX(inst, 1),            \
		.reg_gpotr = (uint8_t *)DT_INST_REG_ADDR_BY_IDX(inst, 2),           \
		.func3_gcr = DT_INST_PROP(inst, func3_gcr),                         \
		.func3_en_mask = DT_INST_PROP(inst, func3_en_mask),                 \
		.func4_gcr = DT_INST_PROP(inst, func4_gcr),                         \
		.func4_en_mask = DT_INST_PROP(inst, func4_en_mask),                 \
		.volt_sel = DT_INST_PROP(inst, volt_sel),                           \
		.volt_sel_mask = DT_INST_PROP(inst, volt_sel_mask),                 \
		.instance = inst,                                                   \
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

