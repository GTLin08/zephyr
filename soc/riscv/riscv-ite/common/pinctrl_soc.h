/*
 * Copyright (c) 2022 ITE Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_RISCV_ITE_IT8XXX2_COMMON_PINCTRL_SOC_H_
#define ZEPHYR_SOC_RISCV_ITE_IT8XXX2_COMMON_PINCTRL_SOC_H_

#include <devicetree.h>
#include <zephyr/types.h>

#include <dt-bindings/pinctrl/it8xxx2-pinctrl.h>

/**
 * @brief ITE IT8XXX2 pin type.
 */
typedef struct pinctrl_soc_pin {
	/** Pinmux settings (port, pin and function). */
	uint32_t pinmux;
	/** Pin configuration (pullup, pulldown, voltate selection). */
	uint32_t pincfg;
} pinctrl_soc_pin_t;

/**
 * @brief Utility macro to initialize pincfg field in #pinctrl_pin_t (F1).
 *
 * @param node_id Node identifier.
 */
#define Z_PINCTRL_IT8XXX2_PINCFG_INIT(node_id)                                                    \
	(((IT8XXX2_NO_PULL * DT_PROP(node_id, bias_disable)) << IT8XXX2_PUPDR_SHIFT) |            \
	 ((IT8XXX2_PULL_UP * DT_PROP(node_id, bias_pull_up)) << IT8XXX2_PUPDR_SHIFT) |            \
	 ((IT8XXX2_PULL_DOWN * DT_PROP(node_id, bias_pull_down)) << IT8XXX2_PUPDR_SHIFT) |        \
	 ((IT8XXX2_PUSH_PULL * DT_PROP(node_id, drive_push_pull)) << IT8XXX2_OTYPER_SHIFT) |      \
	 ((IT8XXX2_OPEN_DRAIN * DT_PROP(node_id, drive_open_drain)) << IT8XXX2_OTYPER_SHIFT) |    \
	 ((IT8XXX2_OVAL_HIGH * DT_PROP(node_id, output_high)) << IT8XXX2_OVAL_SHIFT) |            \
	 ((IT8XXX2_OVAL_LOW * DT_PROP(node_id, output_low)) << IT8XXX2_OVAL_SHIFT) |              \
	 ((IT8XXX2_VOLTAGE_3P3 * DT_PROP(node_id, gpio_voltage_3p3)) << IT8XXX2_VOLTAGE_SHIFT) |  \
	 ((IT8XXX2_VOLTAGE_1P8 * DT_PROP(node_id, gpio_voltage_1p8)) << IT8XXX2_VOLTAGE_SHIFT))

/**
 * @brief Utility macro to initialize pinmux field in #pinctrl_pin_t.
 *
 * @param node_id Node identifier.
 */
#define Z_PINCTRL_IT8XXX2_PINMUX_INIT(node_id) DT_PROP(node_id, pinmux)

/**
 * @brief Utility macro to initialize each pin.
 *
 * @param node_id Node identifier.
 * @param prop Property name.
 * @param idx Property entry index.
 */
#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)          \
	{ .pinmux = Z_PINCTRL_IT8XXX2_PINMUX_INIT(            \
		DT_PROP_BY_IDX(node_id, prop, idx)),          \
	  .pincfg = Z_PINCTRL_IT8XXX2_PINCFG_INIT(            \
		DT_PROP_BY_IDX(node_id, prop, idx)) },

/**
 * @brief Utility macro to initialize state pins contained in a given property.
 *
 * @param node_id Node identifier.
 * @param prop Property name describing state pins.
 */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)              \
	{DT_FOREACH_PROP_ELEM(node_id, prop, Z_PINCTRL_STATE_PIN_INIT)}

#endif /* ZEPHYR_SOC_RISCV_ITE_IT8XXX2_COMMON_PINCTRL_SOC_H_ */
