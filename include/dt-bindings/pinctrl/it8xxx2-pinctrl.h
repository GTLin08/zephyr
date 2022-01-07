/*
 * Copyright (c) 2021 ITE Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_IT8XXX2_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_IT8XXX2_PINCTRL_H_

#define NO_FUNC 0

/**
 * @brief PIN alternate function.
 */
#define IT8XXX2_PINMUX_FUNC_1  0U
#define IT8XXX2_PINMUX_FUNC_2  1U
#define IT8XXX2_PINMUX_FUNC_3  2U
#define IT8XXX2_PINMUX_FUNC_4  3U
#define IT8XXX2_PINMUX_DEFAULT 4U
#define IT8XXX2_PINMUX_PINS    8U

/**
 * @brief PIN configuration bitfield.
 *
 * Pin configuration is coded with the following
 * fields.
 *    GPIO high impedance       [ 0 ]
 *    GPIO pull-up/down config  [ 4 : 5 ]
 *    GPIO input enable config  [ 16 ]
 *    GPIO voltage selection    [ 11 : 12 ]
 */
#define IT8XXX2_HIGH_IMPEDANCE     0x1U
#define IT8XXX2_PULL_PIN_DEFAULT   0x0U
#define IT8XXX2_PULL_UP            0x1U
#define IT8XXX2_PULL_DOWN          0x2U
#define IT8XXX2_VOLTAGE_1P8        0x1U
#define IT8XXX2_VOLTAGE_3P3        0x2U
#define IT8XXX2_INPUT_ENABLE       0x1U
#define IT8XXX2_INPUT_DISABLE      0x2U

/* GPIO tri-state mode. */
#define IT8XXX2_IMPEDANCE_SHIFT    0U
#define IT8XXX2_IMPEDANCE_MASK     0x1U
/* GPIO pull-up or pull-down */
#define IT8XXX2_PUPDR_SHIFT        4U
#define IT8XXX2_PUPDR_MASK         0x3U
/* GPIO 1.8V or 3.3V */
#define IT8XXX2_VOLTAGE_SHIFT      11U
#define IT8XXX2_VOLTAGE_MASK       0x3U
/* GPIO INPUT enable or disable */
#define IT8XXX2_INPUT_SHIFT        16U
#define IT8XXX2_INPUT_MASK         0x3U

/**
 * @brief Utility macro to obtain configuration of tri-state.
 */
#define IT8XXX2_DT_PINCFG_IMPEDANCE(__mode) \
	(((__mode) >> IT8XXX2_IMPEDANCE_SHIFT) & IT8XXX2_IMPEDANCE_MASK)

#endif	/* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_IT8XXX2_PINCTRL_H_ */
