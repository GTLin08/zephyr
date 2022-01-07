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

#define IT8XXX2_PORT_SHIFT 16U
#define IT8XXX2_PORT_MASK  0xFFU
#define IT8XXX2_PIN_SHIFT  8U
#define IT8XXX2_PIN_MASK   0xFFU
#define IT8XXX2_FUNC_SHIFT 0U
#define IT8XXX2_FUNC_MASK  0xFFU

#define IT8XXX2_PINMUX(port, pin, func)					        \
		(((((port) - 'A') & IT8XXX2_PORT_MASK) << IT8XXX2_PORT_SHIFT) | \
		(((pin) & IT8XXX2_PIN_MASK) << IT8XXX2_PIN_SHIFT) |	        \
		(((func) & IT8XXX2_FUNC_MASK) << IT8XXX2_FUNC_SHIFT))

/**
 * @brief Utility macro to obtain IO port number.
 */
#define IT8XXX2_DT_PINMUX_PORT(__port) \
	(((__port) >> IT8XXX2_PORT_SHIFT) & IT8XXX2_PORT_MASK)

/**
 * @brief Utility macro to obtain IO pin number.
 */
#define IT8XXX2_DT_PINMUX_PIN(__pin)   \
	(((__pin) >> IT8XXX2_PIN_SHIFT) & IT8XXX2_PIN_MASK)

/**
 * @brief Utility macro to obtain pin function.
 */
#define IT8XXX2_DT_PINMUX_FUNC(__func) \
	(((__func) >> IT8XXX2_FUNC_SHIFT) & IT8XXX2_FUNC_MASK)

/**
 * @brief PIN configuration bitfield.
 *
 * Pin configuration is coded with the following
 * fields.
 *    GPIO pull-up/down config  [ 0 : 2 ]
 *    GPIO output type config   [ 4 : 5 ]
 *    GPIO output level config  [ 9 : 10 ]
 *    GPIO voltage selection    [ 12 : 14 ]
 */
#define IT8XXX2_NO_PULL          0x0U
#define IT8XXX2_PULL_UP          0x1U
#define IT8XXX2_PULL_DOWN        0x2U
#define IT8XXX2_PUSH_PULL        0x0U
#define IT8XXX2_OPEN_DRAIN       0x1U
#define IT8XXX2_OVAL_HIGH        0x1U
#define IT8XXX2_OVAL_LOW         0x2U
#define IT8XXX2_VOLTAGE_DEFAULT  0x0U
#define IT8XXX2_VOLTAGE_3P3      0x1U
#define IT8XXX2_VOLTAGE_1P8      0x2U

/* GPIO pull-up or pull-down */
#define IT8XXX2_PUPDR_SHIFT      0U
#define IT8XXX2_PUPDR_MASK       0x3U
/* GPIO push-pull or open-drain */
#define IT8XXX2_OTYPER_SHIFT     4U
#define IT8XXX2_OTYPER_MASK      0x3U
/* GPIO output high or low */
#define IT8XXX2_OVAL_SHIFT       8U
#define IT8XXX2_OVAL_MASK        0x3U
/* GPIO 1.8V or 3.3V */
#define IT8XXX2_VOLTAGE_SHIFT    12U
#define IT8XXX2_VOLTAGE_MASK     0x3U

/**
 * @brief Utility macro to obtain configuration of pull-up or pull-down.
 */
#define IT8XXX2_DT_PINCFG_PUPDR(__mode) \
	(((__mode) >> IT8XXX2_PUPDR_SHIFT) & IT8XXX2_PUPDR_MASK)

/**
 * @brief Utility macro to obtain configuration of push-pull or open-drain.
 */
#define IT8XXX2_DT_PINCFG_OTYPER(__mode) \
	(((__mode) >> IT8XXX2_OTYPER_SHIFT) & IT8XXX2_OTYPER_MASK)

/**
 * @brief Utility macro to obtain configuration of output high or low.
 */
#define IT8XXX2_DT_PINCFG_OVAL(__mode) \
	(((__mode) >> IT8XXX2_OVAL_SHIFT) & IT8XXX2_OVAL_MASK)

/**
 * @brief Utility macro to obtain input voltage selection.
 */
#define IT8XXX2_DT_PINCFG_VOLTAGE(__mode) \
	(((__mode) >> IT8XXX2_VOLTAGE_SHIFT) & IT8XXX2_VOLTAGE_MASK)

#endif	/* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_IT8XXX2_PINCTRL_H_ */
