/*
 * Copyright (c) 2020 ITE Corporation. All Rights Reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <init.h>
#include <drivers/pinmux.h>
#include <soc.h>

static int it8xxx2_evb_pinmux_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	//const struct device *p = DEVICE_DT_GET(DT_NODELABEL(pinmux));

	//__ASSERT_NO_MSG(device_is_ready(p));

#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmuxa), okay)
	const struct device *porta =
		device_get_binding(DT_LABEL(DT_NODELABEL(pinmuxa)));
	__ASSERT_NO_MSG(device_is_ready(porta));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmuxb), okay)
	const struct device *portb =
		device_get_binding(DT_LABEL(DT_NODELABEL(pinmuxb)));
	__ASSERT_NO_MSG(device_is_ready(portb));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmuxc), okay)
	const struct device *portc =
		device_get_binding(DT_LABEL(DT_NODELABEL(pinmuxc)));
	__ASSERT_NO_MSG(device_is_ready(portc));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmuxd), okay)
	const struct device *portd =
		device_get_binding(DT_LABEL(DT_NODELABEL(pinmuxd)));
	__ASSERT_NO_MSG(device_is_ready(portd));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmuxe), okay)
	const struct device *porte =
		device_get_binding(DT_LABEL(DT_NODELABEL(pinmuxe)));
	__ASSERT_NO_MSG(device_is_ready(porte));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmuxf), okay)
	const struct device *portf =
		device_get_binding(DT_LABEL(DT_NODELABEL(pinmuxf)));
	__ASSERT_NO_MSG(device_is_ready(portf));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmuxg), okay)
	const struct device *portg =
		device_get_binding(DT_LABEL(DT_NODELABEL(pinmuxg)));
	__ASSERT_NO_MSG(device_is_ready(portg));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmuxh), okay)
	const struct device *porth =
		device_get_binding(DT_LABEL(DT_NODELABEL(pinmuxh)));
	__ASSERT_NO_MSG(device_is_ready(porth));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmuxi), okay)
	const struct device *porti =
		device_get_binding(DT_LABEL(DT_NODELABEL(pinmuxi)));
	__ASSERT_NO_MSG(device_is_ready(porti));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmuxj), okay)
	const struct device *portj =
		device_get_binding(DT_LABEL(DT_NODELABEL(pinmuxj)));
	__ASSERT_NO_MSG(device_is_ready(portj));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pinmuxm), okay)
	const struct device *portm =
		device_get_binding(DT_LABEL(DT_NODELABEL(pinmuxm)));
	__ASSERT_NO_MSG(device_is_ready(portm));
#endif

	//pinmux_pin_set(portb, 0, IT8XXX2_PINMUX_IOF0);	/* SIN0 */
	//pinmux_pin_set(portb, 1, IT8XXX2_PINMUX_IOF0);	/* SOUT0 */
	//pinmux_pin_set(portf, 3, IT8XXX2_PINMUX_IOF0);	/* RTS0# */
	//pinmux_pin_set(portd, 5, IT8XXX2_PINMUX_IOF0);	/* CTS0# */

	pinmux_pin_set(porta, 5, IT8XXX2_PINMUX_IOF1);
	printk("reg1615_val=%x\n", sys_read8(0xF01615));


#if 0
	const struct device *p =
		device_get_binding(CONFIG_PINMUX_ITE_IT8XXX2_NAME);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart1), okay)
	pinmux_pin_set(p, 0, IT8XXX2_PINMUX_IOF1);
	pinmux_pin_set(p, 56, IT8XXX2_PINMUX_IOF1);
#endif	/* DT_NODE_HAS_STATUS(DT_NODELABEL(uart1), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart2), okay)
	pinmux_pin_set(p, 3, IT8XXX2_PINMUX_IOF1);
	pinmux_pin_set(p, 59, IT8XXX2_PINMUX_IOF1);
#endif	/* DT_NODE_HAS_STATUS(DT_NODELABEL(uart2), okay) */
#endif


	return 0;
}

SYS_INIT(it8xxx2_evb_pinmux_init, POST_KERNEL, CONFIG_PINMUX_INIT_PRIORITY);
