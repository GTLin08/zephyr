/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/gpio.h>
#include <device.h>


#define GPIO_DEV_B DT_LABEL(DT_ALIAS(gpiob))
#define GPIO_DEV_F DT_LABEL(DT_ALIAS(gpiof))
#define LED_GPIO_DEV DT_GPIO_LABEL(DT_ALIAS(led0), gpios)
#define LED_GPIO_PIN DT_GPIO_PIN(DT_ALIAS(led0), gpios)
#define LED_GPIO_FLAGS	DT_GPIO_FLAGS(DT_ALIAS(led0), gpios)

void main(void)
{
	const struct device *button;
	int ret;

	printk("Hello World! %s\n", CONFIG_BOARD);


	button = device_get_binding(LED_GPIO_DEV);
	if (button == NULL) {
		printk("Error: didn't find %s device\n", LED_GPIO_DEV);
		return;
	}

	ret = gpio_pin_configure(button, LED_GPIO_PIN, LED_GPIO_FLAGS);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, LED_GPIO_DEV, LED_GPIO_PIN);
		return;
	}

	ret = gpio_pin_interrupt_configure(button,
					   0,
					   GPIO_INT_EDGE_TO_ACTIVE);

	ret = gpio_pin_interrupt_configure(button,
					   2,
					   GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin 2\n",
			ret, GPIO_DEV_F);
		return;
	}

}
