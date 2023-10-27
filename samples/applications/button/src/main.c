/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <inttypes.h>

#define SLEEP_TIME_MS	1

static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(DT_NODELABEL(sw0), gpios);
static struct gpio_callback button1_cb_data;
static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET(DT_NODELABEL(sw1), gpios);
static struct gpio_callback button2_cb_data;

static struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_NODELABEL(led0), gpios);
static struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_NODELABEL(led1), gpios);
static struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_NODELABEL(led2), gpios);

#define thread1_prio  1
static struct k_thread tdata1;

#define thread2_prio  3
static struct k_thread tdata2;

struct k_sem sem_thread1;
struct k_sem sem_thread2;

#define STACK_SIZE  512
K_THREAD_STACK_DEFINE(thread1_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread2_stack, STACK_SIZE);

void thread1_entry(void *p1, void *p2, void *p3) {

	printk("thread1_entry\n");
	while (1) {
		/* Suspend thread1 */
		k_sem_take(&sem_thread1, K_FOREVER);

		printk("Thread 1: Button a0 makes the light flash.\n");
		gpio_pin_toggle_dt(&led1);
		printk("Thread 1: end!\n");
	}
}

void thread2_entry(void *p1, void *p2, void *p3) {

	printk("thread2_entry\n");
	while (1) {
		/* Suspend thread2 */
		k_sem_take(&sem_thread2, K_FOREVER);

		printk("Thread 2: Button a1 makes the light flash.\n");
		gpio_pin_toggle_dt(&led2);
		printk("Thread 2: end!\n");
	}
}

void button1_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	/* Wake up theard1 */
	k_sem_give(&sem_thread1);
}
		    
void button2_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	/* Wake up theard2 */
	k_sem_give(&sem_thread2);
}

void gpio_button1_init(void) {
	int ret;

	if (!gpio_is_ready_dt(&button1)) {
		printk("Error: button device %s is not ready\n",
		       button1.port->name);
		return;
	}

	ret = gpio_pin_configure_dt(&button1, GPIO_INPUT | GPIO_PULL_UP);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button1.port->name, button1.pin);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&button1,
					      GPIO_INT_EDGE_FALLING);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button1.port->name, button1.pin);
		return;
	}

	gpio_init_callback(&button1_cb_data, button1_pressed, BIT(button1.pin));
	gpio_add_callback(button1.port, &button1_cb_data);

	printk("Set up button at %s pin %d\n", button1.port->name, button1.pin);
}

void gpio_button2_init(void) {
	int ret;

	if (!gpio_is_ready_dt(&button2)) {
		printk("Error: button device %s is not ready\n",
		       button2.port->name);
		return;
	}

	ret = gpio_pin_configure_dt(&button2, GPIO_INPUT | GPIO_PULL_UP);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button2.port->name, button2.pin);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&button2,
					      GPIO_INT_EDGE_FALLING);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button2.port->name, button2.pin);
		return;
	}

	gpio_init_callback(&button2_cb_data, button2_pressed, BIT(button2.pin));
	gpio_add_callback(button2.port, &button2_cb_data);

	printk("Set up button at %s pin %d\n", button2.port->name, button2.pin);
}

void gpio_led0_init(void) {
	int ret;

	if (led0.port && !device_is_ready(led0.port)) {
		printk("Error: LED device %s is not ready; ignoring it\n", led1.port->name);
		led0.port = NULL;

		return;
	}
	if (led0.port) {
		ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT);
		if (ret != 0) {
			printk("Error %d: failed to configure LED device %s pin %d\n",
			       ret, led0.port->name, led0.pin);
			led0.port = NULL;

			return;
		} else {
			printk("Set up LED at %s pin %d\n", led0.port->name, led0.pin);
		}
	}
}

void gpio_led1_init(void) {
	int ret;

	if (led1.port && !device_is_ready(led1.port)) {
		printk("Error: LED device %s is not ready; ignoring it\n", led1.port->name);
		led1.port = NULL;

		return;
	}
	if (led1.port) {
		ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT);
		if (ret != 0) {
			printk("Error %d: failed to configure LED device %s pin %d\n",
			       ret, led1.port->name, led1.pin);
			led1.port = NULL;

			return;
		} else {
			printk("Set up LED at %s pin %d\n", led1.port->name, led1.pin);
		}
	}
}

void gpio_led2_init(void) {
	int ret;

	if (led2.port && !device_is_ready(led2.port)) {
		printk("Error: LED device %s is not ready; ignoring it\n", led2.port->name);
		led2.port = NULL;

		return;
	}
	if (led2.port) {
		ret = gpio_pin_configure_dt(&led2, GPIO_OUTPUT);
		if (ret != 0) {
			printk("Error %d: failed to configure LED device %s pin %d\n",
			       ret, led2.port->name, led2.pin);
			led2.port = NULL;

			return;
		} else {
			printk("Set up LED at %s pin %d\n", led2.port->name, led2.pin);
		}
	}
}

int main(void)
{
	/* Button initialization */
	gpio_button1_init();
	gpio_button2_init();

	/* LED initialization */
	gpio_led0_init();
	gpio_led1_init();
	gpio_led2_init();

	k_sem_init(&sem_thread1, 0, 1);
	k_sem_init(&sem_thread2, 0, 1);

	k_thread_create(&tdata1, thread1_stack, STACK_SIZE,
				      thread1_entry, NULL, NULL, NULL,
				      thread1_prio, 0, K_NO_WAIT);

	k_thread_create(&tdata2, thread2_stack, STACK_SIZE,
				      thread2_entry, NULL, NULL, NULL,
				      thread2_prio, 0, K_NO_WAIT);

	k_thread_priority_set(k_current_get(), K_PRIO_PREEMPT(2));
	while (1) {
		for (int i=0;i<10;i++) {
			printk("Thread 0: Flashing at 0.5s intervals.\n");
			gpio_pin_toggle_dt(&led0);

			k_busy_wait(500000);
		}
		k_sleep(K_FOREVER);
	}

	return 0;
}
